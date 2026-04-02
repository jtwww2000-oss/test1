`timescale 1ns / 1ps

module dilithium_verify_top #(
    parameter WIDTH    = 24,
    parameter Q        = 24'd8380417,
    parameter K_PARAM  = 4'd4,
    parameter L_PARAM  = 4'd4,
    parameter TAU      = 8'd39,
    parameter LAMBDA   = 8'd128,
    parameter W1_WIDTH = 4'd6
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          i_start,
    
    input  wire [255:0]  i_M,
    output reg  [9:0]    o_pk_raddr,
    input  wire [31:0]   i_pk_rdata,
    output reg  [11:0]   o_sig_raddr,
    input  wire [31:0]   i_sig_rdata,
    
    output reg           o_done,
    output reg           o_verify_success 
);

    // ==========================================
    // 1. 顶层共享 RAM 声明
    // ==========================================
    wire        t1_we; wire [9:0]  t1_waddr, t1_raddr; wire [23:0] t1_wdata; reg [23:0] t1_rdata;
    reg  [23:0] ram_t1 [0:1023];
    always @(posedge clk) begin if (t1_we) ram_t1[t1_waddr] <= t1_wdata; t1_rdata <= ram_t1[t1_raddr]; end

    wire        z_we; wire [9:0]  z_waddr, z_raddr; wire [23:0] z_wdata; reg [23:0] z_rdata;
    reg  [23:0] ram_z [0:1023];
    always @(posedge clk) begin if (z_we) ram_z[z_waddr] <= z_wdata; z_rdata <= ram_z[z_raddr]; end

    wire        w1_we; wire [9:0]  w1_waddr, w1_raddr; wire [5:0]  w1_wdata; reg [5:0]  w1_rdata;
    reg  [5:0]  ram_w1 [0:1023];
    always @(posedge clk) begin if (w1_we) ram_w1[w1_waddr] <= w1_wdata; w1_rdata <= ram_w1[w1_raddr]; end

    // --- Hint RAM (含清零逻辑) ---
    wire        hint_we;
    wire [9:0]  hint_waddr, hint_raddr;
    wire        hint_wdata;
    reg         hint_rdata;
    reg         ram_hint [0:1023];
    always @(posedge clk) begin
        if (hint_we) ram_hint[hint_waddr] <= hint_wdata;
        hint_rdata <= ram_hint[hint_raddr];
    end

    // ==========================================
    // 2. 状态机定义
    // ==========================================
    localparam ST_IDLE       = 3'd0;
    localparam ST_UNPACK_CORE= 3'd1; 
    localparam ST_UNPACK_HINT= 3'd2; 
    localparam ST_INTER      = 3'd3; 
    localparam ST_W_CALC     = 3'd4; 
    localparam ST_FINAL      = 3'd5; 
    localparam ST_DONE       = 3'd6;

    reg [2:0] state;
    reg  start_unpack, start_hint, start_inter, start_wcalc, start_final;
    wire done_unpack,  done_hint,  done_inter,  done_wcalc,  done_final;
    reg  global_err;

    // ==========================================
    // 3. 子模块实例化
    // ==========================================
    wire [9:0]   up_pk_raddr, up_sig_raddr;
    wire [255:0] rho_val, c_tilde_val;
    wire         unpack_err;
    
    verify_unpack_ntt u_unpack_ntt (
        .clk(clk), .rst_n(rst_n), .start(start_unpack), .done(done_unpack),
        .verify_error(unpack_err), 
        .o_pk_raddr(up_pk_raddr),   .i_pk_rdata(i_pk_rdata),
        .o_sig_raddr(up_sig_raddr), .i_sig_rdata(i_sig_rdata),
        .o_rho(rho_val), .o_c_tilde(c_tilde_val),
        .o_t1_we(t1_we), .o_t1_addr(t1_waddr), .o_t1_wdata(t1_wdata),
        .o_z_we(z_we),   .o_z_addr(z_waddr),   .o_z_wdata(z_wdata)
    );

    // --- Hint 解包模块与自动清零引擎 ---
    wire [6:0] hint_y_raddr; 
    wire [7:0] hint_y_rdata; 
    wire       hint_err;
    wire       hint_unpack_we;
    wire [9:0] hint_unpack_waddr;
    wire       hint_unpack_wdata;

    HintBitUnpack u_hint_unpack (
        .clk(clk), .rst_n(rst_n), .start(start_hint), .done(done_hint), .verify_result(hint_err),
        .y_rd_addr(hint_y_raddr), .y_rd_data(hint_y_rdata),
        .hint_wr_en(hint_unpack_we), .hint_wr_addr(hint_unpack_waddr), .hint_wr_data(hint_unpack_wdata)    
    );

    wire [11:0] hint_sig_raddr = 12'd584 + {5'd0, hint_y_raddr[6:2]};
    reg [1:0] hint_byte_sel_d1;
    always @(posedge clk) hint_byte_sel_d1 <= hint_y_raddr[1:0];
    
    assign hint_y_rdata = (hint_byte_sel_d1 == 2'd0) ? i_sig_rdata[7:0]   :
                          (hint_byte_sel_d1 == 2'd1) ? i_sig_rdata[15:8]  :
                          (hint_byte_sel_d1 == 2'd2) ? i_sig_rdata[23:16] : i_sig_rdata[31:24];

    // 【关键修复】：硬件级 Hint RAM 自动清零器 (利用 Unpack 漫长的时间后台无感清零)
    reg [10:0] clear_cnt;
    wire is_clearing = (state == ST_UNPACK_CORE) && (clear_cnt < 1024);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) clear_cnt <= 0;
        else if (state == ST_IDLE) clear_cnt <= 0;
        else if (is_clearing) clear_cnt <= clear_cnt + 1;
    end

    assign hint_we    = is_clearing ? 1'b1 : hint_unpack_we;
    assign hint_waddr = is_clearing ? clear_cnt[9:0] : hint_unpack_waddr;
    assign hint_wdata = is_clearing ? 1'b0 : hint_unpack_wdata;
    // ------------------------------------

    wire [8:0]   inter_pk_raddr;
    wire [511:0] u_val;
    wire [11:0]  A_raddr;
    wire [23:0]  A_rdata;
    wire [7:0]   c_post_raddr;
    wire [23:0]  c_post_rdata;
    
    verify_intermediate_calc #( .K_PARAM(K_PARAM), .L_PARAM(L_PARAM), .TAU(TAU) ) u_inter_calc (
        .clk(clk), .rst_n(rst_n), .i_start(start_inter), .o_done(done_inter),
        .i_rho(rho_val), .i_c_tilde(c_tilde_val), .i_M(i_M),
        .o_pk_raddr(inter_pk_raddr), .i_pk_rdata(i_pk_rdata),
        .o_u(u_val),
        .i_A_raddr(A_raddr), .o_A_rdata(A_rdata),
        .i_c_raddr(c_post_raddr), .o_c_rdata(c_post_rdata)
    );

    verify_w_calc #( .K_PARAM(K_PARAM), .L_PARAM(L_PARAM) ) u_w_calc (
        .clk(clk), .rst_n(rst_n), .i_start(start_wcalc), .o_done(done_wcalc),
        .o_A_raddr(A_raddr),       .i_A_rdata(A_rdata),
        .o_c_raddr(c_post_raddr),  .i_c_rdata(c_post_rdata),
        .o_t1_raddr(t1_raddr),     .i_t1_rdata(t1_rdata),
        .o_z_raddr(z_raddr),       .i_z_rdata(z_rdata),
        .o_hint_raddr(hint_raddr), .i_hint_rdata(hint_rdata),
        .o_w1_we(w1_we), .o_w1_waddr(w1_waddr), .o_w1_wdata(w1_wdata)
    );

    wire final_success;
    verify_final_check #( .K_PARAM(K_PARAM), .W1_WIDTH(W1_WIDTH), .LAMBDA(LAMBDA) ) u_final_check (
        .clk(clk), .rst_n(rst_n), .i_start(start_final), .o_done(done_final),
        .i_u(u_val), .i_c_tilde(c_tilde_val),
        .o_w1_raddr(w1_raddr), .i_w1_rdata(w1_rdata),
        .o_verify_success(final_success)
    );

    // ==========================================
    // 4. MUX
    // ==========================================
    always @(*) begin
        if (state == ST_UNPACK_CORE) o_pk_raddr = up_pk_raddr;
        else if (state == ST_INTER) o_pk_raddr = {1'b0, inter_pk_raddr};
        else o_pk_raddr = 10'd0;
            
        if (state == ST_UNPACK_CORE) o_sig_raddr = {2'd0, up_sig_raddr}; 
        else if (state == ST_UNPACK_HINT) o_sig_raddr = hint_sig_raddr;  
        else o_sig_raddr = 12'd0;
    end

    // ==========================================
    // 5. 状态机
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; o_done <= 0; o_verify_success <= 0; global_err <= 0;
            start_unpack <= 0; start_hint <= 0; start_inter <= 0; start_wcalc <= 0; start_final <= 0;
        end else begin
            start_unpack <= 0; start_hint <= 0; start_inter <= 0; start_wcalc <= 0; start_final <= 0;
            
            case (state)
                ST_IDLE: begin
                    o_done <= 0; 
                    
                    if (i_start) begin 
                    o_verify_success <= 0; global_err <= 0;
                    start_unpack <= 1; state <= ST_UNPACK_CORE; 
                    end
                end

                ST_UNPACK_CORE: begin
                    if (done_unpack) begin
                        if (unpack_err) global_err <= 1; 
                        start_hint <= 1; state <= ST_UNPACK_HINT;
                    end
                end

                ST_UNPACK_HINT: begin
                    if (done_hint) begin
                        if (hint_err) global_err <= 1;   
                        start_inter <= 1; state <= ST_INTER;
                    end
                end

                ST_INTER: begin
                    if (done_inter) begin start_wcalc <= 1; state <= ST_W_CALC; end
                end

                ST_W_CALC: begin
                    if (done_wcalc) begin start_final <= 1; state <= ST_FINAL; end
                end

                ST_FINAL: begin
                    if (done_final) begin
                        o_verify_success <= final_success & (~global_err);
                        state <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    o_done <= 1;
                    if (!i_start) state <= ST_IDLE;
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule