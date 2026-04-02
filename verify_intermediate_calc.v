`timescale 1ns / 1ps

module verify_intermediate_calc #(
    parameter WIDTH   = 24,
    parameter Q       = 24'd8380417,
    parameter K_PARAM = 4'd4,
    parameter L_PARAM = 4'd4,
    parameter TAU     = 8'd39
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          i_start,
    
    input  wire [255:0]  i_rho,
    input  wire [255:0]  i_c_tilde,
    input  wire [255:0]  i_M,
    
    output wire [8:0]    o_pk_raddr,
    input  wire [31:0]   i_pk_rdata,
    
    output reg           o_done,
    output reg  [511:0]  o_u,
    
    input  wire [11:0]   i_A_raddr,
    output wire [23:0]   o_A_rdata,
    input  wire [7:0]    i_c_raddr,
    output wire [23:0]   o_c_rdata
);

    reg  [2:0]  a_i, a_j; reg a_start; wire a_done; wire a_valid; wire [22:0] a_data;
    
    Rejsam_a u_rejsam_a (
        .clk(clk), .rst_n(rst_n), .i_start(a_start), .i_rho(i_rho),
        .i_row({5'd0, a_i}), .i_column({5'd0, a_j}),
        .o_coeff_valid(a_valid), .o_coeff_data(a_data), .o_done(a_done)
    );

    (* ram_style = "block" *) reg [23:0] A_ram [0:4095];
    reg [7:0] a_coeff_cnt;
    
    always @(posedge clk) begin
        if (a_start) begin
            a_coeff_cnt <= 0;
        end else if (a_valid) begin
            A_ram[((a_i * L_PARAM + a_j) * 256) + a_coeff_cnt] <= {1'b0, a_data};
            a_coeff_cnt <= a_coeff_cnt + 1;
        end
    end
    
    reg [23:0] A_rdata_reg;
    always @(posedge clk) A_rdata_reg <= A_ram[i_A_raddr];
    assign o_A_rdata = A_rdata_reg;

    reg tr_start; wire tr_done; wire [511:0] tr_out;
    calc_tr u_calc_tr (
        .clk(clk), .rst_n(rst_n), .i_start(tr_start),
        .o_pk_raddr(o_pk_raddr), .i_pk_rdata(i_pk_rdata), .o_done(tr_done), .o_tr(tr_out)
    );

    reg u_start; reg u_squeeze_req; wire u_squeeze_valid; wire [511:0] u_shake_out;
    SHAKE256 #( .OUTPUT_LEN_BYTES(64), .ABSORB_LEN(768) ) u_shake_u (
        .clk(clk), .rst_n(rst_n), .i_start(u_start), .i_seed({i_M, tr_out}), 
        .o_busy(), .i_squeeze_req(u_squeeze_req), .o_squeeze_valid(u_squeeze_valid), .o_squeeze_data(u_shake_out)
    );

    // ==========================================
    // ★★★ 核心修复：Ping-Pong MUX 逻辑 ★★★
    // ==========================================
    // ★ 修复点 1：增加 ST_CLEAR_C_RAM 状态
    localparam ST_IDLE=4'd0, ST_GEN_A_START=4'd1, ST_GEN_A_WAIT_ACK=4'd8, ST_GEN_A_WAIT_DONE=4'd9, 
               ST_CALC_TR=4'd2, ST_CALC_U_REQ=4'd3, ST_CALC_U_WAIT=4'd4, 
               ST_CLEAR_C_RAM=4'd10, // 新增的清零状态
               ST_SAMPLE_C=4'd5, ST_NTT_C=4'd6, ST_DONE=4'd7;
    
    reg [3:0] state;
    reg [8:0] clr_cnt; // 用于清零 RAM 的计数器

    wire is_ntt_active    = (state == ST_NTT_C);
    wire is_sample_active = (state == ST_SAMPLE_C);
    wire is_clear_active  = (state == ST_CLEAR_C_RAM); // 判断是否在清零状态

    reg ball_start; wire ball_done; wire c_we; wire [7:0] c_addr; wire [23:0] c_wdata;
    
    // 提前宣告 ram0_rdata_a 供 SampleInBall 读取旧值（如有需要）
    wire [23:0] ram0_rdata_a;
    
    SampleInBall #( .TAU(TAU), .Q(Q) ) u_sample_ball (
        .clk(clk), .rst_n(rst_n), .i_start(ball_start), .i_c1(i_c_tilde), 
        .o_c_we(c_we), .o_c_addr(c_addr), .o_c_wdata(c_wdata), .i_c_rdata(ram0_rdata_a), .o_done(ball_done)
    );

    // NTT 核心接口宣告
    reg ntt_start; wire ntt_done;
    wire [7:0]  ntt_ram0_addr_a, ntt_ram0_addr_b, ntt_ram1_addr_a, ntt_ram1_addr_b;
    wire        ntt_ram0_we_a,   ntt_ram0_we_b,   ntt_ram1_we_a,   ntt_ram1_we_b;
    wire [23:0] ntt_ram0_wdata_a,ntt_ram0_wdata_b,ntt_ram1_wdata_a,ntt_ram1_wdata_b;
    wire [23:0] ram0_rdata_b, ram1_rdata_a, ram1_rdata_b;

    ntt_core #( .WIDTH(WIDTH) ) u_ntt_c (
        .clk(clk), .rst_n(rst_n), .start(ntt_start), .done(ntt_done),
        .ram0_addr_a(ntt_ram0_addr_a), .ram0_we_a(ntt_ram0_we_a), .ram0_wdata_a(ntt_ram0_wdata_a), .ram0_rdata_a(ram0_rdata_a), 
        .ram0_addr_b(ntt_ram0_addr_b), .ram0_we_b(ntt_ram0_we_b), .ram0_wdata_b(ntt_ram0_wdata_b), .ram0_rdata_b(ram0_rdata_b),
        .ram1_addr_a(ntt_ram1_addr_a), .ram1_we_a(ntt_ram1_we_a), .ram1_wdata_a(ntt_ram1_wdata_a), .ram1_rdata_a(ram1_rdata_a), 
        .ram1_addr_b(ntt_ram1_addr_b), .ram1_we_b(ntt_ram1_we_b), .ram1_wdata_b(ntt_ram1_wdata_b), .ram1_rdata_b(ram1_rdata_b)
    );

    // --- RAM0 路由逻辑 ---
    // ★ 修复点 2：把清零状态 (is_clear_active) 加入 RAM MUX
    wire        mux_ram0_we_a    = is_ntt_active ? ntt_ram0_we_a    : (is_clear_active ? 1'b1 : (is_sample_active ? c_we : 1'b0));
    wire [7:0]  mux_ram0_addr_a  = is_ntt_active ? ntt_ram0_addr_a  : (is_clear_active ? clr_cnt[7:0] : (is_sample_active ? c_addr : i_c_raddr));
    wire [23:0] mux_ram0_wdata_a = is_ntt_active ? ntt_ram0_wdata_a : (is_clear_active ? 24'd0 : c_wdata);

    wire        mux_ram0_we_b    = is_ntt_active ? ntt_ram0_we_b : 1'b0;
    wire [7:0]  mux_ram0_addr_b  = is_ntt_active ? ntt_ram0_addr_b : 8'd0;
    wire [23:0] mux_ram0_wdata_b = is_ntt_active ? ntt_ram0_wdata_b : 24'd0;

    tdpram_24x256 u_c_ram0 (
        .clk(clk), 
        .we_a(mux_ram0_we_a), .addr_a(mux_ram0_addr_a), .din_a(mux_ram0_wdata_a), .dout_a(ram0_rdata_a), 
        .we_b(mux_ram0_we_b), .addr_b(mux_ram0_addr_b), .din_b(mux_ram0_wdata_b), .dout_b(ram0_rdata_b)
    );

    // --- RAM1 路由逻辑 (仅供 NTT 独占打乒乓) ---
    wire        mux_ram1_we_a    = is_ntt_active ? ntt_ram1_we_a : 1'b0;
    wire [7:0]  mux_ram1_addr_a  = is_ntt_active ? ntt_ram1_addr_a : 8'd0;
    wire [23:0] mux_ram1_wdata_a = is_ntt_active ? ntt_ram1_wdata_a : 24'd0;

    wire        mux_ram1_we_b    = is_ntt_active ? ntt_ram1_we_b : 1'b0;
    wire [7:0]  mux_ram1_addr_b  = is_ntt_active ? ntt_ram1_addr_b : 8'd0;
    wire [23:0] mux_ram1_wdata_b = is_ntt_active ? ntt_ram1_wdata_b : 24'd0;

    tdpram_24x256 u_c_ram1 (
        .clk(clk), 
        .we_a(mux_ram1_we_a), .addr_a(mux_ram1_addr_a), .din_a(mux_ram1_wdata_a), .dout_a(ram1_rdata_a), 
        .we_b(mux_ram1_we_b), .addr_b(mux_ram1_addr_b), .din_b(mux_ram1_wdata_b), .dout_b(ram1_rdata_b)
    );

    // 最终转出数据口 (由于 NTT 特性，结果必落回 RAM0)
    assign o_c_rdata = ram0_rdata_a; 

    // ==========================================
    // 状态机
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; o_done <= 0; a_start <= 0; tr_start <= 0; u_start <= 0; ball_start <= 0; ntt_start <= 0; u_squeeze_req <= 0; a_i <= 0; a_j <= 0; clr_cnt <= 0;
        end else begin
            a_start <= 0; tr_start <= 0; u_start <= 0; ball_start <= 0; ntt_start <= 0; u_squeeze_req <= 0;
            case (state)
                ST_IDLE: begin o_done <= 0; a_i <= 0; a_j <= 0; clr_cnt <= 0; if (i_start) state <= ST_GEN_A_START; end
                
                ST_GEN_A_START: begin a_start <= 1; state <= ST_GEN_A_WAIT_ACK; end
                ST_GEN_A_WAIT_ACK: begin if (!a_done) state <= ST_GEN_A_WAIT_DONE; end
                ST_GEN_A_WAIT_DONE: begin
                    if (a_done) begin 
                        if (a_j == L_PARAM - 1) begin a_j <= 0;
                            if (a_i == K_PARAM - 1) begin tr_start <= 1; state <= ST_CALC_TR; end 
                            else begin a_i <= a_i + 1; state <= ST_GEN_A_START; end
                        end else begin a_j <= a_j + 1; state <= ST_GEN_A_START; end
                    end
                end
                
                ST_CALC_TR: begin if (tr_done) begin u_start <= 1; state <= ST_CALC_U_REQ; end end
                ST_CALC_U_REQ: begin u_squeeze_req <= 1; state <= ST_CALC_U_WAIT; end
                ST_CALC_U_WAIT: begin 
                    if (u_squeeze_valid) begin 
                        o_u <= u_shake_out; 
                        clr_cnt <= 0; 
                        state <= ST_CLEAR_C_RAM; // ★ 修复点 3：先去清零 RAM
                    end 
                end
                
                // ★ 修复点 4：清零 RAM 的具体执行
                ST_CLEAR_C_RAM: begin
                    if (clr_cnt < 255) begin
                        clr_cnt <= clr_cnt + 1;
                    end else begin
                        ball_start <= 1; // 清零完成，正式启动 SampleInBall
                        state <= ST_SAMPLE_C;
                    end
                end
                
                ST_SAMPLE_C: begin if (ball_done) begin ntt_start <= 1; state <= ST_NTT_C; end end
                ST_NTT_C: begin if (ntt_done) state <= ST_DONE; end
                
                ST_DONE: begin o_done <= 1; if (!i_start) state <= ST_IDLE; end
            endcase
        end
    end
endmodule