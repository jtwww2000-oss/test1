`timescale 1ns / 1ps

module sign_sk_unpack_ntt (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    output reg           done,

    // ==========================================
    // 现有接口：SK 读取与 NTT 输出
    // ==========================================
    output reg  [9:0]    o_sk_raddr,
    input  wire [31:0]   i_sk_rdata,

    output reg  [255:0]  o_rho,
    output reg  [255:0]  o_K,
    output reg  [511:0]  o_tr,

    output reg           o_s1_we,
    output reg  [9:0]    o_s1_addr,
    output reg  [23:0]   o_s1_wdata,

    output reg           o_s2_we,
    output reg  [9:0]    o_s2_addr,
    output reg  [23:0]   o_s2_wdata,

    output reg           o_t0_we,
    output reg  [9:0]    o_t0_addr,
    output reg  [23:0]   o_t0_wdata,

    // ==========================================
    // 新增接口：A 矩阵与 SHAKE256 (u, rho') 结果
    // ==========================================
    input  wire [255:0]  i_M,             // 待签名的消息 M
    
    // A 矩阵输出 RAM 接口
    output reg           o_A_we,
    output reg  [11:0]   o_A_addr,
    output reg  [23:0]   o_A_wdata,

    // u 与 rho_prime 寄存器输出
    output reg  [511:0]  o_u,
    output reg  [511:0]  o_rho_prime
);

    // ==========================================
    // 参数与常量定义
    // ==========================================
    localparam [23:0] Q = 24'd8380417;

    localparam S_IDLE           = 5'd0;
    localparam S_HEADER_REQ     = 5'd1;
    localparam S_HEADER_WAIT1   = 5'd2;
    localparam S_HEADER_WAIT2   = 5'd3;
    localparam S_UNPACK_CHECK   = 5'd4;
    localparam S_UNPACK_COEFF   = 5'd5;
    localparam S_UNPACK_WRITE   = 5'd6;
    localparam S_UNPACK_NEXT    = 5'd7;
    localparam S_FETCH_REQ      = 5'd8;
    localparam S_FETCH_WAIT1    = 5'd9;
    localparam S_FETCH_WAIT2    = 5'd10;
    localparam S_NTT_START      = 5'd11;
    localparam S_NTT_WAIT       = 5'd12;
    localparam S_DUMP_REQ       = 5'd13;
    localparam S_DUMP_WAIT      = 5'd14;
    localparam S_DUMP_STORE     = 5'd15;
    localparam S_DUMP_NEXT      = 5'd16;
    
    localparam S_GEN_A_START    = 5'd17;
    localparam S_GEN_A_WAIT     = 5'd18;
    localparam S_GEN_U_START    = 5'd19;
    localparam S_GEN_U_WAIT     = 5'd20;
    localparam S_GEN_RHO_START  = 5'd21;
    localparam S_GEN_RHO_WAIT   = 5'd22;
    localparam S_DONE           = 5'd23;

    reg [4:0]  state;
    reg [4:0]  ret_state;

    reg [5:0]  header_idx;
    reg [1:0]  mode;         
    reg [2:0]  poly_idx;     
    reg [8:0]  coeff_idx;    
    reg [8:0]  dump_idx;     
    
    reg [9:0]  sk_ptr;
    reg [63:0] bit_buf;
    reg [6:0]  bit_cnt;

    // ==========================================
    // 【修改】Ping-Pong RAM 接口定义
    // ==========================================
    reg         ntt_start;
    wire        ntt_done;
    
    reg         ntt_ram_we_user;
    reg  [7:0]  ntt_ram_addr_user;
    reg  [23:0] ntt_ram_wdata_user;

    // RAM0 (主 RAM)
    reg  [7:0]  ram0_addr_a, ram0_addr_b; 
    reg         ram0_we_a,   ram0_we_b;
    reg  [23:0] ram0_wdata_a, ram0_wdata_b; 
    wire [23:0] ram0_rdata_a, ram0_rdata_b;

    // RAM1 (乒乓附属 RAM)
    reg  [7:0]  ram1_addr_a, ram1_addr_b; 
    reg         ram1_we_a,   ram1_we_b;
    reg  [23:0] ram1_wdata_a, ram1_wdata_b; 
    wire [23:0] ram1_rdata_a, ram1_rdata_b;

    // NTT 到 RAM 的连线
    wire [7:0]  ntt_ram0_addr_a, ntt_ram0_addr_b, ntt_ram1_addr_a, ntt_ram1_addr_b;
    wire        ntt_ram0_we_a,   ntt_ram0_we_b,   ntt_ram1_we_a,   ntt_ram1_we_b;
    wire [23:0] ntt_ram0_wdata_a,ntt_ram0_wdata_b,ntt_ram1_wdata_a,ntt_ram1_wdata_b;

    // 实例化 2块 256深度双端口 RAM
    tdpram_24x256 u_ram0 ( .clk(clk), .we_a(ram0_we_a), .addr_a(ram0_addr_a), .din_a(ram0_wdata_a), .dout_a(ram0_rdata_a), .we_b(ram0_we_b), .addr_b(ram0_addr_b), .din_b(ram0_wdata_b), .dout_b(ram0_rdata_b) );
    tdpram_24x256 u_ram1 ( .clk(clk), .we_a(ram1_we_a), .addr_a(ram1_addr_a), .din_a(ram1_wdata_a), .dout_a(ram1_rdata_a), .we_b(ram1_we_b), .addr_b(ram1_addr_b), .din_b(ram1_wdata_b), .dout_b(ram1_rdata_b) );

    // 实例化双路 Ping-Pong NTT
    ntt_core #( .WIDTH(24) ) u_ntt (
        .clk(clk), .rst_n(rst_n), .start(ntt_start), .done(ntt_done),
        .ram0_addr_a(ntt_ram0_addr_a), .ram0_we_a(ntt_ram0_we_a), .ram0_wdata_a(ntt_ram0_wdata_a), .ram0_rdata_a(ram0_rdata_a),
        .ram0_addr_b(ntt_ram0_addr_b), .ram0_we_b(ntt_ram0_we_b), .ram0_wdata_b(ntt_ram0_wdata_b), .ram0_rdata_b(ram0_rdata_b),
        .ram1_addr_a(ntt_ram1_addr_a), .ram1_we_a(ntt_ram1_we_a), .ram1_wdata_a(ntt_ram1_wdata_a), .ram1_rdata_a(ram1_rdata_a),
        .ram1_addr_b(ntt_ram1_addr_b), .ram1_we_b(ntt_ram1_we_b), .ram1_wdata_b(ntt_ram1_wdata_b), .ram1_rdata_b(ram1_rdata_b)
    );

    // 【新增】RAM 总线复用选择器 (MUX)
    wire use_user = (state != S_NTT_START && state != S_NTT_WAIT);
    
    always @(*) begin
        if (!use_user) begin
            // 移交控制权给 NTT Core
            ram0_addr_a = ntt_ram0_addr_a; ram0_we_a = ntt_ram0_we_a; ram0_wdata_a = ntt_ram0_wdata_a;
            ram0_addr_b = ntt_ram0_addr_b; ram0_we_b = ntt_ram0_we_b; ram0_wdata_b = ntt_ram0_wdata_b;
            ram1_addr_a = ntt_ram1_addr_a; ram1_we_a = ntt_ram1_we_a; ram1_wdata_a = ntt_ram1_wdata_a;
            ram1_addr_b = ntt_ram1_addr_b; ram1_we_b = ntt_ram1_we_b; ram1_wdata_b = ntt_ram1_wdata_b;
        end else begin
            // User 独占 RAM0_A 口进行数据的填入与转出
            ram0_addr_a = ntt_ram_addr_user; ram0_we_a = ntt_ram_we_user; ram0_wdata_a = ntt_ram_wdata_user;
            ram0_addr_b = 8'd0;              ram0_we_b = 1'b0;            ram0_wdata_b = 24'd0;
            
            // RAM1 闲置
            ram1_addr_a = 8'd0; ram1_we_a = 1'b0; ram1_wdata_a = 24'd0;
            ram1_addr_b = 8'd0; ram1_we_b = 1'b0; ram1_wdata_b = 24'd0;
        end
    end

    // ==========================================
    // 实例化：Rejsam_a 模块
    // ==========================================
    reg         rejsam_start;
    wire        rejsam_valid;
    wire [22:0] rejsam_data;
    wire        rejsam_done;
    reg  [2:0]  rejsam_i;
    reg  [2:0]  rejsam_j;

    Rejsam_a u_rejsam_a (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_start       (rejsam_start),
        .i_rho         (o_rho),
        .i_row         ({5'd0, rejsam_i}),
        .i_column      ({5'd0, rejsam_j}),
        .o_coeff_valid (rejsam_valid),
        .o_coeff_data  (rejsam_data),
        .o_done        (rejsam_done)
    );

    // ==========================================
    // 实例化：SHAKE256 for u 
    // ==========================================
    reg          shake_u_start;
    wire         shake_u_valid;
    wire [511:0] shake_u_data;

    SHAKE256 #(
        .OUTPUT_LEN_BYTES(64),
        .ABSORB_LEN(512 + 256) // tr (512) + M (256)
    ) u_shake_u (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (shake_u_start),
        .i_seed          ({i_M, o_tr}), 
        .o_busy          (),
        .i_squeeze_req   (1'b0),        
        .o_squeeze_valid (shake_u_valid),
        .o_squeeze_data  (shake_u_data)
    );

    // ==========================================
    // 实例化：SHAKE256 for rho_prime 
    // ==========================================
    reg          shake_rho_start;
    wire         shake_rho_valid;
    wire [511:0] shake_rho_data;

    SHAKE256 #(
        .OUTPUT_LEN_BYTES(64),
        .ABSORB_LEN(256 + 256 + 512) // K (256) + zeros (256) + u (512)
    ) u_shake_rho_prime (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (shake_rho_start),
        .i_seed          ({o_u, 256'd0, o_K}), 
        .o_busy          (),
        .i_squeeze_req   (1'b0),
        .o_squeeze_valid (shake_rho_valid),
        .o_squeeze_data  (shake_rho_data)
    );

    // 预处理映射逻辑
    wire [3:0]  target_bits = (mode == 2) ? 4'd13 : 4'd3;
    wire [12:0] raw_val     = bit_buf[12:0] & ((1 << target_bits) - 1);
    wire [23:0] pre_ntt_val = (mode == 2) ? 
                         ((raw_val <= 13'd4096) ? (24'd4096 - raw_val) : (Q + 24'd4096 - raw_val)) :
                         ((raw_val <= 13'd2)    ? (24'd2 - raw_val)    : (Q + 24'd2 - raw_val));

    // ==========================================
    // 主状态机
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0;
            o_s1_we <= 0; o_s2_we <= 0; o_t0_we <= 0;
            ntt_start <= 0; ntt_ram_we_user <= 0;
            bit_buf <= 0; bit_cnt <= 0; sk_ptr <= 0;
            o_rho <= 0; o_K <= 0; o_tr <= 0;
            o_A_we <= 0; o_A_addr <= 0; o_u <= 0; o_rho_prime <= 0;
            rejsam_start <= 0; shake_u_start <= 0; shake_rho_start <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0; o_A_we <= 0; o_A_addr <= 0;
                    if (start) begin
                        header_idx <= 0;
                        state <= S_HEADER_REQ;
                    end
                end

                S_HEADER_REQ: begin
                    o_sk_raddr <= header_idx;
                    state <= S_HEADER_WAIT1;
                end
                S_HEADER_WAIT1: state <= S_HEADER_WAIT2; 
                S_HEADER_WAIT2: begin
                    if (header_idx < 8)       o_rho[header_idx*32 +: 32] <= i_sk_rdata;
                    else if (header_idx < 16) o_K[(header_idx-8)*32 +: 32] <= i_sk_rdata;
                    else                      o_tr[(header_idx-16)*32 +: 32] <= i_sk_rdata;

                    if (header_idx == 31) begin
                        state <= S_UNPACK_CHECK;
                        mode <= 0; poly_idx <= 0; sk_ptr <= 10'd32;
                        bit_buf <= 0; bit_cnt <= 0;
                    end else begin
                        header_idx <= header_idx + 1;
                        state <= S_HEADER_REQ;
                    end
                end

                S_UNPACK_CHECK: begin
                    if (poly_idx == 4) begin
                        if (mode == 0) begin mode <= 1; poly_idx <= 0; end      
                        else if (mode == 1) begin mode <= 2; poly_idx <= 0; end 
                        else state <= S_GEN_A_START; 
                    end else begin
                        coeff_idx <= 0;
                        state <= S_UNPACK_COEFF;
                    end
                end

                S_UNPACK_COEFF: begin
                    if (bit_cnt < target_bits) begin
                        o_sk_raddr <= sk_ptr;
                        state <= S_FETCH_WAIT1;
                        ret_state <= S_UNPACK_COEFF;
                    end else begin
                        state <= S_UNPACK_WRITE;
                    end
                end
                S_UNPACK_WRITE: begin
                    ntt_ram_we_user <= 1;
                    ntt_ram_addr_user <= coeff_idx[7:0];
                    ntt_ram_wdata_user <= pre_ntt_val;
                    bit_buf <= bit_buf >> target_bits;
                    bit_cnt <= bit_cnt - target_bits;
                    state <= S_UNPACK_NEXT;
                end
                S_UNPACK_NEXT: begin
                    ntt_ram_we_user <= 0; 
                    if (coeff_idx == 255) state <= S_NTT_START;
                    else begin coeff_idx <= coeff_idx + 1; state <= S_UNPACK_COEFF; end
                end

                S_FETCH_WAIT1: state <= S_FETCH_WAIT2;
                S_FETCH_WAIT2: begin
                    bit_buf <= bit_buf | ({32'd0, i_sk_rdata} << bit_cnt);
                    bit_cnt <= bit_cnt + 32;
                    sk_ptr <= sk_ptr + 1;
                    state <= ret_state;
                end

                S_NTT_START: begin ntt_start <= 1; state <= S_NTT_WAIT; end
                S_NTT_WAIT: begin
                    ntt_start <= 0;
                    if (ntt_done) begin dump_idx <= 0; state <= S_DUMP_REQ; end
                end

                S_DUMP_REQ: begin
                    ntt_ram_we_user <= 0; ntt_ram_addr_user <= dump_idx[7:0]; state <= S_DUMP_WAIT;
                end
                S_DUMP_WAIT: state <= S_DUMP_STORE;
                S_DUMP_STORE: begin
                    // 【核心修改】：转存数据时，从最终落定数据的 RAM0 中读取
                    if (mode == 0) begin o_s1_we <= 1; o_s1_addr <= {poly_idx[1:0], dump_idx[7:0]}; o_s1_wdata <= ram0_rdata_a; end
                    if (mode == 1) begin o_s2_we <= 1; o_s2_addr <= {poly_idx[1:0], dump_idx[7:0]}; o_s2_wdata <= ram0_rdata_a; end
                    if (mode == 2) begin o_t0_we <= 1; o_t0_addr <= {poly_idx[1:0], dump_idx[7:0]}; o_t0_wdata <= ram0_rdata_a; end
                    state <= S_DUMP_NEXT;
                end
                S_DUMP_NEXT: begin
                    o_s1_we <= 0; o_s2_we <= 0; o_t0_we <= 0;
                    if (dump_idx == 255) begin
                        poly_idx <= poly_idx + 1;
                        state <= S_UNPACK_CHECK;
                    end else begin
                        dump_idx <= dump_idx + 1;
                        state <= S_DUMP_REQ;
                    end
                end

                S_GEN_A_START: begin
                    rejsam_i <= 0; rejsam_j <= 0; rejsam_start <= 1; o_A_addr <= 0; o_A_we <= 0; state <= S_GEN_A_WAIT;
                end

                S_GEN_A_WAIT: begin
                    o_A_we <= rejsam_valid;
                    if (rejsam_valid) o_A_wdata <= {1'b0, rejsam_data};
                    if (o_A_we) o_A_addr <= o_A_addr + 1;

                    if (rejsam_done && !rejsam_start) begin
                        if (rejsam_j == 3) begin
                            if (rejsam_i == 3) begin
                                state <= S_GEN_U_START; 
                                rejsam_start <= 0;      
                            end else begin
                                rejsam_i <= rejsam_i + 1; rejsam_j <= 0; rejsam_start <= 1;     
                            end
                        end else begin
                            rejsam_j <= rejsam_j + 1; rejsam_start <= 1;
                        end
                    end else begin
                        rejsam_start <= 0; 
                    end
                end

                S_GEN_U_START: begin
                    o_A_we <= 0; shake_u_start <= 1; state <= S_GEN_U_WAIT;
                end
                S_GEN_U_WAIT: begin
                    shake_u_start <= 0;
                    if (shake_u_valid) begin o_u <= shake_u_data; state <= S_GEN_RHO_START; end
                end

                S_GEN_RHO_START: begin
                    shake_rho_start <= 1; state <= S_GEN_RHO_WAIT;
                end
                S_GEN_RHO_WAIT: begin
                    shake_rho_start <= 0;
                    if (shake_rho_valid) begin o_rho_prime <= shake_rho_data; state <= S_DONE; end
                end

                S_DONE: begin
                    done <= 1;
                    if (!start) state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule