`timescale 1ns / 1ps

module verify_unpack_ntt (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    output reg           done,
    output reg           verify_error,   // 对应 matlab 中的 verify_result

    // ==========================================
    // 接口 1：从 RAM/FIFO 读取 PK (rho, t1)
    // ==========================================
    output reg  [9:0]    o_pk_raddr,
    input  wire [31:0]   i_pk_rdata,

    // ==========================================
    // 接口 2：从 RAM/FIFO 读取 Signature (c_tilde, z)
    // ==========================================
    output reg  [9:0]    o_sig_raddr,
    input  wire [31:0]   i_sig_rdata,

    // ==========================================
    // 输出接口：直接输出的基础参数
    // ==========================================
    output reg  [255:0]  o_rho,
    output reg  [255:0]  o_c_tilde,

    // ==========================================
    // 输出接口：t1_post (写往外部 RAM，k=4)
    // ==========================================
    output reg           o_t1_we,
    output reg  [9:0]    o_t1_addr,      // {poly_idx[1:0], dump_idx[7:0]}
    output reg  [23:0]   o_t1_wdata,

    // ==========================================
    // 输出接口：z_post (写往外部 RAM，l=4)
    // ==========================================
    output reg           o_z_we,
    output reg  [9:0]    o_z_addr,       // {poly_idx[1:0], dump_idx[7:0]}
    output reg  [23:0]   o_z_wdata
);

    // 参数定义 (Security Level 2)
    localparam [23:0] Q            = 24'd8380417;
    localparam [23:0] Z_LOW_BOUND  = 24'd130994;   // y1 - beta = 131072 - 78
    localparam [23:0] Z_HIGH_BOUND = 24'd8249423;  // Q - (y1 - beta) = 8380417 - 130994

    // FSM 状态定义
    localparam S_IDLE           = 5'd0;
    localparam S_RHO_REQ        = 5'd1;
    localparam S_RHO_WAIT       = 5'd2;
    localparam S_RHO_STORE      = 5'd3;
    localparam S_CTILDE_REQ     = 5'd4;
    localparam S_CTILDE_WAIT    = 5'd5;
    localparam S_CTILDE_STORE   = 5'd6;
    
    localparam S_UNPACK_CHK     = 5'd7;
    localparam S_UNPACK_COEFF   = 5'd8;
    localparam S_FETCH_PK_W1    = 5'd9;
    localparam S_FETCH_PK_W2    = 5'd10;
    localparam S_FETCH_SIG_W1   = 5'd11;
    localparam S_FETCH_SIG_W2   = 5'd12;
    localparam S_UNPACK_WRITE   = 5'd13;
    localparam S_UNPACK_NEXT    = 5'd14;
    
    localparam S_NTT_START      = 5'd15;
    localparam S_NTT_WAIT       = 5'd16;
    localparam S_DUMP_REQ       = 5'd17;
    localparam S_DUMP_WAIT      = 5'd18;
    localparam S_DUMP_STORE     = 5'd19;
    localparam S_DUMP_NEXT      = 5'd20;
    localparam S_DONE           = 5'd21;

    reg [4:0]  state;
    reg [2:0]  mode;         // 0: Init/Rho, 1: c_tilde, 2: T1 unpack, 3: Z unpack
    reg [2:0]  poly_idx;     // 0 ~ 3
    reg [8:0]  coeff_idx;    // 0 ~ 255
    reg [8:0]  dump_idx;     // 0 ~ 255
    reg [3:0]  header_idx;

    reg [63:0] bit_buf;
    reg [6:0]  bit_cnt;
    reg [9:0]  pk_ptr;
    reg [9:0]  sig_ptr;

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
    // 核心计算逻辑：位宽截取与模运算预处理
    // ==========================================
    wire [4:0]  target_bits = (mode == 2) ? 5'd10 : 5'd18;

    // 1. T1 预处理: t1_pre_ntt(i,:) = mod(t1_pre_ntt(i,:) * (2^13), mod_val)
    wire [9:0]  raw_t1 = bit_buf[9:0];
    wire [23:0] t1_val = {14'd0, raw_t1} << 13;

//    // 2. Z 预处理: mod(y1 - z_pre, mod_val); y1 = 2^17
//    wire [17:0] raw_z    = bit_buf[17:0];
//    wire [23:0] raw_z_24 = {6'd0, raw_z};
//    wire [23:0] y1_24    = 24'd131072;
    
//// ? 正确的代码：(z_raw - y1) mod Q
//wire [23:0] z_val = (raw_z_24 >= y1_24) ? (raw_z_24 - y1_24) : (Q + raw_z_24 - y1_24);

// 2. Z 预处理: mod(y1 - z_pre, mod_val);
    // y1 = 2^17
    wire [17:0] raw_z    = bit_buf[17:0];
    wire [23:0] raw_z_24 = {6'd0, raw_z};
    wire [23:0] y1_24    = 24'd131072;
    
    // 【修改点】：对应 MATLAB 的 z_pre_ntt(i,:) = mod(y1 - z_pre_ntt(i,:), mod_val);
    wire [23:0] z_val = (y1_24 >= raw_z_24) ? 
                        (y1_24 - raw_z_24) : 
                        (Q + y1_24 - raw_z_24);

    wire [23:0] pre_ntt_val    = (mode == 2) ? t1_val : z_val;
    
    // 3. 安全性检验: z 越界判断 (对应 matlab 的边界 check)
    wire        z_err = (mode == 3) && (z_val >= Z_LOW_BOUND) && (z_val <= Z_HIGH_BOUND);

    // ==========================================
    // FSM 控制
    // ==========================================
// ==========================================
    // FSM 控制
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0; verify_error <= 0;
            o_t1_we <= 0; o_z_we <= 0; ntt_start <= 0; ntt_ram_we_user <= 0;
            bit_buf <= 0; bit_cnt <= 0; pk_ptr <= 0; sig_ptr <= 0;
            o_rho <= 0; o_c_tilde <= 0;
            mode <= 0;
            
            // ★ 新增：给输出地址总线增加显式复位
            o_pk_raddr  <= 10'd0;
            o_sig_raddr <= 10'd0;
            
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    verify_error <= 0; o_t1_we <= 0; o_z_we <= 0;
                    
//                    // ★ 新增：在 IDLE 状态保持地址为 0，防止前一次验证的地址残留
//                    o_pk_raddr  <= 10'd0;
//                    o_sig_raddr <= 10'd0;

                    if (start) begin
                        header_idx <= 0;
                        mode <= 0; // 读取 Rho
                        state <= S_RHO_REQ;
                    end
                end

                // ----------------------------------------
                // 阶段 1: 提取 PK 前 32 Bytes (rho)
                // ----------------------------------------
                S_RHO_REQ: begin
                    o_pk_raddr <= header_idx;
                    state <= S_RHO_WAIT;
                end
                S_RHO_WAIT: state <= S_RHO_STORE;
                S_RHO_STORE: begin
                    o_rho[header_idx*32 +: 32] <= i_pk_rdata;
                    if (header_idx == 7) begin
                        header_idx <= 0;
                        mode <= 1; // 切换为读取 c_tilde
                        state <= S_CTILDE_REQ;
                    end else begin
                        header_idx <= header_idx + 1;
                        state <= S_RHO_REQ;
                    end
                end

                // ----------------------------------------
                // 阶段 2: 提取 Signature 前 32 Bytes (c_tilde)
                // ----------------------------------------
                S_CTILDE_REQ: begin
                    o_sig_raddr <= header_idx;
                    state <= S_CTILDE_WAIT;
                end
                S_CTILDE_WAIT: state <= S_CTILDE_STORE;
                S_CTILDE_STORE: begin
                    o_c_tilde[header_idx*32 +: 32] <= i_sig_rdata;
                    if (header_idx == 7) begin
                        mode <= 2; // 切换为解包 T1
                        poly_idx <= 0;
                        pk_ptr <= 10'd8; // pk(1:256) 占用了 8 个 32-bit 地址
                        bit_buf <= 0; bit_cnt <= 0;
                        state <= S_UNPACK_CHK;
                    end else begin
                        header_idx <= header_idx + 1;
                        state <= S_CTILDE_REQ;
                    end
                end

                // ----------------------------------------
                // 阶段 3 & 4: 解包判断 (T1 或 Z)
                // ----------------------------------------
                S_UNPACK_CHK: begin
                    if (poly_idx == 4) begin 
                        if (mode == 2) begin
                            mode <= 3;     // T1 结束，切换为解包 Z
                            poly_idx <= 0;
                            sig_ptr <= 10'd8; // sig(1:256) 占用了 8 个 32-bit 地址
                            bit_buf <= 0; bit_cnt <= 0;
                            state <= S_UNPACK_CHK;
                        end else begin
                            state <= S_DONE; // 整个流程结束
                        end
                    end else begin
                        coeff_idx <= 0;
                        state <= S_UNPACK_COEFF;
                    end
                end

                // 提取单系数逻辑
                S_UNPACK_COEFF: begin
                    if (bit_cnt < target_bits) begin
                        if (mode == 2) begin // 读 PK RAM
                            o_pk_raddr <= pk_ptr;
                            state <= S_FETCH_PK_W1;
                        end else begin       // 读 SIG RAM
                            o_sig_raddr <= sig_ptr;
                            state <= S_FETCH_SIG_W1;
                        end
                    end else begin
                        state <= S_UNPACK_WRITE;
                    end
                end

                // 流式读取拼接：PK
                S_FETCH_PK_W1: state <= S_FETCH_PK_W2;
                S_FETCH_PK_W2: begin
                    bit_buf <= bit_buf | ({32'd0, i_pk_rdata} << bit_cnt);
                    bit_cnt <= bit_cnt + 32;
                    pk_ptr <= pk_ptr + 1;
                    state <= S_UNPACK_COEFF;
                end

                // 流式读取拼接：SIG
                S_FETCH_SIG_W1: state <= S_FETCH_SIG_W2;
                S_FETCH_SIG_W2: begin
                    bit_buf <= bit_buf | ({32'd0, i_sig_rdata} << bit_cnt);
                    bit_cnt <= bit_cnt + 32;
                    sig_ptr <= sig_ptr + 1;
                    state <= S_UNPACK_COEFF;
                end

                // 计算并写入 NTT RAM
                S_UNPACK_WRITE: begin
                    ntt_ram_we_user <= 1;
                    ntt_ram_addr_user <= coeff_idx[7:0];
                    ntt_ram_wdata_user <= pre_ntt_val;
                    
                    bit_buf <= bit_buf >> target_bits;
                    bit_cnt <= bit_cnt - target_bits;
                    
                    if (z_err) verify_error <= 1'b1; // 一旦发现错界，锁存验证错误信号
                    
                    state <= S_UNPACK_NEXT;
                end
                
                S_UNPACK_NEXT: begin
                    ntt_ram_we_user <= 0;
                    if (coeff_idx == 255) state <= S_NTT_START;
                    else begin coeff_idx <= coeff_idx + 1; state <= S_UNPACK_COEFF; end
                end

                // ----------------------------------------
                // 阶段 5: 执行 NTT 与结果转储
                // ----------------------------------------
                S_NTT_START: begin 
                    ntt_start <= 1; 
                    state <= S_NTT_WAIT; 
                end
                
                S_NTT_WAIT: begin
                    ntt_start <= 0;
                    if (ntt_done) begin 
                        dump_idx <= 0; 
                        state <= S_DUMP_REQ; 
                    end
                end

                S_DUMP_REQ: begin
                    ntt_ram_we_user <= 0;
                    ntt_ram_addr_user <= dump_idx[7:0]; 
                    state <= S_DUMP_WAIT;
                end
                
                S_DUMP_WAIT: state <= S_DUMP_STORE;
                
                S_DUMP_STORE: begin
                    // 【核心修改】：写入时从 ram0_rdata_a 获取数据
                    if (mode == 2) begin
                        o_t1_we <= 1;
                        o_t1_addr <= {poly_idx[1:0], dump_idx[7:0]};
                        o_t1_wdata <= ram0_rdata_a;
                    end else if (mode == 3) begin
                        o_z_we <= 1;
                        o_z_addr <= {poly_idx[1:0], dump_idx[7:0]};
                        o_z_wdata <= ram0_rdata_a;
                    end
                    state <= S_DUMP_NEXT;
                end
                
                S_DUMP_NEXT: begin
                    o_t1_we <= 0; o_z_we <= 0;
                    if (dump_idx == 255) begin
                        poly_idx <= poly_idx + 1;
                        state <= S_UNPACK_CHK;
                    end else begin
                        dump_idx <= dump_idx + 1;
                        state <= S_DUMP_REQ;
                    end
                end

                // 完成
                S_DONE: begin
                    done <= 1;
                    if (!start) state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule