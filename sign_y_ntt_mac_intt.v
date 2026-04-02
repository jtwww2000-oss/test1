`timescale 1ns / 1ps

module sign_y_ntt_mac_intt #(
    parameter WIDTH = 24,
    parameter Q     = 24'd8380417,
    parameter L_PARAM = 4'd4, 
    parameter K_PARAM = 4'd4  
)(
    input  wire                 clk,
    input  wire                 rst_n,
  
    // --- 第一阶段控制与输入 ---
    input  wire                 i_start,
    input  wire [511:0]         i_rho_prime,
    input  wire [15:0]          i_rej_round,
    input  wire                 i_A_valid,
    input  wire [WIDTH-1:0]     i_A_data,
    input  wire [7:0]           i_A_m_idx,
    input  wire [3:0]           i_A_j_idx,
    output wire                 o_ready_for_A, 
    
    // --- 第一阶段 w 与 w1 输出 ---
    output reg                  o_w_valid,
    output reg  [WIDTH-1:0]     o_w_data,
    output reg  [5:0]           o_w1_data,    
    output reg  [3:0]           o_w_poly_idx, 
    output reg  [7:0]           o_w_coeff_idx,
    output reg                  o_w_done,       
  
    // --- 第二阶段控制与输入 ---
    input  wire                 i_c_start,
    input  wire [255:0]         i_c_tilde,      
    
    output wire [9:0]           o_s1_rd_addr,
    input  wire [WIDTH-1:0]     i_s1_rd_data,
    
    output wire [9:0]           o_s2_rd_addr,
    input  wire [WIDTH-1:0]     i_s2_rd_data,

    output wire [9:0]           o_t0_rd_addr,
    input  wire [WIDTH-1:0]     i_t0_rd_data,
    
    // --- 第二阶段各多项式输出 ---
    output reg                  o_z_valid,
    output reg  [WIDTH-1:0]     o_z_data,
    output reg  [3:0]           o_z_poly_idx,
    output reg  [7:0]           o_z_coeff_idx,
    
    output reg                  o_cs2_valid,
    output reg  [WIDTH-1:0]     o_cs2_data,
    output reg  [3:0]           o_cs2_poly_idx,
    output reg  [7:0]           o_cs2_coeff_idx,

    output reg                  o_r0_valid,
    output reg  [WIDTH-1:0]     o_r0_data,
    output reg  [3:0]           o_r0_poly_idx,
    output reg  [7:0]           o_r0_coeff_idx,

    output reg                  o_ct0_valid,
    output reg  [WIDTH-1:0]     o_ct0_data,
    output reg  [3:0]           o_ct0_poly_idx,
    output reg  [7:0]           o_ct0_coeff_idx,

    output wire                 o_hint_pre_valid,
    output wire                 o_hint_pre_data,
    output reg  [3:0]           o_hint_pre_poly_idx,
    output reg  [7:0]           o_hint_pre_coeff_idx,

    // --- 新增: 拒绝采样与完成标志 ---
    output reg                  o_rej_flag,
    output reg                  o_all_done      
);

    // ==========================================
    // 状态机定义
    // ==========================================
    localparam S_IDLE          = 6'd0;
    localparam S_GEN_Y         = 6'd1;
    localparam S_WAIT_Y        = 6'd2;
    localparam S_NTT_START     = 6'd3;
    localparam S_NTT_WAIT      = 6'd4;
    localparam S_COPY_REQ      = 6'd5;
    localparam S_COPY_WAIT     = 6'd6;
    localparam S_COPY_STORE    = 6'd7;
    localparam S_MAC_START     = 6'd8;
    localparam S_MAC_RUN       = 6'd9;
    localparam S_MAC_WAIT_RES  = 6'd10;
    localparam S_INTT_START    = 6'd11;
    localparam S_INTT_WAIT     = 6'd12;
    localparam S_OUT_REQ       = 6'd13;
    localparam S_OUT_WAIT      = 6'd14;
    localparam S_OUT_STORE     = 6'd15;
    localparam S_W_DONE        = 6'd16;
    localparam S_SIB_START     = 6'd17;
    localparam S_SIB_WAIT      = 6'd18;
    localparam S_C_NTT_START   = 6'd19;
    localparam S_C_NTT_WAIT    = 6'd20;
    localparam S_COPY_C        = 6'd21;
    localparam S_CS1_MUL       = 6'd22;
    localparam S_CS1_INTT_START= 6'd23;
    localparam S_CS1_INTT_WAIT = 6'd24;
    localparam S_Z_ADD_OUT     = 6'd25;
    localparam S_CS2_MUL       = 6'd26;
    localparam S_CS2_INTT_START= 6'd27;
    localparam S_CS2_INTT_WAIT = 6'd28;
    localparam S_CS2_OUT       = 6'd29;
    localparam S_CT0_MUL       = 6'd30;
    localparam S_CT0_INTT_START= 6'd31;
    localparam S_CT0_INTT_WAIT = 6'd32;
    localparam S_CT0_OUT       = 6'd33;
    localparam S_ALL_DONE      = 6'd34;
    localparam S_SIB_CLEAR     = 6'd35;
    localparam S_ALL_DONE_WAIT = 6'd36; // <--- 新增的等待状态

    reg [5:0] state;
    reg [3:0] l_cnt;         
    reg [3:0] k_cnt;
    reg [3:0] l_cnt_a;
    reg [8:0] general_cnt;   
    reg [8:0] m_cnt;         
    reg [8:0] mac_out_cnt;

    assign o_ready_for_A = (state == S_MAC_RUN && m_cnt == 0);

    // ==========================================
    // 1. Rejsam_y 与 SampleInBall
    // ==========================================
    reg  rejsam_start;
    wire [15:0] rejsam_row = l_cnt + i_rej_round;
    wire rejsam_valid;
    wire [17:0] rejsam_data;
    wire rejsam_done;

    Rejsam_y u_rejsam_y (
        .clk(clk), .rst_n(rst_n), .i_start(rejsam_start),
        .i_rho_prime(i_rho_prime), .i_row(rejsam_row),
        .o_coeff_valid(rejsam_valid), .o_coeff_data(rejsam_data), .o_done(rejsam_done)
    );

    wire [23:0] y_raw_ext = {6'd0, rejsam_data}; 
    wire [23:0] y_mod_q   = (24'd131072 >= y_raw_ext) ? (24'd131072 - y_raw_ext) : (Q + 24'd131072 - y_raw_ext);

    reg sib_start;
    wire sib_we;
    wire [7:0] sib_addr;
    wire [23:0] sib_wdata;
    wire sib_done;

    wire [23:0] ntt_ram0_rdata_a, ntt_ram0_rdata_b, ntt_ram1_rdata_a, ntt_ram1_rdata_b;
    wire [23:0] intt_ram0_rdata_a, intt_ram0_rdata_b, intt_ram1_rdata_a, intt_ram1_rdata_b;

    SampleInBall #( .TAU(8'd39), .Q(Q) ) u_sib (
        .clk(clk), .rst_n(rst_n), .i_start(sib_start),
        .i_c1(i_c_tilde),
        .o_c_we(sib_we), .o_c_addr(sib_addr), .o_c_wdata(sib_wdata),
        .i_c_rdata(ntt_ram0_rdata_a), .o_done(sib_done)
    );

    // ==========================================
    // 2. RAM 实例化与 Ping-Pong 架构 MUX
    // ==========================================
    reg ntt_start, intt_start;
    wire ntt_done, intt_done;
    
    reg         ntt_we_user, intt_we_user;
    reg  [7:0]  ntt_addr_user, intt_addr_user;
    reg  [23:0] ntt_wdata_user, intt_wdata_user;

    wire use_user_ntt = (state != S_NTT_START && state != S_NTT_WAIT && state != S_C_NTT_START && state != S_C_NTT_WAIT);
    wire use_user_intt = (state != S_INTT_START && state != S_INTT_WAIT && 
                          state != S_CS1_INTT_START && state != S_CS1_INTT_WAIT && 
                          state != S_CS2_INTT_START && state != S_CS2_INTT_WAIT &&
                          state != S_CT0_INTT_START && state != S_CT0_INTT_WAIT);

    wire [7:0]  core_ntt_ram0_addr_a, core_ntt_ram0_addr_b, core_ntt_ram1_addr_a, core_ntt_ram1_addr_b;
    wire        core_ntt_ram0_we_a,   core_ntt_ram0_we_b,   core_ntt_ram1_we_a,   core_ntt_ram1_we_b;
    wire [23:0] core_ntt_ram0_wdata_a,core_ntt_ram0_wdata_b,core_ntt_ram1_wdata_a,core_ntt_ram1_wdata_b;

    wire [7:0]  core_intt_ram0_addr_a, core_intt_ram0_addr_b, core_intt_ram1_addr_a, core_intt_ram1_addr_b;
    wire        core_intt_ram0_we_a,   core_intt_ram0_we_b,   core_intt_ram1_we_a,   core_intt_ram1_we_b;
    wire [23:0] core_intt_ram0_wdata_a,core_intt_ram0_wdata_b,core_intt_ram1_wdata_a,core_intt_ram1_wdata_b;

    wire [7:0]  ntt_ram0_addr_a  = use_user_ntt ? ntt_addr_user  : core_ntt_ram0_addr_a;
    wire        ntt_ram0_we_a    = use_user_ntt ? ntt_we_user    : core_ntt_ram0_we_a;
    wire [23:0] ntt_ram0_wdata_a = use_user_ntt ? ntt_wdata_user : core_ntt_ram0_wdata_a;
    wire [7:0]  ntt_ram0_addr_b  = use_user_ntt ? 8'd0           : core_ntt_ram0_addr_b;
    wire        ntt_ram0_we_b    = use_user_ntt ? 1'b0           : core_ntt_ram0_we_b;
    wire [23:0] ntt_ram0_wdata_b = use_user_ntt ? 24'd0          : core_ntt_ram0_wdata_b;

    wire [7:0]  ntt_ram1_addr_a  = use_user_ntt ? 8'd0           : core_ntt_ram1_addr_a;
    wire        ntt_ram1_we_a    = use_user_ntt ? 1'b0           : core_ntt_ram1_we_a;
    wire [23:0] ntt_ram1_wdata_a = use_user_ntt ? 24'd0          : core_ntt_ram1_wdata_a;
    wire [7:0]  ntt_ram1_addr_b  = use_user_ntt ? 8'd0           : core_ntt_ram1_addr_b;
    wire        ntt_ram1_we_b    = use_user_ntt ? 1'b0           : core_ntt_ram1_we_b;
    wire [23:0] ntt_ram1_wdata_b = use_user_ntt ? 24'd0          : core_ntt_ram1_wdata_b;

    wire [7:0]  intt_ram0_addr_a  = use_user_intt ? intt_addr_user  : core_intt_ram0_addr_a;
    wire        intt_ram0_we_a    = use_user_intt ? intt_we_user    : core_intt_ram0_we_a;
    wire [23:0] intt_ram0_wdata_a = use_user_intt ? intt_wdata_user : core_intt_ram0_wdata_a;
    wire [7:0]  intt_ram0_addr_b  = use_user_intt ? 8'd0            : core_intt_ram0_addr_b;
    wire        intt_ram0_we_b    = use_user_intt ? 1'b0            : core_intt_ram0_we_b;
    wire [23:0] intt_ram0_wdata_b = use_user_intt ? 24'd0           : core_intt_ram0_wdata_b;

    wire [7:0]  intt_ram1_addr_a  = use_user_intt ? 8'd0            : core_intt_ram1_addr_a;
    wire        intt_ram1_we_a    = use_user_intt ? 1'b0            : core_intt_ram1_we_a;
    wire [23:0] intt_ram1_wdata_a = use_user_intt ? 24'd0           : core_intt_ram1_wdata_a;
    wire [7:0]  intt_ram1_addr_b  = use_user_intt ? 8'd0            : core_intt_ram1_addr_b;
    wire        intt_ram1_we_b    = use_user_intt ? 1'b0            : core_intt_ram1_we_b;
    wire [23:0] intt_ram1_wdata_b = use_user_intt ? 24'd0           : core_intt_ram1_wdata_b;

    tdpram_24x256 u_ntt_ram0  ( .clk(clk), .we_a(ntt_ram0_we_a), .addr_a(ntt_ram0_addr_a), .din_a(ntt_ram0_wdata_a), .dout_a(ntt_ram0_rdata_a), .we_b(ntt_ram0_we_b), .addr_b(ntt_ram0_addr_b), .din_b(ntt_ram0_wdata_b), .dout_b(ntt_ram0_rdata_b) );
    tdpram_24x256 u_ntt_ram1  ( .clk(clk), .we_a(ntt_ram1_we_a), .addr_a(ntt_ram1_addr_a), .din_a(ntt_ram1_wdata_a), .dout_a(ntt_ram1_rdata_a), .we_b(ntt_ram1_we_b), .addr_b(ntt_ram1_addr_b), .din_b(ntt_ram1_wdata_b), .dout_b(ntt_ram1_rdata_b) );
    tdpram_24x256 u_intt_ram0 ( .clk(clk), .we_a(intt_ram0_we_a), .addr_a(intt_ram0_addr_a), .din_a(intt_ram0_wdata_a), .dout_a(intt_ram0_rdata_a), .we_b(intt_ram0_we_b), .addr_b(intt_ram0_addr_b), .din_b(intt_ram0_wdata_b), .dout_b(intt_ram0_rdata_b) );
    tdpram_24x256 u_intt_ram1 ( .clk(clk), .we_a(intt_ram1_we_a), .addr_a(intt_ram1_addr_a), .din_a(intt_ram1_wdata_a), .dout_a(intt_ram1_rdata_a), .we_b(intt_ram1_we_b), .addr_b(intt_ram1_addr_b), .din_b(intt_ram1_wdata_b), .dout_b(intt_ram1_rdata_b) );

    // ----- 外部暂存阵列 -----
    reg [23:0] y_post_bram [0:1023];
    reg [23:0] acc_bram [0:255];
    reg [23:0] y_bram [0:1023];
    reg [23:0] c_post_bram [0:255];   

    integer mem_i;
    initial begin
        for(mem_i=0; mem_i<1024; mem_i=mem_i+1) y_post_bram[mem_i] = 24'd0;
        for(mem_i=0; mem_i<1024; mem_i=mem_i+1) y_bram[mem_i] = 24'd0;
        for(mem_i=0; mem_i<256; mem_i=mem_i+1)  acc_bram[mem_i] = 24'd0;
        for(mem_i=0; mem_i<256; mem_i=mem_i+1)  c_post_bram[mem_i] = 24'd0;
    end

    reg         y_post_we;
    reg  [9:0]  y_post_addr_w;
    reg  [23:0] y_post_wdata;
    wire [9:0]  y_post_addr_r;
    reg  [23:0] y_post_rdata;

    always @(posedge clk) begin
        if (y_post_we) y_post_bram[y_post_addr_w] <= y_post_wdata;
        y_post_rdata <= y_post_bram[y_post_addr_r];
    end

    reg         y_we;
    reg  [9:0]  y_addr_w;
    reg  [23:0] y_wdata;
    wire [9:0]  y_addr_r;
    reg  [23:0] y_rdata;

    always @(posedge clk) begin
        if (y_we) y_bram[y_addr_w] <= y_wdata;
        y_rdata <= y_bram[y_addr_r];
    end

    reg         c_post_we;
    reg  [7:0]  c_post_addr_w;
    reg  [23:0] c_post_wdata;
    wire [7:0]  c_post_addr_r;
    reg  [23:0] c_post_rdata;

    always @(posedge clk) begin
        if (c_post_we) c_post_bram[c_post_addr_w] <= c_post_wdata;
        c_post_rdata <= c_post_bram[c_post_addr_r];
    end

    wire        mac_acc_we;
    wire [7:0]  mac_acc_addr, mac_acc_waddr;
    wire [23:0] mac_acc_wdata;
    reg  [23:0] mac_acc_rdata;

    always @(posedge clk) begin
        if (mac_acc_we) begin
            acc_bram[mac_acc_waddr] <= mac_acc_wdata;
        end else if (state == S_COPY_STORE || state == S_OUT_STORE) begin
            acc_bram[general_cnt[7:0]] <= 24'd0;
        end
        mac_acc_rdata <= acc_bram[mac_acc_addr];
    end

    reg         w_we;
    reg  [9:0]  w_addr_w;
    reg  [23:0] w_wdata;
    wire [9:0]  w_addr_r;
    reg  [23:0] w_rdata;
    reg  [23:0] w_bram [0:1023];

    always @(posedge clk) begin
        if (w_we) w_bram[w_addr_w] <= w_wdata;
        w_rdata <= w_bram[w_addr_r];
    end
    assign w_addr_r = {k_cnt[1:0], general_cnt[7:0]};

    // ==========================================
    // ★ 核心修复：对 w_minus_cs2 增加 1 级流水线 ★
    // ==========================================
    wire [23:0] w_minus_cs2;
    mod_sub #( .WIDTH(WIDTH) ) u_mod_sub_r0 (
        .a(w_rdata), .b(intt_ram0_rdata_a), .q(Q), .res(w_minus_cs2)
    );

    reg [23:0] w_minus_cs2_reg;
    reg        r0_pipe_en;
    reg [3:0]  r0_pipe_poly;
    reg [7:0]  r0_pipe_addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_minus_cs2_reg <= 0;
            r0_pipe_en      <= 0;
            r0_pipe_poly    <= 0;
            r0_pipe_addr    <= 0;
        end else begin
            // 寄存减法结果，切断时序链
            w_minus_cs2_reg <= w_minus_cs2;
            
            // 将输出有效信号向后推迟 1 拍
            r0_pipe_en      <= (state == S_CS2_OUT) && out_valid_d1;
            r0_pipe_poly    <= k_cnt;
            r0_pipe_addr    <= out_addr_d1;
        end
    end

    wire [23:0] r0_calc;
    Lowbits #( .WIDTH(WIDTH), .Q(Q) ) u_lowbits_r0 (
        .i_w(w_minus_cs2_reg),  // 使用经过寄存器打拍的输入
        .o_w0(r0_calc)
    );

    // ==========================================
    // 暂存 w_minus_cs2 供 hint_pre 计算
    // ==========================================
    reg         wm_cs2_we;
    reg  [9:0]  wm_cs2_addr_w;
    reg  [23:0] wm_cs2_wdata;
    wire [9:0]  wm_cs2_addr_r;
    reg  [23:0] wm_cs2_rdata;
    reg  [23:0] wm_cs2_bram [0:1023];

    always @(posedge clk) begin
        if (wm_cs2_we) wm_cs2_bram[wm_cs2_addr_w] <= wm_cs2_wdata;
        wm_cs2_rdata <= wm_cs2_bram[wm_cs2_addr_r];
    end
    assign wm_cs2_addr_r = {k_cnt[1:0], general_cnt[7:0]};

    // ==========================================
    // 计算 Makehint_pre
    // ==========================================
    wire [23:0] makehint_A;
    mod_sub #( .WIDTH(WIDTH) ) u_mod_sub_hintA (
        .a(24'd0), .b(intt_ram0_rdata_a), .q(Q), .res(makehint_A)
    );

    wire [23:0] makehint_B;
    mod_add #( .WIDTH(WIDTH) ) u_mod_add_hintB (
        .a(wm_cs2_rdata), .b(intt_ram0_rdata_a), .q(Q), .res(makehint_B)
    );

    // ★ 新增：为 Makehint_pre 插入一级流水线，切断 -0.065ns 的关键路径 ★
    reg [23:0] makehint_A_reg, makehint_B_reg;
    reg        hint_eval_en_reg;
    reg [3:0]  hint_poly_reg;
    reg [7:0]  hint_coeff_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            makehint_A_reg <= 0; makehint_B_reg <= 0;
            hint_eval_en_reg <= 0; hint_poly_reg <= 0; hint_coeff_reg <= 0;
        end else begin
            makehint_A_reg   <= makehint_A;
            makehint_B_reg   <= makehint_B;
            hint_eval_en_reg <= hint_eval_en;
            hint_poly_reg    <= k_cnt;
            hint_coeff_reg   <= out_addr_d1;
        end
    end

    wire hint_eval_en;
    Makehint_pre #( .WIDTH(WIDTH), .Q(Q) ) u_makehint_pre (
        .clk(clk), .rst_n(rst_n),
        .i_valid(hint_eval_en_reg), .i_A(makehint_A_reg), .i_B(makehint_B_reg), // 使用打拍后的数据
        .o_valid(o_hint_pre_valid), .o_hint_pre(o_hint_pre_data)
    );

    reg out_valid_d1;
    reg [7:0] out_addr_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid_d1 <= 0;
            out_addr_d1 <= 0; o_hint_pre_poly_idx <= 0; o_hint_pre_coeff_idx <= 0;
        end else begin
            if (state == S_Z_ADD_OUT || state == S_CS2_OUT || state == S_CT0_OUT) begin
                if (general_cnt < 256) begin
                    out_valid_d1 <= 1'b1;
                    out_addr_d1 <= general_cnt[7:0];
                end else out_valid_d1 <= 1'b0;
            end else out_valid_d1 <= 1'b0;
            
            // 使用同步延迟的使能信号和地址
            if (hint_eval_en_reg) begin
                o_hint_pre_poly_idx <= hint_poly_reg;
                o_hint_pre_coeff_idx <= hint_coeff_reg; 
            end
        end
    end
    
    assign hint_eval_en = (out_valid_d1 && state == S_CT0_OUT);;

    // ==========================================
    // 拒绝采样条件检查 (Security Level 2)
    // ==========================================
    wire z_rej_cond   = (o_z_valid   && (o_z_data >= 24'd130994) && (o_z_data <= 24'd8249423));
    wire r0_rej_cond  = (o_r0_valid  && (o_r0_data >= 24'd95154) && (o_r0_data <= 24'd8285263));
    wire ct0_rej_cond = (o_ct0_valid && (o_ct0_data >= 24'd95232) && (o_ct0_data <= 24'd8285185));

    reg [10:0] hint_cnt;
    reg        rej_flag_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hint_cnt <= 0;
            rej_flag_reg <= 0;
        end else if (i_start || i_c_start) begin
            if (i_start) begin
                hint_cnt <= 0;
                rej_flag_reg <= 0;
            end
        end else begin
            if (o_hint_pre_valid && o_hint_pre_data) 
                hint_cnt <= hint_cnt + 1;
            if (z_rej_cond || r0_rej_cond || ct0_rej_cond)
                rej_flag_reg <= 1'b1;
        end
    end

    // ==========================================
    // 3. 核心 IP 实例化
    // ==========================================
    ntt_core #( .WIDTH(WIDTH) ) u_ntt_core (
        .clk(clk), .rst_n(rst_n), .start(ntt_start), .done(ntt_done),
        .ram0_addr_a(core_ntt_ram0_addr_a), .ram0_we_a(core_ntt_ram0_we_a), .ram0_wdata_a(core_ntt_ram0_wdata_a), .ram0_rdata_a(ntt_ram0_rdata_a),
        .ram0_addr_b(core_ntt_ram0_addr_b), .ram0_we_b(core_ntt_ram0_we_b), .ram0_wdata_b(core_ntt_ram0_wdata_b), .ram0_rdata_b(ntt_ram0_rdata_b),
        .ram1_addr_a(core_ntt_ram1_addr_a), .ram1_we_a(core_ntt_ram1_we_a), .ram1_wdata_a(core_ntt_ram1_wdata_a), .ram1_rdata_a(ntt_ram1_rdata_a),
        .ram1_addr_b(core_ntt_ram1_addr_b), .ram1_we_b(core_ntt_ram1_we_b), .ram1_wdata_b(core_ntt_ram1_wdata_b), .ram1_rdata_b(ntt_ram1_rdata_b)
    );

    intt_core #( .WIDTH(WIDTH) ) u_intt_core (
        .clk(clk), .rst_n(rst_n), .start(intt_start), .done(intt_done),
        .ram0_addr_a(core_intt_ram0_addr_a), .ram0_we_a(core_intt_ram0_we_a), .ram0_wdata_a(core_intt_ram0_wdata_a), .ram0_rdata_a(intt_ram0_rdata_a),
        .ram0_addr_b(core_intt_ram0_addr_b), .ram0_we_b(core_intt_ram0_we_b), .ram0_wdata_b(core_intt_ram0_wdata_b), .ram0_rdata_b(intt_ram0_rdata_b),
        .ram1_addr_a(core_intt_ram1_addr_a), .ram1_we_a(core_intt_ram1_we_a), .ram1_wdata_a(core_intt_ram1_wdata_a), .ram1_rdata_a(intt_ram1_rdata_a),
        .ram1_addr_b(core_intt_ram1_addr_b), .ram1_we_b(core_intt_ram1_we_b), .ram1_wdata_b(core_intt_ram1_wdata_b), .ram1_rdata_b(intt_ram1_rdata_b)
    );

    wire [7:0]  mac_s1_addr;
    wire [3:0]  mac_s1_poly_idx;
    wire        mac_res_valid;
    wire [23:0] mac_res_data;
    wire [7:0]  mac_res_m_idx;
    assign y_post_addr_r = {mac_s1_poly_idx[1:0], mac_s1_addr};

    MatrixVecMul_Core #( .WIDTH(WIDTH), .Q(Q), .MU(26'd33587228) ) u_mac (
        .clk(clk), .rst_n(rst_n), .i_A_valid(i_A_valid), .i_A_data(i_A_data),
        .i_m_idx(i_A_m_idx), .i_j_idx(i_A_j_idx), .i_l_param(L_PARAM),
        .o_s1_addr(mac_s1_addr), .o_s1_poly_idx(mac_s1_poly_idx), .i_s1_rdata(y_post_rdata),
        .o_acc_we(mac_acc_we), .o_acc_addr(mac_acc_addr), .o_acc_waddr(mac_acc_waddr),
        .o_acc_wdata(mac_acc_wdata), .i_acc_rdata(mac_acc_rdata),
        .o_res_valid(mac_res_valid),.o_res_data(mac_res_data),.o_res_m_idx(mac_res_m_idx)
    );

    wire [5:0] w1_calc;
    Highbits #( .WIDTH(WIDTH), .Q(Q) ) u_highbits (
        .i_w(intt_ram0_rdata_a), .o_w1(w1_calc)
    );

    // ==========================================
    // 4. 驱动数据向 RAM0 的调度 MUX
    // ==========================================
    always @(*) begin
        y_we = 1'b0;
        y_addr_w = 10'd0; y_wdata = 24'd0;
        if (state == S_WAIT_Y && rejsam_valid) begin
            y_we = 1'b1;
            y_addr_w = {l_cnt[1:0], general_cnt[7:0]}; y_wdata = y_mod_q;
        end
    end
    assign y_addr_r = {l_cnt[1:0], general_cnt[7:0]};

    always @(*) begin
        c_post_we = 1'b0; c_post_addr_w = 8'd0; c_post_wdata = 24'd0;
        if (state == S_COPY_C && general_cnt > 0) begin
            c_post_we = 1'b1;
            c_post_addr_w = general_cnt[7:0] - 8'd1; c_post_wdata = ntt_ram0_rdata_a;
        end
    end

    always @(*) begin
        ntt_we_user = 1'b0;
        ntt_addr_user = 8'd0; ntt_wdata_user = 24'd0;
        if (state == S_WAIT_Y) begin
            ntt_we_user = rejsam_valid;
            ntt_addr_user = general_cnt[7:0]; ntt_wdata_user = y_mod_q;
        end else if (state == S_COPY_REQ || state == S_COPY_WAIT || state == S_COPY_STORE) begin
            ntt_addr_user = general_cnt[7:0];
        end else if (state == S_SIB_CLEAR) begin
            ntt_we_user = 1'b1;
            ntt_addr_user = general_cnt[7:0];
            ntt_wdata_user = 24'd0; 
        end else if (state == S_SIB_WAIT) begin
            ntt_we_user = sib_we;
            ntt_addr_user = sib_addr; ntt_wdata_user = sib_wdata;
        end else if (state == S_COPY_C) begin
            ntt_addr_user = general_cnt[7:0];
        end
    end

    wire is_mul_state = (state == S_CS1_MUL || state == S_CS2_MUL || state == S_CT0_MUL);
    reg [5:0] mul_valid_sr;
    reg [7:0] mul_addr_sr [0:5];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_valid_sr <= 0;
            mul_addr_sr[0] <= 0; mul_addr_sr[1] <= 0;
            mul_addr_sr[2] <= 0; mul_addr_sr[3] <= 0; mul_addr_sr[4] <= 0; mul_addr_sr[5] <= 0;
        end else begin
            if (is_mul_state) begin
                if (m_cnt < 256) begin mul_valid_sr[0] <= 1'b1;
                mul_addr_sr[0] <= m_cnt[7:0]; end 
                else mul_valid_sr[0] <= 1'b0;
            end else mul_valid_sr[0] <= 1'b0;
            
            mul_valid_sr[5:1] <= mul_valid_sr[4:0];
            mul_addr_sr[1] <= mul_addr_sr[0]; mul_addr_sr[2] <= mul_addr_sr[1];
            mul_addr_sr[3] <= mul_addr_sr[2]; mul_addr_sr[4] <= mul_addr_sr[3];
            mul_addr_sr[5] <= mul_addr_sr[4];
        end
    end

    assign o_s1_rd_addr = {l_cnt[1:0], m_cnt[7:0]};
    assign o_s2_rd_addr = {k_cnt[1:0], m_cnt[7:0]};
    assign o_t0_rd_addr = {k_cnt[1:0], m_cnt[7:0]}; 
    assign c_post_addr_r = m_cnt[7:0];

    reg [23:0] c_val_reg, sx_val_reg;
    reg [47:0] prod_reg;
    
    always @(posedge clk) begin
        c_val_reg <= c_post_rdata;
        sx_val_reg <= (state == S_CS1_MUL) ? i_s1_rd_data : 
                      (state == S_CS2_MUL) ? i_s2_rd_data : i_t0_rd_data;
        prod_reg <= c_val_reg * sx_val_reg;
    end

    wire [23:0] barrett_res;
    Barrett_reduce #( .WIDTH(24) ) u_barrett (
        .clk(clk), .prod(prod_reg), .q(Q), .mu(26'd33587228), .res(barrett_res)
    );

    always @(*) begin
        intt_we_user = 1'b0; intt_addr_user = 8'd0; intt_wdata_user = 24'd0;
        if (state == S_MAC_RUN || state == S_MAC_WAIT_RES) begin
            intt_we_user = mac_res_valid;
            intt_addr_user = mac_res_m_idx; intt_wdata_user = mac_res_data;
        end else if (state == S_OUT_REQ || state == S_OUT_WAIT || state == S_OUT_STORE) begin
            intt_addr_user = general_cnt[7:0];
        end else if (is_mul_state && mul_valid_sr[5]) begin
            intt_we_user = 1'b1;
            intt_addr_user = mul_addr_sr[5]; intt_wdata_user = barrett_res;
        end else if (state == S_Z_ADD_OUT || state == S_CS2_OUT || state == S_CT0_OUT) begin
            intt_addr_user = general_cnt[7:0];
        end
    end

    wire [24:0] z_sum = intt_ram0_rdata_a + y_rdata;
    wire [23:0] z_mod = (z_sum >= Q) ? (z_sum - Q) : z_sum[23:0];

    // ==========================================
    // 5. 巨型主状态机
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_w_valid <= 0; o_w_data <= 0; o_w1_data <= 0; 
            o_z_valid <= 0; o_z_data <= 0; o_cs2_valid <= 0;
            o_cs2_data <= 0;
            o_w_poly_idx <= 0; o_w_coeff_idx <= 0; o_z_poly_idx <= 0; o_z_coeff_idx <= 0;
            o_cs2_poly_idx <= 0;
            o_cs2_coeff_idx <= 0; o_r0_valid <= 0; o_r0_data <= 0;
            o_r0_poly_idx <= 0; o_r0_coeff_idx <= 0; o_ct0_valid <= 0;
            o_ct0_data <= 0;
            o_ct0_poly_idx <= 0; o_ct0_coeff_idx <= 0; w_we <= 0; w_addr_w <= 0; w_wdata <= 0;
            wm_cs2_we <= 0; wm_cs2_addr_w <= 0; wm_cs2_wdata <= 0; o_w_done <= 0;
            o_all_done <= 0; o_rej_flag <= 0;
            l_cnt <= 0; k_cnt <= 0; l_cnt_a <= 0;
            general_cnt <= 0; m_cnt <= 0; mac_out_cnt <= 0;
            rejsam_start <= 0;
            sib_start <= 0; ntt_start <= 0; intt_start <= 0; y_post_we <= 0;
            y_post_addr_w <= 0;
            y_post_wdata <= 0;
        end else begin
            rejsam_start <= 0;
            sib_start <= 0; ntt_start <= 0; intt_start <= 0;
            y_post_we <= 0; w_we <= 0; wm_cs2_we <= 0;
            o_w_valid <= 0; o_z_valid <= 0; 
            o_cs2_valid <= 0; o_r0_valid <= 0; o_ct0_valid <= 0; o_all_done <= 0;
            o_rej_flag <= 0;

            // ★ 核心修复应用：流水线写回 o_r0 和 wm_cs2 ★
            // 不干扰原 FSM 逻辑，仅将输出时机顺延一拍
            if (r0_pipe_en) begin
                o_r0_valid     <= 1'b1;
                o_r0_data      <= r0_calc;
                o_r0_poly_idx  <= r0_pipe_poly;
                o_r0_coeff_idx <= r0_pipe_addr;

                wm_cs2_we      <= 1'b1;
                wm_cs2_addr_w  <= {r0_pipe_poly[1:0], r0_pipe_addr};
                wm_cs2_wdata   <= w_minus_cs2_reg;
            end

            case (state)
                S_IDLE: begin
                    o_w_done <= 0;
                    l_cnt <= 0; k_cnt <= 0; l_cnt_a <= 0;
                    if (i_start) state <= S_GEN_Y;
                end
                
                S_GEN_Y: begin rejsam_start <= 1;
                    general_cnt <= 0; state <= S_WAIT_Y; end
                S_WAIT_Y: begin
                    if (rejsam_valid) general_cnt <= general_cnt + 1;
                    if (rejsam_done)  state <= S_NTT_START;
                end
                S_NTT_START: begin ntt_start <= 1;
                    state <= S_NTT_WAIT; end
                S_NTT_WAIT:  begin if (ntt_done) begin general_cnt <= 0;
                    state <= S_COPY_REQ; end end
                S_COPY_REQ:   state <= S_COPY_WAIT;
                S_COPY_WAIT:  state <= S_COPY_STORE;
                S_COPY_STORE: begin
                    y_post_we <= 1'b1;
                    y_post_addr_w <= {l_cnt[1:0], general_cnt[7:0]}; y_post_wdata <= ntt_ram0_rdata_a; 
                    if (general_cnt == 255) begin
                        general_cnt <= 0;
                        if (l_cnt == L_PARAM - 1) begin 
                            l_cnt_a <= 0;
                            k_cnt <= 0; state <= S_MAC_START; 
                        end else begin l_cnt <= l_cnt + 1; state <= S_GEN_Y;
                        end
                    end else begin general_cnt <= general_cnt + 1;
                        state <= S_COPY_REQ; end
                end

                S_MAC_START: begin m_cnt <= 0;
                    if (l_cnt_a == 0) mac_out_cnt <= 0; state <= S_MAC_RUN;
                end
                S_MAC_RUN: begin
                    if (i_A_valid) m_cnt <= m_cnt + 1;
                    if (mac_res_valid) mac_out_cnt <= mac_out_cnt + 1;
                    if (m_cnt == 256) begin 
                        if (l_cnt_a < L_PARAM - 1) begin l_cnt_a <= l_cnt_a + 1;
                            state <= S_MAC_START; end 
                        else state <= S_MAC_WAIT_RES;
                    end
                end
                S_MAC_WAIT_RES: begin
                    if (mac_res_valid) mac_out_cnt <= mac_out_cnt + 1;
                    if (mac_out_cnt >= 256 || (mac_res_valid && mac_out_cnt == 255)) state <= S_INTT_START;
                end
                S_INTT_START: begin intt_start <= 1;
                    state <= S_INTT_WAIT; end
                S_INTT_WAIT:  begin if (intt_done) begin general_cnt <= 0;
                    state <= S_OUT_REQ; end end
                S_OUT_REQ:   state <= S_OUT_WAIT;
                S_OUT_WAIT:  state <= S_OUT_STORE;
                S_OUT_STORE: begin
                    o_w_valid <= 1'b1;
                    o_w_data <= intt_ram0_rdata_a; o_w1_data <= w1_calc; 
                    o_w_poly_idx <= k_cnt; o_w_coeff_idx <= general_cnt[7:0];
                    w_we <= 1'b1; w_addr_w <= {k_cnt[1:0], general_cnt[7:0]};
                    w_wdata <= intt_ram0_rdata_a;

                    if (general_cnt == 255) begin
                        general_cnt <= 0;
                        if (k_cnt == K_PARAM - 1) state <= S_W_DONE;
                        else begin k_cnt <= k_cnt + 1; l_cnt_a <= 0;
                            state <= S_MAC_START; end
                    end else begin general_cnt <= general_cnt + 1;
                        state <= S_OUT_REQ; end
                end
                S_W_DONE: begin
                    o_w_done <= 1'b1;
                    if (i_c_start) begin 
                        o_w_done <= 0;
                        general_cnt <= 0;     
                        state <= S_SIB_CLEAR; 
                    end
                end

                S_SIB_CLEAR: begin
                    if (general_cnt < 256) begin
                        general_cnt <= general_cnt + 1;
                    end else begin
                        state <= S_SIB_START;
                    end
                end

                S_SIB_START: begin sib_start <= 1;
                    state <= S_SIB_WAIT; end
                
                S_SIB_WAIT:  begin if (sib_done) state <= S_C_NTT_START;
                end
                S_C_NTT_START: begin ntt_start <= 1;
                    state <= S_C_NTT_WAIT; end
                S_C_NTT_WAIT:  begin if (ntt_done) begin general_cnt <= 0;
                    state <= S_COPY_C; end end

                S_COPY_C: begin
                    if (general_cnt < 257) general_cnt <= general_cnt + 1;
                    else begin general_cnt <= 0; m_cnt <= 0; l_cnt <= 0; state <= S_CS1_MUL;
                    end
                end

                S_CS1_MUL: begin
                    if (m_cnt < 256) m_cnt <= m_cnt + 1;
                    else if (mul_valid_sr == 0) state <= S_CS1_INTT_START;
                end
                S_CS1_INTT_START: begin intt_start <= 1;
                    state <= S_CS1_INTT_WAIT; end
                S_CS1_INTT_WAIT:  begin if (intt_done) begin general_cnt <= 0;
                    state <= S_Z_ADD_OUT; end end

                S_Z_ADD_OUT: begin
                    if (general_cnt < 256) begin general_cnt <= general_cnt + 1;
                    end else if (!out_valid_d1) begin
                        general_cnt <= 0;
                        m_cnt <= 0;
                        if (l_cnt == L_PARAM - 1) begin k_cnt <= 0; state <= S_CS2_MUL;
                        end 
                        else begin l_cnt <= l_cnt + 1;
                        state <= S_CS1_MUL; end
                    end
                    if (out_valid_d1) begin
                        o_z_valid <= 1'b1;
                        o_z_data <= z_mod; 
                        o_z_poly_idx <= l_cnt; o_z_coeff_idx <= out_addr_d1;
                    end
                end

                S_CS2_MUL: begin
                    if (m_cnt < 256) m_cnt <= m_cnt + 1;
                    else if (mul_valid_sr == 0) state <= S_CS2_INTT_START;
                end
                S_CS2_INTT_START: begin intt_start <= 1;
                    state <= S_CS2_INTT_WAIT; end
                S_CS2_INTT_WAIT:  begin if (intt_done) begin general_cnt <= 0;
                    state <= S_CS2_OUT; end end

                S_CS2_OUT: begin
                    if (general_cnt < 256) begin general_cnt <= general_cnt + 1;
                    end else if (!out_valid_d1) begin
                        general_cnt <= 0;
                        m_cnt <= 0;
                        if (k_cnt == K_PARAM - 1) begin k_cnt <= 0; state <= S_CT0_MUL;
                        end 
                        else begin k_cnt <= k_cnt + 1;
                        state <= S_CS2_MUL; end
                    end
                    
                    if (out_valid_d1) begin
                        o_cs2_valid <= 1'b1;
                        o_cs2_data <= intt_ram0_rdata_a;
                        o_cs2_poly_idx <= k_cnt; o_cs2_coeff_idx <= out_addr_d1;
                        
                        // o_r0 和 wm_cs2 的写入已经被提取到上面的 r0_pipe_en 流水线逻辑中了
                    end
                end

                S_CT0_MUL: begin
                    if (m_cnt < 256) m_cnt <= m_cnt + 1;
                    else if (mul_valid_sr == 0) state <= S_CT0_INTT_START;
                end
                S_CT0_INTT_START: begin intt_start <= 1;
                    state <= S_CT0_INTT_WAIT; end
                S_CT0_INTT_WAIT:  begin if (intt_done) begin general_cnt <= 0;
                    state <= S_CT0_OUT; end end

                S_CT0_OUT: begin
                    if (general_cnt < 256) begin general_cnt <= general_cnt + 1;
                    end else if (!out_valid_d1) begin
                        general_cnt <= 0;
                        m_cnt <= 0;
                        if (k_cnt == K_PARAM - 1) state <= S_ALL_DONE_WAIT; // <--- 修改：跳转到 WAIT
                        else begin k_cnt <= k_cnt + 1;
                        state <= S_CT0_MUL; end
                    end

                    if (out_valid_d1) begin
                        o_ct0_valid <= 1'b1;
                        o_ct0_data <= intt_ram0_rdata_a;
                        o_ct0_poly_idx <= k_cnt; o_ct0_coeff_idx <= out_addr_d1;
                    end
                end

                // ★ 新增等待状态，补偿流水线引入的 1 拍延迟 ★
                S_ALL_DONE_WAIT: begin
                    state <= S_ALL_DONE;
                end

                S_ALL_DONE: begin 
                    o_all_done <= 1'b1;
                    o_rej_flag <= rej_flag_reg | (hint_cnt > 11'd80);
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule