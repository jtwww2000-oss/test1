`timescale 1ns / 1ps

module KeyGen (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire [255:0]  seed,
    
    output reg           done,
    output reg           busy,
  
    // --- 公钥 PK 流式输出接口 ---
    output wire          pk_valid,
    output wire [8:0]    pk_addr,
    output wire [31:0]   pk,
    
    // --- 私钥 SK 流式输出接口 ---
    output wire          sk_valid,
    output wire [9:0]    sk_addr,
    output wire [31:0]   sk
);

    // =========================================================================
    // 内部线网声明 (替代原先的 output port)
    // =========================================================================
    wire [511:0] internal_tr;   // 替代原来的 o_tr
    
    wire [22:0]  A_data;
    wire         A_valid;
    reg  [2:0]   A_row_idx;
    reg  [2:0]   A_col_idx;
    wire [3:0]   s_raw_data;
    wire         s_raw_valid;
    reg  [3:0]   s_poly_idx;
    wire         s_is_s2;     
    wire [23:0]  s_post_data;
    reg          s_post_valid;
    reg  [3:0]   s_post_poly_idx;
    wire         s_post_is_s2;
    wire         o_AS1_valid;
    wire [23:0]  o_AS1_data;
    wire [7:0]   o_AS1_idx;
    reg          o_dbg_intt_valid;
    wire [23:0]  o_dbg_intt_data;
    wire [7:0]   o_dbg_intt_idx;
    wire         o_p2r_valid;
    wire [9:0]   o_t1_data;
    wire [12:0]  o_t0_data;
    reg  [7:0]   o_p2r_idx;

    // =========================================================================
    // 参数与内部信号
    // =========================================================================
    localparam [23:0] Q = 24'd8380417;
    localparam [3:0] K_PARAM = 4'd4;
    localparam [3:0] L_PARAM = 4'd4;
    localparam [3:0] ETA_VAL = 4'd2;

    localparam S_IDLE           = 6'd0;
    localparam S_EXPAND_SEED    = 6'd1;
    localparam S_WAIT_EXPAND    = 6'd2;
    localparam S_S_INIT         = 6'd3;
    localparam S_S_START        = 6'd4;
    localparam S_S_WAIT_ACK     = 6'd5;
    localparam S_S_FILL_RAM     = 6'd6;
    localparam S_S_NTT_START    = 6'd7;
    localparam S_S_NTT_RUN      = 6'd8;
    localparam S_S_DUMP_INIT    = 6'd9;
    localparam S_S_DUMP_RUN     = 6'd10;
    localparam S_S_DUMP_WAIT    = 6'd11;
    localparam S_S_NEXT         = 6'd12;
    localparam S_A_INIT         = 6'd13;
    localparam S_A_START        = 6'd14;
    localparam S_A_WAIT_ACK     = 6'd15;
    localparam S_A_RUN          = 6'd16;
    localparam S_A_WAIT_RESULT  = 6'd17;
    localparam S_A_INTT_START   = 6'd18; 
    localparam S_A_INTT_RUN     = 6'd19;
    localparam S_A_INTT_CHECK   = 6'd20;
    localparam S_A_ROW_DONE     = 6'd21;
    localparam S_TR_START       = 6'd22;
    localparam S_TR_WAIT        = 6'd23;
    localparam S_SK_START       = 6'd24;
    localparam S_SK_WAIT        = 6'd25;
    localparam S_DONE           = 6'd31;

    reg [5:0] state;

    reg  shake_seed_start;
    wire shake_seed_busy;
    wire shake_seed_valid;
    wire [1023:0] shake_seed_out;
    reg [255:0] rho;
    reg [511:0] rho_prime;
    reg [255:0] key_K; 
    
    reg  rejsam_a_start;
    wire rejsam_a_done; wire rejsam_a_valid; wire [22:0] rejsam_a_data;
    reg  rejsam_s_start; wire rejsam_s_done; wire rejsam_s_valid; wire [3:0]  rejsam_s_data;
    reg  ntt_start; wire ntt_done;
    reg  intt_start; wire intt_done;

    // =========================================================================
    // 双通道 Ping-Pong RAM 接口定义
    // =========================================================================
    reg  [7:0]  top_ram_addr;
    reg top_ram_we; reg [23:0] top_ram_wdata;
    reg  [8:0]  s_sample_cnt; reg [8:0] dump_cnt; reg ram_read_req;
    reg  [7:0]  m_cnt;
    reg  [7:0]  s_dump_addr_reg; reg [8:0] as1_out_cnt; reg [8:0] intt_chk_cnt;

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

    // NTT 连线
    wire [7:0]  ntt_ram0_addr_a, ntt_ram0_addr_b, ntt_ram1_addr_a, ntt_ram1_addr_b;
    wire        ntt_ram0_we_a,   ntt_ram0_we_b,   ntt_ram1_we_a,   ntt_ram1_we_b;
    wire [23:0] ntt_ram0_wdata_a,ntt_ram0_wdata_b,ntt_ram1_wdata_a,ntt_ram1_wdata_b;

    // INTT 连线
    wire [7:0]  intt_ram0_addr_a, intt_ram0_addr_b, intt_ram1_addr_a, intt_ram1_addr_b;
    wire        intt_ram0_we_a,   intt_ram0_we_b,   intt_ram1_we_a,   intt_ram1_we_b;
    wire [23:0] intt_ram0_wdata_a,intt_ram0_wdata_b,intt_ram1_wdata_a,intt_ram1_wdata_b;

    // --- S1 Banks ---
    wire [7:0]  s1_ram_addr; wire [3:0]  s1_ram_sel;
    wire [23:0] s1_ram_rdata;
    wire [23:0] s1_bank_dout [0:7]; wire [7:0] s1_bank_addr; wire s1_bank_we [0:7]; wire [23:0] s1_bank_din;

    // Packer Control
    reg pk_pack_start; reg tr_start; wire tr_done; reg sk_pack_start;
    wire [3:0] sk_s_ram_sel; wire [7:0] sk_s_ram_addr;
    wire [3:0] sk_t0_ram_sel; wire [7:0] sk_t0_ram_addr; wire [12:0] sk_t0_ram_rdata;
    wire sk_packer_done;

    // --- T0 RAM 定义 ---
    reg [7:0]  t0_ram_addr; reg t0_ram_we [0:3]; reg [12:0] t0_ram_wdata;
    wire [12:0] t0_bank_dout [0:3];

    // =========================================================================
    // 内部 PK BRAM 存储 (仅用于内部 calc_tr 计算)
    // =========================================================================
    (* ram_style = "block" *) reg [31:0] pk_bram [0:511];

    always @(posedge clk) begin 
        if (pk_valid) pk_bram[pk_addr] <= pk; 
    end
    
    wire [8:0]  calc_tr_raddr; 
    reg  [31:0] calc_tr_rdata;
    
    wire [8:0]  internal_pk_raddr = (state == S_TR_WAIT || state == S_TR_START) ? calc_tr_raddr : 9'd0;
    
    always @(posedge clk) begin
        calc_tr_rdata <= pk_bram[internal_pk_raddr];
    end


// =========================================================================
    // 新增：专用于存储 pre-NTT 原始 s1, s2 (值为0~4) 的 BRAM，仅供私钥打包使用
    // =========================================================================
    (* ram_style = "block" *) reg [3:0] s_raw_bram [0:2047]; // 8多项式 x 256系数
    reg [3:0] sk_s_raw_out;

    always @(posedge clk) begin
        // 1. 写入端：在拒绝采样模块吐出系数的瞬间录下它 (0, 1, 2, 3, 4)
        if (rejsam_s_valid) begin
            s_raw_bram[{s_poly_idx[2:0], s_sample_cnt[7:0]}] <= rejsam_s_data;
        end
        // 2. 读出端：打包模块读取 (1拍延迟，完美契合 sk_packer 的 WAIT 状态时序)
        sk_s_raw_out <= s_raw_bram[{sk_s_ram_sel[2:0], sk_s_ram_addr}];
    end

    // =========================================================================
    // 模块实例化
    // =========================================================================
    SHAKE256 #( .RATE(1088), .CAPACITY(512), .OUTPUT_LEN_BYTES(128), .ABSORB_LEN(256) ) u_shake_expand (
        .clk(clk), .rst_n(rst_n), .i_start(shake_seed_start), .i_seed(seed), .o_busy(shake_seed_busy), .i_squeeze_req(1'b0), .o_squeeze_valid(shake_seed_valid), .o_squeeze_data(shake_seed_out)
    );

    Rejsam_a u_rejsam_a ( .clk(clk), .rst_n(rst_n), .i_start(rejsam_a_start), .i_rho(rho), .i_row({5'd0, A_row_idx}), .i_column({5'd0, A_col_idx}), .o_coeff_valid(rejsam_a_valid), .o_coeff_data(rejsam_a_data), .o_done(rejsam_a_done) );
    Rejsam_s u_rejsam_s ( .clk(clk), .rst_n(rst_n), .i_start(rejsam_s_start), .i_rho_prime(rho_prime), .i_row({12'd0, s_poly_idx}), .o_coeff_valid(rejsam_s_valid), .o_coeff_data(rejsam_s_data), .o_done(rejsam_s_done) );

    ntt_core #( .WIDTH(24) ) u_ntt ( 
        .clk(clk), .rst_n(rst_n), .start(ntt_start), .done(ntt_done), 
        .ram0_addr_a(ntt_ram0_addr_a), .ram0_we_a(ntt_ram0_we_a), .ram0_wdata_a(ntt_ram0_wdata_a), .ram0_rdata_a(ram0_rdata_a),
        .ram0_addr_b(ntt_ram0_addr_b), .ram0_we_b(ntt_ram0_we_b), .ram0_wdata_b(ntt_ram0_wdata_b), .ram0_rdata_b(ram0_rdata_b),
        .ram1_addr_a(ntt_ram1_addr_a), .ram1_we_a(ntt_ram1_we_a), .ram1_wdata_a(ntt_ram1_wdata_a), .ram1_rdata_a(ram1_rdata_a),
        .ram1_addr_b(ntt_ram1_addr_b), .ram1_we_b(ntt_ram1_we_b), .ram1_wdata_b(ntt_ram1_wdata_b), .ram1_rdata_b(ram1_rdata_b)
    );

    intt_core #( .WIDTH(24) ) u_intt ( 
        .clk(clk), .rst_n(rst_n), .start(intt_start), .done(intt_done), 
        .ram0_addr_a(intt_ram0_addr_a), .ram0_we_a(intt_ram0_we_a), .ram0_wdata_a(intt_ram0_wdata_a), .ram0_rdata_a(ram0_rdata_a),
        .ram0_addr_b(intt_ram0_addr_b), .ram0_we_b(intt_ram0_we_b), .ram0_wdata_b(intt_ram0_wdata_b), .ram0_rdata_b(ram0_rdata_b),
        .ram1_addr_a(intt_ram1_addr_a), .ram1_we_a(intt_ram1_we_a), .ram1_wdata_a(intt_ram1_wdata_a), .ram1_rdata_a(ram1_rdata_a),
        .ram1_addr_b(intt_ram1_addr_b), .ram1_we_b(intt_ram1_we_b), .ram1_wdata_b(intt_ram1_wdata_b), .ram1_rdata_b(ram1_rdata_b)
    );

    tdpram_24x256 u_ram0 ( .clk(clk), .we_a(ram0_we_a), .addr_a(ram0_addr_a), .din_a(ram0_wdata_a), .dout_a(ram0_rdata_a), .we_b(ram0_we_b), .addr_b(ram0_addr_b), .din_b(ram0_wdata_b), .dout_b(ram0_rdata_b) );
    tdpram_24x256 u_ram1 ( .clk(clk), .we_a(ram1_we_a), .addr_a(ram1_addr_a), .din_a(ram1_wdata_a), .dout_a(ram1_rdata_a), .we_b(ram1_we_b), .addr_b(ram1_addr_b), .din_b(ram1_wdata_b), .dout_b(ram1_rdata_b) );

    // -------------------------------------------------------------------------
    // 余下核心连线
    // -------------------------------------------------------------------------
    wire s1_mux_select_packer = (state == S_SK_START || state == S_SK_WAIT);
    wire [7:0] internal_s1_addr = (state == S_A_RUN || state == S_A_WAIT_RESULT) ?
        s1_ram_addr : ((state == S_S_DUMP_RUN || state == S_S_DUMP_WAIT) ? s_dump_addr_reg : (state == S_A_INTT_CHECK ? top_ram_addr : 8'd0));
    assign s1_bank_addr = s1_mux_select_packer ? sk_s_ram_addr : internal_s1_addr;
    assign s1_bank_din  = s_post_data;
    wire [3:0] current_s1_sel = s1_mux_select_packer ?
        sk_s_ram_sel : s1_ram_sel;
    
    genvar k;
    generate
        for (k = 0; k < 8; k = k + 1) begin : gen_s1_ram 
            assign s1_bank_we[k] = (state == S_S_DUMP_RUN || state == S_S_DUMP_WAIT) && s_post_valid && (s_post_poly_idx == k);
            tdpram_24x256 u_s1_ram_inst ( .clk(clk), .we_a(s1_bank_we[k]), .addr_a(s1_bank_addr), .din_a(s1_bank_din), .dout_a(s1_bank_dout[k]), .we_b(1'b0), .addr_b(8'd0), .din_b(24'd0), .dout_b() );
        end
    endgenerate
    
    assign s1_ram_rdata = s1_bank_dout[s1_ram_sel];

    generate
        for (k = 0; k < 4; k = k + 1) begin : gen_t0_ram 
            always @(*) begin t0_ram_we[k] = o_p2r_valid && (A_row_idx == k); end
            wire [7:0] t0_addr_mux = (state == S_SK_START || state == S_SK_WAIT) ?
                sk_t0_ram_addr : o_p2r_idx;
            reg [12:0] mem [0:255];
            always @(posedge clk) begin if (t0_ram_we[k]) mem[o_p2r_idx] <= o_t0_data; end
            assign t0_bank_dout[k] = mem[t0_addr_mux];
        end
    endgenerate

    wire [7:0] acc_raddr, acc_waddr; wire acc_we; wire [23:0] acc_wdata, acc_rdata;
    tdpram_24x256 u_acc_ram ( .clk(clk), .we_a(1'b0), .addr_a(acc_raddr), .din_a(24'd0), .dout_a(acc_rdata), .we_b(acc_we), .addr_b(acc_waddr), .din_b(acc_wdata), .dout_b() );

    MatrixVecMul_Core #( .WIDTH(24) ) u_mat_mul ( .clk(clk), .rst_n(rst_n), .i_A_valid(rejsam_a_valid), .i_A_data ({1'b0, rejsam_a_data}), .i_m_idx(m_cnt), .i_j_idx({1'b0, A_col_idx}), .i_l_param(L_PARAM), .o_s1_addr(s1_ram_addr), .o_s1_poly_idx(s1_ram_sel), .i_s1_rdata(s1_ram_rdata), .o_acc_addr(acc_raddr), .o_acc_waddr(acc_waddr), .o_acc_we(acc_we), .o_acc_wdata(acc_wdata), .i_acc_rdata(acc_rdata), .o_res_valid(o_AS1_valid), .o_res_data (o_AS1_data), .o_res_m_idx(o_AS1_idx) );

    reg [23:0] s_calc_val;
    always @(*) begin
        if ({20'd0, rejsam_s_data} <= {20'd0, ETA_VAL}) s_calc_val = {20'd0, ETA_VAL} - {20'd0, rejsam_s_data};
        else s_calc_val = {20'd0, ETA_VAL} + Q - {20'd0, rejsam_s_data};
    end
    
    wire [23:0] s2_for_t = s1_bank_dout[L_PARAM + A_row_idx];
    wire [23:0] t_val;
    mod_add #( .WIDTH(24) ) u_add_t ( .a(ram0_rdata_a), .b(s2_for_t), .q(Q), .res(t_val) );
    Power2Round #( .WIDTH(24) ) u_power2round ( .clk(clk), .rst_n(rst_n), .i_valid(o_dbg_intt_valid), .i_data(t_val), .o_valid(o_p2r_valid), .o_t1(o_t1_data), .o_t0(o_t0_data) );

    pk_packer u_pk_packer ( 
        .clk(clk), .rst_n(rst_n), .start_pack(pk_pack_start), 
        .rho(rho), .t1_valid(o_p2r_valid), .t1_data(o_t1_data), 
        .o_pk_we(pk_valid), .o_pk_addr(pk_addr), .o_pk_wdata(pk), .o_pk_valid() 
    );

    calc_tr u_calc_tr ( 
        .clk(clk), .rst_n(rst_n), .i_start(tr_start), 
        .o_pk_raddr(calc_tr_raddr), .i_pk_rdata(calc_tr_rdata), 
        .o_done(tr_done), .o_tr(internal_tr) 
    );

sk_packer u_sk_packer ( 
        .clk(clk), .rst_n(rst_n), .i_start(sk_pack_start), 
        .rho(rho), .key_K(key_K), .tr(internal_tr), 
        .o_s_ram_sel(sk_s_ram_sel), .o_s_ram_addr(sk_s_ram_addr), 
        
        // ★★★ 核心修改：不再连接 s1_bank_dout，连接新录制的纯净原始 s
        .i_s_ram_data({20'd0, sk_s_raw_out}), 
        
        .o_t0_ram_sel(sk_t0_ram_sel), .o_t0_ram_addr(sk_t0_ram_addr), 
        .i_t0_ram_data(t0_bank_dout[sk_t0_ram_sel]), 
        .o_sk_we(sk_valid), .o_sk_addr(sk_addr), .o_sk_wdata(sk), .o_sk_valid(sk_packer_done) 
    );

    // =========================================================================
    // RAM 总线复用选择器 (MUX)
    // =========================================================================
    always @(*) begin
        if (state == S_S_NTT_RUN) begin
            ram0_addr_a = ntt_ram0_addr_a;
            ram0_we_a = ntt_ram0_we_a; ram0_wdata_a = ntt_ram0_wdata_a;
            ram0_addr_b = ntt_ram0_addr_b; ram0_we_b = ntt_ram0_we_b; ram0_wdata_b = ntt_ram0_wdata_b;
            ram1_addr_a = ntt_ram1_addr_a;
            ram1_we_a = ntt_ram1_we_a; ram1_wdata_a = ntt_ram1_wdata_a;
            ram1_addr_b = ntt_ram1_addr_b; ram1_we_b = ntt_ram1_we_b; ram1_wdata_b = ntt_ram1_wdata_b;
        end else if (state == S_A_INTT_RUN) begin
            ram0_addr_a = intt_ram0_addr_a;
            ram0_we_a = intt_ram0_we_a; ram0_wdata_a = intt_ram0_wdata_a;
            ram0_addr_b = intt_ram0_addr_b; ram0_we_b = intt_ram0_we_b; ram0_wdata_b = intt_ram0_wdata_b;
            ram1_addr_a = intt_ram1_addr_a;
            ram1_we_a = intt_ram1_we_a; ram1_wdata_a = intt_ram1_wdata_a;
            ram1_addr_b = intt_ram1_addr_b; ram1_we_b = intt_ram1_we_b; ram1_wdata_b = intt_ram1_wdata_b;
        end else begin
            ram1_addr_a = 8'd0;
            ram1_we_a = 1'b0; ram1_wdata_a = 24'd0;
            ram1_addr_b = 8'd0; ram1_we_b = 1'b0; ram1_wdata_b = 24'd0;
            if (state == S_A_RUN || state == S_A_WAIT_RESULT) begin
                ram0_addr_a = o_AS1_idx;
                ram0_we_a = o_AS1_valid; ram0_wdata_a = o_AS1_data;
                ram0_addr_b = 8'd0;      ram0_we_b = 1'b0;        ram0_wdata_b = 24'd0;
            end else if (state == S_A_INTT_CHECK) begin
                ram0_addr_a = top_ram_addr;
                ram0_we_a = 1'b0; ram0_wdata_a = 24'd0;
                ram0_addr_b = 8'd0;         ram0_we_b = 1'b0; ram0_wdata_b = 24'd0;
            end else begin
                ram0_addr_a = top_ram_addr;
                ram0_we_a = top_ram_we; ram0_wdata_a = top_ram_wdata;
                ram0_addr_b = 8'd0;         ram0_we_b = 1'b0;       ram0_wdata_b = 24'd0;
            end
        end
    end

    // --- Output Assignments ---
    assign A_data  = rejsam_a_data;
    assign A_valid = rejsam_a_valid;
    assign s_raw_data  = rejsam_s_data; assign s_raw_valid = rejsam_s_valid; assign s_is_s2 = (s_poly_idx >= L_PARAM);
    always @(posedge clk or negedge rst_n) begin if (!rst_n) s_post_valid <= 0; else s_post_valid <= ram_read_req;
    end
    
    assign s_post_data = ram0_rdata_a;
    assign s_post_is_s2 = (s_post_poly_idx >= L_PARAM);
    always @(posedge clk or negedge rst_n) begin if (!rst_n) o_dbg_intt_valid <= 0;
    else o_dbg_intt_valid <= (state == S_A_INTT_CHECK) && ram_read_req; end
    assign o_dbg_intt_data = t_val;
    assign o_dbg_intt_idx = (top_ram_addr == 0) ? 8'd255 : (top_ram_addr - 1'b1);
    always @(posedge clk) begin o_p2r_idx <= o_dbg_intt_idx;
    end

    // =========================================================================
    // FSM (完全不变)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0; busy <= 0;
            shake_seed_start <= 0; rejsam_a_start <= 0; rejsam_s_start <= 0;
            ntt_start <= 0;
            intt_start <= 0; rho <= 0; rho_prime <= 0; key_K <= 0;
            A_row_idx <= 0; A_col_idx <= 0;
            s_poly_idx <= 0; s_post_poly_idx <= 0;
            top_ram_addr <= 0; top_ram_we <= 0; top_ram_wdata <= 0;
            s_sample_cnt <= 0;
            dump_cnt <= 0; ram_read_req <= 0;
            m_cnt <= 0; s_dump_addr_reg <= 0; as1_out_cnt <= 0; intt_chk_cnt <= 0;
            pk_pack_start <= 0; tr_start <= 0; sk_pack_start <= 0;
        end else begin
            shake_seed_start <= 0;
            rejsam_a_start <= 0; rejsam_s_start <= 0;
            ntt_start <= 0; intt_start <= 0; top_ram_we <= 0; ram_read_req <= 0;
            pk_pack_start <= 0; tr_start <= 0; sk_pack_start <= 0;
            
            case (state)
                S_IDLE: begin done <= 0;
                    if (start) begin busy <= 1; state <= S_EXPAND_SEED; end else busy <= 0;
                end
                S_EXPAND_SEED: begin shake_seed_start <= 1;
                    state <= S_WAIT_EXPAND; end
                S_WAIT_EXPAND: begin
                    if (shake_seed_valid) begin 
                        rho <= shake_seed_out[255:0];
                        rho_prime <= shake_seed_out[767:256]; key_K <= shake_seed_out[1023:768]; 
                        pk_pack_start <= 1; state <= S_S_INIT;
                    end
                end
                S_S_INIT: begin s_poly_idx <= 0;
                    state <= S_S_START; end
                S_S_START: begin rejsam_s_start <= 1;
                    s_sample_cnt <= 0; state <= S_S_WAIT_ACK; end
                S_S_WAIT_ACK: if (!rejsam_s_done) state <= S_S_FILL_RAM;
                S_S_FILL_RAM: begin
                    if (rejsam_s_valid) begin top_ram_we <= 1;
                        top_ram_addr <= s_sample_cnt[7:0]; top_ram_wdata <= s_calc_val; s_sample_cnt <= s_sample_cnt + 1;
                    end
                    if (rejsam_s_done) begin if (s_is_s2) state <= S_S_DUMP_INIT;
                        else state <= S_S_NTT_START; end
                end
                S_S_NTT_START: begin ntt_start <= 1;
                    state <= S_S_NTT_RUN; end
                S_S_NTT_RUN: if (ntt_done) state <= S_S_DUMP_INIT;
                S_S_DUMP_INIT: begin s_post_poly_idx <= s_poly_idx; dump_cnt <= 0; top_ram_we <= 0; top_ram_addr <= 0; ram_read_req <= 1; s_dump_addr_reg <= 0;
                    state <= S_S_DUMP_RUN; end
                S_S_DUMP_RUN: begin
                    if (dump_cnt < 255) begin top_ram_addr <= dump_cnt[7:0] + 1;
                        ram_read_req <= 1; dump_cnt <= dump_cnt + 1; end 
                    else begin ram_read_req <= 0;
                        dump_cnt <= 0; state <= S_S_DUMP_WAIT; end
                    if (s_post_valid) s_dump_addr_reg <= s_dump_addr_reg + 1;
                end
                S_S_DUMP_WAIT: begin if (s_post_valid) s_dump_addr_reg <= s_dump_addr_reg + 1;
                    else state <= S_S_NEXT; end
                S_S_NEXT: begin if (s_poly_idx < (L_PARAM + K_PARAM - 1)) begin s_poly_idx <= s_poly_idx + 1;
                    state <= S_S_START; end else state <= S_A_INIT; end
                S_A_INIT: begin A_row_idx <= 0;
                    A_col_idx <= 0; state <= S_A_START; end
                S_A_START: begin rejsam_a_start <= 1;
                    m_cnt <= 0; if (A_col_idx == 0) as1_out_cnt <= 0; state <= S_A_WAIT_ACK;
                end
                S_A_WAIT_ACK: if (!rejsam_a_done) state <= S_A_RUN;
                S_A_RUN: begin
                    if (rejsam_a_valid) m_cnt <= m_cnt + 1;
                    if (o_AS1_valid) as1_out_cnt <= as1_out_cnt + 1;
                    if (rejsam_a_done) begin if (A_col_idx < L_PARAM - 1) begin A_col_idx <= A_col_idx + 1;
                        state <= S_A_START; end else state <= S_A_WAIT_RESULT; end
                end
                S_A_WAIT_RESULT: begin
                    if (o_AS1_valid) as1_out_cnt <= as1_out_cnt + 1;
                    if (as1_out_cnt >= 256 || (o_AS1_valid && as1_out_cnt == 255)) state <= S_A_INTT_START;
                end
                S_A_INTT_START: begin intt_start <= 1;
                    state <= S_A_INTT_RUN; end
                S_A_INTT_RUN: begin if (intt_done) begin intt_chk_cnt <= 0;
                    top_ram_addr <= 0; ram_read_req <= 1; state <= S_A_INTT_CHECK; end end
                S_A_INTT_CHECK: begin
                    if (intt_chk_cnt < 255) begin top_ram_addr <= intt_chk_cnt[7:0] + 1;
                        ram_read_req <= 1; intt_chk_cnt <= intt_chk_cnt + 1; end 
                    else begin ram_read_req <= 0;
                        top_ram_addr <= 0; 
                        if (intt_chk_cnt == 256) begin state <= S_A_ROW_DONE; intt_chk_cnt <= 0;
                        end else intt_chk_cnt <= intt_chk_cnt + 1; end
                end
                S_A_ROW_DONE: begin
                    A_col_idx <= 0;
                    if (A_row_idx < K_PARAM - 1) begin A_row_idx <= A_row_idx + 1; state <= S_A_START; end else state <= S_TR_START;
                end
                S_TR_START: begin tr_start <= 1;
                    state <= S_TR_WAIT; end
                S_TR_WAIT: begin if (tr_done) state <= S_SK_START;
                end 
                S_SK_START: begin sk_pack_start <= 1;
                    state <= S_SK_WAIT; end
                S_SK_WAIT: begin if (sk_packer_done) state <= S_DONE;
                end
                S_DONE: begin done <= 1;
                    busy <= 0; state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule