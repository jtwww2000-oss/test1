`timescale 1ns / 1ps

module dilithium_sign_core #(
    parameter WIDTH = 24,
    parameter Q     = 24'd8380417,
    parameter L_PARAM = 4'd4,
    parameter K_PARAM = 4'd4,
    parameter OMEGA   = 80
)(
    input  wire          clk,
    input  wire          rst_n,
    
    input  wire          start,  
    input  wire [255:0]  M,            
    
    // --- SK 读取接口 (向外提供地址，读入数据) ---
    output wire [9:0]    sk_addr,
    input  wire [31:0]   sk,
    
    output reg           done,         
    
    // --- 签名 SIG 流式输出接口 ---
    output wire          sig_valid,
    output wire [9:0]    sig_addr,
    output wire [31:0]   sig_data
);

    // ==========================================
    // 1. 内部 RAM
    // ==========================================
    reg [23:0] A_ram  [0:4095];
    reg [23:0] s1_ram [0:1023];
    reg [23:0] s2_ram [0:1023];
    reg [23:0] t0_ram [0:1023];

    reg  sk_start;
    wire sk_done;
    wire sk_s1_we, sk_s2_we, sk_t0_we, sk_A_we;
    wire [9:0]  sk_s1_addr, sk_s2_addr, sk_t0_addr;
    wire [23:0] sk_s1_wdata, sk_s2_wdata, sk_t0_wdata;
    wire [11:0] sk_A_addr;
    wire [23:0] sk_A_wdata;
    wire [511:0] sk_u, sk_rho_prime;

    sign_sk_unpack_ntt u_sk_unpack (
        .clk(clk), .rst_n(rst_n), .start(sk_start), .done(sk_done),
        .o_sk_raddr(sk_addr), .i_sk_rdata(sk),
        .o_rho(), .o_K(), .o_tr(), 
        .o_s1_we(sk_s1_we), .o_s1_addr(sk_s1_addr), .o_s1_wdata(sk_s1_wdata),
        .o_s2_we(sk_s2_we), .o_s2_addr(sk_s2_addr), .o_s2_wdata(sk_s2_wdata),
        .o_t0_we(sk_t0_we), .o_t0_addr(sk_t0_addr), .o_t0_wdata(sk_t0_wdata),
        .i_M(M),
        .o_A_we(sk_A_we), .o_A_addr(sk_A_addr), .o_A_wdata(sk_A_wdata),
        .o_u(sk_u), .o_rho_prime(sk_rho_prime)
    );

    always @(posedge clk) begin
        if (sk_A_we)  A_ram[sk_A_addr]   <= sk_A_wdata;
        if (sk_s1_we) s1_ram[sk_s1_addr] <= sk_s1_wdata;
        if (sk_s2_we) s2_ram[sk_s2_addr] <= sk_s2_wdata;
        if (sk_t0_we) t0_ram[sk_t0_addr] <= sk_t0_wdata;
    end

    // ==========================================
    // 2. 签名核心流水线
    // ==========================================
    reg         sig_start;
    reg  [15:0] sig_rej_round;
    reg         sig_A_valid;
    reg  [23:0] sig_A_data;
    reg  [7:0]  sig_A_m_idx;
    reg  [3:0]  sig_A_j_idx;
    wire        sig_ready_for_A;
    wire        sig_w_valid;     
    wire [5:0]  sig_w1_data;
    wire [3:0]  sig_w_poly_idx;
    wire [7:0]  sig_w_coeff_idx;
    wire        sig_w_done;
    reg         sig_c_start;
    reg  [255:0] c_tilde_reg;
    
    wire [9:0]  sig_s1_rd_addr, sig_s2_rd_addr, sig_t0_rd_addr;
    reg [23:0] s1_rd_data_reg, s2_rd_data_reg, t0_rd_data_reg;

    always @(posedge clk) begin
        s1_rd_data_reg <= s1_ram[sig_s1_rd_addr];
        s2_rd_data_reg <= s2_ram[sig_s2_rd_addr];
        t0_rd_data_reg <= t0_ram[sig_t0_rd_addr];
    end

    wire        sig_rej_flag, sig_all_done;
    wire        sig_z_valid;     wire [23:0] sig_z_data;
    wire [3:0]  sig_z_poly_idx;  wire [7:0]  sig_z_coeff_idx;
    wire        sig_hint_pre_valid; wire       sig_hint_pre_data;
    wire [3:0]  sig_hint_pre_poly_idx; wire [7:0] sig_hint_pre_coeff_idx;

    sign_y_ntt_mac_intt #( .WIDTH(WIDTH), .Q(Q), .L_PARAM(L_PARAM), .K_PARAM(K_PARAM) ) u_sig_loop (
        .clk(clk), .rst_n(rst_n),
        .i_start(sig_start), .i_rho_prime(sk_rho_prime), .i_rej_round(sig_rej_round),
        .i_A_valid(sig_A_valid), .i_A_data(sig_A_data), .i_A_m_idx(sig_A_m_idx), .i_A_j_idx(sig_A_j_idx),
        .o_ready_for_A(sig_ready_for_A),
        
        .o_w_valid(sig_w_valid), .o_w_data(), .o_w1_data(sig_w1_data),
        .o_w_poly_idx(sig_w_poly_idx), .o_w_coeff_idx(sig_w_coeff_idx), .o_w_done(sig_w_done),
        
        .i_c_start(sig_c_start), .i_c_tilde(c_tilde_reg),
        .i_s1_rd_data(s1_rd_data_reg), .o_s1_rd_addr(sig_s1_rd_addr),
        .i_s2_rd_data(s2_rd_data_reg), .o_s2_rd_addr(sig_s2_rd_addr),
        .i_t0_rd_data(t0_rd_data_reg), .o_t0_rd_addr(sig_t0_rd_addr),
        
        .o_z_valid(sig_z_valid), .o_z_data(sig_z_data), .o_z_poly_idx(sig_z_poly_idx), .o_z_coeff_idx(sig_z_coeff_idx),
        .o_cs2_valid(), .o_cs2_data(), .o_cs2_poly_idx(), .o_cs2_coeff_idx(),
        .o_r0_valid(), .o_r0_data(), .o_r0_poly_idx(), .o_r0_coeff_idx(),
        .o_ct0_valid(), .o_ct0_data(), .o_ct0_poly_idx(), .o_ct0_coeff_idx(),
        
        .o_hint_pre_valid(sig_hint_pre_valid), .o_hint_pre_data(sig_hint_pre_data), 
        .o_hint_pre_poly_idx(sig_hint_pre_poly_idx), .o_hint_pre_coeff_idx(sig_hint_pre_coeff_idx),
        
        .o_rej_flag(sig_rej_flag), .o_all_done(sig_all_done)
    );

    wire [15:0] final_round = sig_rej_round;

    // ==========================================
    // 3. W1 缓存与 SHAKE_C 流式吸收
    // ==========================================
    (* ram_style = "block" *) reg [5:0] w1_bram [0:1023];
    reg [5:0] w1_rdata;
    reg [9:0] w1_raddr;

    always @(posedge clk) begin
        if (sig_w_valid) w1_bram[{sig_w_poly_idx[1:0], sig_w_coeff_idx}] <= sig_w1_data;
        w1_rdata <= w1_bram[w1_raddr];
    end

    // 流式 SHAKE256_stream
    reg          shake_c_start;
    wire         shake_c_busy;
    reg          shake_c_absorb_valid;
    reg  [63:0]  shake_c_absorb_data;
    reg  [6:0]   shake_c_absorb_bits;
    reg          shake_c_absorb_last;
    wire         shake_c_absorb_ready;
    wire         shake_c_squeeze_valid;
    wire [255:0] shake_c_squeeze_data;

    SHAKE256_stream #(
        .RATE(1088), .STATE_WIDTH(1600), .OUTPUT_LEN_BYTES(32)
    ) u_shake_c_stream (
        .clk(clk), .rst_n(rst_n),
        .i_start(shake_c_start), .o_busy(shake_c_busy),
        .i_absorb_valid(shake_c_absorb_valid),
        .i_absorb_data(shake_c_absorb_data),
        .i_absorb_bits(shake_c_absorb_bits),
        .i_absorb_last(shake_c_absorb_last),
        .o_absorb_ready(shake_c_absorb_ready),
        .i_squeeze_req(1'b0), // 仅需一次输出 32 bytes
        .o_squeeze_valid(shake_c_squeeze_valid),
        .o_squeeze_data(shake_c_squeeze_data)
    );

    // ========================================================
    // 4. Z_RAM 缓冲与 Hint 缓冲
    // ========================================================
    wire [24:0] z_y1_sub = 25'd131072 - sig_z_data;
    wire [23:0] z_proc   = (24'd131072 >= sig_z_data) ? z_y1_sub[23:0] : (Q + 24'd131072 - sig_z_data);
    (* ram_style = "block" *) reg [17:0] z_ram [0:1023];
    always @(posedge clk) begin
        if (sig_z_valid) z_ram[{sig_z_poly_idx[1:0], sig_z_coeff_idx}] <= z_proc[17:0];
    end

    wire       hint_we;
    wire [7:0] hint_addr;
    wire [7:0] hint_wdata;
    Makehint #(
        .OMEGA(OMEGA), .K_PARAM(K_PARAM)
    ) u_makehint (
        .clk(clk), .rst_n(rst_n), .i_start(sig_c_start),
        .i_valid(sig_hint_pre_valid), .i_hint_pre(sig_hint_pre_data),
        .i_poly_idx(sig_hint_pre_poly_idx), .i_coeff_idx(sig_hint_pre_coeff_idx),
        .o_hint_we(hint_we), .o_hint_addr(hint_addr), .o_hint_wdata(hint_wdata),
        .o_fail(), .o_done()
    );

    reg [671:0] hint_buf;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hint_buf <= 0;
        end else begin
            if (sig_c_start) hint_buf <= 0;
            else if (hint_we) hint_buf[hint_addr * 8 +: 8] <= hint_wdata;
        end
    end

    // ========================================================
    // 5. 实例化 sig_packer 直接穿透至流式输出
    // ========================================================
    reg         sig_pack_start;
    wire [3:0]  packer_z_poly;
    wire [7:0]  packer_z_coeff;
    wire [7:0]  packer_hint_addr;
    wire        sig_packer_done; // 内部打包完成状态

    reg [17:0] z_ram_rdata;
    always @(posedge clk) begin
        z_ram_rdata <= z_ram[{packer_z_poly[1:0], packer_z_coeff}];
    end

    sig_packer u_sig_packer (
        .clk(clk), .rst_n(rst_n), .i_start(sig_pack_start),
        .c_tilde(c_tilde_reg),
        .o_z_poly_idx(packer_z_poly), .o_z_coeff_idx(packer_z_coeff), .i_z_data(z_ram_rdata),
        .o_hint_addr(packer_hint_addr), .i_hint_data(hint_buf[packer_hint_addr * 8 +: 8]),
        // 挂载到顶层流式输出
        .o_sig_we(sig_valid), 
        .o_sig_addr(sig_addr), 
        .o_sig_wdata(sig_data), 
        .o_sig_valid(sig_packer_done) // FSM 判定结束使用
    );

    // (原 signature_bram 已被删除)

    // ==========================================
    // 6. 顶层自动重试 FSM 及 Hash 子状态机
    // ==========================================
    localparam ST_IDLE       = 4'd0;
    localparam ST_UNPACK     = 4'd1;
    localparam ST_SIGN_INIT  = 4'd2;
    localparam ST_SIGN_PH1   = 4'd3;
    localparam ST_HASH_C     = 4'd4;
    localparam ST_SIGN_PH2   = 4'd5;
    localparam ST_PACK       = 4'd6;
    localparam ST_DONE       = 4'd7;

    localparam HS_IDLE    = 4'd0;
    localparam HS_U_SEND  = 4'd1;
    localparam HS_U_WAIT  = 4'd2;   
    localparam HS_W1_RD   = 4'd3;
    localparam HS_W1_WAIT = 4'd4;
    localparam HS_W1_PACK = 4'd5;
    localparam HS_W1_SEND = 4'd6;
    localparam HS_W1_ACK  = 4'd7;
    localparam HS_DONE    = 4'd8;

    reg [3:0] state;
    reg [3:0] stream_i, stream_j;
    reg [8:0] stream_m;
    reg       stream_active;

    reg [6:0] hash_u_cnt;
    reg [9:0] hash_w1_cnt; 
    reg [63:0] pack_buf;
    reg [6:0]  pack_bits;
    reg [3:0]  hash_st;
    reg [3:0]  w1_sub_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            done <= 0; sk_start <= 0;
            sig_start <= 0; sig_c_start <= 0;
            sig_rej_round <= 0;
            stream_active <= 0;
            sig_A_valid <= 0;
            sig_pack_start <= 0;
            
            shake_c_start <= 0;
            shake_c_absorb_valid <= 0;
            hash_st <= HS_IDLE;
            w1_raddr <= 0;
        end else begin
            sk_start <= 0;
            sig_start <= 0;
            sig_c_start <= 0;
            sig_pack_start <= 0;

            case (state)
                ST_IDLE: begin
                    done <= 0;
                    if (start) begin sk_start <= 1; state <= ST_UNPACK; end
                end
                
                ST_UNPACK: begin 
                    if (sk_done) begin sig_rej_round <= 0;
                    state <= ST_SIGN_INIT; end
                end
                
                ST_SIGN_INIT: begin 
                    sig_start <= 1;
                    stream_i <= 0; stream_j <= 0;
                    stream_active <= 0;
                    hash_st <= HS_IDLE;
                    state <= ST_SIGN_PH1;
                end
                
                ST_SIGN_PH1: begin 
                    if (sig_ready_for_A && !stream_active) begin
                        stream_active <= 1;
                        stream_m <= 0;
                    end
                    if (stream_active) begin
                        sig_A_valid <= 1;
                        sig_A_j_idx <= stream_j; sig_A_m_idx <= stream_m[7:0];
                        sig_A_data  <= A_ram[{stream_i[1:0], stream_j[1:0], stream_m[7:0]}];
                        if (stream_m == 255) begin
                            stream_active <= 0;
                            if (stream_j == L_PARAM - 1) begin
                                stream_j <= 0;
                                if (stream_i == K_PARAM - 1) stream_i <= 0; else stream_i <= stream_i + 1;
                            end else stream_j <= stream_j + 1;
                        end else stream_m <= stream_m + 1;
                    end else sig_A_valid <= 0;
                    if (sig_w_done) begin state <= ST_HASH_C; end
                end
                
                ST_HASH_C: begin 
                    case (hash_st)
                        HS_IDLE: begin
                            shake_c_start <= 1;
                            hash_u_cnt <= 0; hash_w1_cnt <= 0;
                            pack_buf <= 0; pack_bits <= 0; w1_sub_cnt <= 0;
                            shake_c_absorb_valid <= 0;
                            hash_st <= HS_U_SEND;
                        end
                        
                        HS_U_SEND: begin
                            shake_c_start <= 0;
                            shake_c_absorb_valid <= 1;
                            shake_c_absorb_data <= sk_u >> (hash_u_cnt * 64);
                            shake_c_absorb_bits <= 64;
                            shake_c_absorb_last <= 0;
                            hash_st <= HS_U_WAIT;
                        end
                        
                        HS_U_WAIT: begin
                            if (shake_c_absorb_ready) begin
                                shake_c_absorb_valid <= 0;
                                if (hash_u_cnt == 7) begin
                                    hash_st <= HS_W1_RD;
                                    w1_raddr <= 0;
                                end else begin
                                    hash_u_cnt <= hash_u_cnt + 1;
                                    hash_st <= HS_U_SEND; 
                                end
                            end
                        end
                        
                        HS_W1_RD: begin
                            hash_st <= HS_W1_WAIT;
                        end
                        
                        HS_W1_WAIT: begin
                            hash_st <= HS_W1_PACK;
                        end
                        
                        HS_W1_PACK: begin
                            pack_buf <= pack_buf |
                            ({58'd0, w1_rdata} << pack_bits);
                            pack_bits <= pack_bits + 6;
                            
                            if (w1_sub_cnt == 9 || hash_w1_cnt == 1023) begin
                                hash_st <= HS_W1_SEND;
                            end else begin
                                w1_sub_cnt <= w1_sub_cnt + 1;
                                hash_w1_cnt <= hash_w1_cnt + 1;
                                w1_raddr <= hash_w1_cnt + 1;
                                hash_st <= HS_W1_WAIT;
                            end
                        end
                        
                        HS_W1_SEND: begin
                            shake_c_absorb_valid <= 1;
                            shake_c_absorb_data <= pack_buf;
                            shake_c_absorb_bits <= pack_bits;
                            shake_c_absorb_last <= (hash_w1_cnt == 1023); 
                            hash_st <= HS_W1_ACK;
                        end
                        
                        HS_W1_ACK: begin
                            if (shake_c_absorb_ready) begin
                                shake_c_absorb_valid <= 0;
                                if (hash_w1_cnt == 1023) begin
                                    hash_st <= HS_DONE;
                                end else begin
                                    pack_buf <= 0;
                                    pack_bits <= 0; w1_sub_cnt <= 0;
                                    hash_w1_cnt <= hash_w1_cnt + 1;
                                    w1_raddr <= hash_w1_cnt + 1;
                                    hash_st <= HS_W1_WAIT;
                                end
                            end
                        end
                        
                        HS_DONE: begin
                            if (shake_c_squeeze_valid) begin
                                sig_c_start <= 1;
                                c_tilde_reg <= shake_c_squeeze_data;
                                state <= ST_SIGN_PH2; 
                            end
                        end
                    endcase
                end
                
                ST_SIGN_PH2: begin 
                    if (sig_all_done) begin
                        if (sig_rej_flag) begin
                            sig_rej_round <= sig_rej_round + L_PARAM;
                            state <= ST_SIGN_INIT;
                        end else begin
                            sig_pack_start <= 1;
                            state <= ST_PACK;
                        end
                    end
                end
                
                ST_PACK: begin
                    if (sig_packer_done) state <= ST_DONE;
                end
                
                ST_DONE: begin
                    done <= 1;
                    if (!start) state <= ST_IDLE; 
                end
            endcase
        end
    end
endmodule