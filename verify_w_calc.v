`timescale 1ns / 1ps

module verify_w_calc #(
    parameter WIDTH   = 24,
    parameter Q       = 24'd8380417,
    parameter K_PARAM = 4'd4,
    parameter L_PARAM = 4'd4
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          i_start,
    
    // 外部 BRAM 接口 (1拍延迟)
    output wire [11:0]   o_A_raddr,    input  wire [23:0]   i_A_rdata,
    output wire [7:0]    o_c_raddr,    input  wire [23:0]   i_c_rdata,
    output wire [9:0]    o_t1_raddr,   input  wire [23:0]   i_t1_rdata,
    output wire [9:0]    o_z_raddr,    input  wire [23:0]   i_z_rdata,
    output wire [9:0]    o_hint_raddr, input  wire          i_hint_rdata,

    output wire          o_w1_we,
    output wire [9:0]    o_w1_waddr,
    output wire [5:0]    o_w1_wdata,
    
    output reg           o_done
);

    localparam ST_IDLE       = 4'd0, ST_CT1_MUL    = 4'd1, ST_AZ_MAC     = 4'd2, ST_INTT_START = 4'd3, ST_INTT_WAIT  = 4'd4, ST_USE_HINT   = 4'd5, ST_NEXT_ROW   = 4'd6, ST_DONE       = 4'd7;
    reg [3:0] state; reg [3:0] i_cnt, j_cnt; reg [8:0] m_cnt;

    // ==========================================
    // INTT RAM (Ping-Pong 架构适配)
    // ==========================================
    reg intt_start; wire intt_done;
    
    // Core 端口宣告
    wire [7:0]  core_ram0_addr_a, core_ram0_addr_b, core_ram1_addr_a, core_ram1_addr_b;
    wire        core_ram0_we_a,   core_ram0_we_b,   core_ram1_we_a,   core_ram1_we_b;
    wire [23:0] core_ram0_wdata_a,core_ram0_wdata_b,core_ram1_wdata_a,core_ram1_wdata_b;
    
    // RAM 读出数据宣告
    wire [23:0] ram0_rdata_a, ram0_rdata_b, ram1_rdata_a, ram1_rdata_b;

    wire user_mode = (state != ST_INTT_START && state != ST_INTT_WAIT);
    wire user_we_a; wire [7:0] user_addr_a, user_addr_b; wire [23:0] user_din_a;

    // --- RAM0 MUX 逻辑 ---
    wire [7:0]  ram0_addr_a  = user_mode ? user_addr_a : core_ram0_addr_a;
    wire        ram0_we_a    = user_mode ? user_we_a   : core_ram0_we_a;
    wire [23:0] ram0_wdata_a = user_mode ? user_din_a  : core_ram0_wdata_a;

    wire [7:0]  ram0_addr_b  = user_mode ? user_addr_b : core_ram0_addr_b;
    wire        ram0_we_b    = user_mode ? 1'b0        : core_ram0_we_b;
    wire [23:0] ram0_wdata_b = user_mode ? 24'd0       : core_ram0_wdata_b;

    tdpram_24x256 u_ram0 (
        .clk(clk), 
        .we_a(ram0_we_a), .addr_a(ram0_addr_a), .din_a(ram0_wdata_a), .dout_a(ram0_rdata_a),
        .we_b(ram0_we_b), .addr_b(ram0_addr_b), .din_b(ram0_wdata_b), .dout_b(ram0_rdata_b)
    );

    // --- RAM1 MUX 逻辑 (INTT 独占) ---
    wire [7:0]  ram1_addr_a  = user_mode ? 8'd0  : core_ram1_addr_a;
    wire        ram1_we_a    = user_mode ? 1'b0  : core_ram1_we_a;
    wire [23:0] ram1_wdata_a = user_mode ? 24'd0 : core_ram1_wdata_a;

    wire [7:0]  ram1_addr_b  = user_mode ? 8'd0  : core_ram1_addr_b;
    wire        ram1_we_b    = user_mode ? 1'b0  : core_ram1_we_b;
    wire [23:0] ram1_wdata_b = user_mode ? 24'd0 : core_ram1_wdata_b;

    tdpram_24x256 u_ram1 (
        .clk(clk), 
        .we_a(ram1_we_a), .addr_a(ram1_addr_a), .din_a(ram1_wdata_a), .dout_a(ram1_rdata_a),
        .we_b(ram1_we_b), .addr_b(ram1_addr_b), .din_b(ram1_wdata_b), .dout_b(ram1_rdata_b)
    );

    // INTT Core 实例化
    intt_core #( .WIDTH(WIDTH) ) u_intt (
        .clk(clk), .rst_n(rst_n), .start(intt_start), .done(intt_done),
        .ram0_addr_a(core_ram0_addr_a), .ram0_we_a(core_ram0_we_a), .ram0_wdata_a(core_ram0_wdata_a), .ram0_rdata_a(ram0_rdata_a),
        .ram0_addr_b(core_ram0_addr_b), .ram0_we_b(core_ram0_we_b), .ram0_wdata_b(core_ram0_wdata_b), .ram0_rdata_b(ram0_rdata_b),
        .ram1_addr_a(core_ram1_addr_a), .ram1_we_a(core_ram1_we_a), .ram1_wdata_a(core_ram1_wdata_a), .ram1_rdata_a(ram1_rdata_a),
        .ram1_addr_b(core_ram1_addr_b), .ram1_we_b(core_ram1_we_b), .ram1_wdata_b(core_ram1_wdata_b), .ram1_rdata_b(ram1_rdata_b)
    );

    // ==========================================
    // 核心：完美对齐的 6 拍 MAC 流水线
    // ==========================================
    wire v_in = (state == ST_CT1_MUL || state == ST_AZ_MAC) && (m_cnt < 256);
    
    assign o_c_raddr  = m_cnt[7:0]; 
    assign o_t1_raddr = (i_cnt * 256) + m_cnt;
    assign o_z_raddr  = (j_cnt * 256) + m_cnt;
    assign o_A_raddr  = ((i_cnt * L_PARAM + j_cnt) * 256) + m_cnt; 

    reg [5:0] v_p;       // valid
    // ... 前面的代码 ...
    reg [5:0] ct1_p;     // ct1 select
    reg [7:0] ad_p [0:5]; // address

    // 修复点：在这里声明 k
    integer k; 

    always @(posedge clk) begin
        v_p[0] <= v_in; v_p[5:1] <= v_p[4:0];
        ct1_p[0] <= (state == ST_CT1_MUL); ct1_p[5:1] <= ct1_p[4:0];
        
        ad_p[0] <= m_cnt[7:0]; 
        // 修复点：移除循环内部的 integer 关键字
        for (k=1; k<6; k=k+1) begin
            ad_p[k] <= ad_p[k-1];
        end
    end
    // ... 后面的代码 ...

    // T2: 数据抓取 (1拍延迟的最佳捕获点)
    reg [23:0] ma, mb;
    always @(posedge clk) begin
        if (v_p[0]) begin 
            ma <= ct1_p[0] ? i_c_rdata  : i_A_rdata;
            mb <= ct1_p[0] ? i_t1_rdata : i_z_rdata;
        end
    end

    // T3: 乘法器
    reg [47:0] prod;
    always @(posedge clk) prod <= ma * mb;

    // T4-T6: Barrett 3 拍延迟
    wire [23:0] barrett_res;
    Barrett_reduce #( .WIDTH(WIDTH) ) u_barrett (
        .clk(clk), .prod(prod), .q(Q), .mu(26'd33587228), .res(barrett_res)
    );

    // T6 时读取累加器旧值 (在 T5 给出地址)
    assign user_addr_b = (state == ST_USE_HINT) ? m_cnt[7:0] : ad_p[4];

    // T7: 组合逻辑累加 & 写入 INTT_BRAM (此时读取自 ram0_rdata_b)
    wire [23:0] n_ct1 = (barrett_res == 0) ? 0 : (Q - barrett_res);
    wire [24:0] sum   = ram0_rdata_b + barrett_res;
    wire [23:0] m_sum = (sum >= Q) ? (sum - Q) : sum[23:0];

    assign user_we_a   = v_p[5];
    assign user_addr_a = ad_p[5];
    // 这里极度精妙：算第一行 c*t1 时，强制覆写 RAM！自动清除了上一行的累加残留！
    assign user_din_a  = ct1_p[5] ? n_ct1 : m_sum;

    wire mac_idle = (v_p == 0) && (v_in == 0);

    // ==========================================
    // Usehint 流水线 (极致精简)
    // ==========================================
    assign o_hint_raddr = (i_cnt * 256) + m_cnt;
    wire uv_in = (state == ST_USE_HINT) && (m_cnt < 256);
    
    // ★ 修复点：将移位寄存器深度增加到 3 级 (uv[2:0] 和 ua[0:2]) ★
    reg [2:0] uv; 
    reg [7:0] ua [0:2];
    
    // ★ 修复点：为 BRAM 读出数据增加一级流水线寄存器 ★
    reg [23:0] ram0_rdata_b_reg;
    reg        i_hint_rdata_reg;

    always @(posedge clk) begin
        uv[0] <= uv_in; uv[1] <= uv[0]; uv[2] <= uv[1];
        ua[0] <= m_cnt[7:0]; ua[1] <= ua[0]; ua[2] <= ua[1];
        
        // 斩断关键路径：打一拍
        ram0_rdata_b_reg <= ram0_rdata_b;
        i_hint_rdata_reg <= i_hint_rdata;
    end

    wire [5:0] r1_c; wire [23:0] r0_c;
    Highbits h_c (.i_w(ram0_rdata_b_reg), .o_w1(r1_c)); // 接到寄存后的数据
    Lowbits  l_c (.i_w(ram0_rdata_b_reg), .o_w0(r0_c));

    reg [5:0] r1; reg [23:0] r0; reg rh;
    // 延迟 1 拍后，使用 uv[1] 来触发采样
    always @(posedge clk) if(uv[1]) begin r1 <= r1_c; r0 <= r0_c; rh <= i_hint_rdata_reg; end

    wire [31:0] w_e;
    Usehint_Core uh_c (.r1({26'd0,r1}), .r0({{8{r0[23]}},r0}), .hint_bit(rh), .w1_approx(w_e));

    // 使用 uv[2] 触发输出写回，对应流水线最后一级
    assign o_w1_we    = uv[2];
    assign o_w1_waddr = (i_cnt * 256) + ua[2];
    assign o_w1_wdata = w_e[5:0];

    // ==========================================
    // FSM
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin 
            state <= ST_IDLE; i_cnt <= 0; j_cnt <= 0; m_cnt <= 0; o_done <= 0; intt_start <= 0; 
        end
        else begin
            intt_start <= 0;
            case (state)
                ST_IDLE: begin o_done <= 0; i_cnt <= 0; j_cnt <= 0; m_cnt <= 0; if (i_start) state <= ST_CT1_MUL; end
                ST_CT1_MUL: if (m_cnt < 256) m_cnt <= m_cnt + 1; else if (mac_idle) begin m_cnt <= 0; j_cnt <= 0; state <= ST_AZ_MAC; end
                ST_AZ_MAC: if (m_cnt < 256) m_cnt <= m_cnt + 1;
                           else if (mac_idle) begin 
                               if (j_cnt < L_PARAM - 1) begin j_cnt <= j_cnt + 1; m_cnt <= 0; end 
                               else begin
                                   // $display 探针已被移除，防止 BRAM 跨模块索引报错
                                   state <= ST_INTT_START; 
                               end
                           end
                ST_INTT_START: begin intt_start <= 1; state <= ST_INTT_WAIT; end
                ST_INTT_WAIT: if (intt_done) begin 
                                m_cnt <= 0; state <= ST_USE_HINT; 
                             end
                ST_USE_HINT: if (m_cnt < 256) m_cnt <= m_cnt + 1; else if (uv == 0) state <= ST_NEXT_ROW;
                ST_NEXT_ROW: if (i_cnt < K_PARAM - 1) begin 
                                i_cnt <= i_cnt + 1; m_cnt <= 0; 
                                state <= ST_CT1_MUL; 
                             end else state <= ST_DONE;
                ST_DONE: begin o_done <= 1; if (!i_start) state <= ST_IDLE; end
            endcase
        end
    end
endmodule
