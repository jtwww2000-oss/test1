`timescale 1ns / 1ps

module SHAKE256_stream #(
    parameter RATE = 1088,
    parameter STATE_WIDTH = 1600,
    parameter OUTPUT_LEN_BYTES = 32
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // 控制接口
    input  wire                 i_start,  
    output reg                  o_busy,   

    // 流式输入接口 (Stream Absorb)
    input  wire                 i_absorb_valid,
    input  wire [63:0]          i_absorb_data,
    input  wire [6:0]           i_absorb_bits, // 本次输入的有效位数 (1~64)
    input  wire                 i_absorb_last, // 是否是最后一块数据
    output reg                  o_absorb_ready,

    // 挤出接口 (Squeeze)
    input  wire                 i_squeeze_req,
    output reg                  o_squeeze_valid,
    output wire [OUTPUT_LEN_BYTES*8-1:0] o_squeeze_data
);

    localparam S_IDLE            = 3'd0;
    localparam S_ABSORB          = 3'd1;
    localparam S_WAIT_KECCAK     = 3'd2;
    localparam S_HANDLE_LEFTOVER = 3'd3;
    localparam S_DO_PAD          = 3'd4;
    localparam S_PAD_EXTRA       = 3'd5;
    localparam S_SQUEEZE         = 3'd6;
    localparam S_CLEAR_WAIT      = 3'd7; // ★ 新增：安全缓冲状态

    reg [2:0] state, next_state_ret;
    reg [STATE_WIDTH-1:0] r_state;
    reg [RATE-1:0]        rate_buf;
    reg [10:0]            rate_bits;
    
    // 用于处理跨块边界残留的数据
    reg [6:0]  leftover_bits;
    reg [63:0] leftover_data;
    reg        leftover_last;

    reg keccak_start;
    reg [STATE_WIDTH-1:0] keccak_in;
    wire keccak_done;
    wire [STATE_WIDTH-1:0] keccak_out;

    Keccak_f #( .STATE_WIDTH(STATE_WIDTH) ) u_keccak (
        .clk(clk), .rst_n(rst_n), .i_start(keccak_start), .i_data(keccak_in),
        .o_valid(keccak_done), .o_data(keccak_out), .o_busy()
    );

    wire [RATE-1:0] pad_in_N = (state == S_PAD_EXTRA) ? {RATE{1'b0}} : rate_buf;
    wire [10:0]     pad_in_m = (state == S_PAD_EXTRA) ? 11'd0 : rate_bits;
    wire [RATE-1:0] pad_out;
    
    pad #( .X(RATE), .MAX_LEN(RATE) ) u_pad (
        .N(pad_in_N), .m(pad_in_m), .P(pad_out)
    );
    
    wire need_extra_pad = (rate_bits > (RATE - 5));

    wire [RATE-1:0] data_padded = { {(RATE-64){1'b0}}, i_absorb_data };
    wire [11:0]     rate_bits_next = rate_bits + i_absorb_bits;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_busy <= 0;
            o_absorb_ready <= 0;
            o_squeeze_valid <= 0;
            keccak_start <= 0;
            keccak_in <= 0;
            rate_buf <= 0;
            rate_bits <= 0;
            leftover_bits <= 0;
            leftover_data <= 0;
            leftover_last <= 0;
        end 
        // ★ 核心修复：无论在什么状态，收到 start 强制打断并重启
        else if (i_start) begin  
            state <= S_CLEAR_WAIT; // ★ 转移至安全缓冲状态
            r_state <= 0;
            rate_buf <= 0;
            rate_bits <= 0;
            o_busy <= 1'b1;
            o_absorb_ready <= 1'b0; // 缓冲期间不接收数据
            o_squeeze_valid <= 0;
            leftover_bits <= 0;
            leftover_data <= 0;
            leftover_last <= 0;
            
            // ★ 双重保险：向下级 Keccak 发送假启动，强行用 0 覆盖掉其残留状态
            keccak_start <= 1'b1;
            keccak_in <= {STATE_WIDTH{1'b0}};
        end 
        else begin
            keccak_start <= 0; // 默认拉低，形成脉冲
            case (state)
                S_IDLE: begin
                    o_squeeze_valid <= 0;
                    o_busy <= 1'b0;
                    o_absorb_ready <= 1'b0;
                end
                
                // ★ 新增：等待底层强行覆盖生效，一拍之后满血复活进入接收状态
                S_CLEAR_WAIT: begin
                    state <= S_ABSORB;
                    o_absorb_ready <= 1'b1;
                end

                S_ABSORB: begin
                    if (i_absorb_valid && o_absorb_ready) begin
                        if (i_absorb_last && rate_bits_next <= RATE) begin
                            // 如果是最后一块并且未越界，填充结束转Padding
                            rate_buf <= rate_buf | (data_padded << rate_bits);
                            rate_bits <= rate_bits_next;
                            o_absorb_ready <= 1'b0;
                            state <= S_DO_PAD;
                        end else if (rate_bits_next >= RATE) begin
                            // 当前块已满，触发 Keccak
                            o_absorb_ready <= 1'b0;
                            keccak_in[RATE-1:0] <= r_state[RATE-1:0] ^ (rate_buf | (data_padded << rate_bits));
                            keccak_in[STATE_WIDTH-1:RATE] <= r_state[STATE_WIDTH-1:RATE];
                            keccak_start <= 1'b1;
                            
                            // 保存溢出到下一个 block 的部分
                            leftover_bits <= rate_bits_next - RATE;
                            leftover_data <= i_absorb_data >> (RATE - rate_bits);
                            leftover_last <= i_absorb_last;
                            
                            next_state_ret <= S_HANDLE_LEFTOVER;
                            state <= S_WAIT_KECCAK;
                        end else begin
                            // 正常累积吸收数据
                            rate_buf <= rate_buf | (data_padded << rate_bits);
                            rate_bits <= rate_bits_next;
                        end
                    end
                end

                S_HANDLE_LEFTOVER: begin
                    // 处理上一块遗留的数据
                    rate_buf <= { {(RATE-64){1'b0}}, leftover_data };
                    rate_bits <= leftover_bits;
                    if (leftover_last) begin
                        state <= S_DO_PAD;
                    end else begin
                        o_absorb_ready <= 1'b1;
                        state <= S_ABSORB;
                    end
                end

                S_DO_PAD: begin
                    keccak_in[RATE-1:0] <= r_state[RATE-1:0] ^ pad_out;
                    keccak_in[STATE_WIDTH-1:RATE] <= r_state[STATE_WIDTH-1:RATE];
                    keccak_start <= 1'b1;
                    if (need_extra_pad) begin
                        next_state_ret <= S_PAD_EXTRA;
                    end else begin
                        next_state_ret <= S_SQUEEZE;
                    end
                    state <= S_WAIT_KECCAK;
                end

                S_PAD_EXTRA: begin
                    keccak_in[RATE-1:0] <= r_state[RATE-1:0] ^ pad_out; // 作为空块 N=0, m=0 填充
                    keccak_in[STATE_WIDTH-1:RATE] <= r_state[STATE_WIDTH-1:RATE];
                    keccak_start <= 1'b1;
                    next_state_ret <= S_SQUEEZE;
                    state <= S_WAIT_KECCAK;
                end

                S_WAIT_KECCAK: begin
                    if (keccak_done) begin
                        r_state <= keccak_out;
                        state <= next_state_ret;
                        if (next_state_ret == S_ABSORB) o_absorb_ready <= 1'b1;
                    end
                end

                S_SQUEEZE: begin
                    o_busy <= 1'b1;
                    o_squeeze_valid <= 1'b1;
                    if (i_squeeze_req) begin
                        o_squeeze_valid <= 1'b0;
                        keccak_in <= r_state;
                        keccak_start <= 1'b1;
                        next_state_ret <= S_SQUEEZE;
                        state <= S_WAIT_KECCAK;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    assign o_squeeze_data = r_state[OUTPUT_LEN_BYTES*8-1:0];

endmodule