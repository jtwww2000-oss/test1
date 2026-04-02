`timescale 1ns / 1ps

module Rejsam_y (
    input  wire                 clk,
    input  wire                 rst_n,
    
    // --- 控制与数据输入接口 ---
    input  wire                 i_start,        // 启动脉冲
    input  wire [511:0]         i_rho_prime,    // rho' (512 bits = 64 bytes)
    input  wire [15:0]          i_row,          // row index (16 bits)
    
    // --- 输出接口 (流式输出 256 个系数) ---
    output reg                  o_coeff_valid,  // 系数有效标志
    output reg  [17:0]          o_coeff_data,   // 生成的系数 (安全等级2为 18 bits)
    output reg                  o_done          // 256个系数生成完毕信号
);

    // --- 参数定义 ---
    localparam CNT_TARGET = 9'd256;
    localparam RATE_BITS  = 1088; // SHAKE256 Rate
    localparam RATE_BYTES = 136;  // 1088 / 8
    localparam ABSORB_LEN = 512 + 16; // rho'(512) + row(16) = 528 bits

    // --- 状态机 ---
    localparam S_IDLE        = 3'd0;
    localparam S_START_SHAKE = 3'd1;
    localparam S_REQ_SQUEEZE = 3'd2;
    localparam S_WAIT_ACK    = 3'd3;
    localparam S_WAIT_DATA   = 3'd4;
    localparam S_PROCESS     = 3'd5;
    localparam S_DONE        = 3'd6;

    reg [2:0] state;

    // --- 内部控制与拼接寄存器 ---
    reg [8:0]    poly_cnt;      // 已生成的系数计数 (0-256)
    reg [1087:0] data_buffer;   // 锁存 SHAKE 吐出的单块 1088 bits 数据
    reg [10:0]   bit_ptr;       // 当前读取指针 (0 ~ 1088)
    
    // 跨块拼接专用的暂存寄存器
    reg [17:0]   leftover_bits; // 存放上一个块剩余不足 18 bits 的数据
    reg [4:0]    leftover_len;  // 剩余的 bit 数量 (0 ~ 17)

    // --- SHAKE256 实例化 ---
    reg  shake_start;
    wire [ABSORB_LEN-1:0] shake_seed;
    reg  shake_squeeze_req;
    wire shake_squeeze_valid;
    wire [RATE_BITS-1:0] shake_out_data;

    // 组合 Seed，由于 Keccak 是 LSB-first 吸收，
    // 将 i_rho_prime 放在低位，i_row 放在高位，完美对应 MATLAB 的 [rho, row_matrix]
    assign shake_seed = {i_row, i_rho_prime}; 

    SHAKE256 #(
        .OUTPUT_LEN_BYTES(RATE_BYTES), 
        .ABSORB_LEN(ABSORB_LEN)        
    ) u_shake256 (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (shake_start),
        .i_seed          (shake_seed),
        .o_busy          (),
        .i_squeeze_req   (shake_squeeze_req),
        .o_squeeze_valid (shake_squeeze_valid),
        .o_squeeze_data  (shake_out_data)
    );

    // --- 核心动态提取与拼接逻辑 (组合逻辑) ---
    wire [10:0] bits_rem = 11'd1088 - bit_ptr;
    wire [1087:0] shifted_data = data_buffer >> bit_ptr;
    wire [17:0] next_bits = shifted_data[17:0]; // 提取窗口内数据

    wire [4:0]  bits_needed = 5'd18 - leftover_len;
    wire [18:0] mask_wide = (19'd1 << bits_needed) - 1'b1;
    wire [17:0] actual_new_bits = next_bits & mask_wide[17:0];

    wire [18:0] rem_mask_wide = (19'd1 << bits_rem) - 1'b1;
    wire [17:0] clean_next_bits = next_bits & rem_mask_wide[17:0];

    // --- 主状态机 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_coeff_valid <= 1'b0;
            o_coeff_data  <= 18'd0;
            o_done <= 1'b0;
            shake_start <= 1'b0;
            shake_squeeze_req <= 1'b0;
            poly_cnt <= 9'd0;
            bit_ptr <= 11'd0;
            leftover_bits <= 18'd0;
            leftover_len <= 5'd0;
        end else begin
            shake_start <= 1'b0;
            o_coeff_valid <= 1'b0; 

            case (state)
                S_IDLE: begin
                    o_done <= 1'b0;
                    poly_cnt <= 9'd0;
                    leftover_bits <= 18'd0;
                    leftover_len <= 5'd0;
                    if (i_start) begin
                        state <= S_START_SHAKE;
                    end
                end

                S_START_SHAKE: begin
                    shake_start <= 1'b1;
                    state <= S_REQ_SQUEEZE;
                end

                S_REQ_SQUEEZE: begin
                    shake_squeeze_req <= 1'b1;
                    state <= S_WAIT_ACK;
                end
                
                S_WAIT_ACK: begin
                    shake_squeeze_req <= 1'b0;
                    if (shake_squeeze_valid == 1'b0) begin
                        state <= S_WAIT_DATA;
                    end
                end
                
                S_WAIT_DATA: begin
                    if (shake_squeeze_valid) begin
                        data_buffer <= shake_out_data;
                        bit_ptr <= 11'd0; 
                        state <= S_PROCESS;
                    end
                end

                S_PROCESS: begin
                    // 如果 256 个系数全部生成完，直接结束
                    if (poly_cnt == CNT_TARGET) begin
                        state <= S_DONE;
                    end 
                    // 当前块剩余的 bit 加上上个块的 leftover，足够拼出 18 bits
                    else if (bits_rem + leftover_len >= 18) begin
                        o_coeff_valid <= 1'b1;
                        // 将新获取的 bit 左移，填补在 leftover_bits 的上方，完成无缝拼接
                        o_coeff_data  <= (actual_new_bits << leftover_len) | leftover_bits;
                        poly_cnt      <= poly_cnt + 1;
                        
                        bit_ptr       <= bit_ptr + bits_needed; // 推进指针
                        leftover_len  <= 5'd0;                  // 拼接完成，清零残留
                        leftover_bits <= 18'd0;
                    end 
                    // 块内数据不足，全部存入 leftover，请求下一次 Squeeze
                    else begin
                        leftover_bits <= (clean_next_bits << leftover_len) | leftover_bits;
                        leftover_len  <= leftover_len + bits_rem[4:0];
                        state         <= S_REQ_SQUEEZE;
                    end
                end

                S_DONE: begin
                    o_done <= 1'b1;
                    if (!i_start) begin
                        state <= S_IDLE; // 等待下一次启动
                    end
                end
            endcase
        end
    end

endmodule