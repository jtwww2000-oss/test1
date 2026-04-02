`timescale 1ns / 1ps

module Power2Round #(
    parameter WIDTH = 24
)(
    input  wire             clk,
    input  wire             rst_n,
    
    // --- 控制信号 ---
    input  wire             i_valid, // 输入数据有效标志
    output reg              o_valid, // 输出数据有效标志
    
    // --- 数据接口 ---
    input  wire [WIDTH-1:0] i_data,  // 输入元素 t
    output reg  [9:0]       o_t1,    // 输出 t1 (修改为 10 bits)
    output reg  [12:0]      o_t0     // 输出 t0 (修改为 13 bits)
);
    // --- 常量定义 ---
    // T0_CUTOFF = 2^12 = 4096
    localparam [12:0] T0_CUTOFF = 13'd4096;
    // Case 1 常数: 4096 - (-8192) = 12288
    localparam [13:0] CONST_CASE1 = 14'd12288;
    // Case 2 常数: 4096
    localparam [12:0] CONST_CASE2 = 13'd4096;

    // --- 内部信号 ---
    wire [12:0] t0_raw;
    wire [WIDTH-1:0] t1_raw;

    // 1. 计算 t mod 2^13 (直接截取低 13 位)
    assign t0_raw = i_data[12:0];
    // 2. 计算 floor(t / 2^13) (直接右移 13 位)
    assign t1_raw = i_data >> 13;

    // --- 主逻辑 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_valid <= 1'b0;
            o_t1    <= 10'd0;
            o_t0    <= 13'd0;
        end else begin
            o_valid <= i_valid;
            if (i_valid) begin
                // MATLAB 逻辑映射:
                // if (t0_raw > 2^12)
                //    t0 = 2^12 - (t0_raw - 2^13) = 12288 - t0_raw
                //    t1 = t1_raw + 1
                // else
                //    t0 = 2^12 - t0_raw = 4096 - t0_raw
                //    t1 = t1_raw
                
                if (t0_raw > T0_CUTOFF) begin
                    // Case 1: t0_raw > 4096
                    // t1 增加 1。虽然 t1_raw 是 24 位宽，但最大值仅为 1023，加 1 后为 1024 (需11位)
                    // 但在 Dilithium 模数下，t1 最大只到 1023，这里直接截断赋值给 10 bits 即可。
                    o_t1 <= t1_raw[9:0] + 1'b1;
                    
                    // t0 计算结果最大为 8191，适合 13 bits
                    o_t0 <= CONST_CASE1 - t0_raw;
                end else begin
                    // Case 2: t0_raw <= 4096
                    o_t1 <= t1_raw[9:0];
                    o_t0 <= CONST_CASE2 - t0_raw;
                end
            end
        end
    end

endmodule