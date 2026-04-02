`timescale 1ns / 1ps

module mod_sub #(
    parameter WIDTH = 24
)(
    input  wire [WIDTH-1:0] a,
    input  wire [WIDTH-1:0] b,
    input  wire [WIDTH-1:0] q,
    output wire [WIDTH-1:0] res
);

    // 扩展 1 位位宽以容纳减法的借位/符号位 (共 25 bits)
    wire [WIDTH:0] diff;
    wire [WIDTH:0] diff_add_q;
    
    // 1. 并行计算两路可能的结果
    assign diff = {1'b0, a} - {1'b0, b};          // 推测 1: a >= b，直接相减
    assign diff_add_q = diff + {1'b0, q};         // 推测 2: a < b，结果为负数，需要加上 Q 恢复到正规域
    
    // 2. 符号位作为极速选择器 (MUX)
    // 如果 diff 的最高位(位24)为 1，代表 a < b 产生了借位，应该采用推测2 (加上 Q)
    // 如果 diff 的最高位(位24)为 0，代表 a >= b 减法结果为正，应该采用推测1 (直接取 diff)
    assign res = diff[WIDTH] ? diff_add_q[WIDTH-1:0] : diff[WIDTH-1:0];

endmodule