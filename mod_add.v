`timescale 1ns / 1ps

module mod_add #(
    parameter WIDTH = 24
)(
    input  wire [WIDTH-1:0] a,
    input  wire [WIDTH-1:0] b,
    input  wire [WIDTH-1:0] q,
    output wire [WIDTH-1:0] res
);

    // 扩展 1 位位宽以容纳加法的进位和减法的符号位 (共 25 bits)
    wire [WIDTH:0] sum;
    wire [WIDTH:0] sum_sub_q;
    
    // 1. 并行计算两路可能的结果
    assign sum = {1'b0, a} + {1'b0, b};           // 推测 1: 结果不需要取模
    assign sum_sub_q = sum - {1'b0, q};           // 推测 2: 结果溢出，需要减去 Q
    
    // 2. 符号位作为极速选择器 (MUX)
    // 如果 sum_sub_q 的最高位(位24)为 1，代表 sum < q，减法产生了借位(负数)，应该采用推测1
    // 如果 sum_sub_q 的最高位(位24)为 0，代表 sum >= q，减法结果为正，应该采用推测2
    assign res = sum_sub_q[WIDTH] ? sum[WIDTH-1:0] : sum_sub_q[WIDTH-1:0];

endmodule