`timescale 1ns / 1ps

module Usehint_Core (
    input  wire [31:0] r1,        // 来自 Decompose 的 r1
    input  wire [31:0] r0,        // 来自 Decompose 的 r0 (注意: 在Dilithium中这可能是有符号或无符号数处理，取决于Decompose的实现)
    input  wire        hint_bit,  // 当前元素对应的 hint 位 (0 或 1)
    output reg  [31:0] w1_approx  // 输出的 w1_approx
);

    // 参数定义 (Security Level 2)
    localparam [31:0] M_VAL = 32'd44;
    localparam [31:0] Q_MINUS_1_HALF = 32'd4190208;

    always @(*) begin
        if (hint_bit == 1'b1) begin
            if (r0 > 32'd0 && r0 < Q_MINUS_1_HALF) begin
                // mod(r1 + 1, m)
                w1_approx = (r1 == (M_VAL - 1)) ? 32'd0 : (r1 + 1'b1);
            end 
            else if (r0 == 32'd0 || r0 >= Q_MINUS_1_HALF) begin
                // mod(r1 - 1, m)
                w1_approx = (r1 == 32'd0) ? (M_VAL - 1) : (r1 - 1'b1);
            end
            else begin
                w1_approx = r1;
            end
        end 
        else begin
            // hint == 0 时，直接等于 r1
            w1_approx = r1;
        end
    end

endmodule