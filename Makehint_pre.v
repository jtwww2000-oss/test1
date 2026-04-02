`timescale 1ns / 1ps

module Makehint_pre #(
    parameter WIDTH = 24,
    parameter Q     = 24'd8380417
)(
    input  wire             clk,
    input  wire             rst_n,
    
    input  wire             i_valid,
    input  wire [WIDTH-1:0] i_A,
    input  wire [WIDTH-1:0] i_B,
    
    output reg              o_valid,
    output reg              o_hint_pre
);

    // 1. Z = mod(B + A, Q)
    wire [24:0] z_sum = i_A + i_B;
    wire [23:0] z_mod = (z_sum >= Q) ? (z_sum - Q) : z_sum[23:0];

    // 2. 提取 Highbits
    wire [5:0] hb_B;
    wire [5:0] hb_Z;

    // 例化两个纯组合逻辑的 Highbits 模块分别计算 B 和 Z
    Highbits #(
        .WIDTH(WIDTH),
        .Q(Q)
    ) u_hb_B (
        .i_w(i_B),
        .o_w1(hb_B)
    );

    Highbits #(
        .WIDTH(WIDTH),
        .Q(Q)
    ) u_hb_Z (
        .i_w(z_mod),
        .o_w1(hb_Z)
    );

    // 3. 比较判断并打拍输出 (1 cycle delay)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_valid    <= 1'b0;
            o_hint_pre <= 1'b0;
        end else begin
            o_valid <= i_valid;
            
            if (i_valid) begin
                // 如果高位不相等，则 hint_pre = 1；相等则为 0
                o_hint_pre <= (hb_B != hb_Z) ? 1'b1 : 1'b0;
            end else begin
                o_hint_pre <= 1'b0;
            end
        end
    end

endmodule