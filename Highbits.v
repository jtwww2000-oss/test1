`timescale 1ns / 1ps

module Highbits #(
    parameter WIDTH = 24,
    parameter Q = 24'd8380417
)(
    input  wire [WIDTH-1:0] i_w,
    output wire [5:0]       o_w1
);

    // ★ 修复 1：为了完美匹配 MATLAB 的严格大于 (w0 > 95232)
    // 根据分解定理，偏移量必须是 190464 - 95233 = 95231
   (* use_dsp = "yes" *) wire [24:0] w_plus_gamma2 = i_w + 24'd95231;

    // ★ 修复 2：废弃严重错误的魔法数近似，直接除以常数。
    // 综合器会自动将常数除法优化为无误差的移位乘法逻辑，绝不消耗除法器 IP
    (* use_dsp = "yes" *)wire [5:0] w1_raw = w_plus_gamma2 / 24'd190464;

    // Dilithium 规约：若算出的值为 44，代表跨过了 Q 的边界，须折叠回 0
    assign o_w1 = (w1_raw >= 6'd44) ? 6'd0 : w1_raw;

endmodule