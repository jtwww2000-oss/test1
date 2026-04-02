`timescale 1ns / 1ps
module Lowbits #(
    parameter WIDTH = 24,
    parameter Q = 24'd8380417
)(
    input  wire [WIDTH-1:0] i_w,
    output wire [WIDTH-1:0] o_w0
);
    // w0 = w - w1 * alpha (mod Q)
    wire [5:0] w1;
    Highbits #( .WIDTH(WIDTH), .Q(Q) ) u_hb (.i_w(i_w), .o_w1(w1));
    
    wire [23:0] w1_alpha = w1 * 24'd190464;
    assign o_w0 = (i_w >= w1_alpha) ? (i_w - w1_alpha) : (Q + i_w - w1_alpha);
endmodule