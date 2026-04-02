`timescale 1ns / 1ps

module Montgomery_mul #(
    parameter WIDTH = 24
)(
    input  wire             clk,
    input  wire [WIDTH-1:0] a,
    input  wire [WIDTH-1:0] b,
    input  wire [WIDTH-1:0] q,       
    input  wire [WIDTH-1:0] q_prime, 
    
    output reg  [WIDTH-1:0] res
);

    // ========================================================
    // Stage 0: 输入寄存器层 (Input Pipelining)
    // ========================================================
    reg [WIDTH-1:0] r0_a, r0_b, r0_q, r0_q_prime;

    always @(posedge clk) begin
        r0_a       <= a;
        r0_b       <= b;
        r0_q       <= q;
        r0_q_prime <= q_prime;
    end

    // ========================================================
    // Stage 1: 计算 prod = A * B
    // ========================================================
    (* use_dsp = "yes" *) reg [2*WIDTH-1:0] r1_prod;
    reg [WIDTH-1:0] r1_q, r1_q_prime;
    always @(posedge clk) begin
        r1_prod    <= r0_a * r0_b; 
        r1_q       <= r0_q;
        r1_q_prime <= r0_q_prime;
    end

    // ========================================================
    // Stage 2: 计算 m = (prod * Q') mod R
    // ========================================================
    (* use_dsp = "yes" *) reg [WIDTH-1:0] r2_m;
    reg [2*WIDTH-1:0] r2_prod;
    reg [WIDTH-1:0]   r2_q;
    always @(posedge clk) begin
        r2_m    <= r1_prod[WIDTH-1:0] * r1_q_prime;
        r2_prod <= r1_prod;
        r2_q    <= r1_q;
    end

    // ========================================================
    // Stage 3: 计算 mq = m * Q
    // ========================================================
    (* use_dsp = "yes" *) reg [2*WIDTH-1:0] r3_mq;
    reg [2*WIDTH-1:0] r3_prod;
    reg [WIDTH-1:0]   r3_q;

    always @(posedge clk) begin
        r3_mq   <= r2_m * r2_q;
        r3_prod <= r2_prod;
        r3_q    <= r2_q;
    end

    // ========================================================
    // Stage 4: 计算 48-bit 大加法并打拍 (切断关键路径)
    // ========================================================
    reg [2*WIDTH:0] r4_sum;
    reg [WIDTH-1:0] r4_q;

    always @(posedge clk) begin
        r4_sum <= r3_prod + r3_mq; // 将加法器隔离在此周期
        r4_q   <= r3_q;
    end

    // ========================================================
    // Stage 5: 比较、减法与归约
    // ========================================================
    wire [WIDTH:0]   t_comb;
    wire [WIDTH-1:0] res_comb;

    assign t_comb   = r4_sum[2*WIDTH : WIDTH]; // 除以 R (右移)
    // 比较与减法隔离在此周期
    assign res_comb = (t_comb >= r4_q) ? (t_comb - r4_q) : t_comb[WIDTH-1:0];

    always @(posedge clk) begin
        res <= res_comb;
    end

endmodule