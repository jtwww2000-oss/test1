`timescale 1ns / 1ps

module MatrixVecMul_Core #(
    parameter WIDTH = 24,
    parameter Q = 24'd8380417,
    parameter MU = 26'd33587228 // 2^48 / Q for Barrett
)(
    input  wire          clk,
    input  wire          rst_n,

    // --- 来自 Rejsam_a 的数据流 (矩阵 A) ---
    input  wire          i_A_valid,
    input  wire [WIDTH-1:0] i_A_data,
    input  wire [7:0]    i_m_idx,     
    input  wire [3:0]    i_j_idx,     
    input  wire [3:0]    i_l_param,   

    // --- s1 RAM 读取接口 ---
    output wire [7:0]    o_s1_addr,   
    output wire [3:0]    o_s1_poly_idx, 
    input  wire [WIDTH-1:0] i_s1_rdata,  

    // --- 累加器 RAM 接口 ---
    output wire          o_acc_we,
    output wire [7:0]    o_acc_addr,  // 读地址 (Port A)
    output wire [7:0]    o_acc_waddr, // 写地址 (Port B)
    output wire [WIDTH-1:0] o_acc_wdata,
    input  wire [WIDTH-1:0] i_acc_rdata, // 读取当前累加值

    // --- 最终结果输出 ---
    output reg           o_res_valid,
    output reg  [WIDTH-1:0] o_res_data,
    output reg  [7:0]    o_res_m_idx
);

    // ============================================================
    // 1. 流水线延迟定义
    // ============================================================
    localparam PIPE_DEPTH = 6; 

    // ============================================================
    // 2. 数据通路 - 阶段 1: s1 读取与乘法准备
    // ============================================================
    assign o_s1_poly_idx = i_j_idx; 
    assign o_s1_addr = i_m_idx; 

    reg [WIDTH-1:0] r_A_data_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) r_A_data_d1 <= 0;
        else r_A_data_d1 <= i_A_data;
    end

    // ============================================================
    // 3. 数据通路 - 阶段 2: 模乘 (A * s1 % q)
    // ============================================================
    reg [WIDTH-1:0] mul_op_a, mul_op_b;
    (* use_dsp = "yes" *) reg [2*WIDTH-1:0] prod_reg;
    
    // 【核心修复】：彻底去掉数据通路的异步复位
    // 这样 mul_op_a 和 mul_op_b 会自动变成 DSP48 内部的 AREG 和 BREG
    // prod_reg 会变成 DSP48 内部的 PREG
    always @(posedge clk) begin
        mul_op_a <= r_A_data_d1;
        mul_op_b <= i_s1_rdata; 
        prod_reg <= mul_op_a * mul_op_b;
    end

    wire [WIDTH-1:0] barrett_res;
    // Barrett 约减 (延迟约 3 拍，结果在 Stage 5 有效)
    Barrett_reduce #( .WIDTH(WIDTH) ) u_barrett (
        .clk(clk),
        .prod(prod_reg),
        .q(Q),
        .mu(MU),
        .res(barrett_res)
    );

    // ============================================================
    // 4. 控制信号延迟线
    // ============================================================
    reg [7:0] pipe_m_idx [0:PIPE_DEPTH];
    reg [3:0] pipe_j_idx [0:PIPE_DEPTH];
    reg       pipe_valid [0:PIPE_DEPTH];
    reg [3:0] pipe_l_param [0:PIPE_DEPTH];
    
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for(k=0; k<=PIPE_DEPTH; k=k+1) begin
                pipe_m_idx[k]   <= 8'd0;
                pipe_j_idx[k]   <= 4'd0;
                pipe_valid[k]   <= 1'b0;
                pipe_l_param[k] <= 4'd0;
            end
        end else begin
            pipe_m_idx[0] <= i_m_idx;
            pipe_j_idx[0] <= i_j_idx;
            pipe_valid[0] <= i_A_valid;
            pipe_l_param[0] <= i_l_param;
            
            for(k=0; k<PIPE_DEPTH; k=k+1) begin
                pipe_m_idx[k+1] <= pipe_m_idx[k];
                pipe_j_idx[k+1] <= pipe_j_idx[k];
                pipe_valid[k+1] <= pipe_valid[k];
                pipe_l_param[k+1] <= pipe_l_param[k];
            end
        end
    end

    // ============================================================
    // 5. 数据通路 - 阶段 3: 读取 & 累加 (Stage 5)
    // ============================================================
    
    // [关键修改 1] 读地址提前 1 拍 (使用 Stage 4 的地址)
    // 这样数据会在 Stage 5 (PIPE_DEPTH-1) 准备好
    assign o_acc_addr = pipe_m_idx[PIPE_DEPTH-2]; 
    
    // 在 Stage 5 进行加法计算
    // 此时 i_acc_rdata 是有效的 (对应 pipe_m_idx[PIPE_DEPTH-1])
    // 此时 barrett_res 也是有效的 (假设 Barrett 输出对齐到这里)
    
    // 如果是第一列 (j=0)，忽略 RAM 数据，初始值为 0
    wire [WIDTH-1:0] acc_operand;
    assign acc_operand = (pipe_j_idx[PIPE_DEPTH-1] == 4'd0) ? 24'd0 : i_acc_rdata;

    wire [WIDTH-1:0] add_res;
    mod_add #( .WIDTH(WIDTH) ) u_mod_add (
        .a(barrett_res), // 直接使用 Barrett 结果，不再额外延迟
        .b(acc_operand),
        .q(Q),
        .res(add_res)
    );

    // [关键修改 2] 将加法结果寄存 1 拍
    // 这样写操作发生在 Stage 6，数据来源于 Stage 5 的计算结果
    reg [WIDTH-1:0] r_final_res;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) r_final_res <= 0;
        else r_final_res <= add_res;
    end

    // ============================================================
    // 6. 写回与输出 (Stage 6)
    // ============================================================
    
    // 写使能和地址保持在 Stage 6
    assign o_acc_we = pipe_valid[PIPE_DEPTH];
    assign o_acc_waddr = pipe_m_idx[PIPE_DEPTH]; 
    
    // [关键修改 3] 写数据使用寄存后的结果
    assign o_acc_wdata = r_final_res;

    // 最终输出逻辑
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            o_res_valid <= 0;
            o_res_data <= 0;
            o_res_m_idx <= 0;
        end else begin
            if (pipe_valid[PIPE_DEPTH] && (pipe_j_idx[PIPE_DEPTH] == pipe_l_param[PIPE_DEPTH] - 1)) begin
                o_res_valid <= 1'b1;
                o_res_data  <= r_final_res; // 输出也要用寄存后的值
                o_res_m_idx <= pipe_m_idx[PIPE_DEPTH];
            end else begin
                o_res_valid <= 1'b0;
            end
        end
    end

endmodule