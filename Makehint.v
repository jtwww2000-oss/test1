`timescale 1ns / 1ps

module Makehint #(
    parameter OMEGA   = 80, // Security level 2: omega = 80
    parameter K_PARAM = 4   // Security level 2: k = 4
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       i_start,      
    input  wire       i_valid,
    input  wire       i_hint_pre,   
    input  wire [3:0] i_poly_idx,   
    input  wire [7:0] i_coeff_idx,  

    // 统一恢复为单写端口，彻底避免顶层并发写入冲突
    output reg        o_hint_we,
    output reg  [7:0] o_hint_addr,  
    output reg  [7:0] o_hint_wdata, 
    
    output reg        o_fail,       
    output reg        o_done        
);

    reg [7:0] omega_cnt;
    
    // 缓冲队列：用于存放需要向后延期写入的 K 统计值
    reg       k_queue_valid;
    reg [7:0] k_queue_addr;
    reg [7:0] k_queue_data;
    reg       k_queue_is_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            omega_cnt       <= 8'd0;
            k_queue_valid   <= 1'b0;
            k_queue_addr    <= 8'd0;
            k_queue_data    <= 8'd0;
            k_queue_is_last <= 1'b0;
            
            o_hint_we       <= 1'b0;
            o_hint_addr     <= 8'd0;
            o_hint_wdata    <= 8'd0;
            o_fail          <= 1'b0;
            o_done          <= 1'b0;
        end else begin
            o_hint_we <= 1'b0;
            o_done    <= 1'b0;

            if (i_start) begin
                omega_cnt     <= 8'd0;
                o_fail        <= 1'b0;
                k_queue_valid <= 1'b0;
            end 
            else begin
                // ===============================================
                // 通路 1 (最高优先级)：绝不漏掉任何输入的 1
                // 对应 MATLAB 的 if(hint_pre(i,j) == 1) 逻辑
                // ===============================================
                if (i_valid && i_hint_pre) begin
                    if (omega_cnt < OMEGA) begin
                        o_hint_we    <= 1'b1;
                        o_hint_addr  <= omega_cnt;       // 对应 hint_omega_flag
                        o_hint_wdata <= i_coeff_idx;     // 对应 j-1
                    end else begin
                        o_fail <= 1'b1;
                    end
                    omega_cnt <= omega_cnt + 1;
                end
                
                // ===============================================
                // 通路 2 (见缝插针)：当没有任何 1 输入时，立刻把 K 值写进去
                // 对应 MATLAB 的 hint(hint_k_flag...) 逻辑
                // ===============================================
                else if (k_queue_valid) begin
                    o_hint_we     <= 1'b1;
                    o_hint_addr   <= k_queue_addr;
                    o_hint_wdata  <= k_queue_data;
                    k_queue_valid <= 1'b0; // 写入完成，清空队列
                    if (k_queue_is_last) begin
                        o_done <= 1'b1;
                    end
                end

                // ===============================================
                // 通路 3：当到达多项式结尾时，计算总数并压入缓冲队列
                // ===============================================
                if (i_valid && i_coeff_idx == 8'd255) begin
                    k_queue_valid   <= 1'b1;
                    k_queue_addr    <= OMEGA + i_poly_idx; // 对应 hint_k_flag
                    
                    // 核心细节：如果这最后一位刚好是 1，要把即将 +1 的总数写进去
                    k_queue_data    <= i_hint_pre ? (omega_cnt + 1) : omega_cnt;
                    k_queue_is_last <= (i_poly_idx == K_PARAM - 1);
                end
            end
        end
    end
endmodule