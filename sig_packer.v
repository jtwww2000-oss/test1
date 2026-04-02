`timescale 1ns / 1ps

module sig_packer (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          i_start,
    
    // 输入1：Hash 产生的 c_tilde
    input  wire [255:0]  c_tilde,
    
    // 输入2：Z 数组读取接口 (1024 * 18-bit)
    output reg  [3:0]    o_z_poly_idx,
    output reg  [7:0]    o_z_coeff_idx,
    input  wire [17:0]   i_z_data,
    
    // 输入3：Hint 数组读取接口 (84 * 8-bit)
    output reg  [7:0]    o_hint_addr,
    input  wire [7:0]    i_hint_data,
    
    // 输出：对接顶层 BRAM
    output reg           o_sig_we,
    output reg  [9:0]    o_sig_addr,
    output reg  [31:0]   o_sig_wdata,
    output reg           o_sig_valid
);

    localparam S_IDLE         = 4'd0;
    localparam S_CTILDE       = 4'd1;
    localparam S_Z_REQ        = 4'd2;
    localparam S_Z_WAIT       = 4'd3;
    localparam S_Z_CONSUME    = 4'd4;
    localparam S_HINT_REQ     = 4'd5;
    localparam S_HINT_WAIT    = 4'd6;
    localparam S_HINT_CONSUME = 4'd7;
    localparam S_FLUSH        = 4'd8;

    reg [3:0]  state;
    reg [63:0] buffer;     // 64位缓冲器，防止拼接时溢出
    reg [6:0]  bit_count;
    reg [4:0]  word_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_sig_we <= 0; o_sig_addr <= 10'h3FF; o_sig_wdata <= 0; o_sig_valid <= 0;
            buffer <= 0; bit_count <= 0; word_cnt <= 0;
            o_z_poly_idx <= 0; o_z_coeff_idx <= 0; o_hint_addr <= 0;
        end else begin
            o_sig_we <= 0; o_sig_valid <= 0;

            case (state)
                S_IDLE: begin
                    if (i_start) begin
                        state <= S_CTILDE;
                        word_cnt <= 0; o_sig_addr <= 10'h3FF;
                        buffer <= 0; bit_count <= 0;
                    end
                end
                
                // 1. 组装 c_tilde (256 bits = 8 个 32-bit word)
                S_CTILDE: begin
                    o_sig_we <= 1;
                    o_sig_addr <= o_sig_addr + 1;
                    o_sig_wdata <= c_tilde[word_cnt * 32 +: 32];
                    if (word_cnt == 7) begin
                        state <= S_Z_REQ;
                        o_z_poly_idx <= 0; o_z_coeff_idx <= 0;
                    end else begin
                        word_cnt <= word_cnt + 1;
                    end
                end
                
                // 2. 组装 Z (每个 18-bit)
                S_Z_REQ:   state <= S_Z_WAIT;
                S_Z_WAIT:  state <= S_Z_CONSUME; // 等待BRAM读出
                S_Z_CONSUME: begin
                    if (bit_count >= 32) begin
                        o_sig_we <= 1; o_sig_addr <= o_sig_addr + 1; o_sig_wdata <= buffer[31:0];
                        // 发送满32位后，将剩余位右移，同时拼入新的 18-bit z数据
                        buffer <= (buffer >> 32) | ({46'd0, i_z_data} << (bit_count - 32));
                        bit_count <= bit_count - 32 + 18;
                    end else begin
                        buffer <= buffer | ({46'd0, i_z_data} << bit_count);
                        bit_count <= bit_count + 18;
                    end
                    
                    if (o_z_coeff_idx == 255) begin
                        o_z_coeff_idx <= 0;
                        if (o_z_poly_idx == 3) begin // Sec Level 2: k=4
                            state <= S_HINT_REQ;
                            o_hint_addr <= 0;
                        end else begin
                            o_z_poly_idx <= o_z_poly_idx + 1;
                            state <= S_Z_REQ;
                        end
                    end else begin
                        o_z_coeff_idx <= o_z_coeff_idx + 1;
                        state <= S_Z_REQ;
                    end
                end
                
                // 3. 组装 Hint (共 84 个 8-bit)
                S_HINT_REQ:   state <= S_HINT_WAIT;
                S_HINT_WAIT:  state <= S_HINT_CONSUME;
                S_HINT_CONSUME: begin
                    if (bit_count >= 32) begin
                        o_sig_we <= 1; o_sig_addr <= o_sig_addr + 1; o_sig_wdata <= buffer[31:0];
                        buffer <= (buffer >> 32) | ({56'd0, i_hint_data} << (bit_count - 32));
                        bit_count <= bit_count - 32 + 8;
                    end else begin
                        buffer <= buffer | ({56'd0, i_hint_data} << bit_count);
                        bit_count <= bit_count + 8;
                    end
                    
                    if (o_hint_addr == 83) begin // K(4) + Omega(80) - 1 = 83
                        state <= S_FLUSH;
                    end else begin
                        o_hint_addr <= o_hint_addr + 1;
                        state <= S_HINT_REQ;
                    end
                end
                
                // 4. 清空残余缓冲
                S_FLUSH: begin
                    if (bit_count > 0) begin
                        o_sig_we <= 1; o_sig_addr <= o_sig_addr + 1; o_sig_wdata <= buffer[31:0];
                        buffer <= buffer >> 32;
                        if (bit_count > 32) bit_count <= bit_count - 32;
                        else bit_count <= 0;
                    end else begin
                        o_sig_valid <= 1;
                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end
endmodule