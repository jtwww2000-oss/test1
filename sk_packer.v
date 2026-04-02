`timescale 1ns / 1ps

module sk_packer (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          i_start,
    
    input  wire [255:0]  rho,
    input  wire [255:0]  key_K,
    input  wire [511:0]  tr,
    
    output reg  [3:0]    o_s_ram_sel, 
    output reg  [7:0]    o_s_ram_addr,  
    input  wire [23:0]   i_s_ram_data,  
    
    output reg  [3:0]    o_t0_ram_sel,  
    output reg  [7:0]    o_t0_ram_addr, 
    input  wire [12:0]   i_t0_ram_data, 
    
    output reg           o_sk_we,
    output reg  [9:0]    o_sk_addr,
    output reg  [31:0]   o_sk_wdata,
    output reg           o_sk_valid
);
    localparam S_IDLE = 4'd0, S_RHO = 4'd1, S_K = 4'd2, S_TR = 4'd3;
    // 【修改点 1】：新增一个 S_PACK_S_WAIT2 (用 4'd11) 状态
    localparam S_PACK_S_REQ = 4'd4, S_PACK_S_WAIT = 4'd5, S_PACK_S_WAIT2 = 4'd11, S_PACK_S_CONSUME = 4'd6;
    localparam S_PACK_T_REQ = 4'd7, S_PACK_T_WAIT = 4'd8, S_PACK_T_CONSUME = 4'd9;
    localparam S_FLUSH = 4'd10;

    reg [3:0] state;
    reg [63:0] buffer;
    reg [6:0]  bit_count;
    reg [4:0]  word_cnt;
    reg [3:0]  poly_cnt;
    reg [8:0]  coeff_cnt;
    
    reg [2:0] recovered_s;
    
    // 【修改点 2】：直接原样提取低3位！因为 i_s_ram_data 已经是纯净的 0,1,2,3,4 了
//    always @(*) begin
//        if (i_s_ram_data <= 24'd2) 
//            recovered_s = 3'd2 - i_s_ram_data[2:0];
//        else 
//            recovered_s = (24'd8380419 - i_s_ram_data); 
//    end
    always @(*) begin
        recovered_s = i_s_ram_data[2:0]; 
    end
    

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_sk_we <= 0; o_sk_addr <= 10'h3FF; o_sk_wdata <= 0; o_sk_valid <= 0;
            buffer <= 0; bit_count <= 0;
            o_s_ram_sel <= 0; o_s_ram_addr <= 0; o_t0_ram_sel <= 0; o_t0_ram_addr <= 0;
        end else begin
            o_sk_we <= 0; o_sk_valid <= 0;
            case (state)
                S_IDLE: begin
                    if (i_start) begin
                        state <= S_RHO;
                        word_cnt <= 0; o_sk_addr <= 10'h3FF;
                        buffer <= 0; bit_count <= 0;
                    end
                end
                
                S_RHO: begin
                    o_sk_we <= 1;
                    o_sk_addr <= o_sk_addr + 1; o_sk_wdata <= rho[word_cnt * 32 +: 32];
                    if (word_cnt == 7) begin state <= S_K;
                        word_cnt <= 0; end else word_cnt <= word_cnt + 1;
                end
                
                S_K: begin
                    o_sk_we <= 1;
                    o_sk_addr <= o_sk_addr + 1; o_sk_wdata <= key_K[word_cnt * 32 +: 32];
                    if (word_cnt == 7) begin state <= S_TR;
                        word_cnt <= 0; end else word_cnt <= word_cnt + 1;
                end
                
                S_TR: begin
                    o_sk_we <= 1;
                    o_sk_addr <= o_sk_addr + 1; o_sk_wdata <= tr[word_cnt * 32 +: 32];
                    if (word_cnt == 15) begin 
                        state <= S_PACK_S_REQ;
                        poly_cnt <= 0; coeff_cnt <= 0; 
                    end else word_cnt <= word_cnt + 1;
                end
                
                S_PACK_S_REQ: begin
                    o_s_ram_sel <= poly_cnt;
                    o_s_ram_addr <= coeff_cnt; 
                    state <= S_PACK_S_WAIT;
                end
                S_PACK_S_WAIT: begin 
                    // 【修改点 3】：进入第二拍等待
                    state <= S_PACK_S_WAIT2; 
                end
                S_PACK_S_WAIT2: begin 
                    // 第二拍等待结束，数据此时绝对稳定
                    state <= S_PACK_S_CONSUME; 
                end
                
                S_PACK_S_CONSUME: begin
                    if (bit_count >= 32) begin
                        o_sk_we <= 1; o_sk_addr <= o_sk_addr + 1; o_sk_wdata <= buffer[31:0];
                        buffer <= (buffer >> 32) | ({61'd0, recovered_s} << (bit_count - 32));
                        bit_count <= bit_count - 32 + 3;
                    end else begin
                        buffer <= buffer |
                        ({61'd0, recovered_s} << bit_count);
                        bit_count <= bit_count + 3;
                    end
                    
                    if (coeff_cnt == 255) begin
                        coeff_cnt <= 0;
                        if (poly_cnt == 7) begin 
                            state <= S_PACK_T_REQ;
                            poly_cnt <= 0;
                        end else begin
                            poly_cnt <= poly_cnt + 1;
                            state <= S_PACK_S_REQ;
                        end
                    end else begin
                        coeff_cnt <= coeff_cnt + 1;
                        state <= S_PACK_S_REQ;
                    end
                end
                
                S_PACK_T_REQ: begin
                    o_t0_ram_sel <= poly_cnt;
                    o_t0_ram_addr <= coeff_cnt; state <= S_PACK_T_WAIT;
                end
                S_PACK_T_WAIT: begin state <= S_PACK_T_CONSUME;
                end
                
                S_PACK_T_CONSUME: begin
                    if (bit_count >= 32) begin
                        o_sk_we <= 1;
                        o_sk_addr <= o_sk_addr + 1; o_sk_wdata <= buffer[31:0];
                        buffer <= (buffer >> 32) | ({51'd0, i_t0_ram_data} << (bit_count - 32));
                        bit_count <= bit_count - 32 + 13;
                    end else begin
                        buffer <= buffer |
                        ({51'd0, i_t0_ram_data} << bit_count);
                        bit_count <= bit_count + 13;
                    end
                    
                    if (coeff_cnt == 255) begin
                        coeff_cnt <= 0;
                        if (poly_cnt == 3) begin
                            state <= S_FLUSH;
                        end else begin
                            poly_cnt <= poly_cnt + 1;
                            state <= S_PACK_T_REQ;
                        end
                    end else begin
                        coeff_cnt <= coeff_cnt + 1;
                        state <= S_PACK_T_REQ;
                    end
                end
                
                S_FLUSH: begin
                    if (bit_count > 0) begin
                        
                        o_sk_we <= 1; o_sk_addr <= o_sk_addr + 1; o_sk_wdata <= buffer[31:0];
                        buffer <= buffer >> 32;
                        if (bit_count > 32) bit_count <= bit_count - 32;
                        else bit_count <= 0;
                    end else begin
                        o_sk_valid <= 1;
                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end
endmodule
 