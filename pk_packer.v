`timescale 1ns / 1ps

module pk_packer (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start_pack, 
    
    input  wire [255:0]  rho,
    input  wire          t1_valid,
    input  wire [9:0]    t1_data,
    
    output reg           o_pk_we,
    output reg  [8:0]    o_pk_addr,
    output reg  [31:0]   o_pk_wdata,
    output reg           o_pk_valid    
);

    localparam S_IDLE = 2'd0, S_RHO = 2'd1, S_T1 = 2'd2, S_DONE = 2'd3;
    reg [1:0]  state;
    
    reg [63:0] buffer;
    reg [6:0]  bit_count;
    reg [3:0]  rho_word_cnt;
    reg [10:0] t1_cnt; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_pk_we <= 0; o_pk_addr <= 9'h1FF; o_pk_wdata <= 0; o_pk_valid <= 0;
            buffer <= 0; bit_count <= 0; rho_word_cnt <= 0; t1_cnt <= 0;
        end else begin
            o_pk_we <= 0; o_pk_valid <= 0;
            
            case (state)
                S_IDLE: begin
                    if (start_pack) begin
                        state <= S_RHO; rho_word_cnt <= 0; o_pk_addr <= 9'h1FF; 
                        buffer <= 0; bit_count <= 0; t1_cnt <= 0;
                    end
                end
                
                S_RHO: begin
                    o_pk_we <= 1; o_pk_addr <= o_pk_addr + 1; o_pk_wdata <= rho[rho_word_cnt * 32 +: 32];
                    if (rho_word_cnt == 7) state <= S_T1; else rho_word_cnt <= rho_word_cnt + 1;
                end
                
                S_T1: begin
                    if (bit_count >= 32) begin
                        o_pk_we <= 1; o_pk_addr <= o_pk_addr + 1; o_pk_wdata <= buffer[31:0];
                        if (t1_valid) begin
                            buffer <= (buffer >> 32) | ({54'd0, t1_data} << (bit_count - 32));
                            bit_count <= bit_count - 32 + 10;
                            t1_cnt <= t1_cnt + 1;
                        end else begin
                            buffer <= buffer >> 32; bit_count <= bit_count - 32;
                        end
                    end else begin
                        if (t1_valid) begin
                            buffer <= buffer | ({54'd0, t1_data} << bit_count);
                            bit_count <= bit_count + 10;
                            t1_cnt <= t1_cnt + 1;
                        end else if (t1_cnt >= 1024 && bit_count == 0) begin
                            state <= S_DONE;
                        end
                    end
                end
                
                S_DONE: begin
                    o_pk_valid <= 1; state <= S_IDLE;
                end
            endcase
        end
    end
endmodule