`timescale 1ns / 1ps

module uart_tx #(
    parameter CLK_FREQ = 100_000_000,
    parameter BAUD_RATE = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx,
    output reg        tx_done
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    
    reg [2:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  tx_data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 0;
            tx <= 1;
            tx_done <= 0;
            clk_cnt <= 0;
            bit_idx <= 0;
            tx_data_reg <= 0;
        end else begin
            tx_done <= 0;
            case (state)
                0: begin // Idle
                    tx <= 1;
                    if (tx_start) begin
                        tx_data_reg <= tx_data;
                        state <= 1;
                        clk_cnt <= CLKS_PER_BIT - 1;
                    end
                end
                1: begin // Start bit
                    tx <= 0;
                    if (clk_cnt == 0) begin
                        state <= 2;
                        clk_cnt <= CLKS_PER_BIT - 1;
                    end else clk_cnt <= clk_cnt - 1;
                end
                2: begin // Data bits
                    tx <= tx_data_reg[bit_idx];
                    if (clk_cnt == 0) begin
                        clk_cnt <= CLKS_PER_BIT - 1;
                        if (bit_idx < 7) bit_idx <= bit_idx + 1;
                        else begin
                            bit_idx <= 0;
                            state <= 3;
                        end
                    end else clk_cnt <= clk_cnt - 1;
                end
                3: begin // Stop bit
                    tx <= 1;
                    if (clk_cnt == 0) begin
                        tx_done <= 1;
                        state <= 0;
                    end else clk_cnt <= clk_cnt - 1;
                end
            endcase
        end
    end
endmodule