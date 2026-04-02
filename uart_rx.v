`timescale 1ns / 1ps

module uart_rx #(
    parameter CLK_FREQ = 100_000_000, // 假设 50MHz 时钟
    parameter BAUD_RATE = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    
    reg [2:0] r_rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) r_rx_sync <= 3'b111;
        else r_rx_sync <= {r_rx_sync[1:0], rx};
    end
    wire rx_in = r_rx_sync[2];

    reg [2:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 0;
            clk_cnt <= 0;
            bit_idx <= 0;
            rx_valid <= 0;
            rx_data <= 0;
        end else begin
            rx_valid <= 0;
            case (state)
                0: begin // 等待起始位
                    if (rx_in == 1'b0) begin
                        state <= 1;
                        clk_cnt <= CLKS_PER_BIT / 2; // 定位到数据位中间
                    end
                end
                1: begin // 检查起始位
                    if (clk_cnt == 0) begin
                        if (rx_in == 1'b0) begin
                            state <= 2;
                            clk_cnt <= CLKS_PER_BIT - 1;
                            bit_idx <= 0;
                        end else state <= 0;
                    end else clk_cnt <= clk_cnt - 1;
                end
                2: begin // 接收数据位
                    if (clk_cnt == 0) begin
                        rx_data[bit_idx] <= rx_in;
                        clk_cnt <= CLKS_PER_BIT - 1;
                        if (bit_idx < 7) bit_idx <= bit_idx + 1;
                        else begin
                            state <= 3;
                            bit_idx <= 0;
                        end
                    end else clk_cnt <= clk_cnt - 1;
                end
                3: begin // 停止位
                    if (clk_cnt == 0) begin
                        rx_valid <= 1;
                        state <= 0;
                    end else clk_cnt <= clk_cnt - 1;
                end
            endcase
        end
    end
endmodule