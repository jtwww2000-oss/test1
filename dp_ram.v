`timescale 1ns / 1ps

module dp_ram #(
    parameter ADDR_WIDTH = 10,
    parameter DATA_WIDTH = 32
)(
    input  wire                  clk,
    // 端口 A：写入（连接密码核心）
    input  wire                  wea,
    input  wire [ADDR_WIDTH-1:0] addra,
    input  wire [DATA_WIDTH-1:0] dina,
    // 端口 B：读取（连接 UART 发送端）
    input  wire [ADDR_WIDTH-1:0] addrb,
    output reg  [DATA_WIDTH-1:0] doutb
);

    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

    // 写操作
    always @(posedge clk) begin
        if (wea) begin
            ram[addra] <= dina;
        end
    end

    // 读操作
    always @(posedge clk) begin
        doutb <= ram[addrb];
    end

endmodule