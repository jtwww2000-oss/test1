module tdpram_24x256 (
    input wire clk,
    
    input wire we_a,
    input wire [7:0] addr_a,
    input wire [23:0] din_a,
    output reg [23:0] dout_a,
    
    input wire we_b,
    input wire [7:0] addr_b,
    input wire [23:0] din_b,
    output reg [23:0] dout_b
);

    reg [23:0] ram [0:255];
// --- [新增] 仿真初始化 ---
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            ram[i] = 24'd0;
        end
    end
    // Port A
    always @(posedge clk) begin
        if (we_a)
            ram[addr_a] <= din_a;
        dout_a <= ram[addr_a];
    end

    // Port B
    always @(posedge clk) begin
        if (we_b)
            ram[addr_b] <= din_b;
        dout_b <= ram[addr_b];
    end

endmodule