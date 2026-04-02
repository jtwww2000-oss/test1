`timescale 1ns / 1ps

module dilithium_verify_wrapper (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          i_start,
    input  wire [255:0]  i_M,

    input  wire          i_pk_en,   
    input  wire          i_sig_en,  

    input  wire          i_pk_valid,
    input  wire [8:0]    i_pk_addr,
    input  wire [31:0]   i_pk_data,

    input  wire          i_sig_valid,
    input  wire [9:0]    i_sig_addr,
    input  wire [31:0]   i_sig_data,

    output wire          o_done,
    output wire          o_verify_success
);

    // ★ 修复 1：强行恢复 Block RAM 约束，消除海量 LUT 造成的时序灾难
    (* ram_style = "block" *) reg [31:0] pk_ram  [0:511];
    (* ram_style = "block" *) reg [31:0] sig_ram [0:1023];

    integer i;
    initial begin
        for(i=0; i<512; i=i+1)  pk_ram[i]  = 32'd0;
        for(i=0; i<1024; i=i+1) sig_ram[i] = 32'd0;
    end

    // ★ 修复 2：保留这个逻辑修复，确保数据能真正写入 RAM
    always @(posedge clk) begin
        if (i_pk_valid)  pk_ram[i_pk_addr] <= i_pk_data;
        if (i_sig_valid) sig_ram[i_sig_addr] <= i_sig_data;
    end

    wire [9:0]  orig_pk_raddr;
    wire [11:0] orig_sig_raddr;

    wire [8:0] addr_pk_safe  = (^orig_pk_raddr === 1'bx)  ? 9'd0 : orig_pk_raddr[8:0];
    wire [9:0] addr_sig_safe = (^orig_sig_raddr === 1'bx) ? 10'd0 : orig_sig_raddr[9:0];

    // ★ 修复 3：恢复 1 拍延迟的同步读取，完美映射到内部 BRAM 硬核
    reg [31:0] i_pk_rdata_to_module;
    reg [31:0] i_sig_rdata_to_module;

    always @(posedge clk) begin
        i_pk_rdata_to_module  <= pk_ram[addr_pk_safe];
        i_sig_rdata_to_module <= sig_ram[addr_sig_safe];
    end

    dilithium_verify_top #(
        .WIDTH(24), .Q(24'd8380417), .K_PARAM(4), .L_PARAM(4), 
        .TAU(39), .LAMBDA(128), .W1_WIDTH(6)
    ) u_original_top (
        .clk               (clk), 
        .rst_n             (rst_n), 
        .i_start           (i_start), 
        .i_M               (i_M),
        
        .o_pk_raddr        (orig_pk_raddr),   
        .i_pk_rdata        (i_pk_rdata_to_module), 
        .o_sig_raddr       (orig_sig_raddr), 
        .i_sig_rdata       (i_sig_rdata_to_module),
        
        .o_done            (o_done), 
        .o_verify_success  (o_verify_success)
    );
endmodule