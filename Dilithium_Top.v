`timescale 1ns / 1ps

module Dilithium_Top (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire [255:0]  seed,
    input  wire [255:0]  message,
    
    output reg           done,
    output reg           verify_success,

    // ===== 新增：用于外部截获数据的输出端口 =====
    output wire          kg_pk_valid,
    output wire [8:0]    kg_pk_addr,
    output wire [31:0]   kg_pk_data,
    
    output wire          kg_sk_valid, 
    output wire [9:0]    kg_sk_addr,
    output wire [31:0]   kg_sk_data,
    
    output wire          sign_sig_valid, 
    output wire [9:0]    sign_sig_addr, 
    output wire [31:0]   sign_sig_data
);

    (* ram_style = "block" *) reg [31:0] sk_ram [0:1023]; 
    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) sk_ram[i] = 32'd0;
    end

    localparam S_IDLE       = 3'd0;
    localparam S_KEYGEN_RUN = 3'd1;
    localparam S_SIGN_RUN   = 3'd2;
    localparam S_WAIT_FLUSH = 3'd3; // ★ 新增：等待 BRAM 落盘状态
    localparam S_VERIFY_RUN = 3'd4;
    localparam S_DONE       = 3'd5;
    
    reg [2:0] state;
    reg [5:0] wait_cnt;             // ★ 新增：等待计数器
    
    reg         kg_start;
    wire        kg_done;
    
    reg         sign_start;
    wire        sign_done;
    wire [9:0]  sign_sk_addr;
    reg  [31:0] sign_sk_data;
    
    reg         ver_start;   
    wire        ver_done;
    wire        ver_success;

    // 物理隔绝：仅在 KeyGen 态允许写 SK
    always @(posedge clk) begin
        if ((kg_sk_valid == 1'b1) && (state == S_KEYGEN_RUN)) 
            sk_ram[kg_sk_addr] <= kg_sk_data;
    end

    always @(posedge clk) begin
        sign_sk_data <= sk_ram[sign_sk_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0; verify_success <= 0;
            kg_start <= 0; sign_start <= 0; ver_start <= 0;
            wait_cnt <= 0; // ★ 复位计数器
        end else begin
            kg_start <= 0; sign_start <= 0; ver_start <= 0;
            case (state)
                S_IDLE: begin
                    done <= 0; 
                    wait_cnt <= 0;
                    if (start) begin 
                    verify_success <= 0;
                    kg_start <= 1; 
                    state <= S_KEYGEN_RUN; 
                    end
                end
                
                S_KEYGEN_RUN: begin
                    // 这里 KeyGen 结束后也可以等，但通常 Sign 比较皮实，可以直连
                    if (kg_done) begin sign_start <= 1; state <= S_SIGN_RUN; end
                end
                
                S_SIGN_RUN: begin
                    // ★ 修复：Sign 结束后，绝对不能立刻拉高 ver_start，进入等待状态！
                    if (sign_done) begin 
                        wait_cnt <= 0;
                        state <= S_WAIT_FLUSH; 
                    end
                end
                
                // ★ 新增的等待落盘状态
                S_WAIT_FLUSH: begin
                    // 等待 32 个时钟周期，确保最后一口数据写进 RAM，且总线稳定
                    if (wait_cnt < 6'd32) begin
                        wait_cnt <= wait_cnt + 1;
                    end else begin
                        ver_start <= 1;        // 32 拍后，安稳地启动验签
                        state <= S_VERIFY_RUN;
                    end
                end
                
                S_VERIFY_RUN: begin
                    if (ver_done) begin
                        verify_success <= ver_success;
                        done <= 1; 
                        state <= S_DONE;
                    end
                end
                
                S_DONE: begin
                    if (!start) state <= S_IDLE;
                end
            endcase
        end
    end

    KeyGen u_keygen (
        .clk(clk), .rst_n(rst_n), .start(kg_start), .seed(seed), .done(kg_done), .busy(), 
        .pk_valid(kg_pk_valid), .pk_addr(kg_pk_addr), .pk(kg_pk_data),
        .sk_valid(kg_sk_valid), .sk_addr(kg_sk_addr), .sk(kg_sk_data)
    );

    dilithium_sign_core u_sign (
        .clk(clk), .rst_n(rst_n), .start(sign_start), .M(message),
        .sk_addr(sign_sk_addr), .sk(sign_sk_data), .done(sign_done),
        .sig_valid(sign_sig_valid), .sig_addr(sign_sig_addr), .sig_data(sign_sig_data)
    );

    dilithium_verify_wrapper u_verify_wrapper (
        .clk              (clk), 
        .rst_n            (rst_n),
        .i_start          (ver_start), 
        .i_M              (message),
        .i_pk_en          (state == S_KEYGEN_RUN),
        .i_sig_en         (state == S_SIGN_RUN),
        .i_pk_valid       (kg_pk_valid), 
        .i_pk_addr        (kg_pk_addr), 
        .i_pk_data        (kg_pk_data),
        .i_sig_valid      (sign_sig_valid), 
        .i_sig_addr       (sign_sig_addr), 
        .i_sig_data       (sign_sig_data),
        .o_done           (ver_done), 
        .o_verify_success (ver_success)
    );
endmodule