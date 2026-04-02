
`timescale 1ns / 1ps

module Dilithium_System_Top (
    input  wire sys_clk_p, 
    input  wire sys_clk_n,     
    input  wire rst_n,    
    input  wire rx,       
    output wire tx
//    output reg  led        
);

    // ==========================================
    // 0. 例化时钟管理单元 (Clocking Wizard)
    // ==========================================
    wire clk;   // 内部 50MHz 核心时钟
    wire locked;    // PLL 锁定信号，高电平表示时钟已稳定
    
    // 注意：Xilinx Clocking Wizard IP 的 reset 默认是高电平有效
    // 所以我们将低电平有效的 ext_rst_n 取反后接入
//    clk_wiz_0 u_clk_wiz (
//        .clk_in1  (sys_clk),
//        .reset    (~ext_rst_n), 
//        .clk_out1 (clk),
//        .locked   (locked)
//    );
      clk_wiz_0 u_clk_wiz
   (
    // Clock out ports
    .clk_out1(clk),     // output clk_out1
    // Status and control signals
    .reset(~rst_n), // input reset
    .locked(locked),       // output locked
   // Clock in ports
    .clk_in1_p(sys_clk_p),    // input clk_in1_p
    .clk_in1_n(sys_clk_n));    // input clk_in1_n

    // ==========================================
    // 1. UART 接收 64 Bytes (Seed + Message)
    // ==========================================
    wire [7:0] rx_data;
    wire       rx_valid;
    reg [255:0] seed_reg;
    reg [255:0] msg_reg;
    reg [6:0]   rx_byte_cnt;
    reg         dili_start;

    uart_rx u_uart_rx (
        .clk(clk), 
        .rst_n(rst_n), 
        .rx(rx), 
        .rx_data(rx_data), 
        .rx_valid(rx_valid)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_byte_cnt <= 7'd0;
            dili_start  <= 1'b0;
            seed_reg    <= 256'd0;
            msg_reg     <= 256'd0;
        end else begin
            dili_start <= 1'b0;
            if (rx_valid) begin
                if (rx_byte_cnt < 7'd32) begin
                    seed_reg <= {seed_reg[247:0], rx_data};
                    rx_byte_cnt <= rx_byte_cnt + 7'd1;
                end else if (rx_byte_cnt < 7'd64) begin
                    msg_reg  <= {msg_reg[247:0], rx_data};
                    rx_byte_cnt <= rx_byte_cnt + 7'd1;
                end
                
                if (rx_byte_cnt == 7'd63) begin
                    dili_start <= 1'b1; 
                    rx_byte_cnt <= 7'd0;
                end
            end
        end
    end

    // ==========================================
    // 2. 例化 Dilithium 核心并引出截获信号
    // ==========================================
    wire core_done;
    wire verify_success;
    wire kg_pk_valid;   wire [8:0] kg_pk_addr;   wire [31:0] kg_pk_data;
    wire kg_sk_valid;   wire [9:0] kg_sk_addr;   wire [31:0] kg_sk_data;
    wire sign_sig_valid;wire [9:0] sign_sig_addr;wire [31:0] sign_sig_data;

    Dilithium_Top u_core (
        .clk(clk), 
        .rst_n(rst_n),
        .start(dili_start), 
        .seed(seed_reg), 
        .message(msg_reg),
        .done(core_done), 
        .verify_success(verify_success),
        
        .kg_pk_valid(kg_pk_valid), .kg_pk_addr(kg_pk_addr), .kg_pk_data(kg_pk_data),
        .kg_sk_valid(kg_sk_valid), .kg_sk_addr(kg_sk_addr), .kg_sk_data(kg_sk_data),
        .sign_sig_valid(sign_sig_valid), .sign_sig_addr(sign_sig_addr), .sign_sig_data(sign_sig_data)
    );

    // ==========================================
    // 3. 数据缓存 RAM
    // ==========================================
    wire [8:0] tx_pk_read_addr;  wire [31:0] tx_pk_read_data;
    wire [9:0] tx_sk_read_addr;  wire [31:0] tx_sk_read_data;
    wire [9:0] tx_sig_read_addr; wire [31:0] tx_sig_read_data;

    dp_ram #(.ADDR_WIDTH(9), .DATA_WIDTH(32)) u_pk_ram (
        .clk(clk), .wea(kg_pk_valid), .addra(kg_pk_addr), .dina(kg_pk_data), 
        .addrb(tx_pk_read_addr), .doutb(tx_pk_read_data)
    );
    dp_ram #(.ADDR_WIDTH(10), .DATA_WIDTH(32)) u_sk_ram (
        .clk(clk), .wea(kg_sk_valid), .addra(kg_sk_addr), .dina(kg_sk_data), 
        .addrb(tx_sk_read_addr), .doutb(tx_sk_read_data)
    );
    dp_ram #(.ADDR_WIDTH(10), .DATA_WIDTH(32)) u_sig_ram (
        .clk(clk), .wea(sign_sig_valid), .addra(sign_sig_addr), .dina(sign_sig_data), 
        .addrb(tx_sig_read_addr), .doutb(tx_sig_read_data)
    );

    // ==========================================
    // 4. 字符串打印 ROM 字典 (严格 Verilog 标准)
    // ==========================================
    reg [7:0] msg_rom [0:127];
    integer i; // 遵循严格的 Verilog 语法，在 initial 块外部声明循环变量

    initial begin
        for(i = 0; i < 128; i = i + 1) begin
            msg_rom[i] = 8'h00; // 0 作为字符串结束符
        end
        
        // 字典 0: "Start KeyGen...\r\n" (开始密钥生成) | 地址: 0
        msg_rom[0]="S"; msg_rom[1]="t"; msg_rom[2]="a"; msg_rom[3]="r"; msg_rom[4]="t"; msg_rom[5]=" "; 
        msg_rom[6]="K"; msg_rom[7]="e"; msg_rom[8]="y"; msg_rom[9]="G"; msg_rom[10]="e"; msg_rom[11]="n"; 
        msg_rom[12]="."; msg_rom[13]="."; msg_rom[14]="\r"; msg_rom[15]="\n"; msg_rom[16]=8'h00;

        // 字典 1: "KeyGen Success! pk: " (密钥生成成功！pk：) | 地址: 20
        msg_rom[20]="K"; msg_rom[21]="e"; msg_rom[22]="y"; msg_rom[23]="G"; msg_rom[24]="e"; msg_rom[25]="n";
        msg_rom[26]=" "; msg_rom[27]="S"; msg_rom[28]="u"; msg_rom[29]="c"; msg_rom[30]="c"; msg_rom[31]="e"; 
        msg_rom[32]="s"; msg_rom[33]="s"; msg_rom[34]="!"; msg_rom[35]=" "; msg_rom[36]="p"; msg_rom[37]="k"; 
        msg_rom[38]=":"; msg_rom[39]=" "; msg_rom[40]=8'h00;

        // 字典 2: "\r\nsk: " (sk：) | 地址: 50
        msg_rom[50]="\r"; msg_rom[51]="\n"; msg_rom[52]="s"; msg_rom[53]="k"; msg_rom[54]=":"; msg_rom[55]=" "; msg_rom[56]=8'h00;

        // 字典 3: "\r\nStart Sign...\r\nsign: " (开始签名 sign：) | 地址: 60
        msg_rom[60]="\r"; msg_rom[61]="\n"; msg_rom[62]="S"; msg_rom[63]="t"; msg_rom[64]="a"; msg_rom[65]="r"; 
        msg_rom[66]="t"; msg_rom[67]=" "; msg_rom[68]="S"; msg_rom[69]="i"; msg_rom[70]="g"; msg_rom[71]="n"; 
        msg_rom[72]="."; msg_rom[73]="."; msg_rom[74]="\r"; msg_rom[75]="\n"; msg_rom[76]="s"; msg_rom[77]="i"; 
        msg_rom[78]="g"; msg_rom[79]="n"; msg_rom[80]=":"; msg_rom[81]=" "; msg_rom[82]=8'h00;

        // 字典 4: "\r\nVerify Success!\r\n" (验签成功) | 地址: 90
        msg_rom[90]="\r"; msg_rom[91]="\n"; msg_rom[92]="V"; msg_rom[93]="e"; msg_rom[94]="r"; msg_rom[95]="i"; 
        msg_rom[96]="f"; msg_rom[97]="y"; msg_rom[98]=" "; msg_rom[99]="S"; msg_rom[100]="u"; msg_rom[101]="c"; 
        msg_rom[102]="c"; msg_rom[103]="e"; msg_rom[104]="s"; msg_rom[105]="s"; msg_rom[106]="!"; msg_rom[107]="\r"; msg_rom[108]="\n"; msg_rom[109]=8'h00;

        // 字典 5: "\r\nVerify Failed!\r\n" (验签失败) | 地址: 110
        msg_rom[110]="\r"; msg_rom[111]="\n"; msg_rom[112]="V"; msg_rom[113]="e"; msg_rom[114]="r"; msg_rom[115]="i"; 
        msg_rom[116]="f"; msg_rom[117]="y"; msg_rom[118]=" "; msg_rom[119]="F"; msg_rom[120]="a"; msg_rom[121]="i"; 
        msg_rom[122]="l"; msg_rom[123]="e"; msg_rom[124]="d"; msg_rom[125]="!"; msg_rom[126]="\r"; msg_rom[127]="\n"; msg_rom[128]=8'h00;
    end

    // ==========================================
    // 5. Hex 到 ASCII 转换逻辑
    // ==========================================
    reg [9:0] read_addr;      
    reg [2:0] nibble_idx;     
    wire [31:0] tx_ram_rdata; 



    // 提取当前准备发送的 4-bit 原始数据 (纯 Verilog 组合逻辑)
    wire [3:0] current_nibble;
    assign current_nibble = (nibble_idx == 3'd7) ? tx_ram_rdata[31:28] :
                            (nibble_idx == 3'd6) ? tx_ram_rdata[27:24] :
                            (nibble_idx == 3'd5) ? tx_ram_rdata[23:20] :
                            (nibble_idx == 3'd4) ? tx_ram_rdata[19:16] :
                            (nibble_idx == 3'd3) ? tx_ram_rdata[15:12] :
                            (nibble_idx == 3'd2) ? tx_ram_rdata[11:8]  :
                            (nibble_idx == 3'd1) ? tx_ram_rdata[7:4]   : tx_ram_rdata[3:0];

    // 0~9 映射到 '0'~'9'，10~15 映射到 'A'~'F'
    wire [7:0] ascii_char;
    assign ascii_char = (current_nibble <= 4'd9) ? ({4'h0, current_nibble} + 8'h30) : ({4'h0, current_nibble} - 8'd10 + 8'h41);

    // ==========================================
    // 6. UART TX 发送机调度
    // ==========================================
    reg [3:0] tx_state;
    reg       tx_start;
    reg [7:0] tx_data;
    wire      tx_done;
    reg       tx_busy;

    uart_tx u_uart_tx (
        .clk(clk), 
        .rst_n(rst_n), 
        .tx_start(tx_start), 
        .tx_data(tx_data), 
        .tx(tx), 
        .tx_done(tx_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tx_busy <= 1'b0;
        else if (tx_start) tx_busy <= 1'b1;
        else if (tx_done) tx_busy <= 1'b0;
    end

    localparam S_IDLE       = 4'd0;
    localparam S_STR_START  = 4'd1;
    localparam S_WAIT_CORE  = 4'd2;
    localparam S_STR_PK     = 4'd3;
    localparam S_DUMP_PK    = 4'd4;
    localparam S_STR_SK     = 4'd5;
    localparam S_DUMP_SK    = 4'd6;
    localparam S_STR_SIG    = 4'd7;
    localparam S_DUMP_SIG   = 4'd8;
    localparam S_STR_VER    = 4'd9;

    reg [6:0] str_ptr;
    reg [10:0] words_to_send;

    // RAM 数据与地址复用逻辑
    assign tx_pk_read_addr  = (tx_state == S_DUMP_PK) ? read_addr[8:0] : 9'd0;
    assign tx_sk_read_addr  = (tx_state == S_DUMP_SK) ? read_addr : 10'd0;
    assign tx_sig_read_addr = (tx_state == S_DUMP_SIG)? read_addr : 10'd0;
    assign tx_ram_rdata     = (tx_state == S_DUMP_PK) ? tx_pk_read_data :
                              (tx_state == S_DUMP_SK) ? tx_sk_read_data : tx_sig_read_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= S_IDLE; 
            tx_start <= 1'b0;
            str_ptr  <= 7'd0;
            read_addr <= 10'd0;
            nibble_idx <= 3'd0;
            words_to_send <= 11'd0;
            tx_data <= 8'd0;
        end else begin
            tx_start <= 1'b0;
            case(tx_state)
                S_IDLE: begin
                    if (dili_start) begin
                        tx_state <= S_STR_START; 
                        str_ptr <= 7'd0; // 指向 "Start KeyGen"
                    end
                end
                
                S_STR_START: begin
                    if (msg_rom[str_ptr] == 8'h00) tx_state <= S_WAIT_CORE;
                    else if (!tx_busy && !tx_start) begin
                        tx_data <= msg_rom[str_ptr]; 
                        tx_start <= 1'b1; 
                        str_ptr <= str_ptr + 7'd1;
                    end
                end
                
                S_WAIT_CORE: begin
                    if (core_done) begin
                        tx_state <= S_STR_PK; 
                        str_ptr <= 7'd20; 
                    end
                end

// ... S_IDLE, S_STR_START, S_WAIT_CORE 保持不变 ...
                
                S_STR_PK: begin
                    if (msg_rom[str_ptr] == 8'h00) begin
                        tx_state <= S_DUMP_PK; 
                        words_to_send <= 11'd328;
                        read_addr <= 10'd327; // <--- 修改：从最大地址开始读
                        nibble_idx <= 3'd7; 
                    end else if (!tx_busy && !tx_start) begin
                        tx_data <= msg_rom[str_ptr]; 
                        tx_start <= 1'b1; 
                        str_ptr <= str_ptr + 7'd1;
                    end
                end

                S_DUMP_PK: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data <= ascii_char; 
                        tx_start <= 1'b1;
                        if (nibble_idx == 3'd0) begin
                            nibble_idx <= 3'd7;
                            if (read_addr == 10'd0) begin // <--- 修改：地址减到 0 时结束当前段
                                tx_state <= S_STR_SK; 
                                str_ptr <= 7'd50; 
                            end
                            else read_addr <= read_addr - 10'd1; // <--- 修改：地址递减
                        end else nibble_idx <= nibble_idx - 3'd1;
                    end
                end

                S_STR_SK: begin
                    if (msg_rom[str_ptr] == 8'h00) begin
                        tx_state <= S_DUMP_SK; 
                        words_to_send <= 11'd640;
                        read_addr <= 10'd639; // <--- 修改：从最大地址开始读
                        nibble_idx <= 3'd7; 
                    end else if (!tx_busy && !tx_start) begin
                        tx_data <= msg_rom[str_ptr]; 
                        tx_start <= 1'b1; 
                        str_ptr <= str_ptr + 7'd1;
                    end
                end

                S_DUMP_SK: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data <= ascii_char; 
                        tx_start <= 1'b1;
                        if (nibble_idx == 3'd0) begin
                            nibble_idx <= 3'd7;
                            if (read_addr == 10'd0) begin // <--- 修改：地址减到 0 时结束当前段
                                tx_state <= S_STR_SIG; 
                                str_ptr <= 7'd60; 
                            end
                            else read_addr <= read_addr - 10'd1; // <--- 修改：地址递减
                        end else nibble_idx <= nibble_idx - 3'd1;
                    end
                end

                S_STR_SIG: begin
                    if (msg_rom[str_ptr] == 8'h00) begin
                        tx_state <= S_DUMP_SIG; 
                        words_to_send <= 11'd605;
                        read_addr <= 10'd604; // <--- 修改：从最大地址开始读
                        nibble_idx <= 3'd7; 
                    end else if (!tx_busy && !tx_start) begin
                        tx_data <= msg_rom[str_ptr]; 
                        tx_start <= 1'b1; 
                        str_ptr <= str_ptr + 7'd1;
                    end
                end

                S_DUMP_SIG: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data <= ascii_char; 
                        tx_start <= 1'b1;
                        if (nibble_idx == 3'd0) begin
                            nibble_idx <= 3'd7;
                            if (read_addr == 10'd0) begin // <--- 修改：地址减到 0 时结束当前段
                                tx_state <= S_STR_VER; 
                                str_ptr <= verify_success ? 7'd90 : 7'd110; 
                            end else read_addr <= read_addr - 10'd1; // <--- 修改：地址递减
                        end else nibble_idx <= nibble_idx - 3'd1;
                    end
                end
                
                // ... S_STR_VER 等后续逻辑保持不变 ...

                S_STR_VER: begin
                    if (msg_rom[str_ptr] == 8'h00) tx_state <= S_IDLE;
                    else if (!tx_busy && !tx_start) begin
                        tx_data <= msg_rom[str_ptr]; 
                        tx_start <= 1'b1; 
                        str_ptr <= str_ptr + 7'd1;
                    end
                end
                
                default: tx_state <= S_IDLE;
            endcase
        end
    end
    
//    always@(posedge clk or negedge rst_n)
//    if(!rst_n)
//        led <= 1'b0;
//    else if(verify_success)
//        led <= ~led;
        
endmodule