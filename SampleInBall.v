`timescale 1ns / 1ps

module SampleInBall #(
    parameter TAU = 8'd39,
    parameter Q   = 24'd8380417
)(
    input  wire          clk,
    input  wire          rst_n,

    // --- 控制与输入 ---
    input  wire          i_start,
    input  wire [255:0]  i_c1,       // 256-bit hash 种子
    
    // --- c 数组外部 RAM 接口 (256 x 24-bit) ---
    // 要求: 外部 RAM 应当具备 1 周期读取延迟 (1-Cycle Latency)
    output reg           o_c_we,
    output reg  [7:0]    o_c_addr,
    output reg  [23:0]   o_c_wdata,
    input  wire [23:0]   i_c_rdata,  // 读回的 c[j]

    output reg           o_done
);

    // FSM 状态定义
    localparam S_IDLE        = 4'd0;
    localparam S_INIT_RAM    = 4'd1;
    localparam S_START_SHAKE = 4'd2;
    localparam S_REQ_SQUEEZE = 4'd3;
    localparam S_WAIT_ACK    = 4'd4;
    localparam S_WAIT_DATA   = 4'd5;
    localparam S_CHECK_J     = 4'd6;
    localparam S_READ_J_WAIT = 4'd7;
    localparam S_WRITE_I     = 4'd8;
    localparam S_WRITE_J     = 4'd9;
    localparam S_NEXT_I      = 4'd10;
    localparam S_DONE        = 4'd11;

    reg [3:0] state;

    // --- 内部寄存器 ---
    reg [7:0]    i_reg;          // 对应循环变量 i
    reg [7:0]    j_reg;          // 采样得到的 j
    reg          s_val;          // 采样得到的符号位

    reg [63:0]   sign_bits;      // 存储最初产生的 64 个符号位
    reg [1087:0] byte_buffer;    // 存储剩余的随机字节流
    reg [7:0]    bytes_avail;    // 当前缓冲区中可用的字节数
    reg          first_squeeze;  // 标记是否为第一次 Squeeze

    // --- SHAKE256 接口 ---
    reg  shake_start;
    reg  shake_squeeze_req;
    wire shake_squeeze_valid;
    wire [1087:0] shake_out_data;

    SHAKE256 #(
        .OUTPUT_LEN_BYTES(136), // 一次吐出 1088 bits
        .ABSORB_LEN(256)        // 吸收 c1 为 256 bits
    ) u_shake (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (shake_start),
        .i_seed          (i_c1),
        .o_busy          (),
        .i_squeeze_req   (shake_squeeze_req),
        .o_squeeze_valid (shake_squeeze_valid),
        .o_squeeze_data  (shake_out_data)
    );

    wire [7:0] current_j = byte_buffer[7:0];

    // --- 主状态机 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_c_we <= 0; o_c_addr <= 0; o_c_wdata <= 0;
            o_done <= 0; shake_start <= 0; shake_squeeze_req <= 0;
            i_reg <= 0; j_reg <= 0; s_val <= 0;
            sign_bits <= 0; byte_buffer <= 0; bytes_avail <= 0;
            first_squeeze <= 0;
        end else begin
            shake_start <= 0;
            o_c_we <= 0; // 默认拉低写使能

            case (state)
                S_IDLE: begin
                    o_done <= 0;
                    if (i_start) begin
                        i_reg <= 8'd0;
                        first_squeeze <= 1'b1;
                        state <= S_INIT_RAM;
                    end
                end

                // 1. 初始化数组 c 为全 0 (耗时 256 个周期)
                S_INIT_RAM: begin
                    o_c_we <= 1'b1;
                    o_c_addr <= i_reg;
                    o_c_wdata <= 24'd0;
                    if (i_reg == 8'd255) begin
                        i_reg <= 8'd217; // 赋值为 256 - 39 = 217
                        state <= S_START_SHAKE;
                    end else begin
                        i_reg <= i_reg + 1;
                    end
                end

                // 2. 启动 SHAKE256 吸收并准备第一次释放
                S_START_SHAKE: begin
                    shake_start <= 1'b1;
                    state <= S_REQ_SQUEEZE;
                end

                // 3. 请求获取伪随机数据块
                S_REQ_SQUEEZE: begin
                    shake_squeeze_req <= 1'b1;
                    state <= S_WAIT_ACK;
                end
                S_WAIT_ACK: begin
                    shake_squeeze_req <= 1'b0;
                    if (!shake_squeeze_valid) state <= S_WAIT_DATA;
                end
                S_WAIT_DATA: begin
                    if (shake_squeeze_valid) begin
                        if (first_squeeze) begin
                            // 第一批数据：前 8 字节为 sign_bits，剩下 128 字节用于 j
                            sign_bits   <= shake_out_data[63:0];
                            byte_buffer <= {64'd0, shake_out_data[1087:64]}; 
                            bytes_avail <= 8'd128;
                            first_squeeze <= 1'b0;
                        end else begin
                            // 后续数据：全部 136 字节用于 j
                            byte_buffer <= shake_out_data;
                            bytes_avail <= 8'd136;
                        end
                        state <= S_CHECK_J;
                    end
                end

                // 4. 拒绝采样逻辑核心
                S_CHECK_J: begin
                    if (bytes_avail == 0) begin
                        state <= S_REQ_SQUEEZE; // 当前块用完了，去要下一块
                    end else begin
                        // 相当于 MATLAB 中的 j = sum(...)
                        if (current_j > i_reg) begin
                            // j > i, 拒绝。丢弃当前字节
                            byte_buffer <= {8'd0, byte_buffer[1087:8]};
                            bytes_avail <= bytes_avail - 1;
                        end else begin
                            // 接受当前 j
                            j_reg <= current_j;
                            s_val <= sign_bits[0]; // 获取对应符号位
                            
                            // 移位丢弃已被消耗的数据
                            sign_bits <= {1'b0, sign_bits[63:1]};
                            byte_buffer <= {8'd0, byte_buffer[1087:8]};
                            bytes_avail <= bytes_avail - 1;
                            
                            // 读取 c[j] 的内容准备覆盖
                            o_c_addr <= current_j;
                            state <= S_READ_J_WAIT;
                        end
                    end
                end

                // 等待 RAM 读取 c[j] 
                S_READ_J_WAIT: begin
                    state <= S_WRITE_I;
                end

                // 5. 写入 c[i] = c[j]
                S_WRITE_I: begin
                    o_c_we    <= 1'b1;
                    o_c_addr  <= i_reg;
                    o_c_wdata <= i_c_rdata; // 这是刚刚读出来的 c[j]
                    state     <= S_WRITE_J;
                end

                // 6. 写入 c[j] = s ? 8380416 : 1
                S_WRITE_J: begin
                    o_c_we    <= 1'b1;
                    o_c_addr  <= j_reg;
                    o_c_wdata <= s_val ? 24'd8380416 : 24'd1;
                    state     <= S_NEXT_I;
                end

                // 7. 循环步进
                S_NEXT_I: begin
                    if (i_reg == 8'd255) begin
                        state <= S_DONE;
                    end else begin
                        i_reg <= i_reg + 1;
                        state <= S_CHECK_J;
                    end
                end

                S_DONE: begin
                    o_done <= 1'b1;
                    if (!i_start) state <= S_IDLE;
                end
            endcase
        end
    end

endmodule