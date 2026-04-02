`timescale 1ns / 1ps

module verify_final_check #(
    parameter WIDTH   = 24,
    parameter K_PARAM = 4,
    parameter W1_WIDTH = 6,
    parameter LAMBDA  = 128
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          i_start,
    
    input  wire [511:0]  i_u,   
    input  wire [255:0]  i_c_tilde,     
    
    output reg  [9:0]    o_w1_raddr,
    input  wire [5:0]    i_w1_rdata,
    
    output reg           o_done,
    output reg           o_verify_success 
);

    reg  [255:0]  o_calculated_c_tilde;
    
    localparam OUTPUT_BITS = 2 * LAMBDA;

    // --- 状态机定义 ---
    localparam ST_IDLE       = 4'd0;
    localparam ST_START_SHAKE= 4'd1;
    // 吸收 U 阶段
    localparam ST_PREP_U     = 4'd2;
    localparam ST_PUSH_U     = 4'd3;
    // 吸收 W1 阶段
    localparam ST_READ_W1    = 4'd4;
    localparam ST_WAIT_RAM   = 4'd5;
    localparam ST_PREP_W1    = 4'd6;
    localparam ST_PUSH_W1    = 4'd7;
    // 结束比对阶段
    localparam ST_WAIT_SHAKE = 4'd8;
    localparam ST_COMPARE    = 4'd9;
    localparam ST_DONE       = 4'd10;

    reg [3:0] state;
    reg [2:0] u_cnt;       // 用于计数 U 的 8 个 64-bit 块 (0~7)
    reg [9:0] coeff_cnt;   // 用于计数 W1 的 1024 个系数 (0~1023)
    
    // --- SHAKE256 流式接口信号 ---
    reg         shake_start;
    wire        shake_busy;
    
    reg         shake_absorb_valid;
    reg [63:0]  shake_absorb_data;
    reg [6:0]   shake_absorb_bits;
    reg         shake_absorb_last;
    wire        shake_absorb_ready;
    
    wire        shake_squeeze_valid;
    wire [255:0] shake_squeeze_data;

    // 实例化流式 SHAKE256
    SHAKE256_stream #(
        .OUTPUT_LEN_BYTES(OUTPUT_BITS/8)
    ) u_shake_final (
        .clk(clk),
        .rst_n(rst_n),
        .i_start(shake_start),
        .o_busy(shake_busy),
        
        // 吸收接口
        .i_absorb_valid(shake_absorb_valid),
        .i_absorb_data(shake_absorb_data),
        .i_absorb_bits(shake_absorb_bits),
        .i_absorb_last(shake_absorb_last),
        .o_absorb_ready(shake_absorb_ready),
        
        // 挤出接口 (自动获取第一块数据)
        .i_squeeze_req(1'b0), 
        .o_squeeze_valid(shake_squeeze_valid),
        .o_squeeze_data(shake_squeeze_data)
    );

    // --- U 数据切片多路选择器 ---
    // 避免使用动态索引 i_u[cnt*64 +: 64]，改成硬件友好的 case 选择
    reg [63:0] current_u_chunk;
    always @(*) begin
        case(u_cnt)
            3'd0: current_u_chunk = i_u[63:0];
            3'd1: current_u_chunk = i_u[127:64];
            3'd2: current_u_chunk = i_u[191:128];
            3'd3: current_u_chunk = i_u[255:192];
            3'd4: current_u_chunk = i_u[319:256];
            3'd5: current_u_chunk = i_u[383:320];
            3'd6: current_u_chunk = i_u[447:384];
            3'd7: current_u_chunk = i_u[511:448];
            default: current_u_chunk = 64'd0;
        endcase
    end

    // --- 主控制状态机 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            o_done <= 0; 
            o_verify_success <= 0;
            o_w1_raddr <= 0;
            
            shake_start <= 0; 
            u_cnt <= 0; 
            coeff_cnt <= 0;
            
            shake_absorb_valid <= 0;
            shake_absorb_data <= 0;
            shake_absorb_bits <= 0;
            shake_absorb_last <= 0;
        end else begin
            shake_start <= 0; // 默认拉低脉冲
            
            case (state)
                ST_IDLE: begin
                    o_done <= 0;
                    shake_absorb_valid <= 0;
                    
                    // 【核心保护】：不要在这里无条件清零！保留上一次结果给串口读
                    if (i_start) begin
                        o_calculated_c_tilde <= 256'd0; // 只有新一轮开始才清空
                        o_verify_success     <= 1'b0;
                        u_cnt <= 0;
                        coeff_cnt <= 0;
                        state <= ST_START_SHAKE;
                    end
                end

                ST_START_SHAKE: begin
                    shake_start <= 1'b1; // 复位/启动 SHAKE 核心
                    state <= ST_PREP_U;
                end

                // ==========================================
                // 阶段 1：吸收 512-bit 的 i_u (分 8 次 64-bit)
                // ==========================================
                ST_PREP_U: begin
                    shake_absorb_data <= current_u_chunk;
                    shake_absorb_bits <= 7'd64;     // 每次送满 64 位
                    shake_absorb_last <= 1'b0;      // 还没完
                    shake_absorb_valid <= 1'b1;
                    state <= ST_PUSH_U;
                end

                ST_PUSH_U: begin
                    // 标准 valid-ready 握手
                    if (shake_absorb_valid && shake_absorb_ready) begin
                        shake_absorb_valid <= 1'b0; // 握手成功，撤销 valid
                        if (u_cnt == 3'd7) begin
                            state <= ST_READ_W1;    // U 吸收完毕，去读 W1
                        end else begin
                            u_cnt <= u_cnt + 1;
                            state <= ST_PREP_U;     // 准备下一块 U
                        end
                    end
                end

                // ==========================================
                // 阶段 2：流式读取并吸收 RAM 中的 W1
                // ==========================================
                ST_READ_W1: begin
                    o_w1_raddr <= coeff_cnt;
                    state <= ST_WAIT_RAM;
                end

                ST_WAIT_RAM: begin
                    state <= ST_PREP_W1; // 等待 1 拍 RAM 延迟
                end

                ST_PREP_W1: begin
                    shake_absorb_data <= {58'd0, i_w1_rdata}; // 送入刚读出的 6 位数据
                    shake_absorb_bits <= 7'd6;                // 通知底层本次只有 6 位有效
                    shake_absorb_valid <= 1'b1;
                    
                    if (coeff_cnt == 10'd1023) begin
                        shake_absorb_last <= 1'b1; // 发送最后一块标志！
                    end else begin
                        shake_absorb_last <= 1'b0;
                    end
                    state <= ST_PUSH_W1;
                end

                ST_PUSH_W1: begin
                    // 等待 SHAKE 核心收下这 6 bit
                    if (shake_absorb_valid && shake_absorb_ready) begin
                        shake_absorb_valid <= 1'b0; 
                        if (coeff_cnt == 10'd1023) begin
                            state <= ST_WAIT_SHAKE; // W1 吸收完毕，等结果
                        end else begin
                            coeff_cnt <= coeff_cnt + 1;
                            state <= ST_READ_W1;    // 去读下一个字
                        end
                    end
                end

                // ==========================================
                // 阶段 3：等待 Hash 运算结束并比对
                // ==========================================

                ST_WAIT_SHAKE: begin
                    shake_absorb_last <= 1'b0;
                    if (shake_squeeze_valid) begin // 拿到 256 位哈希输出
                        o_calculated_c_tilde <= shake_squeeze_data; // 恢复你的正确锁存
                        state <= ST_COMPARE;
                    end
                end

                ST_COMPARE: begin
                    // 恢复你的打拍比对逻辑 (能算出 2EB0... 证明这步是绝配)
                    o_verify_success <= (o_calculated_c_tilde == i_c_tilde);
                    state <= ST_DONE;
                end

                ST_DONE: begin
                    o_done <= 1'b1;
                    if (!i_start) state <= ST_IDLE;
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule