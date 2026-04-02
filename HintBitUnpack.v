`timescale 1ns / 1ps

module HintBitUnpack (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // 启动信号
    
    // 连接到输入 y 数组(84 bytes)的读接口
    output reg  [6:0]  y_rd_addr,      // 地址 0~83
    input  wire [7:0]  y_rd_data,      // 读出的字节数据
    
    // 连接到输出 hint BRAM 的写接口 (深度 1024, 位宽 1)
    output reg  [9:0]  hint_wr_addr,   // i * 256 + j
    output reg         hint_wr_en,
    output wire        hint_wr_data,   // 恒定为 1 (只写1)
    
    // 状态和结果输出
    output reg         done,           // 解码完成标志
    output reg         verify_result   // 0: 成功, 1: 失败 (格式错误或1的数量超出omega)
);

    // 参数定义 (Security Level 2)
    localparam K     = 4;
    localparam OMEGA = 80;

    // 状态机定义
    localparam IDLE           = 3'd0;
    localparam READ_BOUNDARY  = 3'd1; // 读 y(omega + i)
    localparam WAIT_BOUNDARY  = 3'd2; 
    localparam READ_HINT_POS  = 3'd3; // 读 y(index)
    localparam WAIT_HINT_POS  = 3'd4;
    localparam CHECK_REMAIN   = 3'd5; // 检查剩余填充位是否为0
    localparam WAIT_REMAIN    = 3'd6;
    localparam DONE           = 3'd7;

    reg [2:0] state;
    reg [2:0] i_cnt;       // 相当于 i, 范围 0 到 k-1
    reg [6:0] index_cnt;   // 相当于 index, 最大为 omega(80)
    reg [6:0] cur_boundary;

    assign hint_wr_data = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            y_rd_addr     <= 7'd0;
            hint_wr_addr  <= 10'd0;
            hint_wr_en    <= 1'b0;
            done          <= 1'b0;
            verify_result <= 1'b0;
            i_cnt         <= 3'd0;
            index_cnt     <= 7'd0;
            cur_boundary  <= 7'd0;
        end else begin
            hint_wr_en <= 1'b0; // 默认不写
            
            case (state)
                IDLE: begin
                    done          <= 1'b0;
                    verify_result <= 1'b0;
                    i_cnt         <= 3'd0;
                    index_cnt     <= 7'd0;
                    if (start) begin
                        state     <= READ_BOUNDARY;
                        y_rd_addr <= OMEGA + i_cnt; // 对应 Matlab: y(omega + i)
                    end
                end

                READ_BOUNDARY: begin
                    state <= WAIT_BOUNDARY; // 假设 BRAM 读延迟 1 周期
                end

                WAIT_BOUNDARY: begin
                    cur_boundary <= y_rd_data;
                    // 检查边界有效性： y(omega + i) < index || y(omega + i) > omega
                    if (y_rd_data < index_cnt || y_rd_data > OMEGA) begin
                        verify_result <= 1'b1; // 验签失败
                        state         <= DONE;
                    end else begin
                        if (index_cnt < y_rd_data) begin
                            state     <= READ_HINT_POS;
                            y_rd_addr <= index_cnt;
                        end else begin
                            // 当前 i 处理完，进入下一个 i
                            if (i_cnt == K - 1) begin
                                state <= CHECK_REMAIN;
                                y_rd_addr <= index_cnt;
                            end else begin
                                i_cnt <= i_cnt + 1'b1;
                                state <= READ_BOUNDARY;
                                y_rd_addr <= OMEGA + i_cnt + 1'b1;
                            end
                        end
                    end
                end

                READ_HINT_POS: begin
                    state <= WAIT_HINT_POS;
                end

                WAIT_HINT_POS: begin
                    // 写入 hint = 1
                    hint_wr_addr <= {i_cnt[1:0], y_rd_data}; // i * 256 + y(index)
                    hint_wr_en   <= 1'b1;
                    
                    index_cnt <= index_cnt + 1'b1;
                    
                    if ((index_cnt + 1'b1) < cur_boundary) begin
                        state     <= READ_HINT_POS;
                        y_rd_addr <= index_cnt + 1'b1;
                    end else begin
                        // 当前 i 的边界达到，进入下一个 i
                        if (i_cnt == K - 1) begin
                            state     <= CHECK_REMAIN;
                            y_rd_addr <= index_cnt + 1'b1;
                        end else begin
                            i_cnt     <= i_cnt + 1'b1;
                            state     <= READ_BOUNDARY;
                            y_rd_addr <= OMEGA + i_cnt + 1'b1;
                        end
                    end
                end

                CHECK_REMAIN: begin
                    if (index_cnt < OMEGA) begin
                        state <= WAIT_REMAIN;
                    end else begin
                        state <= DONE; // 解码全部成功
                    end
                end

                WAIT_REMAIN: begin
                    if (y_rd_data != 8'd0) begin
                        verify_result <= 1'b1; // 填充位不为0，失败
                        state         <= DONE;
                    end else begin
                        index_cnt <= index_cnt + 1'b1;
                        if ((index_cnt + 1'b1) < OMEGA) begin
                            state     <= CHECK_REMAIN;
                            y_rd_addr <= index_cnt + 1'b1;
                        end else begin
                            state <= DONE;
                        end
                    end
                end

                DONE: begin
                    done <= 1'b1;
                    if (!start) begin // 等待 start 撤销后再回到 IDLE
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule