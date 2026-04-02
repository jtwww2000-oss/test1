`timescale 1ns / 1ps

module SHAKE256 #(
    parameter RATE = 1088,             
    parameter CAPACITY = 512,          
    parameter STATE_WIDTH = 1600,
    parameter OUTPUT_LEN_BYTES = 128,  
    parameter ABSORB_LEN = 256         
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 i_start,  
    input  wire [ABSORB_LEN-1:0] i_seed,   
    output reg                  o_busy,   

    input  wire                 i_squeeze_req,
    output reg                  o_squeeze_valid,
    output wire [OUTPUT_LEN_BYTES*8-1:0] o_squeeze_data
);

    localparam S_IDLE           = 3'd0;
    localparam S_WAIT_KECCAK    = 3'd1; 
    localparam S_PAD_EXTRA      = 3'd2;
    localparam S_SQUEEZE        = 3'd3; 
    localparam S_ABSORB_EVAL    = 3'd4; // 新增：分块切片状态

    reg [2:0] current_state;
    reg [STATE_WIDTH-1:0] r_state;          
    reg [2:0] r_next_action;
    
    reg                     keccak_start;
    reg  [STATE_WIDTH-1:0]  keccak_in_data;
    wire                    keccak_done;
    wire [STATE_WIDTH-1:0]  keccak_out_data;
    wire                    keccak_busy;

    // 安全的移位寄存器宽度（防止 ABSORB_LEN < RATE 时越界报错）
    localparam SHIFT_W = (ABSORB_LEN > RATE) ? ABSORB_LEN : RATE;
    reg [SHIFT_W-1:0] shift_seed;
    reg [15:0] remaining_bits;

    wire [RATE-1:0] pad_in_N;
    wire [10:0]     pad_in_m;
    wire [RATE-1:0] pad_out_P;
    wire            w_need_extra_pad;

    Keccak_f #( .STATE_WIDTH(STATE_WIDTH) ) u_keccak (
        .clk(clk), .rst_n(rst_n), .i_start(keccak_start), .i_data(keccak_in_data),
        .o_valid(keccak_done), .o_data(keccak_out_data), .o_busy(keccak_busy)
    );

    pad #( .X(RATE), .MAX_LEN(RATE) ) u_pad (
        .N(pad_in_N), .m(pad_in_m), .P(pad_out_P)
    );

    wire [10:0] cur_m = remaining_bits[10:0];
    // 如果剩下的位数离 RATE 太近，没有空间塞下填充后缀，就需要额外一个全填充块
    assign w_need_extra_pad = (cur_m > (RATE - 5)); 
    
    assign pad_in_N = (current_state == S_PAD_EXTRA) ? {RATE{1'b0}} : shift_seed[RATE-1:0];
    assign pad_in_m = (current_state == S_PAD_EXTRA) ? 11'd0       : cur_m;

    // --- 主状态机 (支持无限长度的分块 Absorb) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state   <= S_IDLE;
            r_state         <= {STATE_WIDTH{1'b0}};
            r_next_action   <= 3'd0;
            o_busy          <= 1'b0;
            o_squeeze_valid <= 1'b0;
            keccak_start    <= 1'b0;
            keccak_in_data  <= {STATE_WIDTH{1'b0}};
            shift_seed      <= 0;
            remaining_bits  <= 0;
        end else begin
            keccak_start <= 1'b0;

            if (i_start) begin
                o_busy <= 1'b1;
                o_squeeze_valid <= 1'b0;

                // 载入完整数据
                shift_seed <= i_seed;
                remaining_bits <= ABSORB_LEN;
                r_state <= {STATE_WIDTH{1'b0}};
                
                current_state <= S_ABSORB_EVAL;
            end else begin
                case (current_state)
                    S_IDLE: begin
                        o_busy <= 1'b0;
                        o_squeeze_valid <= 1'b0;
                    end

                    // ★ 核心修复：切片分块吸收逻辑 ★
                    S_ABSORB_EVAL: begin
                        if (remaining_bits >= RATE) begin
                            // 数据大于 RATE，截取低 1088 位进行异或
                            keccak_in_data[RATE-1:0] <= r_state[RATE-1:0] ^ shift_seed[RATE-1:0];
                            keccak_in_data[STATE_WIDTH-1:RATE] <= r_state[STATE_WIDTH-1:RATE];
                            
                            // 移位丢弃已经吸收的数据
                            shift_seed <= shift_seed >> RATE;
                            remaining_bits <= remaining_bits - RATE;
                            
                            keccak_start <= 1'b1;
                            r_next_action <= S_ABSORB_EVAL; // 完成本块后继续回来切片
                            current_state <= S_WAIT_KECCAK;
                        end else begin
                            // 数据不足 RATE，进入尾部填充计算
                            keccak_in_data[RATE-1:0] <= r_state[RATE-1:0] ^ pad_out_P;
                            keccak_in_data[STATE_WIDTH-1:RATE] <= r_state[STATE_WIDTH-1:RATE];
                            
                            keccak_start <= 1'b1;
                            if (w_need_extra_pad)
                                r_next_action <= S_PAD_EXTRA;
                            else
                                r_next_action <= S_SQUEEZE;
                            
                            current_state <= S_WAIT_KECCAK;
                        end
                    end

                    S_WAIT_KECCAK: begin
                        if (keccak_done) begin
                            r_state <= keccak_out_data;
                            current_state <= r_next_action;
                        end
                    end

                    S_PAD_EXTRA: begin
                        keccak_in_data[RATE-1:0] <= r_state[RATE-1:0] ^ pad_out_P; 
                        keccak_in_data[STATE_WIDTH-1:RATE] <= r_state[STATE_WIDTH-1:RATE];
                        keccak_start   <= 1'b1;
                        r_next_action  <= S_SQUEEZE;
                        current_state  <= S_WAIT_KECCAK;
                    end

                    S_SQUEEZE: begin
                        o_busy          <= 1'b1;
                        o_squeeze_valid <= 1'b1;

                        if (i_squeeze_req) begin
                            o_squeeze_valid <= 1'b0;
                            keccak_in_data <= r_state; 
                            keccak_start   <= 1'b1;
                            r_next_action <= S_SQUEEZE;
                            current_state <= S_WAIT_KECCAK;
                        end
                    end

                    default: current_state <= S_IDLE;
                endcase
            end
        end
    end

    assign o_squeeze_data = r_state[OUTPUT_LEN_BYTES*8-1:0];
endmodule