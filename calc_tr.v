`timescale 1ns / 1ps

module calc_tr (
    input  wire           clk,
    input  wire           rst_n,
    input  wire           i_start,
    
    output reg  [8:0]     o_pk_raddr,
    input  wire [31:0]    i_pk_rdata,
    
    output reg            o_done,
    output wire [511:0]   o_tr
);

    localparam RATE = 1088;
    localparam STATE_WIDTH = 1600;
    localparam TOTAL_BLKS = 5'd9;
    localparam REM_BITS   = 11'd704;

    localparam S_IDLE = 3'd0, S_FETCH = 3'd1, S_WAIT_RAM = 3'd2, S_ABSORB = 3'd3, S_ABSORB_PAD = 3'd4, S_PERM = 3'd5;
    
    reg [2:0] state;
    reg [4:0] blk_cnt;
    reg [5:0] word_cnt; 
    
    reg [RATE-1:0] current_chunk;
    wire [RATE-1:0] padded_chunk;
    
    reg [STATE_WIDTH-1:0] r_state;
    reg  keccak_start;
    wire keccak_done;
    wire [STATE_WIDTH-1:0] keccak_out;
    reg [STATE_WIDTH-1:0] keccak_in;

    pad #( .X(RATE), .MAX_LEN(RATE) ) u_pad (
        .N(current_chunk), .m(REM_BITS), .P(padded_chunk)
    );

    Keccak_f #( .STATE_WIDTH(STATE_WIDTH) ) u_keccak (
        .clk(clk), .rst_n(rst_n), .i_start(keccak_start), .i_data(keccak_in),
        .o_valid(keccak_done), .o_data(keccak_out), .o_busy()
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; blk_cnt <= 0; word_cnt <= 0; o_pk_raddr <= 0;
            current_chunk <= 0; r_state <= 0; keccak_start <= 0; o_done <= 0;
        end else begin
            keccak_start <= 0;
            case (state)
                S_IDLE: begin
                    o_done <= 0;
                    if (i_start) begin
                        state <= S_FETCH; blk_cnt <= 0; word_cnt <= 0; o_pk_raddr <= 0; r_state <= 0;
                        current_chunk <= 0; // 初始化清零
                    end
                end
                
                S_FETCH: begin
                    if (blk_cnt == TOTAL_BLKS && word_cnt == 22) begin
                        state <= S_ABSORB_PAD;
                    end else if (word_cnt == 34) begin
                        state <= S_ABSORB;
                    end else begin
                        state <= S_WAIT_RAM;
                    end
                end
                
                S_WAIT_RAM: begin
                    current_chunk[word_cnt * 32 +: 32] <= i_pk_rdata;
                    word_cnt <= word_cnt + 1;
                    o_pk_raddr <= o_pk_raddr + 1;
                    state <= S_FETCH;
                end
                
                S_ABSORB: begin
                    keccak_in[RATE-1:0] = r_state[RATE-1:0] ^ current_chunk;
                    keccak_in[STATE_WIDTH-1:RATE] = r_state[STATE_WIDTH-1:RATE];
                    keccak_start <= 1;
                    state <= S_PERM;
                end
                
                S_ABSORB_PAD: begin
                    keccak_in[RATE-1:0] = r_state[RATE-1:0] ^ padded_chunk;
                    keccak_in[STATE_WIDTH-1:RATE] = r_state[STATE_WIDTH-1:RATE];
                    keccak_start <= 1;
                    state <= S_PERM;
                    blk_cnt <= blk_cnt + 1;
                end
                
                S_PERM: begin
                    if (keccak_done) begin
                        r_state <= keccak_out;
                        current_chunk <= 0; // 核心修复：清空残余数据，防止干扰下一块的 Padding
                        if (blk_cnt < TOTAL_BLKS) begin
                            blk_cnt <= blk_cnt + 1; word_cnt <= 0; state <= S_FETCH;
                        end else begin
                            o_done <= 1; state <= S_IDLE;
                        end
                    end
                end
            endcase
        end
    end
    assign o_tr = r_state[511:0];
endmodule