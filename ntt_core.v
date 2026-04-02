`timescale 1ns / 1ps

module ntt_core #(
    parameter WIDTH = 24
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    output reg           done,
    
    // --- 乒乓 RAM 0 接口 ---
    output reg  [7:0]    ram0_addr_a,
    output reg           ram0_we_a,       
    output reg  [WIDTH-1:0] ram0_wdata_a,
    input  wire [WIDTH-1:0] ram0_rdata_a,
    
    output reg  [7:0]    ram0_addr_b,
    output reg           ram0_we_b,       
    output reg  [WIDTH-1:0] ram0_wdata_b,
    input  wire [WIDTH-1:0] ram0_rdata_b,

    // --- 乒乓 RAM 1 接口 ---
    output reg  [7:0]    ram1_addr_a,
    output reg           ram1_we_a,       
    output reg  [WIDTH-1:0] ram1_wdata_a,
    input  wire [WIDTH-1:0] ram1_rdata_a,
    
    output reg  [7:0]    ram1_addr_b,
    output reg           ram1_we_b,       
    output reg  [WIDTH-1:0] ram1_wdata_b,
    input  wire [WIDTH-1:0] ram1_rdata_b
);

    localparam [WIDTH-1:0] Q       = 24'd8380417;
    localparam [WIDTH-1:0] Q_PRIME = 24'd8380415; 

    localparam S_IDLE           = 4'd0;
    localparam S_PRE_RUN        = 4'd1;  
    localparam S_PRE_FLUSH      = 4'd2;
    localparam S_BR_RUN         = 4'd3;
    localparam S_BR_FLUSH       = 4'd4;
    localparam S_NTT_RUN        = 4'd5;  
    localparam S_NTT_FLUSH      = 4'd6;
    localparam S_DONE           = 4'd7;
    
    reg [3:0] state;
    reg [8:0] cnt;
    reg [2:0] stage_cnt;    
    reg [6:0] bf_cnt;       
    reg       pipe_en;      

    // 【修改点】：深度+1，适应6拍乘法器
    reg [7:0] pre_sr_val;
    reg [7:0] pre_sr_addr [0:7];  
    reg [7:0] pre_rom_addr_reg;   

    reg [1:0] br_sr_val;
    reg [7:0] br_sr_addr [0:1];

    // 【修改点】：深度+1，适应6拍乘法器
    reg [8:0] sr_val;       
    reg [8:0] sr_ping_pong; 
    reg [7:0] sr_addr_u [0:8];
    reg [7:0] sr_addr_v [0:8];
    reg [WIDTH-1:0] u_d1, u_d2, u_d3, u_d4, u_d5, u_d6; 
    reg [WIDTH-1:0] res_u_reg, res_v_reg;   
    reg [7:0] rom_addr_reg; 

    wire [2:0] stage_k = stage_cnt;
    wire [6:0] H = bf_cnt >> stage_k;
    wire [6:0] L = bf_cnt & ((1 << stage_k) - 1);
    wire [7:0] addr_u_gen   = (H << (stage_k + 1)) | L;
    wire [7:0] addr_v_gen   = (H << (stage_k + 1)) | (1 << stage_k) | L;
    wire [7:0] rom_addr_gen = (1 << stage_cnt) + L; 

    wire [WIDTH-1:0] pre_rom_data, twiddle_data, mont_res, mod_add_out, mod_sub_out;
    pre_rom u_pre_rom (.clka(clk), .addra(pre_rom_addr_reg), .douta(pre_rom_data));
    twiddle_rom u_twiddle_rom (.clka(clk), .addra(rom_addr_reg), .douta(twiddle_data));

    wire current_pp = sr_ping_pong[0];
    wire [WIDTH-1:0] current_u = (current_pp == 0) ? ram0_rdata_a : ram1_rdata_a;
    wire [WIDTH-1:0] current_v = (current_pp == 0) ? ram0_rdata_b : ram1_rdata_b;

    wire [WIDTH-1:0] mont_a_in = (state == S_PRE_RUN || state == S_PRE_FLUSH) ? ram0_rdata_a : current_v;
    wire [WIDTH-1:0] mont_b_in = (state == S_PRE_RUN || state == S_PRE_FLUSH) ? pre_rom_data : twiddle_data;
    
    Montgomery_mul #( .WIDTH(WIDTH) ) u_mont_mul (
        .clk(clk), .a(mont_a_in), .b(mont_b_in), .q(Q), .q_prime(Q_PRIME), .res(mont_res)
    );
    // 【修改点】：输入数据延迟对齐为 u_d6
    mod_add #( .WIDTH(WIDTH) ) u_mod_add ( .a(u_d6), .b(mont_res), .q(Q), .res(mod_add_out) );
    mod_sub #( .WIDTH(WIDTH) ) u_mod_sub ( .a(u_d6), .b(mont_res), .q(Q), .res(mod_sub_out) );

    function [7:0] bit_reverse(input [7:0] in);
        integer k;
        for (k = 0; k < 8; k = k + 1) bit_reverse[k] = in[7-k];
    endfunction

    integer i; 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0;
            ram0_we_a <= 0; ram0_we_b <= 0; ram1_we_a <= 0; ram1_we_b <= 0;
            ram0_addr_a <= 0; ram0_addr_b <= 0; ram1_addr_a <= 0; ram1_addr_b <= 0;
            ram0_wdata_a <= 0; ram0_wdata_b <= 0; ram1_wdata_a <= 0; ram1_wdata_b <= 0;
            
            cnt <= 0; pipe_en <= 0; stage_cnt <= 0; bf_cnt <= 0;
            sr_val <= 0; pre_sr_val <= 0; br_sr_val <= 0; sr_ping_pong <= 0;
            rom_addr_reg <= 0; pre_rom_addr_reg <= 0; 
            
            br_sr_addr[0] <= 0; br_sr_addr[1] <= 0;
            
            u_d1 <= 0; u_d2 <= 0; u_d3 <= 0; u_d4 <= 0; u_d5 <= 0; u_d6 <= 0;
            res_u_reg <= 0; res_v_reg <= 0;
            
            for(i=0; i<8; i=i+1) pre_sr_addr[i] <= 0;
            for(i=0; i<9; i=i+1) begin sr_addr_u[i] <= 0; sr_addr_v[i] <= 0; end
        end else begin
            ram0_we_a <= 0; ram0_we_b <= 0; ram1_we_a <= 0; ram1_we_b <= 0;
        
            if (state == S_BR_RUN || state == S_BR_FLUSH) begin
                br_sr_val <= {br_sr_val[0], pipe_en};
                br_sr_addr[0] <= bit_reverse(cnt[7:0]);
                br_sr_addr[1] <= br_sr_addr[0];
            end

            if (state == S_PRE_RUN || state == S_PRE_FLUSH) begin
                pre_sr_val <= {pre_sr_val[6:0], pipe_en}; // 移位7位
                pre_sr_addr[0] <= cnt[7:0];
                for (i=1; i<8; i=i+1) pre_sr_addr[i] <= pre_sr_addr[i-1];
            end

            if (state == S_NTT_RUN || state == S_NTT_FLUSH) begin
                sr_val <= {sr_val[7:0], pipe_en};         // 移位8位
                sr_ping_pong <= {sr_ping_pong[7:0], stage_cnt[0]};
                
                sr_addr_u[0] <= addr_u_gen; sr_addr_v[0] <= addr_v_gen;
                for (i=1; i<9; i=i+1) begin
                    sr_addr_u[i] <= sr_addr_u[i-1];
                    sr_addr_v[i] <= sr_addr_v[i-1];
                end
                
                u_d1 <= current_u;
                u_d2 <= u_d1; u_d3 <= u_d2; u_d4 <= u_d3; u_d5 <= u_d4; u_d6 <= u_d5;
                res_u_reg <= mod_add_out; res_v_reg <= mod_sub_out;
            end
            
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin 
                        cnt <= 0; pipe_en <= 1; state <= S_PRE_RUN; 
                        stage_cnt <= 0; bf_cnt <= 0;
                        sr_val <= 0; pre_sr_val <= 0; br_sr_val <= 0; sr_ping_pong <= 0;
                        u_d1 <= 0; u_d2 <= 0; u_d3 <= 0; u_d4 <= 0; u_d5 <= 0; u_d6 <= 0;
                        res_u_reg <= 0; res_v_reg <= 0;
                    end
                end

                S_PRE_RUN: begin
                    if (pipe_en) begin
                        ram0_addr_a <= cnt[7:0]; pre_rom_addr_reg <= cnt[7:0];
                        if (cnt == 255) begin pipe_en <= 0; state <= S_PRE_FLUSH; end 
                        else cnt <= cnt + 1;
                    end
                    if (pre_sr_val[7]) begin
                        ram1_we_a <= 1;
                        ram1_addr_a <= pre_sr_addr[7]; ram1_wdata_a <= mont_res; 
                    end
                end
                S_PRE_FLUSH: begin
                    if (pre_sr_val[7]) begin
                        ram1_we_a <= 1;
                        ram1_addr_a <= pre_sr_addr[7]; ram1_wdata_a <= mont_res;
                    end
                    if (pre_sr_val == 0) begin cnt <= 0; pipe_en <= 1; state <= S_BR_RUN; end
                end

                S_BR_RUN: begin
                    if (pipe_en) begin
                        ram1_addr_a <= cnt[7:0];
                        if (cnt == 255) begin pipe_en <= 0; state <= S_BR_FLUSH; end 
                        else cnt <= cnt + 1;
                    end
                    if (br_sr_val[1]) begin
                        ram0_we_a <= 1; ram0_addr_a <= br_sr_addr[1]; ram0_wdata_a <= ram1_rdata_a;
                    end
                end
                S_BR_FLUSH: begin
                    if (br_sr_val[1]) begin
                        ram0_we_a <= 1; ram0_addr_a <= br_sr_addr[1]; ram0_wdata_a <= ram1_rdata_a;
                    end
                    if (br_sr_val == 0) begin
                        stage_cnt <= 0; bf_cnt <= 0; pipe_en <= 1; sr_val <= 0; state <= S_NTT_RUN;
                    end
                end

                S_NTT_RUN: begin
                    if (pipe_en) begin
                        if (stage_cnt[0] == 0) begin
                            ram0_addr_a <= addr_u_gen; ram0_addr_b <= addr_v_gen;
                        end else begin
                            ram1_addr_a <= addr_u_gen; ram1_addr_b <= addr_v_gen;
                        end
                        rom_addr_reg <= rom_addr_gen;
                        if (bf_cnt == 127) begin
                            bf_cnt <= 0; pipe_en <= 0; state <= S_NTT_FLUSH;
                        end else bf_cnt <= bf_cnt + 1;
                    end
                    
                    if (sr_val[8]) begin
                        if (sr_ping_pong[8] == 0) begin 
                            ram1_we_a <= 1; ram1_addr_a <= sr_addr_u[8]; ram1_wdata_a <= res_u_reg;
                            ram1_we_b <= 1; ram1_addr_b <= sr_addr_v[8]; ram1_wdata_b <= res_v_reg;
                        end else begin                  
                            ram0_we_a <= 1; ram0_addr_a <= sr_addr_u[8]; ram0_wdata_a <= res_u_reg;
                            ram0_we_b <= 1; ram0_addr_b <= sr_addr_v[8]; ram0_wdata_b <= res_v_reg;
                        end
                    end
                end
                S_NTT_FLUSH: begin
                    if (sr_val[8]) begin 
                        if (sr_ping_pong[8] == 0) begin
                            ram1_we_a <= 1; ram1_addr_a <= sr_addr_u[8]; ram1_wdata_a <= res_u_reg;
                            ram1_we_b <= 1; ram1_addr_b <= sr_addr_v[8]; ram1_wdata_b <= res_v_reg;
                        end else begin
                            ram0_we_a <= 1; ram0_addr_a <= sr_addr_u[8]; ram0_wdata_a <= res_u_reg;
                            ram0_we_b <= 1; ram0_addr_b <= sr_addr_v[8]; ram0_wdata_b <= res_v_reg;
                        end
                    end
                    
                    if (sr_val == 9'b0) begin 
                        if (stage_cnt == 7) state <= S_DONE;
                        else begin
                            stage_cnt <= stage_cnt + 1; pipe_en <= 1; state <= S_NTT_RUN;
                        end
                    end
                end

                S_DONE: begin
                    done <= 1;
                    if (start == 0) state <= S_IDLE;
                end
            endcase
        end
    end
endmodule