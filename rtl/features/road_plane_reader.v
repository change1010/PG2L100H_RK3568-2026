`timescale 1ns/1ps
// DDR 突发读取失败时仅输出结束错误标记，不输出旧数据。
// 事务保持到 rd_finish，本地超时不能把未完成事务交给其他请求者。
// rd_fault 为 AXI 粘滞故障标志。
module road_plane_reader #(
    parameter integer LENGTH_BITS=16,
    parameter [LENGTH_BITS-1:0] TOTAL_BEATS=7200,
    parameter [9:0] BURST_BEATS=10'd16
)(
    input mem_clk, input rst_n, input start, input abort,
    input [24:0] cfg_base_beat, input [LENGTH_BITS-1:0] cfg_slot_beats, input [1:0] slot_idx,
    output reg rd_req, output reg [24:0] rd_addr,
    input rd_valid, input [255:0] rd_data, input rd_finish, input rd_fault,
    output [127:0] data, output data_valid, output data_error, input data_ready,
    output reg done, output reg [LENGTH_BITS-1:0] bursts_done, output reg error, output busy
);
    localparam [1:0] S_IDLE=0, S_COLLECT=1, S_EMIT=2, S_ERROR=3;
    reg [1:0] state;
    reg [24:0] base_hold;
    reg [LENGTH_BITS-1:0] burst_idx;
    reg [5:0] beat_cnt;
    reg [4:0] emit_cnt;
    reg burst_bad, cancelled;
    reg [255:0] beat_mem[0:15];
    wire [255:0] cur_beat=beat_mem[emit_cnt[4:1]];
    wire [6:0] received={1'b0,beat_cnt}+rd_valid;
    assign data=emit_cnt[0] ? cur_beat[255:128] : cur_beat[127:0];
    assign busy=state!=S_IDLE;
    assign data_valid=(state==S_EMIT || state==S_ERROR) && !abort;
    assign data_error=state==S_ERROR;
    always @(posedge mem_clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=S_IDLE; base_hold<=0; burst_idx<=0; beat_cnt<=0; emit_cnt<=0;
            rd_req<=0; rd_addr<=0; done<=0; bursts_done<=0; error<=0; burst_bad<=0; cancelled<=0;
        end else begin
            done<=0;
            case(state)
                S_IDLE: if(start && !abort) begin
                    base_hold<=cfg_base_beat+slot_idx*cfg_slot_beats;
                    rd_addr<=cfg_base_beat+slot_idx*cfg_slot_beats;
                    burst_idx<=0; beat_cnt<=0; bursts_done<=0; error<=0;
                    burst_bad<=0; cancelled<=0; rd_req<=1; state<=S_COLLECT;
                end
                S_COLLECT: begin
                    if(abort) cancelled<=1;
                    if(rd_fault) burst_bad<=1;
                    if(rd_valid) begin
                        if(beat_cnt<BURST_BEATS) beat_mem[beat_cnt[3:0]]<=rd_data;
                        else burst_bad<=1;
                        if(beat_cnt<63) beat_cnt<=beat_cnt+1'b1;
                    end
                    // 同时处理空突发以及最后一拍与完成信号重合的情况。
                    if(rd_finish) begin
                        rd_req<=0; emit_cnt<=0;
                        if(cancelled || abort) state<=S_IDLE;
                        else if(received!=BURST_BEATS || burst_bad || rd_fault) begin
                            error<=1; state<=S_ERROR;
                        end else state<=S_EMIT;
                    end
                end
                S_EMIT: if(abort) state<=S_IDLE;
                else if(data_ready) begin
                    if(emit_cnt==BURST_BEATS*2-1) begin
                        bursts_done<=bursts_done+1'b1; beat_cnt<=0;
                        if(burst_idx==TOTAL_BEATS/BURST_BEATS-1) begin
                            done<=1; state<=S_IDLE;
                        end else begin
                            burst_idx<=burst_idx+1'b1;
                            rd_addr<=base_hold+(burst_idx+1'b1)*BURST_BEATS;
                            rd_req<=1; burst_bad<=0; state<=S_COLLECT;
                        end
                    end else emit_cnt<=emit_cnt+1'b1;
                end
                S_ERROR: if(abort) state<=S_IDLE;
                else if(data_ready) begin done<=1; state<=S_IDLE; end
                default: begin rd_req<=0; state<=S_IDLE; end
            endcase
        end
    end
endmodule
