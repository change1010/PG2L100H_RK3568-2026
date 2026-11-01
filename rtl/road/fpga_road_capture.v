`timescale 1ns/1ps
// 在像素域合并检测结果，按 16 位源帧序号配对。
// 两个检测器可能在同一 VS 边界前后的不同时钟拍发布结果。
module fpga_road_capture #(
    parameter integer WIDTH=1920, HEIGHT=1080, PERIOD=1
) (
    input clk, rst_n, boundary, de,
    input [31:0] source_epoch, source_sequence,
    input [63:0] source_timestamp,
    input lane_update, cross_update,
    input [1023:0] lane_result,
    input [127:0] cross_result,
    output reg record_valid,
    output reg [1535:0] record,
    input record_ready,
    output reg [31:0] dropped
);
    reg seen, selected, waiting, bad, de_d;
    reg [7:0] cadence;
    reg [12:0] column, lines;
    reg [31:0] epoch, sequence_id, pending_epoch, pending_sequence;
    reg [63:0] timestamp, pending_timestamp;
    reg [1023:0] lane_hold;
    reg [127:0] cross_hold;
    reg lane_ready, cross_ready;
    reg [15:0] lane_seq, cross_seq;
    wire complete = !bad && !de && !de_d && column==0 && lines==HEIGHT;

    wire [15:0] lane_update_seq=lane_result[255:240];
    wire [15:0] cross_update_seq=cross_result[127:112];
    wire lane_hold_match_boundary=lane_ready && lane_seq==sequence_id[15:0];
    wire cross_hold_match_boundary=cross_ready && cross_seq==sequence_id[15:0];
    wire lane_event_match_boundary=lane_update && lane_update_seq==sequence_id[15:0];
    wire cross_event_match_boundary=cross_update && cross_update_seq==sequence_id[15:0];
    wire boundary_lane_available=lane_hold_match_boundary || lane_event_match_boundary;
    wire boundary_cross_available=cross_hold_match_boundary || cross_event_match_boundary;
    wire [1023:0] boundary_lane_value=lane_event_match_boundary ? lane_result : lane_hold;
    wire [127:0] boundary_cross_value=cross_event_match_boundary ? cross_result : cross_hold;

    wire lane_hold_match_pending=lane_ready && lane_seq==pending_sequence[15:0];
    wire cross_hold_match_pending=cross_ready && cross_seq==pending_sequence[15:0];
    wire lane_event_match_pending=lane_update && lane_update_seq==pending_sequence[15:0];
    wire cross_event_match_pending=cross_update && cross_update_seq==pending_sequence[15:0];
    wire pending_lane_available=lane_hold_match_pending || lane_event_match_pending;
    wire pending_cross_available=cross_hold_match_pending || cross_event_match_pending;
    wire [1023:0] pending_lane_value=lane_event_match_pending ? lane_result : lane_hold;
    wire [127:0] pending_cross_value=cross_event_match_pending ? cross_result : cross_hold;

    function [1535:0] make_record;
        input [31:0] record_epoch, record_sequence;
        input [63:0] record_timestamp;
        input [1023:0] record_lane;
        input [127:0] record_cross;
        begin
            // 按小端 32 位字打包标识、接口版本、原生 epoch/帧序号及时间戳；
            // 源为 HDMI，附完成标志；车道 128 字节、斑马线 16 字节、保留 16 字节。
            make_record={128'd0,record_cross,record_lane,32'd3,32'd2,
                record_timestamp,record_sequence,record_epoch,32'd1,32'h31445246};
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seen<=0; selected<=0; waiting<=0;
            cadence<=0; column<=0; lines<=0; bad<=0; de_d<=0;
            epoch<=0; sequence_id<=0; timestamp<=0;
            pending_epoch<=0; pending_sequence<=0; pending_timestamp<=0;
            lane_hold<=0; cross_hold<=0; lane_ready<=0; cross_ready<=0;
            lane_seq<=0; cross_seq<=0;
            record<=0; record_valid<=0; dropped<=0;
        end else begin
            record_valid<=0; de_d<=de;
            if (boundary) begin
                if (waiting) dropped<=dropped+1'b1;
                if (seen && selected && complete) begin
                    pending_epoch<=epoch;
                    pending_sequence<=sequence_id;
                    pending_timestamp<=timestamp;
                    if (boundary_lane_available && boundary_cross_available) begin
                        if (record_ready) begin
                            record<=make_record(epoch,sequence_id,timestamp,
                                boundary_lane_value,boundary_cross_value);
                            record_valid<=1;
                        end else dropped<=dropped+1'b1;
                        waiting<=0;
                        lane_ready<=0;
                        cross_ready<=0;
                    end else waiting<=1;
                end else begin
                    waiting<=0;
                    if (seen && selected && !complete) dropped<=dropped+1'b1;
                end
                if (lane_update && !(seen && selected && complete &&
                                     boundary_lane_available && lane_event_match_boundary)) begin
                    lane_hold<=lane_result; lane_seq<=lane_update_seq; lane_ready<=1;
                end
                if (cross_update && !(seen && selected && complete &&
                                      boundary_cross_available && cross_event_match_boundary)) begin
                    cross_hold<=cross_result; cross_seq<=cross_update_seq; cross_ready<=1;
                end
                seen<=1; column<=0; lines<=0; bad<=0;
                epoch<=source_epoch; sequence_id<=source_sequence; timestamp<=source_timestamp;
                if (!seen || epoch!=source_epoch) begin
                    selected<=1; cadence<=PERIOD-1;
                end else if (cadence==0) begin selected<=1; cadence<=PERIOD-1; end
                else begin selected<=0; cadence<=cadence-1'b1; end
            end else begin
                if (de) begin
                    column<=column+1'b1;
                    if (column>=WIDTH || lines>=HEIGHT) bad<=1;
                end else column<=0;
                if (de_d && !de) begin
                    lines<=lines+1'b1;
                    if (column!=WIDTH) bad<=1;
                end
                if (waiting) begin
                    if (lane_update) begin
                        lane_hold<=lane_result; lane_seq<=lane_update_seq; lane_ready<=1;
                    end
                    if (cross_update) begin
                        cross_hold<=cross_result; cross_seq<=cross_update_seq; cross_ready<=1;
                    end
                    if (pending_lane_available && pending_cross_available) begin
                        waiting<=0;
                        if (record_ready) begin
                            record<=make_record(pending_epoch,pending_sequence,pending_timestamp,
                                pending_lane_value,pending_cross_value);
                            record_valid<=1;
                        end
                        else dropped<=dropped+1'b1;
                        lane_ready<=lane_update && !lane_event_match_pending;
                        cross_ready<=cross_update && !cross_event_match_pending;
                    end
                end else begin
                    if (lane_update) begin
                        lane_hold<=lane_result; lane_seq<=lane_update_seq; lane_ready<=1;
                    end
                    if (cross_update) begin
                        cross_hold<=cross_result; cross_seq<=cross_update_seq; cross_ready<=1;
                    end
                end
            end
        end
    end
endmodule
