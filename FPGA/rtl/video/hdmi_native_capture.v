`timescale 1ns/1ps
// 用消隐期内的 VS 上升沿划分帧，不依赖行计数生成帧起始。

module hdmi_native_capture #(
    parameter integer WIDTH = 1920, HEIGHT = 1080
) (
    input wire clk, rst_n, vs, de,
    input wire [15:0] pixel,
    output wire token_valid,
    input wire token_ready,
    output wire [314:0] token,
    output reg [31:0] rejected_frames,
    output frame_boundary, output [31:0] source_sequence, source_epoch,
    output [63:0] source_timestamp_ticks
);
    reg vs_d, de_d, armed, bad;
    reg [12:0] column, lines;
    reg [24:0] pixels, offset;
    reg [3:0] lane;
    reg [255:0] pack;
    reg [31:0] frame_sequence;
    reg [31:0] stream_epoch, frame_epoch;
    reg [63:0] ticks, frame_timestamp;
    reg [23:0] gap;
    reg lost;
    reg [31:0] left_crc;
    function [31:0] crc16;
        input [31:0] previous;
        input [15:0] value;
        reg [31:0] c;
        integer b;
        begin
            c=previous ^ {16'd0,value};
            for(b=0;b<16;b=b+1) c=c[0] ? (c>>1)^32'hedb88320 : c>>1;
            crc16=c;
        end
    endfunction
    wire boundary = vs && !vs_d;
    assign frame_boundary=boundary;
    assign source_sequence=boundary ? frame_sequence+1'b1 : frame_sequence;
    assign source_epoch=boundary ? stream_epoch : frame_epoch;
    assign source_timestamp_ticks=boundary ? ticks : frame_timestamp;
    wire word_valid = armed && de && !bad && pixels < WIDTH*HEIGHT && lane == 15;
    wire complete = !bad && !de && !de_d && pixels == WIDTH*HEIGHT &&
                    lines == HEIGHT && column == 0 && lane == 0;
    assign token_valid = (boundary && armed) || (!boundary && word_valid);
    // 打包格式：{类型(DATA=1/END=2), 帧序号, 拍偏移, 数据(END 的位 0 表示帧有效)}
    assign token = boundary ? {2'd2, frame_sequence, offset, 127'd0, ~left_crc, frame_timestamp, frame_epoch, complete} :
                             {2'd1, frame_sequence, offset, pixel, pack[239:0]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_d <= 0; de_d <= 0; armed <= 0; bad <= 0;
            column <= 0; lines <= 0; pixels <= 0; offset <= 0;
            lane <= 0; pack <= 0; frame_sequence <= 0; rejected_frames <= 0;
            stream_epoch<=1; frame_epoch<=1; ticks<=0; frame_timestamp<=0; gap<=0; lost<=0;
            left_crc<=32'hffffffff;
        end else begin
            vs_d <= vs; de_d <= de;
            ticks<=ticks+1'b1;
            if(de) begin
                gap<=0;
                if(lost) begin stream_epoch<=stream_epoch+1'b1;lost<=0;end
            end else if(gap<24'd14850000) gap<=gap+1'b1;
            else lost<=1;
            if (boundary) begin
                if (armed && (!complete || !token_ready))
                    rejected_frames <= rejected_frames + 1'b1;
                armed <= 1; bad <= de;
                column <= 0; lines <= 0; pixels <= 0; offset <= 0;
                lane <= 0; frame_sequence <= frame_sequence + 1'b1;
                frame_epoch<=stream_epoch; frame_timestamp<=ticks;
                left_crc<=32'hffffffff;
            end else if (armed) begin
                if(de && column<128 && column<WIDTH && lines<HEIGHT)
                    left_crc<=crc16(left_crc,pixel);
                if (de_d && !de) begin
                    if (column != WIDTH || lines >= HEIGHT) bad <= 1;
                    column <= 0;
                    if (lines <= HEIGHT) lines <= lines + 1'b1;
                end
                if (de) begin
                    if (column <= WIDTH) column <= column + 1'b1;
                    if (pixels <= WIDTH*HEIGHT) pixels <= pixels + 1'b1;
                    if (column >= WIDTH || lines >= HEIGHT || pixels >= WIDTH*HEIGHT)
                        bad <= 1;
                    if (!bad && pixels < WIDTH*HEIGHT) begin
                        pack[lane*16 +: 16] <= pixel;
                        lane <= lane + 1'b1;
                        if (lane == 15) begin
                            if (token_ready) offset <= offset + 1'b1;
                            else bad <= 1;
                        end
                    end
                end
            end
        end
    end
endmodule
