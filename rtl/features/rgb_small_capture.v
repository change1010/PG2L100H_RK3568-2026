`timescale 1ns/1ps
// 从公共 RGB565 画布抽样并打包 RGB888，不阻塞上游视频。
// END 校验完整源帧，包括最后一个抽样点之后的像素。
module rgb_small_capture #(
    parameter integer WIDTH = 1920, HEIGHT = 1080
) (
    input clk, rst_n, boundary, de, invalid,
    input [15:0] pixel,
    input enable, input [3:0] period_div,
    input [31:0] source_sequence, source_epoch, source_config, image_generation,
    input [2:0] source_layout,
    output token_valid, input token_ready, output [381:0] token,
    output reg [31:0] rejected_frames
);
    localparam integer FRAME_BYTES = (WIDTH/3)*(HEIGHT/3)*3;
    reg active, bad, de_d;
    reg [12:0] column, lines;
    reg [24:0] pixels, offset;
    reg [1:0] x_phase, y_phase;
    reg [5:0] fill;
    reg [255:0] pack;
    reg [31:0] seq_hold, epoch_hold, config_hold, image_hold;
    reg [2:0] layout_hold;
    (* ASYNC_REG = "TRUE" *) reg [1:0] enable_sync;
    (* ASYNC_REG = "TRUE" *) reg [3:0] period_s1, period_s2;
    reg [3:0] skipped;
    wire identity_bad = source_config != config_hold ||
        image_generation != image_hold || source_layout != layout_hold;
    wire sample = active && !bad && !identity_bad && !invalid && de &&
        column < WIDTH && lines < HEIGHT && x_phase == 1 && y_phase == 1;
    wire [23:0] rgb = {{pixel[4:0],pixel[4:2]},
                       {pixel[10:5],pixel[10:9]},
                       {pixel[15:11],pixel[15:13]}};
    wire [279:0] merged = {24'd0,pack} | ({256'd0,rgb} << (fill*8));
    wire word_valid = sample && fill >= 29;
    wire complete = !bad && !de && !de_d && pixels == WIDTH*HEIGHT &&
        column == 0 && lines == HEIGHT && fill == 0 && offset == FRAME_BYTES/32;
    assign token_valid = boundary ? active : word_valid;
    // 记录类型为 DATA=1/END=2，并携带完整帧标识、32 字节偏移及数据或帧有效位。
    assign token = {boundary ? 2'd2 : 2'd1, seq_hold, epoch_hold, config_hold,
        layout_hold, offset, boundary ? {255'd0,complete} : merged[255:0]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active<=0; bad<=0; de_d<=0; column<=0; lines<=0;
            pixels<=0; offset<=0; x_phase<=0; y_phase<=0; fill<=0; pack<=0;
            seq_hold<=0; epoch_hold<=0; config_hold<=0; image_hold<=0; layout_hold<=0;
            enable_sync<=0; period_s1<=2; period_s2<=2; skipped<=0; rejected_frames<=0;
        end else begin
            enable_sync<={enable_sync[0],enable}; period_s1<=period_div; period_s2<=period_s1;
            de_d<=de;
            if (boundary) begin
                if (active && (!complete || !token_ready))
                    rejected_frames<=rejected_frames+1'b1;
                active<=enable_sync[1] && skipped==0;
                if (!enable_sync[1]) skipped<=0;
                else if (skipped!=0) skipped<=skipped-1'b1;
                else skipped<=period_s2<=1 ? 0 : period_s2-1'b1;
                bad<=de; column<=0; lines<=0; pixels<=0; offset<=0;
                x_phase<=0; y_phase<=0; fill<=0; pack<=0;
                seq_hold<=source_sequence; epoch_hold<=source_epoch;
                config_hold<=source_config; image_hold<=image_generation; layout_hold<=source_layout;
            end else if (active) begin
                if (identity_bad || (invalid && de)) bad<=1;
                if (de_d && !de) begin
                    if (column!=WIDTH || lines>=HEIGHT) bad<=1;
                    column<=0; x_phase<=0;
                    if (lines<=HEIGHT) lines<=lines+1'b1;
                    y_phase<=y_phase==2 ? 0 : y_phase+1'b1;
                end
                if (de) begin
                    if (column<=WIDTH) column<=column+1'b1;
                    if (pixels<=WIDTH*HEIGHT) pixels<=pixels+1'b1;
                    x_phase<=x_phase==2 ? 0 : x_phase+1'b1;
                    if (column>=WIDTH || lines>=HEIGHT || pixels>=WIDTH*HEIGHT) bad<=1;
                end
                if (sample) begin
                    if (word_valid) begin
                        pack<={232'd0,merged[279:256]}; fill<=fill-29;
                        if (token_ready) offset<=offset+1'b1;
                        else bad<=1;
                    end else begin pack<=merged[255:0]; fill<=fill+3; end
                end
            end
        end
    end
`ifndef SYNTHESIS
    initial if (WIDTH%3!=0 || HEIGHT%3!=0 || FRAME_BYTES%512!=0)
        $fatal(1,"RGB geometry must form whole 16-beat DDR bursts");
`endif
endmodule
