`timescale 1ns / 1ps

module quad_channel_display(
    input               video_clk,
    input               rst,
    input               display_enable,

    input               timing_hs,
    input               timing_vs,
    input               timing_de,

    output              ch0_read_req,
    input               ch0_read_req_ack,
    output              ch0_read_en,
    input      [15:0]   ch0_read_data,

    output              ch1_read_req,
    input               ch1_read_req_ack,
    output              ch1_read_en,
    input      [15:0]   ch1_read_data,

    output              ch2_read_req,
    input               ch2_read_req_ack,
    output              ch2_read_en,
    input      [15:0]   ch2_read_data,

    output              ch3_read_req,
    input               ch3_read_req_ack,
    output              ch3_read_en,
    input      [15:0]   ch3_read_data,

    output              hs,
    output              vs,
    output              de,
    output     [15:0]   vout_data
);

localparam [11:0] WIN_W = 12'd960;
localparam [11:0] WIN_H = 12'd540;

wire [15:0] bg_data;
wire [15:0] v0_data;
wire [15:0] v1_data;
wire [15:0] v2_data;

wire bg_hs;
wire bg_vs;
wire bg_de;
wire v1_hs;
wire v1_vs;
wire v1_de;
wire v2_hs;
wire v2_vs;
wire v2_de;

assign bg_data = 16'd0;

video_rect_read_data_m0 u_quad_win0 (
    .display_enable     (display_enable),
    .video_clk          (video_clk),
    .rst                (rst),
    .video_left_offset  (12'd0),
    .video_top_offset   (12'd0),
    .video_width        (WIN_W),
    .video_height       (WIN_H),
    .read_req           (ch0_read_req),
    .read_req_ack       (ch0_read_req_ack),
    .read_en            (ch0_read_en),
    .read_data          (ch0_read_data),
    .timing_hs          (timing_hs),
    .timing_vs          (timing_vs),
    .timing_de          (timing_de),
    .timing_data        (bg_data),
    .hs                 (bg_hs),
    .vs                 (bg_vs),
    .de                 (bg_de),
    .vout_data          (v0_data)
);

video_rect_read_data_m1 u_quad_win1 (
    .display_enable     (display_enable),
    .video_clk          (video_clk),
    .rst                (rst),
    .video_left_offset  (12'd960),
    .video_top_offset   (12'd0),
    .video_width        (WIN_W),
    .video_height       (WIN_H),
    .read_req           (ch1_read_req),
    .read_req_ack       (ch1_read_req_ack),
    .read_en            (ch1_read_en),
    .read_data          (ch1_read_data),
    .timing_hs          (bg_hs),
    .timing_vs          (bg_vs),
    .timing_de          (bg_de),
    .timing_data        (v0_data),
    .hs                 (v1_hs),
    .vs                 (v1_vs),
    .de                 (v1_de),
    .vout_data          (v1_data)
);

video_rect_read_data_m1 u_quad_win2 (
    .display_enable     (display_enable),
    .video_clk          (video_clk),
    .rst                (rst),
    .video_left_offset  (12'd0),
    .video_top_offset   (12'd540),
    .video_width        (WIN_W),
    .video_height       (WIN_H),
    .read_req           (ch2_read_req),
    .read_req_ack       (ch2_read_req_ack),
    .read_en            (ch2_read_en),
    .read_data          (ch2_read_data),
    .timing_hs          (v1_hs),
    .timing_vs          (v1_vs),
    .timing_de          (v1_de),
    .timing_data        (v1_data),
    .hs                 (v2_hs),
    .vs                 (v2_vs),
    .de                 (v2_de),
    .vout_data          (v2_data)
);

video_rect_read_data_m1 u_quad_win3 (
    .display_enable     (display_enable),
    .video_clk          (video_clk),
    .rst                (rst),
    .video_left_offset  (12'd960),
    .video_top_offset   (12'd540),
    .video_width        (WIN_W),
    .video_height       (WIN_H),
    .read_req           (ch3_read_req),
    .read_req_ack       (ch3_read_req_ack),
    .read_en            (ch3_read_en),
    .read_data          (ch3_read_data),
    .timing_hs          (v2_hs),
    .timing_vs          (v2_vs),
    .timing_de          (v2_de),
    .timing_data        (v2_data),
    .hs                 (hs),
    .vs                 (vs),
    .de                 (de),
    .vout_data          (vout_data)
);

endmodule
