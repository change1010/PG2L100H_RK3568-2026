`timescale 1ns / 1ps
module single_channel_fullres_display(
    input                video_clk,
    input                rst,
    input                display_enable,
    input      [2:0]      i_display_sel_mode,
    input                timing_hs,
    input                timing_vs,
    input                timing_de,
    output reg           ch0_read_req,
    input                ch0_read_req_ack,
    output               ch0_read_en,
    input      [15:0]     ch0_read_data,
    output reg           ch1_read_req,
    input                ch1_read_req_ack,
    output               ch1_read_en,
    input      [15:0]     ch1_read_data,
    output reg           ch2_read_req,
    input                ch2_read_req_ack,
    output               ch2_read_en,
    input      [15:0]     ch2_read_data,
    output reg           ch3_read_req,
    input                ch3_read_req_ack,
    output               ch3_read_en,
    input      [15:0]     ch3_read_data,
    output     [2:0]      o_display_sel,
    output               hs,
    output               vs,
    output               de,
    output reg [15:0]     vout_data
);

    reg [1:0] hs_pipe;
    reg [1:0] vs_pipe;
    reg [1:0] de_pipe;
    reg [11:0] out_x;
    reg [11:0] out_y;
    reg [15:0] line_buf [0:959];
    reg [15:0] line_pixel;
    reg [15:0] current_pixel;
    reg [9:0] source_x_d0;
    reg line_odd_d0;
    reg pixel_odd_d0;
    reg [1:0] ch0_ack_sync;
    reg [1:0] ch1_ack_sync;
    reg [1:0] ch2_ack_sync;
    reg [1:0] ch3_ack_sync;

    wire frame_start;
    wire read_pixel;
    wire [15:0] selected_data;
    wire valid_mode = (i_display_sel_mode <= 3'd3);

    assign hs = hs_pipe[1];
    assign vs = vs_pipe[1];
    assign de = de_pipe[1];
    assign o_display_sel = i_display_sel_mode;
    assign frame_start = vs_pipe[0] && !timing_vs;
    assign read_pixel = !rst && display_enable && valid_mode && timing_de && !out_y[0] && !out_x[0];
    assign ch0_read_en = read_pixel && (i_display_sel_mode == 3'd0);
    assign ch1_read_en = read_pixel && (i_display_sel_mode == 3'd1);
    assign ch2_read_en = !rst && display_enable && timing_de && (i_display_sel_mode == 3'd2);
    assign ch3_read_en = read_pixel && (i_display_sel_mode == 3'd3);
    assign selected_data = (i_display_sel_mode == 3'd0) ? ch0_read_data :
                           (i_display_sel_mode == 3'd1) ? ch1_read_data :
                           (i_display_sel_mode == 3'd2) ? ch2_read_data :
                           (i_display_sel_mode == 3'd3) ? ch3_read_data : 16'd0;

    always @(posedge video_clk or posedge rst) begin
        if (rst) begin
            hs_pipe <= 2'b00;
            vs_pipe <= 2'b00;
            de_pipe <= 2'b00;
            out_x <= 12'd0;
            out_y <= 12'd0;
            source_x_d0 <= 10'd0;
            line_odd_d0 <= 1'b0;
            pixel_odd_d0 <= 1'b0;
            ch0_ack_sync <= 2'b00;
            ch1_ack_sync <= 2'b00;
            ch2_ack_sync <= 2'b00;
            ch3_ack_sync <= 2'b00;
        end else begin
            hs_pipe <= {hs_pipe[0], timing_hs};
            vs_pipe <= {vs_pipe[0], timing_vs};
            de_pipe <= {de_pipe[0], timing_de};
            ch0_ack_sync <= {ch0_ack_sync[0], ch0_read_req_ack};
            ch1_ack_sync <= {ch1_ack_sync[0], ch1_read_req_ack};
            ch2_ack_sync <= {ch2_ack_sync[0], ch2_read_req_ack};
            ch3_ack_sync <= {ch3_ack_sync[0], ch3_read_req_ack};
            source_x_d0 <= out_x[10:1];
            line_odd_d0 <= out_y[0];
            pixel_odd_d0 <= out_x[0];
            if (timing_de)
                out_x <= out_x + 12'd1;
            else
                out_x <= 12'd0;
            if (frame_start)
                out_y <= 12'd0;
            else if (de_pipe[0] && !timing_de)
                out_y <= out_y + 12'd1;
        end
    end

    always @(posedge video_clk or posedge rst) begin
        if (rst) begin
            ch0_read_req <= 1'b0;
            ch1_read_req <= 1'b0;
            ch2_read_req <= 1'b0;
            ch3_read_req <= 1'b0;
        end else if (!display_enable || !valid_mode) begin
            ch0_read_req <= 1'b0;
            ch1_read_req <= 1'b0;
            ch2_read_req <= 1'b0;
            ch3_read_req <= 1'b0;
        end else if (frame_start) begin
            ch0_read_req <= (i_display_sel_mode == 3'd0);
            ch1_read_req <= (i_display_sel_mode == 3'd1);
            ch2_read_req <= (i_display_sel_mode == 3'd2);
            ch3_read_req <= (i_display_sel_mode == 3'd3);
        end else begin
            if (ch0_ack_sync[1]) ch0_read_req <= 1'b0;
            if (ch1_ack_sync[1]) ch1_read_req <= 1'b0;
            if (ch2_ack_sync[1]) ch2_read_req <= 1'b0;
            if (ch3_ack_sync[1]) ch3_read_req <= 1'b0;
        end
    end

    always @(posedge video_clk) begin
        if (!rst && display_enable && valid_mode && i_display_sel_mode != 3'd2 && timing_de && out_y[0])
            line_pixel <= line_buf[out_x[10:1]];
        if (!rst && display_enable && valid_mode && i_display_sel_mode != 3'd2 && de_pipe[0] && !line_odd_d0 && !pixel_odd_d0)
            line_buf[source_x_d0] <= selected_data;
    end

    always @(posedge video_clk or posedge rst) begin
        if (rst) begin
            current_pixel <= 16'd0;
            vout_data <= 16'd0;
        end else if (display_enable && valid_mode && de_pipe[0]) begin
            if (i_display_sel_mode == 3'd2)
                vout_data <= ch2_read_data;
            else if (line_odd_d0)
                vout_data <= line_pixel;
            else if (!pixel_odd_d0) begin
                current_pixel <= selected_data;
                vout_data <= selected_data;
            end else
                vout_data <= current_pixel;
        end else begin
            vout_data <= 16'd0;
        end
    end

endmodule
