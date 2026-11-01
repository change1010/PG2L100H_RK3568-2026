`timescale 1ns/1ps
// 改编自官方 2_Demo： 3_2_HDMI_IN_DDR3_YCbCr/3_2_HDMI_IN_DDR3_YCbCr/HDMI_IN_DDR3_YCbCr/source/img_process/RGB2YCbCr.v
module stage1_rgb_to_ycbcr
(

    input               clk             ,
    input               rst_n           ,

    input               vsync_in ,
    input               hsync_in ,
    input               de_in    ,
    input       [4:0]   red         ,
    input       [5:0]   green       ,
    input       [4:0]   blue        ,

    output              vsync_out,
    output              hsync_out,
    output              de_out   ,
    output      [7:0]   y           ,
    output      [7:0]   cb          ,
    output      [7:0]   cr
);

reg  [15:0]   rgb_r_m0, rgb_r_m1, rgb_r_m2;
reg  [15:0]   rgb_g_m0, rgb_g_m1, rgb_g_m2;
reg  [15:0]   rgb_b_m0, rgb_b_m1, rgb_b_m2;
reg  [15:0]   y0 ;
reg  [15:0]   cb0;
reg  [15:0]   cr0;
reg  [ 7:0]   y1 ;
reg  [ 7:0]   cb1;
reg  [ 7:0]   cr1;
reg  [ 2:0]   vsync_in_d;
reg  [ 2:0]   hsync_in_d;
reg  [ 2:0]   de_in_d   ;

wire [ 7:0]   rgb888_r;
wire [ 7:0]   rgb888_g;
wire [ 7:0]   rgb888_b;

assign rgb888_r         = {red  , red[4:2]  };
assign rgb888_g         = {green, green[5:4]};
assign rgb888_b         = {blue , blue[4:2] };

assign vsync_out = vsync_in_d[2]      ;
assign hsync_out = hsync_in_d[2]      ;
assign de_out    = de_in_d[2]         ;
assign y            = de_out ? y1 : 8'd0;
assign cb           = de_out ? cb1: 8'd0;
assign cr           = de_out ? cr1: 8'd0;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rgb_r_m0 <= 16'd0;
        rgb_r_m1 <= 16'd0;
        rgb_r_m2 <= 16'd0;
        rgb_g_m0 <= 16'd0;
        rgb_g_m1 <= 16'd0;
        rgb_g_m2 <= 16'd0;
        rgb_b_m0 <= 16'd0;
        rgb_b_m1 <= 16'd0;
        rgb_b_m2 <= 16'd0;
    end
    else begin
        rgb_r_m0 <= rgb888_r * 8'd77 ;
        rgb_r_m1 <= rgb888_r * 8'd43 ;
        rgb_r_m2 <= rgb888_r << 3'd7 ;
        rgb_g_m0 <= rgb888_g * 8'd150;
        rgb_g_m1 <= rgb888_g * 8'd85 ;
        rgb_g_m2 <= rgb888_g * 8'd107;
        rgb_b_m0 <= rgb888_b * 8'd29 ;
        rgb_b_m1 <= rgb888_b << 3'd7 ;
        rgb_b_m2 <= rgb888_b * 8'd21 ;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        y0  <= 16'd0;
        cb0 <= 16'd0;
        cr0 <= 16'd0;
    end
    else begin
        y0  <= rgb_r_m0 + rgb_g_m0 + rgb_b_m0;
        cb0 <= rgb_b_m1 - rgb_r_m1 - rgb_g_m1 + 16'd32768;
        cr0 <= rgb_r_m2 - rgb_g_m2 - rgb_b_m2 + 16'd32768;
    end

end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        y1  <= 8'd0;
        cb1 <= 8'd0;
        cr1 <= 8'd0;
    end
    else begin
        y1  <= y0 [15:8];
        cb1 <= cb0[15:8];
        cr1 <= cr0[15:8];
    end
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        vsync_in_d <= 3'd0;
        hsync_in_d <= 3'd0;
        de_in_d    <= 3'd0;
    end
    else begin
        vsync_in_d <= {vsync_in_d[1:0], vsync_in};
        hsync_in_d <= {hsync_in_d[1:0], hsync_in};
        de_in_d    <= {de_in_d[1:0]   , de_in   };
    end
end

endmodule
