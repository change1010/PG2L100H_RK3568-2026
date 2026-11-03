`timescale 1ns/1ps
// 改编自官方 2_Demo： 3_4_HDMI_IN_DDR3_bin/3_4_HDMI_IN_DDR3_bin/HDMI_IN_DDR3_bin/source/img_process/binarization.v
module stage1_binarization(

    input               clk             ,
    input               rst_n           ,

    input               vsync_in     ,
    input               hsync_in     ,
    input               de_in        ,
    input   [7:0]       y_in       ,

    output              vsync_out      ,
    output              hsync_out      ,
    output              de_out         ,
    output   reg        pix
);

reg    vsync_in_d;
reg    hsync_in_d;
reg    de_in_d   ;

parameter Binar_THRESHOLD = 128;

assign  vsync_out = vsync_in_d  ;
assign  hsync_out = hsync_in_d  ;
assign  de_out    = de_in_d     ;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        pix <= 1'b0;
    else if(y_in > Binar_THRESHOLD)
        pix <= 1'b1;
    else
        pix <= 1'b0;
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        vsync_in_d <= 1'd0;
        hsync_in_d <= 1'd0;
        de_in_d    <= 1'd0;
    end
    else begin
        vsync_in_d <= vsync_in;
        hsync_in_d <= hsync_in;
        de_in_d    <= de_in   ;
    end
end

endmodule
