`ifndef LANE_CONTOUR_SAMPLES_V
`define LANE_CONTOUR_SAMPLES_V
// 每条活动轨迹按32行分桶保留真实中心点；同步读写RAM，不复位整块存储器。
// 起始行及帧标记防止已释放轨迹/上帧的样本混入当前轮廓。
module lane_contour_samples #(parameter ROI_TOP=540)(
    input clk,write_enable,
    input [3:0] write_slot,epoch,read_slot,
    input [11:0] write_start,write_y,write_center,read_mid_y,
    output reg [39:0] sample_data
);
    wire [11:0] write_row=write_y-ROI_TOP,read_row=read_mid_y-ROI_TOP;
    wire [8:0] wa={write_slot,write_row[9:5]},ra={read_slot,read_row[9:5]};
    reg [39:0] memory[0:511];
    always @(posedge clk) begin
        if(write_enable) memory[wa]<={epoch,write_start,write_y,write_center};
        sample_data<=memory[ra];
    end
endmodule
`endif
