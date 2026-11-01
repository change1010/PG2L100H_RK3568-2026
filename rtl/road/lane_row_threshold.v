`ifndef LANE_ROW_THRESHOLD_V
`define LANE_ROW_THRESHOLD_V
// 32桶行灰度中位数，阈值=clip(上一有效行中位桶中心+22,95,190)。
// 水平消隐内扫描并清空直方图；不缩放、不丢有效像素、不缓存整帧。
// 这是在线近似；不是 OpenCV 当前行256级精确中位数的等价移植。
module lane_row_threshold #(parameter INITIAL_THRESHOLD=170)(
    input clk,rst_n,frame_start,de,
    input [7:0] gray,
    output reg [7:0] threshold,
    output reg ready
);
    reg [11:0] histogram[0:31];
    reg [12:0] pixels,cumulative;
    reg [4:0] scan;
    reg scanning,found,de_q;
    reg [8:0] next_floor;
    integer i;
    always @* begin
        next_floor={1'b0,scan,3'b100}+9'd22;
        if(next_floor<95) next_floor=95;
        if(next_floor>190) next_floor=190;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            threshold<=INITIAL_THRESHOLD;ready<=0;pixels<=0;cumulative<=0;
            scan<=0;scanning<=0;found<=0;de_q<=0;
            for(i=0;i<32;i=i+1) histogram[i]<=0;
        end else begin
            de_q<=de;
            if(frame_start) begin
                threshold<=INITIAL_THRESHOLD;ready<=0;pixels<=0;cumulative<=0;
                scan<=0;scanning<=0;found<=0;
                for(i=0;i<32;i=i+1) histogram[i]<=0;
            end else if(de) begin
                // 异常短消隐不应用未完成统计，继续使用上一次可靠阈值。
                if(!scanning) begin
                    histogram[gray[7:3]]<=histogram[gray[7:3]]+1'b1;
                    pixels<=pixels+1'b1;
                end
            end else if(de_q) begin
                scanning<=1;scan<=0;cumulative<=0;found<=0;
            end else if(scanning) begin
                cumulative<=cumulative+histogram[scan];
                if(!found&&pixels!=0&&
                   cumulative+histogram[scan]>(pixels>>1)) begin
                    threshold<=next_floor[7:0];ready<=1;found<=1;
                end
                histogram[scan]<=0;
                if(scan==31) begin scanning<=0;pixels<=0;end
                else scan<=scan+1'b1;
            end
        end
    end
endmodule
`endif
