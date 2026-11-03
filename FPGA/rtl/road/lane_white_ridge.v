`ifndef LANE_WHITE_RIDGE_V
`define LANE_WHITE_RIDGE_V
// 水平顶帽：灰度减去开运算，拒绝连入车道线的宽路面亮区。
// 每级只比较当前值和 2^级 个时钟前的值；窗口宽度为 2^LOG_WINDOW。
// 逐级流水，不使用乘法器；灰度、色差、DE、VS 与开运算中心严格对齐。
module lane_white_ridge #(parameter LOG_WINDOW=7)(
    input clk,rst_n,in_vs,in_de,
    input [7:0] in_gray,in_chroma,
    output out_vs,out_de,
    output [7:0] out_gray,out_chroma,
    output ridge,road_pixel,
    output reg out_advance /* synthesis syn_maxfan = 32 */
);
    localparam WINDOW=1<<LOG_WINDOW;
    localparam LATENCY=WINDOW-1+2*LOG_WINDOW;
    localparam FLANK=LOG_WINDOW>=7?44:8;
    wire [7:0] erosion[0:LOG_WINDOW],opening[0:LOG_WINDOW];
    reg [16*(LATENCY+FLANK)-1:0] original;
    reg [2*LATENCY-1:0] control;
    assign erosion[0]=in_gray;
    assign opening[0]=erosion[LOG_WINDOW];
    genvar stage;
    generate for(stage=0;stage<LOG_WINDOW;stage=stage+1) begin: levels
        lane_ridge_reduce #(.DELAY(1<<stage),.MAXIMUM(0)) erode(
            .clk(clk),.value(erosion[stage]),.reduced(erosion[stage+1]));
        lane_ridge_reduce #(.DELAY(1<<stage),.MAXIMUM(1)) dilate(
            .clk(clk),.value(opening[stage]),.reduced(opening[stage+1]));
    end endgenerate
    // 数据延时无复位：输入场消隐足够填满窗口后才进入有效像素。
    // 控制按相同延迟单独复位，避免复位后残留 VS 生成虚假的帧边界。
    always @(posedge clk) begin
        original<={original[16*(LATENCY+FLANK-1)-1:0],in_gray,in_chroma};
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) control<=0;
        else control<={control[2*(LATENCY-1)-1:0],in_vs,in_de};
    end
    // 从倒数第二级预译码下一拍队列使能；与原 out_de && !(out_vs && !旧out_vs) 同拍。
    // 高扇出队列直接由寄存器驱动，避免末级控制输出再串联帧起始组合译码。
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) out_advance<=0;
        else out_advance<=control[2*LATENCY-4]&&
            !(control[2*LATENCY-3]&&!control[2*LATENCY-1]);
    end
    assign {out_gray,out_chroma}=original[16*LATENCY-1-:16];
    assign {out_vs,out_de}=control[2*LATENCY-1-:2];
    wire [7:0] left_gray=original[16*(LATENCY+FLANK)-1-:8];
    wire [7:0] right_gray=original[16*(LATENCY-FLANK)-1-:8];
    wire [7:0] left_difference=out_gray>=left_gray?out_gray-left_gray:left_gray-out_gray;
    wire [7:0] right_difference=out_gray>=right_gray?out_gray-right_gray:right_gray-out_gray;
    // 路面证据与绝对白色阈值无关，阴影白漆仍需与两侧路面作比较。
    assign road_pixel=out_chroma<58&&left_difference<28&&right_difference<28;
    assign ridge={1'b0,out_gray}>={1'b0,opening[LOG_WINDOW]}+9'd15;
endmodule

module lane_ridge_reduce #(parameter DELAY=1,MAXIMUM=0)(
    input clk,input [7:0] value,output reg [7:0] reduced
);
    reg [8*DELAY-1:0] history;
    always @(posedge clk) begin
        history<=(history<<8)|value;
        if(MAXIMUM) reduced<=(value>history[8*DELAY-1-:8])?value:history[8*DELAY-1-:8];
        else reduced<=(value<history[8*DELAY-1-:8])?value:history[8*DELAY-1-:8];
    end
endmodule
`endif
