`ifndef LANE_GAP_EVIDENCE_V
`define LANE_GAP_EVIDENCE_V
// 原分辨率输入的8x2白漆证据位图，只有分类查询降采样，显示/轨迹坐标不缩放。
// 间隙取7个中心点：>=6点白漆则连接；<=1点白漆才允许作为虚线空隙。
// 中间状态不提供虚线证据；没有完整帧时拒绝使用未初始化RAM。
module lane_gap_evidence #(
    parameter WIDTH=1920,HEIGHT=1080,ROI_TOP=540,PREPARED_INPUTS=0
)(
    input clk,rst_n,frame_start,de,
    input [11:0] pixel_x,pixel_y,
    input paint,road,
    input start,
    input [79:0] a,b,
    output reg busy,done,road_gap,paint_gap,
    output reg [79:0] merged
);
    localparam XS=WIDTH>=960?3:0,YS=HEIGHT>=720?1:0;
    localparam XMASK=(1<<XS)-1,YMASK=(1<<YS)-1;
    localparam COLUMNS=(WIDTH+XMASK)>>XS,ROWS=(HEIGHT-ROI_TOP+YMASK)>>YS,CELLS=COLUMNS*ROWS;
    reg [1:0] paint_memory[0:CELLS-1];
    reg group_white,group_road,frame_complete,stored_ready;
    reg [2:0] state,sample;
    reg [3:0] hits,road_hits;
    reg known;
    reg signed [16:0] xp,yp,dx,dy;
    // 12位非负坐标/中心与有符号差均落在15位范围，避免查询启动扩成32位加减链。
    reg signed [14:0] upper_x,lower_x,upper_y,lower_y;
    integer read_x,read_y,read_address,write_address;
    reg [1:0] map_bit;
    reg read_ok;
    reg [79:0] upper,lower;
    // 调用者在start前至少保持输入两拍；利用关系核等待期提前准备查询几何。
    // 默认接口仍在start拍直接采样，查询握手、七个采样点及完成拍数不变。
    reg signed [14:0] upper_x_q,lower_x_q,upper_y_q,lower_y_q;
    reg signed [16:0] xp_q,yp_q,dx_q,dy_q;
    reg [79:0] merged_q,merged_qq;
    generate if(PREPARED_INPUTS) begin: prepared_geometry
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                upper_x_q<=0;lower_x_q<=0;upper_y_q<=0;lower_y_q<=0;
                xp_q<=0;yp_q<=0;dx_q<=0;dy_q<=0;merged_q<=0;merged_qq<=0;
            end else begin
                upper_x_q<=upper_x;lower_x_q<=lower_x;upper_y_q<=upper_y;lower_y_q<=lower_y;
                dx_q<=lower_x_q-upper_x_q;dy_q<=lower_y_q-upper_y_q;
                xp_q<=(upper_x_q<<3)+(lower_x_q-upper_x_q);
                yp_q<=(upper_y_q<<3)+(lower_y_q-upper_y_q);
                merged_q<={1'b1,1'b0,upper[77:54],lower[53:30],upper[29:18],lower[17:6],6'd0};
                merged_qq<=merged_q;
            end
        end
    end endgenerate
    always @* begin
        if(a[17:6]<b[29:18]) begin upper=a;lower=b;end
        else begin upper=b;lower=a;end
        upper_x=({1'b0,upper[53:42]}+upper[41:30])>>1;
        lower_x=({1'b0,lower[77:66]}+lower[65:54])>>1;
        upper_y=upper[17:6];lower_y=lower[29:18];
        read_x=xp>>>3;read_y=yp>>>3;
        read_address=((read_y-ROI_TOP)>>YS)*COLUMNS+(read_x>>XS);
        write_address=((pixel_y-ROI_TOP)>>YS)*COLUMNS+(pixel_x>>XS);
    end
    // 同步单bit读口，按上一拍查询捕获，避免组合RAM输出进入分类使能。
    always @(posedge clk) begin
        if(de&&pixel_y>=ROI_TOP&&pixel_y<HEIGHT&&(pixel_y&YMASK)==(ROI_TOP&YMASK)&&pixel_x<WIDTH) begin
            if((pixel_x&XMASK)==XMASK||pixel_x==WIDTH-1)
                paint_memory[write_address]<={XS==0?road:(group_road&road),XS==0?paint:(group_white|paint)};
        end
        if(state==1) begin
            read_ok<=read_x>=0&&read_x<WIDTH&&read_y>=ROI_TOP&&read_y<HEIGHT&&stored_ready;
            if(read_x>=0&&read_x<WIDTH&&read_y>=ROI_TOP&&read_y<HEIGHT&&stored_ready)
                map_bit<=paint_memory[read_address];
            else map_bit<=0;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;done<=0;road_gap<=0;paint_gap<=0;merged<=0;
            state<=0;sample<=0;hits<=0;road_hits<=0;known<=0;xp<=0;yp<=0;dx<=0;dy<=0;
            group_white<=0;group_road<=0;frame_complete<=0;stored_ready<=0;
        end else begin
            done<=0;
            if(de) begin
                if((pixel_x&XMASK)==0) begin group_white<=paint;group_road<=road;end
                else begin group_white<=group_white|paint;group_road<=group_road&road;end
                if(pixel_x==WIDTH-1&&pixel_y==HEIGHT-1) frame_complete<=1;
            end
            if(frame_start) begin stored_ready<=frame_complete;frame_complete<=0;group_white<=0;end
            if(start&&!busy) begin
                busy<=1;road_gap<=0;paint_gap<=0;sample<=1;hits<=0;road_hits<=0;known<=stored_ready;
                dx<=PREPARED_INPUTS?dx_q:(lower_x-upper_x);dy<=PREPARED_INPUTS?dy_q:(lower_y-upper_y);
                xp<=PREPARED_INPUTS?xp_q:((upper_x<<3)+(lower_x-upper_x));
                yp<=PREPARED_INPUTS?yp_q:((upper_y<<3)+(lower_y-upper_y));
                merged<=PREPARED_INPUTS?merged_qq:{1'b1,1'b0,upper[77:54],lower[53:30],upper[29:18],lower[17:6],6'd0};
                state<=1;
            end else if(state==1) state<=2;
            else if(state==2) begin
                if(map_bit[0]&&read_ok) hits<=hits+1'b1;
                if(map_bit[1]&&read_ok) road_hits<=road_hits+1'b1;
                if(!read_ok) known<=0;
                if(sample==7) state<=3;
                else begin sample<=sample+1'b1;xp<=xp+dx;yp<=yp+dy;state<=1;end
            end else if(state==3) begin
                paint_gap<=known&&hits>=6;road_gap<=known&&road_hits>=6&&hits<=1;
                busy<=0;done<=1;state<=0;
            end
        end
    end
endmodule
`endif
