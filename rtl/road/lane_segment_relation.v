`ifndef LANE_SEGMENT_RELATION_V
`define LANE_SEGMENT_RELATION_V
// 帧尾串行几何核：用双向直线外推代替粗斜率分桶，不消耗 APM。
// 同时算十个13位移位加法乘积，每次关系判断固定16拍。
module lane_segment_relation #(
    parameter WIDTH=1920, HEIGHT=1080, MIN_GAP=8
)(
    input clk,rst_n,start,valid_pair,
    input [79:0] a,b,
    output reg busy,done,aligned
);
    reg [25:0] acc[0:9],shifted[0:9];
    reg [12:0] multiplier[0:9];
    reg [4:0] step;
    reg plausible;
    reg valid_q,central_a,central_b;
    reg signed [14:0] sa_q,sb_q,da_q,db_q,g_q,dx_q,wa_q,wb_q;
    integer i;
    // 12位坐标的中心/差/线宽和最大8192，15位有符号数可完整表达。
    // 避免integer把几何和容差链扩成32位；乘4的最大绝对值16380仍在15位范围。
    reg signed [14:0] at,ab,bt,bb,sa,sb,da,db,g,dx,wa,wb,tol;
    reg ordered_a;
    reg [25:0] error_a,error_b,slope_error;
    always @* begin
        at=({1'b0,a[77:66]}+{1'b0,a[65:54]})>>1;
        ab=({1'b0,a[53:42]}+{1'b0,a[41:30]})>>1;
        bt=({1'b0,b[77:66]}+{1'b0,b[65:54]})>>1;
        bb=({1'b0,b[53:42]}+{1'b0,b[41:30]})>>1;
        sa=a[17:6]-a[29:18];sb=b[17:6]-b[29:18];
        da=ab-at;db=bb-bt;
        ordered_a=a[17:6]<b[29:18];
        if(ordered_a) begin g=b[29:18]-a[17:6];dx=bt-ab;end
        else begin g=a[29:18]-b[17:6];dx=at-bb;end
        wa=((a[65:54]-a[77:66]+1)+(a[41:30]-a[53:42]+1))>>1;
        wb=((b[65:54]-b[77:66]+1)+(b[41:30]-b[53:42]+1))>>1;
        // 启动拍先保存两段平均宽，原step0再求和限幅，握手及乘积拍数不变。
        tol=wa_q+wb_q;
        if(tol<WIDTH/192) tol=WIDTH/192;
        if(tol<3) tol=3;
        error_a=(acc[2]>=acc[3])?acc[2]-acc[3]:acc[3]-acc[2];
        error_b=(acc[4]>=acc[5])?acc[4]-acc[5]:acc[5]-acc[4];
        slope_error=(acc[0]>=acc[1])?acc[0]-acc[1]:acc[1]-acc[0];
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;done<=0;aligned<=0;step<=0;plausible<=0;
            valid_q<=0;central_a<=0;central_b<=0;sa_q<=0;sb_q<=0;
            da_q<=0;db_q<=0;g_q<=0;dx_q<=0;wa_q<=0;wb_q<=0;
            for(i=0;i<10;i=i+1) begin acc[i]<=0;shifted[i]<=0;multiplier[i]<=0;end
        end else begin
            done<=0;
            if(start&&!busy) begin
                busy<=1;aligned<=0;step<=0;
                valid_q<=valid_pair;central_a<=at>=WIDTH*43/100&&at<=WIDTH*65/100;
                central_b<=bt>=WIDTH*43/100&&bt<=WIDTH*65/100;
                sa_q<=sa;sb_q<=sb;da_q<=da;db_q<=db;g_q<=g;dx_q<=dx;wa_q<=wa;wb_q<=wb;
                for(i=0;i<10;i=i+1) acc[i]<=0;
            end else if(busy) begin
                if(step==0) begin
                plausible<=valid_q&&sa_q>0&&sb_q>0&&g_q>=MIN_GAP&&g_q<=HEIGHT*23/100&&
                    ((da_q>0&&db_q>0&&dx_q>0)||(da_q<0&&db_q<0&&dx_q<0)||
                     (central_a&&central_b&&
                      (da_q<<<2)>=-sa_q&&(da_q<<<2)<=sa_q&&(db_q<<<2)>=-sb_q&&(db_q<<<2)<=sb_q&&dx_q>=-tol&&dx_q<=tol));
                shifted[0]<=(da_q<0)?-da_q:da_q; multiplier[0]<=sb_q;
                shifted[1]<=(db_q<0)?-db_q:db_q; multiplier[1]<=sa_q;
                shifted[2]<=(dx_q<0)?-dx_q:dx_q; multiplier[2]<=sa_q;
                shifted[3]<=(da_q<0)?-da_q:da_q; multiplier[3]<=g_q;
                shifted[4]<=(dx_q<0)?-dx_q:dx_q; multiplier[4]<=sb_q;
                shifted[5]<=(db_q<0)?-db_q:db_q; multiplier[5]<=g_q;
                shifted[6]<=tol;multiplier[6]<=sa_q;
                shifted[7]<=tol;multiplier[7]<=sb_q;
                shifted[8]<=sa_q;multiplier[8]<=sb_q;
                shifted[9]<=(da_q<0)?-da_q:da_q;multiplier[9]<=(db_q<0)?-db_q:db_q;
                    step<=1;
                end else if(step<13) begin
                    for(i=0;i<10;i=i+1) begin
                        if(multiplier[i][0]) acc[i]<=acc[i]+shifted[i];
                        shifted[i]<=shifted[i]<<1;multiplier[i]<=multiplier[i]>>1;
                    end
                    step<=step+1'b1;
                end else begin
                    // 两端都外推到对方近端，避免邻近另一车道或箭头被宽松配对。
                    aligned<=plausible&&error_a<=({1'b0,acc[6]}<<1)&&
                        error_b<=({1'b0,acc[7]}<<1)&&
                        ({2'b0,slope_error}<<2)<=({2'b0,acc[8]}+acc[9]);
                    busy<=0;done<=1;
                end
            end
        end
    end
endmodule
`endif
