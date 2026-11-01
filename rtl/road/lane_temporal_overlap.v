`ifndef LANE_TEMPORAL_OVERLAP_V
`define LANE_TEMPORAL_OVERLAP_V
// 阴影改变可见起止行时，在共同可见行比较位置，而非直接比较不同高度的端点。
// 只扩展长段确认：至少32行重叠、同向、斜率比<=2、公共中间行位置相差<=24px。
// 四路移位累加+两路恢复除法，零APM；只在帧尾空闲期工作，不预测/保留已消失的段。
module lane_temporal_overlap #(parameter WIDTH=1920, PREPARED_INPUTS=0)(
    input clk,rst_n,start,valid_pair,
    input [79:0] a,b,
    output reg busy,done,aligned
);
    reg signed [14:0] at,ab,bt,bb,sa,sb,da,db,lo,hi,mid;
    // 可选提前计算：调用者须保证a/b在start前至少一拍稳定。
    // 主检测器在关系核完成后才启动本核，利用既有等待时间，不增加查询拍数。
    reg signed [14:0] at_q,bt_q,sa_q,sb_q,da_q,db_q,mid_q;
    reg overlap_q,direction_q;
    wire signed [14:0] use_at=PREPARED_INPUTS?at_q:at;
    wire signed [14:0] use_bt=PREPARED_INPUTS?bt_q:bt;
    wire signed [14:0] use_sa=PREPARED_INPUTS?sa_q:sa;
    wire signed [14:0] use_sb=PREPARED_INPUTS?sb_q:sb;
    wire signed [14:0] use_da=PREPARED_INPUTS?da_q:da;
    wire signed [14:0] use_db=PREPARED_INPUTS?db_q:db;
    wire signed [14:0] use_mid=PREPARED_INPUTS?mid_q:mid;
    wire use_overlap=PREPARED_INPUTS?overlap_q:(hi-lo>=32);
    wire use_direction=PREPARED_INPUTS?direction_q:((ab<at&&bb<bt)||(ab>at&&bb>bt));
    integer i;
    localparam TOL=(WIDTH/80>3)?WIDTH/80:3;
    reg [25:0] acc[0:3],shifted[0:3];
    reg [12:0] factor[0:3];
    reg [11:0] top_a,top_b,span_a,span_b;
    reg negative_a,negative_b;
    reg [12:0] remainder_a,remainder_b;
    reg [5:0] step;
    reg dividing;
    wire [12:0] trial_a={remainder_a[11:0],acc[0][25]};
    wire [12:0] trial_b={remainder_b[11:0],acc[1][25]};
    wire [12:0] point_a=negative_a?{1'b0,top_a}-acc[0][12:0]:{1'b0,top_a}+acc[0][12:0];
    wire [12:0] point_b=negative_b?{1'b0,top_b}-acc[1][12:0]:{1'b0,top_b}+acc[1][12:0];
    wire [12:0] point_error=point_a>=point_b?point_a-point_b:point_b-point_a;
    always @* begin
        at=({1'b0,a[77:66]}+a[65:54])>>1;ab=({1'b0,a[53:42]}+a[41:30])>>1;
        bt=({1'b0,b[77:66]}+b[65:54])>>1;bb=({1'b0,b[53:42]}+b[41:30])>>1;
        sa=a[17:6]-a[29:18];sb=b[17:6]-b[29:18];da=ab-at;db=bb-bt;
        lo=a[29:18]>b[29:18]?a[29:18]:b[29:18];
        hi=a[17:6]<b[17:6]?a[17:6]:b[17:6];mid=(lo+hi)>>1;
    end
    generate if(PREPARED_INPUTS) begin: prepared_geometry
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                at_q<=0;bt_q<=0;sa_q<=0;sb_q<=0;da_q<=0;db_q<=0;mid_q<=0;
                overlap_q<=0;direction_q<=0;
            end else begin
                at_q<=at;bt_q<=bt;sa_q<=sa;sb_q<=sb;da_q<=da;db_q<=db;mid_q<=mid;
                overlap_q<=hi-lo>=32;
                direction_q<=(ab<at&&bb<bt)||(ab>at&&bb>bt);
            end
        end
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;done<=0;aligned<=0;step<=0;dividing<=0;
            top_a<=0;top_b<=0;span_a<=0;span_b<=0;negative_a<=0;negative_b<=0;
            remainder_a<=0;remainder_b<=0;
            for(i=0;i<4;i=i+1) begin acc[i]<=0;shifted[i]<=0;factor[i]<=0;end
        end else begin
            done<=0;
            if(start&&!busy) begin
                aligned<=0;step<=0;dividing<=0;
                // 无条件初始化内部算术寄存器；有效性只控制busy/done，避免长判定驱动数百CE。
                top_a<=use_at;top_b<=use_bt;span_a<=use_sa;span_b<=use_sb;
                negative_a<=use_da<0;negative_b<=use_db<0;
                shifted[0]<=use_da<0?-use_da:use_da;factor[0]<=use_mid-a[29:18];
                shifted[1]<=use_db<0?-use_db:use_db;factor[1]<=use_mid-b[29:18];
                shifted[2]<=use_da<0?-use_da:use_da;factor[2]<=use_sb;
                shifted[3]<=use_db<0?-use_db:use_db;factor[3]<=use_sa;
                for(i=0;i<4;i=i+1) acc[i]<=0;
                if(valid_pair&&a[79]&&b[79]&&use_sa>=32&&use_sb>=32&&use_overlap&&use_direction) busy<=1;
                else begin busy<=0;done<=1;end
            end else if(busy) begin
                if(!dividing) begin
                    if(step<13) begin
                        for(i=0;i<4;i=i+1) begin
                            if(factor[i][0]) acc[i]<=acc[i]+shifted[i];
                            factor[i]<=factor[i]>>1;shifted[i]<=shifted[i]<<1;
                        end
                        step<=step+1'b1;
                    end else begin
                        dividing<=1;step<=0;remainder_a<=0;remainder_b<=0;
                    end
                end else if(step<26) begin
                    remainder_a<=trial_a>=span_a?trial_a-span_a:trial_a;
                    remainder_b<=trial_b>=span_b?trial_b-span_b:trial_b;
                    acc[0]<={acc[0][24:0],trial_a>=span_a};
                    acc[1]<={acc[1][24:0],trial_b>=span_b};
                    step<=step+1'b1;
                end else begin
                    aligned<=point_error<=TOL&&{1'b0,acc[2]}<=({1'b0,acc[3]}<<1)&&
                             {1'b0,acc[3]}<=({1'b0,acc[2]}<<1);
                    busy<=0;done<=1;
                end
            end
        end
    end
endmodule
`endif
