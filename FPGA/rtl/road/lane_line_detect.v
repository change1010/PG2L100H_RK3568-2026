`timescale 1ns/1ps
`ifndef LANE_LINE_DETECT_V
`define LANE_LINE_DETECT_V
// 多条白色窄脊独立跟踪；消隐期筛选、分类，帧间只确认当前可见证据。
// 不使用乘法器/除法器/整帧缓存。低256位保留诊断摘要。
// [256+80*k+:80]={valid,dashed,top_left,top_right,bottom_left,bottom_right,y0,y1,knee_dy}
// [900+12*k+:12]=实线中部真实中心x，0表示无折点；knee_dy为相对中间行的6位有符号偏移。
module lane_line_detect #(
    parameter WIDTH=1920,HEIGHT=1080,ROI_TOP=540,BAND_H=8,
    parameter WHITE_TH=170,CHROMA_MAX=55,MIN_RUN=3,MAX_RUN=80,
    parameter MIN_ROWS=10,MIN_BANDS=2,MAX_DRIFT=4,MIN_SHIFT=4,
    parameter MAX_MISS=2,DASH_MISS=8,CONFIRM_FRAMES=2,ADAPTIVE_WHITE=1
)(
    input clk,rst_n,vsync,de,
    input [31:0] source_sequence,
    input [23:0] rgb,
    output reg [1023:0] result,
    output reg result_update
);
    localparam N=8,NT=16,MIN_SPAN=BAND_H*MIN_BANDS;
    localparam WIDE_DIST=(WIDTH>=1920)?32:((WIDTH>=960)?16:8);
    localparam SOLID_SPAN=(HEIGHT/12>MIN_SPAN)?HEIGHT/12:MIN_SPAN;
    reg vs_pre,de_pre;
    reg [7:0] gray_pre,chroma_pre;
    wire vs_q,de_q,ridge,road_pixel,candidate_advance;
    wire [7:0] gray_q,chroma_q;
    localparam RIDGE_LOG_WINDOW=WIDTH>=960?7:4;
    localparam RIDGE_LATENCY=(1<<RIDGE_LOG_WINDOW)-1+2*RIDGE_LOG_WINDOW;
    lane_white_ridge #(.LOG_WINDOW(RIDGE_LOG_WINDOW)) u_ridge(
        .clk(clk),.rst_n(rst_n),.in_vs(vs_pre),.in_de(de_pre),
        .in_gray(gray_pre),.in_chroma(chroma_pre),.out_vs(vs_q),.out_de(de_q),
        .out_gray(gray_q),.out_chroma(chroma_q),.ridge(ridge),.road_pixel(road_pixel),
        .out_advance(candidate_advance));
    wire [9:0] sum={2'b0,rgb[23:16]}+{1'b0,rgb[15:8],1'b0}+{2'b0,rgb[7:0]};
    wire [7:0] vmax=(rgb[23:16]>rgb[15:8]) ?
        ((rgb[23:16]>rgb[7:0])?rgb[23:16]:rgb[7:0]):
        ((rgb[15:8]>rgb[7:0])?rgb[15:8]:rgb[7:0]);
    wire [7:0] vmin=(rgb[23:16]<rgb[15:8]) ?
        ((rgb[23:16]<rgb[7:0])?rgb[23:16]:rgb[7:0]):
        ((rgb[15:8]<rgb[7:0])?rgb[15:8]:rgb[7:0]);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin vs_pre<=0;de_pre<=0;gray_pre<=0;chroma_pre<=0;end
        else begin vs_pre<=vsync;de_pre<=de;gray_pre<=sum[9:2];chroma_pre<=vmax-vmin;end
    end
    reg vs_d,de_d,seen;
    reg [11:0] x,y,active_width;
    reg [20:0] white_count;
    reg [15:0] seq,current_source_sequence;
    wire frame_start /* synthesis syn_maxfan = 32 */ =vs_q&&!vs_d;
    wire line_end=de_d&&!de_q;
    // y > ty1+MAX_MISS+1 等价于 y>MAX_MISS+1 且 ty1<y-MAX_MISS-1。
    // 与y同时更新行阈值，移除16路轨迹选择后的加法，包含12位行计数回绕。
    wire [11:0] next_y=frame_start?12'd0:(line_end?(y+12'd1):y);
    reg [11:0] expiry_floor_y;
    reg expiry_y_valid;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin expiry_floor_y<=-(MAX_MISS+1);expiry_y_valid<=0;end
        else begin
            expiry_floor_y<=next_y-(MAX_MISS+1);
            expiry_y_valid<={20'd0,next_y}>MAX_MISS+1;
        end
    end
    wire [7:0] adaptive_threshold;
    wire threshold_ready;
    lane_row_threshold #(.INITIAL_THRESHOLD(WHITE_TH)) u_threshold(
        .clk(clk),.rst_n(rst_n),.frame_start(frame_start),.de(de_q),.gray(gray_q),
        .threshold(adaptive_threshold),.ready(threshold_ready));
    wire [7:0] white_threshold=ADAPTIVE_WHITE?adaptive_threshold:WHITE_TH;
    wire white=de_q&&y>=ROI_TOP&&y<HEIGHT&&gray_q>white_threshold&&chroma_q<=CHROMA_MAX&&ridge;
    wire [7:0] gray_near,gray_tail;
    reg run;
    reg [11:0] run_x0,run_x1;
    reg [7:0] flank_left,flank_wide,peak;
    reg [WIDE_DIST-1:0] candidate_valid;
    wire [43:0] candidate_near,candidate_tail;
    reg event_valid;
    reg [11:0] ex0,ex1,ey;
    wire [12:0] run_width={1'b0,run_x1}-{1'b0,run_x0}+1'b1;
    wire [12:0] run_sum={1'b0,run_x1}+{1'b0,run_x0};
    wire [11:0] max_width=12+((y-ROI_TOP)>>3);
    wire finish_accepted=de_q&&!white&&run&&run_width>=MIN_RUN&&run_width<=MAX_RUN&&
        run_width<=max_width&&run_sum[12:1]>=WIDTH/16&&
        run_sum[12:1]<WIDTH-WIDTH/16&&run_x0>=WIDE_DIST;
    // 候选队列使能在ridge末级提前译码，保持原de_q&&!frame_start条件和生效拍数。
    // 原队列只读取4拍候选、8拍灰度及WIDE_DIST拍尾端，循环RAM保留相同有效拍语义。
    lane_delay_taps #(.WIDTH(44),.DEPTH(WIDE_DIST),.TAP(4)) u_candidate_delay(
        .clk(clk),.rst_n(rst_n),.advance(candidate_advance),.value({run_x0,run_x1,y,peak}),
        .near_value(candidate_near),.tail_value(candidate_tail));
    lane_delay_taps #(.WIDTH(8),.DEPTH(WIDE_DIST),.TAP(8)) u_gray_delay(
        .clk(clk),.rst_n(rst_n),.advance(candidate_advance),.value(gray_q),
        .near_value(gray_near),.tail_value(gray_tail));
    integer d;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vs_d<=0;de_d<=0;x<=0;y<=0;active_width<=0;white_count<=0;
            run<=0;run_x0<=0;run_x1<=0;flank_left<=0;flank_wide<=0;peak<=0;
            candidate_valid<=0;
            event_valid<=0;ex0<=0;ex1<=0;ey<=0;
        end else begin
            vs_d<=vs_q;de_d<=de_q;event_valid<=0;
            if(frame_start) begin
                x<=0;y<=0;active_width<=0;white_count<=0;run<=0;candidate_valid<=0;
            end else begin
                if(de_q) begin
                    x<=x+1'b1;
                    if(white&&white_count!=21'h1fffff) white_count<=white_count+1'b1;
                    // 每个候选独立延迟，避免等待宽背景时被后来的白块覆盖。
                    candidate_valid[0]<=finish_accepted&&
                        {1'b0,peak}>={1'b0,flank_left}+9'd18&&
                        {1'b0,peak}>={1'b0,flank_wide}+9'd32;
                    for(d=1;d<WIDE_DIST;d=d+1) begin
                        if(d==4) candidate_valid[d]<=candidate_valid[d-1]&&
                            {1'b0,candidate_near[7:0]}>={1'b0,gray_q}+9'd18;
                        else candidate_valid[d]<=candidate_valid[d-1];
                    end
                    if(candidate_valid[WIDE_DIST-1]&&
                       {1'b0,candidate_tail[7:0]}>={1'b0,gray_q}+9'd32) begin
                        event_valid<=1;ex0<=candidate_tail[43:32];
                        ex1<=candidate_tail[31:20];ey<=candidate_tail[19:8];
                    end
                    if(white) begin
                        if(!run) begin run<=1;run_x0<=x;flank_left<=gray_near;flank_wide<=gray_tail;peak<=gray_q;end
                        else if(gray_q>peak) peak<=gray_q;
                        run_x1<=x;
                    end else if(run) begin
                        run<=0;
                    end
                end else begin x<=0;run<=0;candidate_valid<=0;end
                if(line_end) begin y<=y+1'b1;if(x>active_width) active_width<=x;end
            end
        end
    end
    // 同时维护8条局部轨迹，关联相邻行最近中心，允许两行阈值孔洞。
    reg [NT-1:0] alive;
    reg [11:0] tx0[0:NT-1],tx1[0:NT-1],bx0[0:NT-1],bx1[0:NT-1];
    reg [11:0] ty0[0:NT-1],ty1[0:NT-1],rows[0:NT-1];
    wire [12:0] event_sum={1'b0,ex0}+{1'b0,ex1};
    wire [11:0] ec=event_sum[12:1];
    integer match_slot;
    wire [4:0] free_slot;
    reg [12:0] distance_tmp,center_tmp,dy_tmp;
    localparam RUN_WIDTH_BITS=(MAX_RUN<256)?8:13;
    reg [RUN_WIDTH_BITS-1:0] event_width_tmp,track_width_tmp;
    reg [12:0] distances[0:15],pair_distance[0:7],half_distance[0:3],quarter_distance[0:1];
    reg [3:0] pair_index[0:7],half_index[0:3],quarter_index[0:1];
    reg [3:0] match_pipe;
    reg [11:0] event_x0[0:3],event_x1[0:3],event_y[0:3];
    reg [4:0] free_index[0:3];
    reg [15:0] free_masks[0:3];
    wire [15:0] free_slot_mask;
    wire match_event=match_pipe[3];
    wire [11:0] mx0=event_x0[3],mx1=event_x1[3],my=event_y[3];
    integer p;
    // 槽号与独热预订同拍传递，分配反馈无需再把5位槽号译成16位。
    lane_slot_allocator #(.MASK_INPUTS(1)) u_slots(.alive(alive),.pending(match_pipe),
        .reserved({free_index[3],free_index[2],free_index[1],free_index[0]}),
        .reserved_masks({free_masks[3],free_masks[2],free_masks[1],free_masks[0]}),
        .slot(free_slot),.slot_mask(free_slot_mask));
    // 四组最小距离并行比大小，寄存最终槽号及独热写使能；同距仍优先低组/低槽。
    wire [3:0] group_winner;
    assign group_winner[0]=half_distance[0]<=half_distance[1]&&half_distance[0]<=half_distance[2]&&half_distance[0]<=half_distance[3]&&half_distance[0]!=13'h1fff;
    assign group_winner[1]=half_distance[1]<half_distance[0]&&half_distance[1]<=half_distance[2]&&half_distance[1]<=half_distance[3]&&half_distance[1]!=13'h1fff;
    assign group_winner[2]=half_distance[2]<half_distance[0]&&half_distance[2]<half_distance[1]&&half_distance[2]<=half_distance[3]&&half_distance[2]!=13'h1fff;
    assign group_winner[3]=half_distance[3]<half_distance[0]&&half_distance[3]<half_distance[1]&&half_distance[3]<half_distance[2]&&half_distance[3]!=13'h1fff;
    reg [4:0] match_index_q /* synthesis syn_maxfan = 16 */;
    reg [15:0] match_mask_q;
    integer mi;
    always @* match_slot=(match_index_q==31)?-1:match_index_q;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin match_index_q<=31;match_mask_q<=0;end
        else begin
            match_index_q<=group_winner[0]?{1'b0,half_index[0]}:
                group_winner[1]?{1'b0,half_index[1]}:group_winner[2]?{1'b0,half_index[2]}:
                group_winner[3]?{1'b0,half_index[3]}:5'd31;
            for(mi=0;mi<16;mi=mi+1) match_mask_q[mi]<=
                (group_winner[0]&&half_index[0]==mi)||(group_winner[1]&&half_index[1]==mi)||
                (group_winner[2]&&half_index[2]==mi)||(group_winner[3]&&half_index[3]==mi);
        end
    end
    // 距离计算、两级最小值树分拍，避免8路串联比较落在一个像素周期。
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            match_pipe<=0;
            for(p=0;p<16;p=p+1) distances[p]<=13'h1fff;
            for(p=0;p<8;p=p+1) begin pair_distance[p]<=13'h1fff;pair_index[p]<=15;end
            for(p=0;p<4;p=p+1) begin half_distance[p]<=13'h1fff;half_index[p]<=15;end
            for(p=0;p<2;p=p+1) begin quarter_distance[p]<=13'h1fff;quarter_index[p]<=15;end
            for(p=0;p<4;p=p+1) begin event_x0[p]<=0;event_x1[p]<=0;event_y[p]<=0;free_index[p]<=31;free_masks[p]<=0;end
        end else begin
            match_pipe<={match_pipe[2:0],event_valid};
            event_x0[0]<=ex0;event_x1[0]<=ex1;event_y[0]<=ey;
            free_index[0]<=free_slot;
            free_masks[0]<=free_slot_mask;
            for(p=1;p<4;p=p+1) begin
                event_x0[p]<=event_x0[p-1];event_x1[p]<=event_x1[p-1];event_y[p]<=event_y[p-1];
                free_index[p]<=free_index[p-1];
                free_masks[p]<=free_masks[p-1];
            end
            for(p=0;p<16;p=p+1) begin
                center_tmp=({1'b0,bx0[p]}+{1'b0,bx1[p]})>>1;
                distance_tmp=(ec>=center_tmp)?ec-center_tmp:center_tmp-ec;
                dy_tmp=ey-ty1[p];
                event_width_tmp={1'b0,ex1}-ex0+1'b1;
                track_width_tmp={1'b0,bx1[p]}-bx0[p]+1'b1;
                distances[p]<=(alive[p]&&ey>ty1[p]&&dy_tmp<=MAX_MISS+1&&
                    (({3'b0,event_width_tmp}<<1)+event_width_tmp)<=({3'b0,track_width_tmp}<<3)&&
                    (({3'b0,track_width_tmp}<<1)+track_width_tmp)<=({3'b0,event_width_tmp}<<3)&&
                    distance_tmp<=MAX_DRIFT*dy_tmp)?distance_tmp:13'h1fff;
            end
            for(p=0;p<8;p=p+1) begin
                if(distances[p*2]<=distances[p*2+1]) begin pair_distance[p]<=distances[p*2];pair_index[p]<=p*2;end
                else begin pair_distance[p]<=distances[p*2+1];pair_index[p]<=p*2+1;end
            end
            for(p=0;p<4;p=p+1) begin
                if(pair_distance[p*2]<=pair_distance[p*2+1]) begin half_distance[p]<=pair_distance[p*2];half_index[p]<=pair_index[p*2];end
                else begin half_distance[p]<=pair_distance[p*2+1];half_index[p]<=pair_index[p*2+1];end
            end
            for(p=0;p<2;p=p+1) begin
                if(half_distance[p*2]<=half_distance[p*2+1]) begin quarter_distance[p]<=half_distance[p*2];quarter_index[p]<=half_index[p*2];end
                else begin quarter_distance[p]<=half_distance[p*2+1];quarter_index[p]<=half_index[p*2+1];end
            end
        end
    end
    reg [7:0] cv,pv;
    reg [79:0] boxes[0:N-1],previous[0:N-1],raw_boxes[0:N-1];
    reg [23:0] knots[0:N-1]; // {实测y,实测中心x}；不改变CDC/PIO的1024位宽。
    reg [7:0] raw_valid;
    reg [11:0] support[0:N-1];
    reg [11:0] quality[0:N-1];
    reg [11:0] quality_pair[0:3],quality_half[0:1];
    reg [2:0] quality_pair_index[0:3],quality_half_index[0:1];
    reg [1:0] collect_step;
    reg good_q;
    reg [2:0] bin_q;
    reg [79:0] candidate_q;
    reg [23:0] knot_q;
    reg [11:0] rows_q,quality_q;
    reg [2:0] insert_slot_q;
    reg [7:0] replace_allowed_q;
    reg [2:0] slope_bin[0:N-1];
    reg [3:0] scan;
    reg [2:0] phase;
    reg [3:0] state;
    reg [1023:0] assembled;
    // 坐标滤波单元独立保留供实验；已测连续帧未触发，不占最终器件资源。
    // 当前可见证据的稳定性由线宽关联与共同可见行确认保证。
    reg [11:0] best_left,best_right;
    reg [3:0] count;
    integer slot,j,weak_slot;
    reg [11:0] min_support,span,shift,top_c,bottom_c;
    reg [11:0] required_rows,required_span,required_shift;
    reg good,direction_right,dashed,confirmed,central_track;
    reg [2:0] bin;
    reg [11:0] gap,xgap;
    reg [79:0] b,other;
    reg [11:0] ct,cb,ot,ob;
    function [11:0] absdiff;
        input [11:0] a,bv;
        begin absdiff=(a>=bv)?a-bv:bv-a;end
    endfunction
    wire contour_write=state==0&&match_event&&(match_slot>=0||free_index[3]!=31);
    wire [3:0] contour_slot=(match_slot>=0)?match_slot:free_index[3][3:0];
    wire [11:0] contour_start=(match_slot>=0)?ty0[contour_slot]:my;
    wire [12:0] contour_sum={1'b0,mx0}+mx1;
    wire [12:0] contour_mid_sum={1'b0,ty0[scan]}+ty1[scan];
    wire [11:0] contour_mid_y=contour_mid_sum[12:1];
    wire [39:0] contour_data;
    wire [3:0] contour_epoch=seq[3:0]-(state==1);
    lane_contour_samples #(.ROI_TOP(ROI_TOP)) u_contour(
        .clk(clk),.write_enable(contour_write),.write_slot(contour_slot),.epoch(seq[3:0]),
        .write_start(contour_start),.write_y(my),.write_center(contour_sum[12:1]),
        .read_slot(scan),.read_mid_y(contour_mid_y),.sample_data(contour_data));
    always @* begin
        span=ty1[scan]-ty0[scan];
        top_c=({1'b0,tx0[scan]}+{1'b0,tx1[scan]})>>1;
        bottom_c=({1'b0,bx0[scan]}+{1'b0,bx1[scan]})>>1;
        shift=absdiff(top_c,bottom_c);direction_right=bottom_c>top_c;
        // 中央消失点附近的车道可接近竖直；外侧仍必须向道路两边展开。
        central_track=top_c>=WIDTH*43/100&&top_c<=WIDTH*65/100&&
                      bottom_c>=WIDTH*43/100&&bottom_c<=WIDTH*65/100&&
                      (span<SOLID_SPAN||ty0[scan]>=HEIGHT*2/3||ty1[scan]>=HEIGHT*4/5);
        // 远处短虚线被透视压缩，缩小行数/跨度门槛；仍须通过线宽、方向、同车道配对及跨帧确认。
        required_rows=(ty0[scan]<HEIGHT*4/5&&MIN_ROWS>6)?6:MIN_ROWS;
        // n行连续支持的坐标跨度为n-1，避免刚够行数的短虚线再被跨度条件删掉。
        required_span=(ty0[scan]<HEIGHT*4/5&&MIN_SPAN>5)?5:
            ((MIN_SPAN>=MIN_ROWS&&MIN_ROWS>1)?MIN_ROWS-1:MIN_SPAN);
        required_shift=(ty0[scan]<HEIGHT*4/5&&MIN_SHIFT>2)?2:MIN_SHIFT;
        good=alive[scan]&&rows[scan]>=required_rows&&span>=required_span&&(central_track||shift>=required_shift)&&
             (({2'b0,bx1[scan]}-bx0[scan]+1)<<3)>=
                 (({2'b0,tx1[scan]}-tx0[scan]+1)*3)&&
             ((tx1[scan]-tx0[scan]+1)+(bx1[scan]-bx0[scan]+1))>=
                 (2*MIN_RUN+(((ty0[scan]+ty1[scan])>>1)-ROI_TOP)/32)&&
             {2'b0,shift}<=(({2'b0,span}<<1)+span)&&(central_track||({2'b0,shift}<<2)>={2'b0,span})&&
             ({2'b0,rows[scan]}<<2)>=(({2'b0,span}+1)*3)&&
             ((({1'b0,top_c}+bottom_c)>>1)>=WIDTH*43/100||!direction_right)&&
             ((({1'b0,top_c}+bottom_c)>>1)<=WIDTH*65/100||direction_right);
        if(({1'b0,shift}<<1)<=span) bin=0;
        else if(shift<=span) bin=1;
        else if(shift<=({1'b0,span}<<1)) bin=2;
        else bin=3;
        weak_slot=(quality_half[0]<=quality_half[1])?quality_half_index[0]:quality_half_index[1];
        min_support=(quality_half[0]<=quality_half[1])?quality_half[0]:quality_half[1];
        slot=weak_slot;
        for(j=N-1;j>=0;j=j-1) if(!cv[j]) slot=j;
    end
    // 三拍候选筛选：先取轨迹（含同拍匹配写入旁路），再算几何量，最后判定。
    // 使用现有收集拍0/1/2，第3拍插入及帧结果发布周期保持不变。
    reg [11:0] gx0_q,gx1_q,gb0_q,gb1_q,gy0_q,gy1_q,grows_q;
    reg galive_q;
    wire [12:0] gtop_sum={1'b0,gx0_q}+gx1_q,gbottom_sum={1'b0,gb0_q}+gb1_q;
    wire [11:0] gtop=gtop_sum[12:1],gbottom=gbottom_sum[12:1];
    wire [11:0] gspan=gy1_q-gy0_q,gshift=absdiff(gtop,gbottom);
    reg [11:0] span_g,shift_g,rows_g,required_rows_g,required_span_g,required_shift_g;
    reg alive_g,central_g,width_ok_g,width_sum_ok_g,density_g,position_g;
    wire good_g=alive_g&&rows_g>=required_rows_g&&span_g>=required_span_g&&
        (central_g||shift_g>=required_shift_g)&&width_ok_g&&width_sum_ok_g&&
        {2'b0,shift_g}<=(({2'b0,span_g}<<1)+span_g)&&
        (central_g||({2'b0,shift_g}<<2)>={2'b0,span_g})&&density_g&&position_g;
    wire [2:0] bin_g=(({1'b0,shift_g}<<1)<=span_g)?0:
        ((shift_g<=span_g)?1:((shift_g<=({1'b0,span_g}<<1))?2:3));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            gx0_q<=0;gx1_q<=0;gb0_q<=0;gb1_q<=0;gy0_q<=0;gy1_q<=0;grows_q<=0;galive_q<=0;
            span_g<=0;shift_g<=0;rows_g<=0;required_rows_g<=0;required_span_g<=0;required_shift_g<=0;
            alive_g<=0;central_g<=0;width_ok_g<=0;width_sum_ok_g<=0;density_g<=0;position_g<=0;
        end else begin
            // 持续读取也覆盖消隐中途被有效视频打断的情况。
            gx0_q<=tx0[scan];gx1_q<=tx1[scan];gb0_q<=bx0[scan];gb1_q<=bx1[scan];
            gy0_q<=ty0[scan];gy1_q<=ty1[scan];grows_q<=rows[scan];galive_q<=alive[scan];
            if(state==0&&match_event) begin
                if(match_slot>=0) begin
                    if(match_slot==scan) begin gb0_q<=mx0;gb1_q<=mx1;gy1_q<=my;grows_q<=rows[scan]+1'b1;end
                end else if(free_index[3]!=31&&free_index[3]==scan) begin
                    galive_q<=1;gx0_q<=mx0;gx1_q<=mx1;gb0_q<=mx0;gb1_q<=mx1;
                    gy0_q<=my;gy1_q<=my;grows_q<=1;
                end
            end
            if((state==0||state==1)&&(state==1||!de_q)&&collect_step==1) begin
                span_g<=gspan;shift_g<=gshift;rows_g<=grows_q;alive_g<=galive_q;
                required_rows_g<=(gy0_q<HEIGHT*4/5&&MIN_ROWS>6)?6:MIN_ROWS;
                required_span_g<=(gy0_q<HEIGHT*4/5&&MIN_SPAN>5)?5:
                    ((MIN_SPAN>=MIN_ROWS&&MIN_ROWS>1)?MIN_ROWS-1:MIN_SPAN);
                required_shift_g<=(gy0_q<HEIGHT*4/5&&MIN_SHIFT>2)?2:MIN_SHIFT;
                // 中央短虚线保留近竖直豁免；远处孤立长杆不能仅靠中央位置成为实线。
                // 长轨迹须进入较低路面区域，否则沿用道路向两侧展开的斜率约束。
                central_g<=gtop>=WIDTH*43/100&&gtop<=WIDTH*65/100&&gbottom>=WIDTH*43/100&&gbottom<=WIDTH*65/100&&
                    (gspan<SOLID_SPAN||gy0_q>=HEIGHT*2/3||gy1_q>=HEIGHT*4/5);
                width_ok_g<=(({2'b0,gb1_q}-gb0_q+1)<<3)>=(({2'b0,gx1_q}-gx0_q+1)*3);
                width_sum_ok_g<=((gx1_q-gx0_q+1)+(gb1_q-gb0_q+1))>=
                    (2*MIN_RUN+(((gy0_q+gy1_q)>>1)-ROI_TOP)/32);
                density_g<=({2'b0,grows_q}<<2)>=(({2'b0,gspan}+1)*3);
                position_g<=((({1'b0,gtop}+gbottom)>>1)>=WIDTH*43/100||!(gbottom>gtop))&&
                    ((({1'b0,gtop}+gbottom)>>1)<=WIDTH*65/100||gbottom>gtop);
            end
        end
    end
    integer q;
    // 候选评分加入位置权重，避免上方长噪声挤掉远处短线；最小值树分拍。
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(q=0;q<4;q=q+1) begin quality_pair[q]<=0;quality_pair_index[q]<=q*2;end
            for(q=0;q<2;q=q+1) begin quality_half[q]<=0;quality_half_index[q]<=q*4;end
        end else begin
            for(q=0;q<4;q=q+1) begin
                if(quality[q*2]<=quality[q*2+1]) begin quality_pair[q]<=quality[q*2];quality_pair_index[q]<=q*2;end
                else begin quality_pair[q]<=quality[q*2+1];quality_pair_index[q]<=q*2+1;end
            end
            for(q=0;q<2;q=q+1) begin
                if(quality_pair[q*2]<=quality_pair[q*2+1]) begin quality_half[q]<=quality_pair[q*2];quality_half_index[q]<=quality_pair_index[q*2];end
                else begin quality_half[q]<=quality_pair[q*2+1];quality_half_index[q]<=quality_pair_index[q*2+1];end
            end
        end
    end
    // 帧间/段间关系逐对计算，避免8路几何条件叠在同一像素周期。
    reg [79:0] current_box,confirm_box,neighbor_box,old_box;
    reg [2:0] neighbor_slope,pair_slot;
    wire [11:0] final_span=current_box[17:6]-current_box[29:18];
    wire final_central=ct>=WIDTH*43/100&&ct<=WIDTH*65/100&&
        cb>=WIDTH*43/100&&cb<=WIDTH*65/100&&
        (current_box[29:18]>=HEIGHT*2/3||current_box[17:6]>=HEIGHT*4/5);
    // 合并会把短候选变成长实线；发布前重新检查方向和有效支持覆盖率。
    wire solid_coverage_ok=({1'b0,support[phase]}<<1)>=({1'b0,final_span}+13'd1);
    wire solid_direction_ok=final_central||({2'b0,absdiff(ct,cb)}<<2)>={2'b0,final_span};
    reg gap_direction_ok;
    reg [23:0] current_knot;
    reg neighbor_valid,old_valid;
    reg pair_dash,pair_confirm,pair_dash_q,pair_confirm_q;
    reg [7:0] dash_flags,confirm_flags;
    reg [7:0] span_flags;
    // 每槽分类结束时保存发布所需的方向与折点判定，state8只读标志。
    reg [7:0] left_flags,right_flags,knee_flags;
    reg [5:0] knee_offsets[0:7];
    // 邻段从state5起稳定，原跨帧核完成前先分拍比较四个端点，不改变确认拍数。
    reg [3:0] confirm_bounds_q;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) confirm_bounds_q<=0;
        else begin
            confirm_bounds_q[0]<=absdiff(confirm_box[77:66],old_box[77:66])<=WIDTH/24;
            confirm_bounds_q[1]<=absdiff(confirm_box[53:42],old_box[53:42])<=WIDTH/24;
            confirm_bounds_q[2]<=absdiff(confirm_box[29:18],old_box[29:18])<=HEIGHT/12;
            confirm_bounds_q[3]<=absdiff(confirm_box[17:6],old_box[17:6])<=HEIGHT/12;
        end
    end
    reg paint_join_q;
    wire [79:0] gap_merged;
    reg [23:0] merge_knot,knot_option;
    reg [11:0] merge_mid;
    reg [12:0] merge_distance,option_distance;
    reg signed [13:0] knee_delta;
    reg [2:0] knot_option_index;
    // 间隙证据查询至少等待十余拍，利用既有等待预存六个测量点和距离。
    // state12仍按原顺序逐拍比较，同距不替换，不增加状态或改变发布周期。
    wire [23:0] knot_options[0:5];
    assign knot_options[0]=current_knot;
    assign knot_options[1]=knots[pair_slot];
    assign knot_options[2]={current_box[29:18],ct};
    assign knot_options[3]={current_box[17:6],cb};
    assign knot_options[4]={neighbor_box[29:18],ot};
    assign knot_options[5]={neighbor_box[17:6],ob};
    reg [23:0] knot_options_q[0:5];
    reg [12:0] knot_distances_q[0:5];
    reg [5:0] knot_inside_q;
    reg [11:0] merge_mid_q,merge_top_q,merge_bottom_q;
    integer kp;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            merge_mid_q<=0;merge_top_q<=0;merge_bottom_q<=0;knot_inside_q<=0;
            for(kp=0;kp<6;kp=kp+1) begin knot_options_q[kp]<=0;knot_distances_q[kp]<=0;end
        end else begin
            merge_mid_q<=merge_mid;merge_top_q<=gap_merged[29:18];merge_bottom_q<=gap_merged[17:6];
            for(kp=0;kp<6;kp=kp+1) begin
                knot_options_q[kp]<=knot_options[kp];
                knot_distances_q[kp]<=(knot_options_q[kp][23:12]>=merge_mid_q)?
                    knot_options_q[kp][23:12]-merge_mid_q:merge_mid_q-knot_options_q[kp][23:12];
                knot_inside_q[kp]<=knot_options_q[kp][11:0]!=0&&
                    knot_options_q[kp][23:12]>merge_top_q&&knot_options_q[kp][23:12]<merge_bottom_q;
            end
        end
    end
    always @* begin
        merge_mid=({1'b0,gap_merged[29:18]}+gap_merged[17:6])>>1;
        // 合并时逐拍检查六个实际测量点，共用一个距离计算器。
        case(knot_option_index)
              0:knot_option=current_knot;
              1:knot_option=knots[pair_slot];
              2:knot_option={current_box[29:18],ct};
              3:knot_option={current_box[17:6],cb};
              4:knot_option={neighbor_box[29:18],ot};
              default:knot_option={neighbor_box[17:6],ob};
        endcase
        option_distance=(knot_option[23:12]>=merge_mid)?knot_option[23:12]-merge_mid:merge_mid-knot_option[23:12];
        knee_delta=$signed({1'b0,current_knot[23:12]})-
                   (({1'b0,current_box[29:18]}+current_box[17:6])>>1);
    end
    always @* begin
        b=current_box;other=neighbor_box;
        ct=({1'b0,b[77:66]}+{1'b0,b[65:54]})>>1;
        cb=({1'b0,b[53:42]}+{1'b0,b[41:30]})>>1;
        ot=({1'b0,other[77:66]}+{1'b0,other[65:54]})>>1;
        ob=({1'b0,other[53:42]}+{1'b0,other[41:30]})>>1;
        if(b[29:18]>other[17:6]) begin
            gap=b[29:18]-other[17:6];xgap=absdiff(ct,ob);
            gap_direction_ok=(ct>=WIDTH/2)?ct>ob:ct<ob;
        end else begin
            gap=other[29:18]-b[17:6];xgap=absdiff(ot,cb);
            gap_direction_ok=(ct>=WIDTH/2)?ot>cb:ot<cb;
        end
        pair_dash=neighbor_valid&&pair_slot!=phase&&((ct>=WIDTH/2)==(ot>=WIDTH/2))&&
            ((cb>ct)==(ob>ot))&&gap_direction_ok&&gap>=DASH_MISS&&gap<=HEIGHT/3&&
            xgap>=((slope_bin[phase]==0&&neighbor_slope==0)?(gap>>2):(gap>>1))&&
            {2'b0,xgap}<=({2'b0,gap}<<2)&&
            slope_bin[phase]<=neighbor_slope+1&&neighbor_slope<=slope_bin[phase]+1;
        pair_confirm=old_valid&&(&confirm_bounds_q);
    end
    wire relation_busy,relation_done,relation_aligned;
    wire relation_start=state==6&&!relation_busy&&!relation_done;
    lane_segment_relation #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MIN_GAP(DASH_MISS)) u_relation(
        .clk(clk),.rst_n(rst_n),.start(relation_start),
        .valid_pair(cv[phase]&&neighbor_valid&&pair_slot!=phase),.a(current_box),.b(neighbor_box),
        .busy(relation_busy),.done(relation_done),.aligned(relation_aligned));
    wire temporal_busy,temporal_done,temporal_aligned;
    lane_temporal_overlap #(.WIDTH(WIDTH),.PREPARED_INPUTS(1)) u_temporal(
        .clk(clk),.rst_n(rst_n),.start(state==11&&!temporal_busy&&!temporal_done),
        .valid_pair(cv[phase]&&old_valid),.a(confirm_box),.b(old_box),
        .busy(temporal_busy),.done(temporal_done),.aligned(temporal_aligned));
    wire gap_busy,gap_done,gap_road,gap_paint;
    lane_gap_evidence #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.ROI_TOP(ROI_TOP),.PREPARED_INPUTS(1)) u_gap(
        .clk(clk),.rst_n(rst_n),.frame_start(frame_start),.de(de_q),.pixel_x(x),.pixel_y(y),
        .paint((gray_q>white_threshold||(ridge&&gray_q>=95))&&chroma_q<62&&!road_pixel),
        .road(road_pixel),
        .start(state==9&&!gap_busy&&!gap_done),.a(current_box),.b(neighbor_box),
        .busy(gap_busy),.done(gap_done),.road_gap(gap_road),.paint_gap(gap_paint),.merged(gap_merged));
    integer a;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            alive<=0;cv<=0;pv<=0;scan<=0;phase<=0;state<=0;seen<=0;seq<=0;
            current_source_sequence<=0;
            result<=0;result_update<=0;assembled<=0;best_left<=0;best_right<=0;count<=0;
            collect_step<=0;good_q<=0;bin_q<=0;candidate_q<=0;knot_q<=0;rows_q<=0;quality_q<=0;
            insert_slot_q<=0;replace_allowed_q<=0;
            current_box<=0;confirm_box<=0;current_knot<=0;neighbor_box<=0;old_box<=0;neighbor_valid<=0;old_valid<=0;
            neighbor_slope<=0;pair_slot<=0;dashed<=0;confirmed<=0;pair_dash_q<=0;pair_confirm_q<=0;
            dash_flags<=0;confirm_flags<=0;span_flags<=0;paint_join_q<=0;raw_valid<=0;
            left_flags<=0;right_flags<=0;knee_flags<=0;
            merge_distance<=4096;merge_knot<=0;knot_option_index<=0;
            for(a=0;a<NT;a=a+1) begin
                tx0[a]<=0;tx1[a]<=0;bx0[a]<=0;bx1[a]<=0;ty0[a]<=0;ty1[a]<=0;rows[a]<=0;
            end
            for(a=0;a<N;a=a+1) begin
                boxes[a]<=0;knots[a]<=0;previous[a]<=0;raw_boxes[a]<=0;support[a]<=0;quality[a]<=0;slope_bin[a]<=0;
                knee_offsets[a]<=0;
            end
        end else begin
            result_update<=0;
            if(frame_start&&state==0) begin
                seen<=1;scan<=0;collect_step<=0;
                if(seen) begin
                    state<=1;assembled<=0;seq<=seq+1'b1;best_left<=0;best_right<=0;count<=0;dash_flags<=0;confirm_flags<=0;span_flags<=0;
                    // 发布刚结束帧的结果；源帧号可能不连续，须保留其帧首采样值。
                    assembled[255:240]<=current_source_sequence;
                    assembled[139:128]<=active_width;assembled[127:116]<=y;
                    assembled[115:95]<=white_count;assembled[1023:1008]<=16'h4c32;
                end else begin alive<=0;cv<=0;end
                current_source_sequence<=source_sequence[15:0];
            end else if(state==0||state==1) begin
                if(state==1||!de_q) begin
                    collect_step<=collect_step+1'b1;
                    if(collect_step==1) begin
                        rows_q<=rows[scan];
                        quality_q<=rows[scan]+((ty0[scan]-ROI_TOP)>>2);
                        candidate_q<={1'b1,1'b0,tx0[scan],tx1[scan],bx0[scan],bx1[scan],ty0[scan],ty1[scan],6'd0};
                        if(contour_data[39:36]==contour_epoch&&contour_data[35:24]==ty0[scan]&&
                           contour_data[23:12]>ty0[scan]&&contour_data[23:12]<ty1[scan]) knot_q<=contour_data[23:0];
                        else knot_q<=0;
                    end
                    // 四拍收集的第2拍锁定槽位，八路替换比较并行；第3拍仅选一位。
                    // quality/cv只在第3拍更新，最小值树在下一次第2拍前已稳定。
                    if(collect_step==2) begin
                        good_q<=good_g;bin_q<=bin_g;
                        insert_slot_q<=slot;
                        for(a=0;a<N;a=a+1) replace_allowed_q[a]<=!cv[a]||quality_q>quality[a];
                    end
                    if(collect_step==3) begin
                    if(alive[scan]&&(state==1||(expiry_y_valid&&ty1[scan]<expiry_floor_y))) begin
                        alive[scan]<=0;
                        if(good_q&&replace_allowed_q[insert_slot_q]) begin
                            cv[insert_slot_q]<=1;support[insert_slot_q]<=rows_q;quality[insert_slot_q]<=quality_q;slope_bin[insert_slot_q]<=bin_q;
                            boxes[insert_slot_q]<=candidate_q;knots[insert_slot_q]<=knot_q;
                        end
                    end
                    if(scan==NT-1) begin scan<=0;if(state==1) begin state<=2;phase<=0;end end
                    else scan<=scan+1'b1;
                    end
                end
                if(state==0&&match_event) begin
                    if(match_slot>=0) begin
                        for(a=0;a<NT;a=a+1) if(match_mask_q[a]) begin
                            bx0[a]<=mx0;bx1[a]<=mx1;ty1[a]<=my;rows[a]<=rows[a]+1'b1;
                        end
                    end else if(free_index[3]!=31) begin
                        alive[free_index[3]]<=1;tx0[free_index[3]]<=mx0;tx1[free_index[3]]<=mx1;
                        bx0[free_index[3]]<=mx0;bx1[free_index[3]]<=mx1;ty0[free_index[3]]<=my;ty1[free_index[3]]<=my;
                        rows[free_index[3]]<=1;
                    end
                end
            end else if(state==2) begin
                if(phase==0) begin
                    raw_valid<=cv;
                    for(a=0;a<N;a=a+1) raw_boxes[a]<=boxes[a];
                end
                current_box<=boxes[phase];confirm_box<=boxes[phase];current_knot<=knots[phase];dashed<=0;confirmed<=(CONFIRM_FRAMES<=1);
                pair_slot<=0;state<=5;
            end else if(state==5) begin
                neighbor_box<=boxes[pair_slot];old_box<=previous[pair_slot];
                neighbor_valid<=cv[pair_slot];old_valid<=pv[pair_slot];neighbor_slope<=slope_bin[pair_slot];
                state<=6;
            end else if(state==6) begin
                if(relation_done) begin
                    pair_dash_q<=0;paint_join_q<=0;state<=11;
                end
            end else if(state==11) begin
                if(temporal_done) begin
                    pair_confirm_q<=pair_confirm||temporal_aligned;
                    if(relation_aligned) state<=9;else state<=7;
                end
            end else if(state==9) begin
                if(gap_done) begin pair_dash_q<=gap_road;paint_join_q<=gap_paint;state<=7;end
            end else if(state==7) begin
                dashed<=dashed||pair_dash_q;confirmed<=confirmed||pair_confirm_q;
                if(paint_join_q&&cv[phase]&&cv[pair_slot]) begin
                    knot_option_index<=0;merge_distance<=4096;merge_knot<=0;state<=12;
                end else begin
                    if(pair_slot==N-1) state<=10;else begin pair_slot<=pair_slot+1'b1;state<=5;end
                end
            end else if(state==12) begin
                if(knot_inside_q[knot_option_index]&&knot_distances_q[knot_option_index]<merge_distance) begin
                    merge_distance<=knot_distances_q[knot_option_index];merge_knot<=knot_options_q[knot_option_index];
                end
                if(knot_option_index==5) state<=13;else knot_option_index<=knot_option_index+1'b1;
            end else if(state==13) begin
                current_box<=gap_merged;boxes[phase]<=gap_merged;cv[pair_slot]<=0;
                current_knot<=(merge_distance<=31)?merge_knot:24'd0;
                knots[phase]<=(merge_distance<=31)?merge_knot:24'd0;
                support[phase]<=support[phase]+support[pair_slot];
                if(pair_slot==N-1) state<=10;else begin pair_slot<=pair_slot+1'b1;state<=5;end
            end else if(state==10) begin
                dash_flags[phase]<=dashed;confirm_flags[phase]<=confirmed;
                // 此槽合并已结束；后续仅可能被其他槽撤销cv，几何不再改变。
                // 提前保存原发布条件的跨度项，避免12位减法后驱动整组结果CE。
                span_flags[phase]<=dashed||(final_span>=SOLID_SPAN&&solid_direction_ok&&solid_coverage_ok);
                left_flags[phase]<=cb<ct;right_flags[phase]<=cb>ct;
                knee_flags[phase]<=!dashed&&current_knot[11:0]!=0&&knee_delta>=-32&&knee_delta<=31;
                knee_offsets[phase]<=knee_delta[5:0];
                if(phase==N-1) begin phase<=0;current_box<=boxes[0];current_knot<=knots[0];state<=8;end
                else begin phase<=phase+1'b1;state<=2;end
            end else if(state==8) begin
                if(cv[phase]&&confirm_flags[phase]&&span_flags[phase]) begin
                    assembled[256+phase*80+:80]<={1'b1,dash_flags[phase],b[77:0]};count<=count+1'b1;
                    if(knee_flags[phase]) begin
                        assembled[900+phase*12+:12]<=current_knot[11:0];
                        assembled[256+phase*80+:6]<=knee_offsets[phase];
                    end
                    if(left_flags[phase]&&support[phase]>=best_left) begin
                        best_left<=support[phase];assembled[94:83]<=support[phase];
                        assembled[239:190]<={1'b1,dash_flags[phase],(b[77:66]<b[53:42]?b[77:66]:b[53:42]),
                            (b[65:54]>b[41:30]?b[65:54]:b[41:30]),b[29:18],b[17:6]};
                    end else if(right_flags[phase]&&support[phase]>=best_right) begin
                        best_right<=support[phase];assembled[82:71]<=support[phase];
                        assembled[189:140]<={1'b1,dash_flags[phase],(b[77:66]<b[53:42]?b[77:66]:b[53:42]),
                            (b[65:54]>b[41:30]?b[65:54]:b[41:30]),b[29:18],b[17:6]};
                    end
                end
                if(phase==N-1) state<=3;else begin phase<=phase+1'b1;current_box<=boxes[phase+1'b1];current_knot<=knots[phase+1'b1];state<=8;end
            end else if(state==3) begin
                assembled[899:896]<=count;state<=4;pv<=raw_valid;
                for(a=0;a<N;a=a+1) previous[a]<=raw_boxes[a];
            end else if(state==4) begin
                result<=assembled;result_update<=1;state<=0;alive<=0;cv<=0;
                for(a=0;a<N;a=a+1) begin support[a]<=0;quality[a]<=0;end
            end
        end
    end
endmodule
`endif
