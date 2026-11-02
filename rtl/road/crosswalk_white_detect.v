`timescale 1ns/1ps
`ifndef CROSSWALK_WHITE_DETECT_V
`define CROSSWALK_WHITE_DETECT_V
// 白漆列统计识别：独立于已否决的梯度峰/省乘法器自相关方案。
// 每 16 行统计各列灰度>185的像素数；双缓冲使扫描与下一带采集并行，不丢行。
// 默认1080p下半画面，ROI从y=540起；仅对完整的16行带做判决。
// 填最多4列暗洞 → 白条宽25..110 → 中心距80..180且相邻变化<=25%。
// 至少4条且两个相邻行带有重叠证据才检出。所有坐标为源图有效区坐标。
// result = {status, {x1,x0}, {y1,y0}, pitch}，每项32位；帧边界原子更新。
module crosswalk_white_detect #(
    parameter WIDTH=1920, HEIGHT=1080, ROI_TOP=540,
    parameter BAND_H=16, WHITE_TH=185, WHITE_ROWS=3, MIN_MEAN=90,
    parameter WIDTH_MIN=25, WIDTH_MAX=110, PITCH_MIN=80, PITCH_MAX=180,
    parameter MAX_HOLE=4, MIN_STRIPES=4, MIN_BANDS=2
)(
    input clk, rst_n, vsync, de,
    input [31:0] source_sequence,
    input [23:0] rgb,
    output reg [127:0] result,
    output reg result_update
);
    // 两种亮度证据共用列 RAM：计数到 WHITE_ROWS 后饱和，两个 2 位计数
    // 替代原来的一个 5 位计数，默认仍只占同样的两块双银行 RAM。
    localparam CW=$clog2(WHITE_ROWS+1);
    localparam LOW_WHITE_TH=165;
    reg vs_d, de_d, seen_frame, bad_frame;
    reg [11:0] x, y;
    wire frame_start=vsync && !vs_d;
    wire line_end=de_d && !de;
    wire in_roi=(y>=ROI_TOP && y<HEIGHT);
    wire [3:0] row_in_band=(y-ROI_TOP) % BAND_H;
    wire scan_start=line_end && in_roi && row_in_band==BAND_H-1;
    wire [9:0] gray_sum={2'b0,rgb[23:16]}+{1'b0,rgb[15:8],1'b0}+{2'b0,rgb[7:0]};
    wire [7:0] gray=gray_sum[9:2];
    reg [23:0] band_sum;
    reg write_bank;
    reg [15:0] completed_frame_sequence;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vs_d<=0;de_d<=0;x<=0;y<=0;seen_frame<=0;bad_frame<=0;
            band_sum<=0;write_bank<=0;
        end else begin
            vs_d<=vsync;de_d<=de;
            if(frame_start) begin
                x<=0;y<=0;seen_frame<=1;bad_frame<=0;band_sum<=0;write_bank<=0;
            end else begin
                if(de) x<=x+1'b1; else x<=0;
                if(line_end) begin
                    y<=y+1'b1;
                    if(x!=WIDTH) bad_frame<=1;
                end
                if(de && in_roi && x<WIDTH) begin
                    if(row_in_band==0 && x==0) band_sum<=gray;
                    else band_sum<=band_sum+gray;
                end
                if(scan_start) write_bank<=~write_bank;
            end
        end
    end

    // 两个同步读写 RAM。首行覆盖写，后续行累加；无需上电清 RAM。
    reg [2*CW-1:0] white0[0:WIDTH-1], white1[0:WIDTH-1];
    reg [2*CW-1:0] q0,q1;
    reg we_d, bank_d, first_d, white_d, low_white_d;
    reg [10:0] wa_d;
    reg scan_active, scan_bank, scan_v, scan_mean;
    // 每个16行带有35200个时钟；两次1920列扫描只需约3900个时钟。
    // 复用算术/游程逻辑，原判据和褪色多尺度分支分别保存跨行带状态，共用优先级结果。
    reg scan_mode;
    reg [3:0] scan_gap;
    reg scan_restart;
    reg [10:0] scan_addr, scan_x;
    reg [10:0] scan_y;
    wire [10:0] addr0=(scan_active && !scan_bank) ? scan_addr : x[10:0];
    wire [10:0] addr1=(scan_active && scan_bank) ? scan_addr : x[10:0];
    always @(posedge clk) begin
        q0<=white0[addr0];
        if(we_d && !bank_d) begin
            white0[wa_d][CW-1:0]<=first_d ? white_d :
                (q0[CW-1:0]>=WHITE_ROWS ? q0[CW-1:0] : q0[CW-1:0]+white_d);
            white0[wa_d][2*CW-1:CW]<=first_d ? low_white_d :
                (q0[2*CW-1:CW]>=WHITE_ROWS ? q0[2*CW-1:CW] : q0[2*CW-1:CW]+low_white_d);
        end
    end
    always @(posedge clk) begin
        q1<=white1[addr1];
        if(we_d && bank_d) begin
            white1[wa_d][CW-1:0]<=first_d ? white_d :
                (q1[CW-1:0]>=WHITE_ROWS ? q1[CW-1:0] : q1[CW-1:0]+white_d);
            white1[wa_d][2*CW-1:CW]<=first_d ? low_white_d :
                (q1[2*CW-1:CW]>=WHITE_ROWS ? q1[2*CW-1:CW] : q1[2*CW-1:CW]+low_white_d);
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            we_d<=0;bank_d<=0;first_d<=0;white_d<=0;low_white_d<=0;wa_d<=0;
            scan_active<=0;scan_bank<=0;scan_v<=0;scan_addr<=0;scan_x<=0;
            scan_y<=0;scan_mean<=0;scan_mode<=0;scan_gap<=0;scan_restart<=0;
        end else begin
            scan_restart<=0;
            we_d<=de && in_roi && x<WIDTH && !frame_start;
            bank_d<=write_bank;first_d<=row_in_band==0;
            white_d<=gray>WHITE_TH;low_white_d<=gray>LOW_WHITE_TH;wa_d<=x[10:0];
            scan_v<=scan_active;scan_x<=scan_addr;
            if(scan_active) begin
                if(scan_addr==WIDTH-1) begin scan_active<=0;if(!scan_mode)scan_gap<=8;end
                else scan_addr<=scan_addr+1'b1;
            end
            if(scan_gap!=0)begin
                scan_gap<=scan_gap-1'b1;
                if(scan_gap==1)begin
                    scan_mode<=scan_mode+1'b1;scan_active<=1;scan_addr<=0;scan_restart<=1;
                end
            end
            if(scan_start) begin
                scan_mode<=0;scan_gap<=0;
                scan_active<=1;scan_addr<=0;scan_bank<=write_bank;
                scan_y<=y-BAND_H+1;
                scan_mean<=band_sum>=MIN_MEAN*WIDTH*BAND_H;
            end
            if(frame_start) begin scan_active<=0;scan_v<=0;we_d<=0;scan_mode<=0;scan_gap<=0;scan_restart<=0;end
        end
    end

    // 只统计完整的白条；图像右端的白条和最后一个像素也必须结算。
    // RAM输出先寄存白色分类与对应列号，避免RAM Tco串入宽度/间距/最优候选CE。
    reg is_white;
    reg scan_valid_p,scan_reset_p;
    reg [10:0] scan_column_p;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin is_white<=0;scan_valid_p<=0;scan_reset_p<=0;scan_column_p<=0;end
        else begin
            is_white<=scan_mode ?
                ((scan_bank?q1[2*CW-1:CW]:q0[2*CW-1:CW])>=WHITE_ROWS) :
                ((scan_bank?q1[CW-1:0]:q0[CW-1:0])>=WHITE_ROWS);
            scan_valid_p<=scan_v&&!frame_start;scan_column_p<=scan_x;
            scan_reset_p<=scan_start||scan_restart;
        end
    end
    reg run_active;
    reg [10:0] run_x0, last_white;
    reg [3:0] holes;
    wire close_run=scan_valid_p && run_active &&
        ((!is_white && holes==MAX_HOLE) || scan_column_p==WIDTH-1);
    wire [10:0] end_x=is_white ? scan_column_p : last_white;
    wire [11:0] stripe_width={1'b0,end_x}-{1'b0,run_x0}+12'd1;
    wire [11:0] center_sum={1'b0,run_x0}+{1'b0,end_x};
    wire [10:0] center=center_sum[11:1];
    reg [10:0] prev_center, prev_pitch, seq_x0,prev_width;
    reg [7:0] seq_count, best_count;
    reg [10:0] best_x0,best_x1,best_pitch;
    reg stripe_event,stripe_ok,finish_scan;
    reg [10:0] stripe_center,stripe_left,stripe_right,stripe_width_q;
    wire [10:0] gap=stripe_center-prev_center;
    wire [11:0] pitch_sum={1'b0,prev_pitch}+{1'b0,gap};
    // 箭头头部和相邻细车道线可形成等间距，但宽度交替，不属于同组斑马线。
    // 相邻条带宽度比限定在3/5..5/3，且两条宽度均须占中心距至少1/4。
    // 常数乘法由移位加法构成，不增加 APM 或图像缓存。
    wire widths_match=(({3'b0,stripe_width_q}<<2)+stripe_width_q)>=
            (({3'b0,prev_width}<<1)+prev_width) &&
        (({3'b0,prev_width}<<2)+prev_width)>=
            (({3'b0,stripe_width_q}<<1)+stripe_width_q);
    wire gap_ok=seq_count!=0 && gap>=(scan_mode==0?PITCH_MIN:56) && gap<=(scan_mode==0?PITCH_MAX:440) &&
        widths_match && ({2'b0,stripe_width_q}<<2)>={2'b0,gap} &&
        ({2'b0,prev_width}<<2)>={2'b0,gap} &&
        (seq_count==1 || (gap>=prev_pitch-(prev_pitch>>2) &&
                         gap<=prev_pitch+(prev_pitch>>2)));
    wire [7:0] next_count=gap_ok ? (seq_count==255 ? 8'd255 : seq_count+8'd1) : 8'd1;
    // 用序列内中心距的指数平均，避免单个偏斜白条的末端间距主导跨带匹配。
    wire [10:0] next_pitch=seq_count==1 ? gap : pitch_sum[11:1];
    reg finish_band;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            run_active<=0;run_x0<=0;last_white<=0;holes<=0;
            stripe_event<=0;stripe_ok<=0;stripe_center<=0;stripe_left<=0;stripe_right<=0;stripe_width_q<=0;finish_scan<=0;
        end else begin
            stripe_event<=close_run;stripe_ok<=stripe_width>=(scan_mode==0?WIDTH_MIN:16)&&stripe_width<=(scan_mode==0?WIDTH_MAX:300);
            stripe_center<=center;stripe_left<=run_x0;stripe_right<=end_x;stripe_width_q<=stripe_width;
            finish_scan<=scan_valid_p&&scan_column_p==WIDTH-1;
            if(scan_reset_p||frame_start) begin
                run_active<=0;holes<=0;stripe_event<=0;finish_scan<=0;
            end else if(scan_valid_p) begin
                if(is_white) begin
                    if(!run_active) begin run_active<=1;run_x0<=scan_column_p;end
                    last_white<=scan_column_p;holes<=0;
                end else if(run_active) holes<=holes+1'b1;
                if(close_run) begin run_active<=0;holes<=0;end
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            prev_center<=0;prev_pitch<=0;seq_x0<=0;seq_count<=0;prev_width<=0;
            best_count<=0;best_x0<=0;best_x1<=0;best_pitch<=0;finish_band<=0;
        end else begin
            finish_band<=finish_scan;
            if(scan_reset_p||frame_start) begin
                seq_count<=0;prev_pitch<=0;best_count<=0;finish_band<=0;
            end else if(stripe_event) begin
                if(stripe_ok) begin
                    prev_center<=stripe_center;prev_width<=stripe_width_q;
                    prev_pitch<=gap_ok?next_pitch:11'd0;
                    seq_count<=next_count;
                    if(!gap_ok) seq_x0<=stripe_left;
                    if(next_count>best_count) begin
                        best_count<=next_count;best_x0<=gap_ok?seq_x0:stripe_left;
                        best_x1<=stripe_right;best_pitch<=gap_ok?next_pitch:11'd0;
                    end
                end else begin seq_count<=0;prev_pitch<=0;end
            end
        end
    end

    // 行带结束延后一拍使用最终分割结果，避免漏掉右端白条的非阻塞赋值。
    // 两份上下文存在小块RAM中。只把当前分支载入工作寄存器，避免
    // 宽寄存器的动态读写选择器挤占LUT5放置资源。
    (* syn_ramstyle = "block_ram" *) reg [68:0] contexts[0:1];
    reg [68:0] context_q;
    reg [2:0] load_context;
    reg save_context;
    reg [5:0] group_bands;
    reg [7:0] group_count;
    reg [10:0] gx0,gx1,gy0,gy1,gpitch;
    wire [68:0] active_context={group_bands,group_count,gx0,gx1,gy0,gy1,gpitch};
    // 保存比finish_band晚一拍，取得更新后的完整状态；8拍分支间隔保证无竞争。
    always @(posedge clk) begin
        context_q<=contexts[scan_mode];
        if(save_context) contexts[scan_mode]<=active_context;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin load_context<=0;save_context<=0;end
        else begin
            load_context<={load_context[1:0],scan_start||scan_restart};
            save_context<=finish_band;
            if(frame_start)begin load_context<=0;save_context<=0;end
        end
    end
    reg selected_valid;
    reg selected_mode;
    reg [68:0] selected_frame;
    // 多尺度(低阈值)分支的同一组斑马线可能因车辆遮挡或斜向投影，在若干
    // 行带内断开，且其左右两段不重叠。允许最多跨过两个无证据行带合并；
    // 节距一致性仍作为关联条件，严格分支保持原有相邻/重叠规则。
    wire band_y_match=scan_mode ?
        (scan_y>gy1 && scan_y<=gy1+BAND_H*2+1) :
        (scan_y==gy1+1'b1);
    wire band_x_match=scan_mode || (best_x0<=gx1 && best_x1>=gx0);
    wire join_group=group_bands!=0 && band_y_match && band_x_match &&
        best_pitch>=gpitch-(gpitch>>2) && best_pitch<=gpitch+(gpitch>>2);
    wire [5:0] joined_bands=group_bands+6'd1;
    wire [7:0] joined_count=group_count>best_count ? group_count : best_count;
    wire [10:0] joined_x0=gx0<best_x0 ? gx0 : best_x0;
    wire [10:0] joined_x1=gx1>best_x1 ? gx1 : best_x1;
    wire [5:0] next_bands=join_group ? joined_bands : 6'd1;
    wire [7:0] group_next_count=join_group ? joined_count : best_count;
    wire [10:0] nx0=join_group ? joined_x0 : best_x0;
    wire [10:0] nx1=join_group ? joined_x1 : best_x1;
    wire [10:0] ny0=join_group ? gy0 : scan_y;
    wire [10:0] ny1=scan_y+BAND_H-1;
    // 低阈值分支跟随当前行带的尺度变化；向画面底部节距逐渐增大。
    wire [10:0] npitch=join_group && !scan_mode ? gpitch : best_pitch;
    wire dimensions_ok=!bad_frame && y==HEIGHT;
    wire detected=dimensions_ok && selected_valid;
    wire strong_band=best_count>=MIN_STRIPES;
    // 两三个白条只延伸已由两个强行带确认的目标，不能单独触发检测。
    // 弱证据必须与已有框重叠，防止把路面箭头当成第二个确认行带。
    wire extend_band=scan_mode && best_count>=2 && group_bands>=MIN_BANDS &&
        join_group && best_x0<=gx1 && best_x1>=gx0;
    wire accept_band=finish_band && scan_mean && (strong_band || extend_band);
    // Compare both candidates before join_group, keeping the original arithmetic widths.
    wire joined_frame_ok=joined_bands>=MIN_BANDS &&
        (scan_mode==0 || joined_x1-joined_x0>=320) &&
        (!selected_valid || scan_mode<selected_mode || (scan_mode==selected_mode &&
         (joined_bands>selected_frame[60:55] ||
          (joined_bands==selected_frame[60:55] && joined_count>selected_frame[68:61]))));
    wire fresh_frame_ok=6'd1>=MIN_BANDS &&
        (scan_mode==0 || best_x1-best_x0>=320) &&
        (!selected_valid || scan_mode<selected_mode || (scan_mode==selected_mode &&
         (6'd1>selected_frame[60:55] ||
          (6'd1==selected_frame[60:55] && best_count>selected_frame[68:61]))));
    wire accept_frame=accept_band && (join_group ? joined_frame_ok : fresh_frame_ok);
    // 缓存当前帧中优先级最高分支的最佳框；帧边界原子发布，不增加结果延迟。
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            group_bands<=0;group_count<=0;gx0<=0;gx1<=0;gy0<=0;gy1<=0;gpitch<=0;
            selected_valid<=0;selected_mode<=1;selected_frame<=0;
            completed_frame_sequence<=0;result<=0;result_update<=0;
        end else begin
            result_update<=0;
            if(frame_start)begin
                selected_valid<=0;selected_mode<=1;selected_frame<=0;
                if(seen_frame)begin
                    result_update<=1;
                    result<={completed_frame_sequence,
                        detected ? selected_frame[68:61] : 8'd0,
                        detected ? selected_frame[60:55] : 6'd0,dimensions_ok,detected,
                        5'd0,detected ? selected_frame[54:44] : 11'd0,
                        5'd0,detected ? selected_frame[43:33] : 11'd0,
                        5'd0,detected ? selected_frame[32:22] : 11'd0,
                        5'd0,detected ? selected_frame[21:11] : 11'd0,
                        21'd0,detected ? selected_frame[10:0] : 11'd0};
                end
                completed_frame_sequence<=source_sequence[15:0];
            end else if(load_context[2])begin
                // 帧内首个行带直接初始化，绝不依赖未初始化的RAM或上一帧结果。
                if(scan_y==ROI_TOP)begin
                    group_bands<=0;group_count<=0;gx0<=0;gx1<=0;gy0<=0;gy1<=0;gpitch<=0;
                end else begin
                    {group_bands,group_count,gx0,gx1,gy0,gy1,gpitch}<=context_q;
                end
            end else if(accept_band)begin
                group_bands<=next_bands;group_count<=group_next_count;
                gx0<=nx0;gx1<=nx1;gy0<=ny0;gy1<=ny1;gpitch<=npitch;
                if(accept_frame)begin
                    selected_valid<=1;selected_mode<=scan_mode;
                    selected_frame<={group_next_count,next_bands,nx1,nx0,ny1,ny0,npitch};
                end
            end
        end
    end
endmodule
`endif
