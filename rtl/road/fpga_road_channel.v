`timescale 1ns/1ps
module fpga_road_channel #(
    parameter integer WIDTH=1920, HEIGHT=1080, ROI_TOP=540, PERIOD=1
) (
    input rst_n, pix_clk, bus_clk,
    input boundary, de, input [23:0] rgb,
    input [31:0] source_epoch, source_sequence,
    input [63:0] source_timestamp,
    input read_en, reg_wr,
    input [31:0] read_addr, reg_addr, reg_data,
    output reg [127:0] read_data
);
    localparam [31:0] FRAME_PERIOD = PERIOD;
    (* ASYNC_REG="TRUE" *) reg [2:0] pix_reset, bus_reset;
    always @(posedge pix_clk or negedge rst_n)
        if (!rst_n) pix_reset<=0; else pix_reset<={pix_reset[1:0],1'b1};
    always @(posedge bus_clk or negedge rst_n)
        if (!rst_n) bus_reset<=0; else bus_reset<={bus_reset[1:0],1'b1};
    wire [1023:0] lane;
    wire [127:0] crosswalk;
    wire lane_update, cross_update, record_valid, record_ready;
    wire [1535:0] record;
    reg [1535:0] head, serial_data;
    reg head_valid, sending;
    reg [3:0] send_index, receive_index;
    wire fifo_ready, fifo_valid;
    wire [127:0] fifo_data;
    wire [31:0] dropped;
    wire [127:0] counters;
    reg [31:0] consumed;
    // 本地译码读地址，避免原始地址扇出到 1536 位可变切片，
    // 并隔离能力寄存器与状态数据的选择逻辑。
    wire [13:0] read_word_selected /* synthesis syn_keep=1 */;
    wire [127:0] packet_read_words [0:11];
    wire [127:0] register_read_data;
    genvar read_word;
    generate
        for (read_word=0;read_word<14;read_word=read_word+1) begin : g_read_decode
            localparam [31:0] WORD_ADDRESS=32'h0e0+read_word;
            assign read_word_selected[read_word]=(read_addr==WORD_ADDRESS);
        end
        for (read_word=0;read_word<12;read_word=read_word+1) begin : g_packet_read
            assign packet_read_words[read_word]=
                {128{read_word_selected[read_word+2] && head_valid}} &
                head[read_word*128+:128];
        end
    endgenerate
    assign register_read_data=
        ({128{read_word_selected[0]}} & {32'd1,32'd192,FRAME_PERIOD,32'h31445246}) |
        ({128{read_word_selected[1]}} & {consumed,counters[31:0],31'd0,head_valid,32'd0}) |
        packet_read_words[0] | packet_read_words[1] |
        packet_read_words[2] | packet_read_words[3] |
        packet_read_words[4] | packet_read_words[5] |
        packet_read_words[6] | packet_read_words[7] |
        packet_read_words[8] | packet_read_words[9] |
        packet_read_words[10] | packet_read_words[11];
    wire pop=reg_wr && reg_addr==32'h0ee && head_valid && reg_data==head[127:96];
    assign record_ready=!sending;
    lane_line_detect #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.ROI_TOP(ROI_TOP)) u_lane (
        .clk(pix_clk),.rst_n(pix_reset[2]),.vsync(boundary),.de(de),.rgb(rgb),
        .source_sequence(source_sequence),
        .result(lane),.result_update(lane_update)
    );
    crosswalk_white_detect #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.ROI_TOP(ROI_TOP)) u_crosswalk (
        .clk(pix_clk),.rst_n(pix_reset[2]),.vsync(boundary),.de(de),.rgb(rgb),
        .source_sequence(source_sequence),
        .result(crosswalk),.result_update(cross_update)
    );
    fpga_road_capture #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.PERIOD(PERIOD)) u_capture (
        .clk(pix_clk),.rst_n(pix_reset[2]),.boundary(boundary),.de(de),
        .source_epoch(source_epoch),.source_sequence(source_sequence),.source_timestamp(source_timestamp),
        .lane_update(lane_update),.cross_update(cross_update),.lane_result(lane),.cross_result(crosswalk),
        .record_valid(record_valid),.record(record),.record_ready(record_ready),.dropped(dropped)
    );
    // 每条结果拆为 12 拍入队，避免浅而宽的 FIFO 浪费块 RAM。
    // 仅完整包可以成为主机可见的队首。
    video_token_fifo_bram #(.WIDTH(128),.ADDR_BITS(8)) u_results (
        .wr_clk(pix_clk),.wr_rst_n(pix_reset[2]),.wr_valid(sending),.wr_data(serial_data[127:0]),
        .wr_ready(fifo_ready),.rd_clk(bus_clk),.rd_rst_n(bus_reset[2]),
        .rd_valid(fifo_valid),.rd_data(fifo_data),.rd_ready(!head_valid),.rd_count()
    );
    always @(posedge pix_clk or negedge pix_reset[2]) begin
        if (!pix_reset[2]) begin sending<=0; send_index<=0; serial_data<=0; end
        else if (record_valid) begin sending<=1; send_index<=0; serial_data<=record; end
        else if (sending && fifo_ready) begin
            serial_data<=serial_data>>128;
            if (send_index==11) sending<=0;
            else send_index<=send_index+1'b1;
        end
    end
    stage1_status_cdc u_counters (
        .rst_n(rst_n),.src_clk(pix_clk),.dst_clk(bus_clk),
        .source_state({96'd0,dropped}),.state_snapshot(counters),.generation()
    );
    always @(posedge bus_clk or negedge bus_reset[2]) begin
        if (!bus_reset[2]) begin
            read_data<=0; consumed<=0; head<=0; head_valid<=0; receive_index<=0;
        end
        else begin
            if (pop) begin consumed<=consumed+1'b1; head_valid<=0; end
            if (!head_valid && fifo_valid) begin
                head[receive_index*128+:128]<=fifo_data;
                if (receive_index==11) begin head_valid<=1; receive_index<=0; end
                else receive_index<=receive_index+1'b1;
            end
            if (read_en) read_data<=register_read_data;
        end
    end
endmodule
