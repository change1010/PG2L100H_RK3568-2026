`timescale 1ns/1ps

// 分发暗光 v2 和 Y/C 控制请求；已退役的旧 ROAD 窗口读零。
// 保留公共画布接口和暗光控制层级。
module feature_control_mux #(
    parameter integer CANVAS_WIDTH=1920, CANVAS_HEIGHT=1080,
    parameter integer GRID_COLS=32, GRID_ROWS=18,
    parameter integer LEASE_CYCLES=625000000,
    parameter integer ENABLE_PLANE=1
)(
    input rst_n, input pix_clk, input bus_clk, input run,
    input control_boundary,
    input capture_boundary, input capture_begin, input capture_end, input capture_invalid,
    input [31:0] capture_sequence, input [31:0] capture_epoch,
    input [2:0] capture_source, input pixel_de, input [15:0] pixel_rgb565,
    input [31:0] image_generation, input [31:0] image_request,
    output [31:0] capture_config_generation,
    output sample_enable, output [7:0] sample_threshold, output [7:0] sample_strength,
    output [31:0] sample_generation, output [31:0] sample_request,
    input reg_wr, input [31:0] reg_addr, input [31:0] reg_data,
    input read_en, input [31:0] read_addr, output [127:0] read_data,
    // ---- 车道灰度平面通道（BAR0 0x127-0x13F）----
    output plane_enable, output [3:0] plane_period_div,
    output [24:0] plane_base, output [15:0] plane_slot_beats, output [1:0] plane_slot_count,
    output [15:0] plane_geom_w, output [15:0] plane_geom_h, output [3:0] plane_format,
    output [63:0] plane_host_addr, output [63:0] plane_completion,
    output [31:0] plane_consumer,
    input [191:0] plane_diagnostics,
    input [31:0] plane_frame_count, input [31:0] plane_color_gen,
    input [1:0] plane_slot_index, input plane_busy, input plane_overflow,
    input plane_src_lost, input [7:0] plane_overflow_row, input [31:0] plane_dropped
);
    wire dark_write = reg_addr>=32'h160 && reg_addr<=32'h176;
    wire dark_read = read_addr>=32'h160 && read_addr<=32'h176;
    wire plane_write = reg_addr >=32'h127 && reg_addr <=32'h13f;
    wire plane_read  = read_addr>=32'h127 && read_addr<=32'h13f;
    wire [127:0] dark_data, plane_data;
    reg [1:0] selected;
    (* ASYNC_REG = "TRUE" *) reg [2:0] bus_reset;
    wire bus_rst_n = bus_reset[2];
    always @(posedge bus_clk or negedge rst_n)
        if (!rst_n) bus_reset <= 0; else bus_reset <= {bus_reset[1:0],1'b1};
    always @(posedge bus_clk or negedge bus_rst_n)
        if (!bus_rst_n) selected <= 0;
        else if (read_en) selected <= dark_read ? 2'd1 : plane_read ? 2'd3 : 2'd0;
    assign read_data = selected==1 ? dark_data : selected==3 ? plane_data : 128'd0;
    // 旧 ROAD 配置已退役；暗光配置保留独立版本号，
    // 帧存储通过公共 epoch 跟踪图像变化。
    assign capture_config_generation = 32'd0;
    dark_enhance_control_bank dark (
        .rst_n(rst_n), .bus_clk(bus_clk), .pix_clk(pix_clk),
        .reg_wr(reg_wr && dark_write), .reg_addr(reg_addr), .reg_data(reg_data),
        .read_en(read_en && dark_read), .read_addr(read_addr), .read_data(dark_data),
        .control_boundary(control_boundary), .capture_boundary(capture_boundary),
        .capture_begin(capture_begin), .capture_epoch(capture_epoch),
        .capture_sequence(capture_sequence), .capture_source(capture_source),
        .image_generation(image_generation), .image_request(image_request),
        .sample_enable(sample_enable), .sample_threshold(sample_threshold),
        .sample_strength(sample_strength), .sample_generation(sample_generation),
        .sample_request(sample_request)
    );
    generate if (ENABLE_PLANE != 0) begin : g_plane_control
    road_plane_ctl u_plane_ctl (
        .rst_n(rst_n), .bus_clk(bus_clk),
        .reg_wr(reg_wr && plane_write), .reg_addr(reg_addr), .reg_data(reg_data),
        .read_en(read_en && plane_read), .read_addr(read_addr), .read_data(plane_data),
        .plane_enable(plane_enable), .period_div(plane_period_div), .plane_base(plane_base),
        .slot_beats(plane_slot_beats), .slot_count(plane_slot_count),
        .geom_w(plane_geom_w), .geom_h(plane_geom_h), .plane_format(plane_format),
        .host_addr(plane_host_addr), .completion_base(plane_completion),
        .plane_consumer(plane_consumer),
        .frame_count(plane_frame_count), .color_gen(plane_color_gen), .diagnostics(plane_diagnostics),
        .slot_index(plane_slot_index), .busy(plane_busy), .overflow(plane_overflow),
        .src_lost(plane_src_lost), .overflow_row(plane_overflow_row), .dropped(plane_dropped)
    );
    end else begin : g_no_plane_control
        assign plane_data=128'd0;
        assign plane_enable=1'b0;
        assign plane_period_div=4'd1;
        assign plane_base=25'd0;
        assign plane_slot_beats=16'd0;
        assign plane_slot_count=2'd0;
        assign plane_geom_w=16'd0;
        assign plane_geom_h=16'd0;
        assign plane_format=4'd0;
        assign plane_host_addr=64'd0;
        assign plane_completion=64'd0;
        assign plane_consumer=32'd0;
    end endgenerate
endmodule
