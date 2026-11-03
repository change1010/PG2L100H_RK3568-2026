`timescale 1ns/1ps

// 暗光增强 v2 寄存器组，保留 v1 命令语义。
// 读请求延迟一拍响应；帧关联信息来自像素流水线之后。
module dark_enhance_control_bank (
    input rst_n, input bus_clk, input pix_clk,
    input reg_wr, input [31:0] reg_addr, input [31:0] reg_data,
    input read_en, input [31:0] read_addr, output [127:0] read_data,
    input control_boundary,
    input capture_boundary, input capture_begin,
    input [31:0] capture_epoch, input [31:0] capture_sequence,
    input [2:0] capture_source,
    input [31:0] image_generation, input [31:0] image_request,
    output sample_enable, output [7:0] sample_threshold, output [7:0] sample_strength,
    output [31:0] sample_generation, output [31:0] sample_request
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] bus_reset;
    wire bus_rst_n = bus_reset[2];
    always @(posedge bus_clk or negedge rst_n)
        if (!rst_n) bus_reset <= 0; else bus_reset <= {bus_reset[1:0],1'b1};
    wire [127:0] base_read_data;
    wire base_write = reg_wr && (reg_addr==32'h163 || reg_addr==32'h164 ||
        reg_addr==32'h165 || reg_addr==32'h169);
    wire override_read = read_addr==32'h161 || read_addr==32'h16b ||
        read_addr>=32'h16d;
    dark_enhance_ctrl ctrl (
        .rst_n(rst_n), .bus_clk(bus_clk), .pix_clk(pix_clk),
        .reg_wr(base_write), .reg_addr(reg_addr), .reg_data(reg_data),
        .read_en(read_en && !override_read), .read_addr(read_addr), .read_data(base_read_data),
        .frame_boundary(control_boundary), .sample_enable(sample_enable),
        .sample_threshold(sample_threshold), .sample_strength(sample_strength),
        .sample_generation(sample_generation), .sample_request(sample_request)
    );

    wire applied_valid, first_capture_valid;
    wire [31:0] applied_generation, applied_request, applied_epoch, applied_sequence;
    wire [31:0] first_capture_epoch, first_capture_sequence;
    wire [2:0] applied_source, first_capture_source;
    dark_enhance_frame_tracker tracker (
        .rst_n(rst_n), .pix_clk(pix_clk), .capture_boundary(capture_boundary),
        .capture_begin(capture_begin), .capture_epoch(capture_epoch),
        .capture_sequence(capture_sequence), .capture_source(capture_source),
        .image_generation(image_generation), .image_request(image_request),
        .applied_valid(applied_valid), .applied_generation(applied_generation),
        .applied_request(applied_request), .applied_epoch(applied_epoch),
        .applied_sequence(applied_sequence), .applied_source(applied_source),
        .first_capture_valid(first_capture_valid), .first_capture_epoch(first_capture_epoch),
        .first_capture_sequence(first_capture_sequence), .first_capture_source(first_capture_source)
    );
    // 九个 32 位字对应偏移 0x16d..0x175，低地址字放低位。
    wire [287:0] image_state = {
        29'd0,first_capture_source,first_capture_sequence,first_capture_epoch,
        29'd0,applied_source,applied_sequence,applied_epoch,applied_request,applied_generation,
        30'd0,first_capture_valid,applied_valid};
    wire [287:0] image_snapshot;
    wire [31:0] image_revision;
    dark_enhance_snapshot_cdc snapshots (
        .rst_n(rst_n), .pix_clk(pix_clk), .bus_clk(bus_clk), .state_pix(image_state),
        .snapshot_bus(image_snapshot), .revision_bus(image_revision)
    );
    reg override_selected;
    reg [31:0] override_data;
    assign read_data = override_selected ? {96'd0,override_data} : base_read_data;
    always @(posedge bus_clk or negedge bus_rst_n) begin
        if (!bus_rst_n) begin override_selected <= 0; override_data <= 0; end
        else if (read_en) begin
            override_selected <= override_read;
            override_data <= 0;
            // 固定切片避免共享 PIO 地址驱动 32 位可变移位器。
            case (read_addr)
                32'h161: override_data <= 32'd2;
                32'h16b: override_data <= 32'h0002007f;
                32'h16d: override_data <= image_snapshot[31:0];
                32'h16e: override_data <= image_snapshot[63:32];
                32'h16f: override_data <= image_snapshot[95:64];
                32'h170: override_data <= image_snapshot[127:96];
                32'h171: override_data <= image_snapshot[159:128];
                32'h172: override_data <= image_snapshot[191:160];
                32'h173: override_data <= image_snapshot[223:192];
                32'h174: override_data <= image_snapshot[255:224];
                32'h175: override_data <= image_snapshot[287:256];
                32'h176: override_data <= image_revision;
                default: ;
            endcase
        end
    end
endmodule
