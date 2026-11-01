`timescale 1ns/1ps

// 暗光命令通过握手从总线域传到像素域，仅在帧边界生效。
// 握手完成前保持命令数据稳定，保证参数整体跨域。
module dark_enhance_ctrl #(
    parameter [7:0] DEFAULT_THRESHOLD = 8'd64,
    parameter [7:0] DEFAULT_STRENGTH = 8'd128
)(
    input rst_n,
    input bus_clk,
    input pix_clk,
    input reg_wr,
    input [31:0] reg_addr,
    input [31:0] reg_data,
    input read_en,
    input [31:0] read_addr,
    output reg [127:0] read_data,
    input frame_boundary,
    output reg active_enable,
    output reg [7:0] active_threshold,
    output reg [7:0] active_strength,
    output reg [31:0] active_generation,
    output reg [31:0] active_request,
    output pending,
    // 给出本拍生效的参数，包含帧边界提交值；
    // 下游须与对应像素同步延迟。
    output frame_commit,
    output sample_enable,
    output [7:0] sample_threshold,
    output [7:0] sample_strength,
    output [31:0] sample_generation,
    output [31:0] sample_request
);
    localparam [31:0] REG_MAGIC = 32'h160;
    localparam [31:0] REG_VERSION = 32'h161;
    localparam [31:0] REG_STATUS = 32'h162;
    localparam [31:0] REG_THRESHOLD_SHADOW = 32'h163;
    localparam [31:0] REG_STRENGTH_SHADOW = 32'h164;
    localparam [31:0] REG_CONTROL = 32'h165;
    localparam [31:0] REG_THRESHOLD_ACTIVE = 32'h166;
    localparam [31:0] REG_STRENGTH_ACTIVE = 32'h167;
    localparam [31:0] REG_GENERATION = 32'h168;
    localparam [31:0] REG_REQUEST_SHADOW = 32'h169;
    localparam [31:0] REG_ERROR = 32'h16a;
    localparam [31:0] REG_CAPS = 32'h16b;
    localparam [31:0] REG_REQUEST_ACTIVE = 32'h16c;

    localparam [1:0] CMD_APPLY = 2'd1;
    localparam [1:0] CMD_DISABLE = 2'd2;

    (* ASYNC_REG = "TRUE" *) reg [2:0] bus_reset_pipe;
    (* ASYNC_REG = "TRUE" *) reg [2:0] pix_reset_pipe;
    wire bus_rst_n = bus_reset_pipe[2];
    wire pix_rst_n = pix_reset_pipe[2];
    always @(posedge bus_clk or negedge rst_n)
        if (!rst_n) bus_reset_pipe <= 3'b000;
        else bus_reset_pipe <= {bus_reset_pipe[1:0],1'b1};
    always @(posedge pix_clk or negedge rst_n)
        if (!rst_n) pix_reset_pipe <= 3'b000;
        else pix_reset_pipe <= {pix_reset_pipe[1:0],1'b1};

    reg [7:0] threshold_shadow_bus, strength_shadow_bus;
    reg [31:0] request_shadow_bus;
    reg [31:0] error_bus;
    reg [1:0] command_type_bus;
    reg [7:0] command_threshold_bus, command_strength_bus;
    reg [31:0] command_request_bus;
    reg command_toggle_bus;
    reg command_pending_bus;
    assign pending = command_pending_bus;

    // 同步级使用独立标量，便于 Pango 在综合前后仅约束异步入口。
    
    (* ASYNC_REG = "TRUE" *) reg command_toggle_meta_pix, command_toggle_synced_pix;
    (* ASYNC_REG = "TRUE" *) reg status_toggle_meta_bus, status_toggle_synced_bus;
    wire [1:0] command_toggle_sync_pix = {command_toggle_synced_pix,command_toggle_meta_pix};
    wire [1:0] status_toggle_sync_bus = {status_toggle_synced_bus,status_toggle_meta_bus};
    reg status_toggle_pix;
    reg command_seen_pix, status_toggle_seen_bus;
    reg command_capture_wait_pix, command_pending_pix;
    reg [1:0] command_type_pix;
    reg [7:0] command_threshold_pix, command_strength_pix;
    reg [31:0] command_request_pix;
    reg status_capture_wait_bus, status_snapshot_valid_bus;
    (* ASYNC_REG = "TRUE" *) reg [1:0] active_enable_sync_bus;
    (* ASYNC_REG = "TRUE" *) reg [7:0] active_threshold_meta_bus, active_threshold_sync_bus;
    (* ASYNC_REG = "TRUE" *) reg [7:0] active_strength_meta_bus, active_strength_sync_bus;
    (* ASYNC_REG = "TRUE" *) reg [31:0] active_generation_meta_bus, active_generation_sync_bus;
    (* ASYNC_REG = "TRUE" *) reg [31:0] active_request_meta_bus, active_request_sync_bus;
    reg active_enable_snapshot_bus;
    reg [7:0] active_threshold_snapshot_bus, active_strength_snapshot_bus;
    reg [31:0] active_generation_snapshot_bus, active_request_snapshot_bus;

    assign frame_commit = pix_rst_n && command_pending_pix && frame_boundary;
    assign sample_enable = frame_commit ? command_type_pix == CMD_APPLY : active_enable;
    assign sample_threshold = frame_commit && command_type_pix == CMD_APPLY ?
        command_threshold_pix : active_threshold;
    assign sample_strength = frame_commit && command_type_pix == CMD_APPLY ?
        command_strength_pix : active_strength;
    assign sample_generation = frame_commit ?
        (active_generation == 32'hffffffff ? 32'd1 : active_generation + 1'b1) : active_generation;
    assign sample_request = frame_commit ? command_request_pix : active_request;

    wire [31:0] next_request_id = active_request_snapshot_bus == 32'hffffffff ?
        32'd1 : active_request_snapshot_bus + 1'b1;
    wire request_id_valid = request_shadow_bus != 32'd0 &&
        request_shadow_bus == next_request_id;

    // 总线域保持待执行命令，并接收像素域的帧结果快照。
    always @(posedge bus_clk or negedge bus_rst_n) begin
        if (!bus_rst_n) begin
            threshold_shadow_bus <= DEFAULT_THRESHOLD;
            strength_shadow_bus <= DEFAULT_STRENGTH;
            request_shadow_bus <= 32'd0;
            error_bus <= 32'd0;
            command_type_bus <= 0;
            command_threshold_bus <= DEFAULT_THRESHOLD;
            command_strength_bus <= DEFAULT_STRENGTH;
            command_request_bus <= 0;
            command_toggle_bus <= 0;
            command_pending_bus <= 0;
            status_toggle_meta_bus <= 0; status_toggle_synced_bus <= 0;
            status_toggle_seen_bus <= 0;
            status_capture_wait_bus <= 0;
            status_snapshot_valid_bus <= 1'b1;
            active_enable_sync_bus <= 0;
            active_threshold_meta_bus <= DEFAULT_THRESHOLD;
            active_threshold_sync_bus <= DEFAULT_THRESHOLD;
            active_strength_meta_bus <= DEFAULT_STRENGTH;
            active_strength_sync_bus <= DEFAULT_STRENGTH;
            active_generation_meta_bus <= 0;
            active_generation_sync_bus <= 0;
            active_request_meta_bus <= 0;
            active_request_sync_bus <= 0;
            active_enable_snapshot_bus <= 0;
            active_threshold_snapshot_bus <= DEFAULT_THRESHOLD;
            active_strength_snapshot_bus <= DEFAULT_STRENGTH;
            active_generation_snapshot_bus <= 0;
            active_request_snapshot_bus <= 0;
            read_data <= 0;
        end else begin
            status_toggle_meta_bus <= status_toggle_pix;
            status_toggle_synced_bus <= status_toggle_meta_bus;
            active_enable_sync_bus <= {active_enable_sync_bus[0],active_enable};
            active_threshold_meta_bus <= active_threshold;
            active_threshold_sync_bus <= active_threshold_meta_bus;
            active_strength_meta_bus <= active_strength;
            active_strength_sync_bus <= active_strength_meta_bus;
            active_generation_meta_bus <= active_generation;
            active_generation_sync_bus <= active_generation_meta_bus;
            active_request_meta_bus <= active_request;
            active_request_sync_bus <= active_request_meta_bus;

            if (!status_capture_wait_bus &&
                status_toggle_sync_bus[1] != status_toggle_seen_bus) begin
                status_toggle_seen_bus <= status_toggle_sync_bus[1];
                status_capture_wait_bus <= 1'b1;
                status_snapshot_valid_bus <= 1'b0;
            end else if (status_capture_wait_bus) begin
                // 像素域发出翻转通知前已冻结数据，此时可整体采样。
                active_enable_snapshot_bus <= active_enable_sync_bus[1];
                active_threshold_snapshot_bus <= active_threshold_sync_bus;
                active_strength_snapshot_bus <= active_strength_sync_bus;
                active_generation_snapshot_bus <= active_generation_sync_bus;
                active_request_snapshot_bus <= active_request_sync_bus;
                status_capture_wait_bus <= 1'b0;
                status_snapshot_valid_bus <= 1'b1;
                if (command_pending_bus &&
                    status_toggle_seen_bus == command_toggle_bus)
                    command_pending_bus <= 1'b0;
            end

            if (reg_wr) begin
                case (reg_addr)
                    REG_THRESHOLD_SHADOW: begin
                        if (command_pending_bus) error_bus <= 32'd2;
                        else if (reg_data[31:8] != 0) error_bus <= 32'd1;
                        else begin threshold_shadow_bus <= reg_data[7:0]; error_bus <= 0; end
                    end
                    REG_STRENGTH_SHADOW: begin
                        if (command_pending_bus) error_bus <= 32'd2;
                        else if (reg_data[31:8] != 0) error_bus <= 32'd1;
                        else begin strength_shadow_bus <= reg_data[7:0]; error_bus <= 0; end
                    end
                    REG_REQUEST_SHADOW: begin
                        if (command_pending_bus) error_bus <= 32'd2;
                        else begin request_shadow_bus <= reg_data; error_bus <= 0; end
                    end
                    REG_CONTROL: begin
                        if (command_pending_bus) error_bus <= 32'd2;
                        else if (reg_data == 32'd4) begin
                            threshold_shadow_bus <= DEFAULT_THRESHOLD;
                            strength_shadow_bus <= DEFAULT_STRENGTH;
                            error_bus <= 0;
                        end else if (reg_data != 32'd1 && reg_data != 32'd2) begin
                            error_bus <= 32'd1;
                        end else if (!request_id_valid) begin
                            error_bus <= 32'd1;
                        end else begin
                            command_type_bus <= reg_data[1:0];
                            command_threshold_bus <= threshold_shadow_bus;
                            command_strength_bus <= strength_shadow_bus;
                            command_request_bus <= request_shadow_bus;
                            command_toggle_bus <= ~command_toggle_bus;
                            command_pending_bus <= 1'b1;
                            error_bus <= 0;
                        end
                    end
                    default: begin end
                endcase
            end

            if (read_en) begin
                read_data <= 128'd0;
                case (read_addr)
                    REG_MAGIC: read_data <= {96'd0,32'h4441524b};
                    REG_VERSION: read_data <= {96'd0,32'd1};
                    REG_STATUS: read_data <= {123'd0,status_snapshot_valid_bus,
                        (error_bus != 0),(active_generation_snapshot_bus != 0),
                        command_pending_bus,active_enable_snapshot_bus};
                    REG_THRESHOLD_SHADOW: read_data <= {120'd0,threshold_shadow_bus};
                    REG_STRENGTH_SHADOW: read_data <= {120'd0,strength_shadow_bus};
                    REG_THRESHOLD_ACTIVE: read_data <= {120'd0,active_threshold_snapshot_bus};
                    REG_STRENGTH_ACTIVE: read_data <= {120'd0,active_strength_snapshot_bus};
                    REG_GENERATION: read_data <= {96'd0,active_generation_snapshot_bus};
                    REG_REQUEST_SHADOW: read_data <= {96'd0,request_shadow_bus};
                    REG_ERROR: read_data <= {96'd0,error_bus};
                    REG_CAPS: read_data <= {96'd0,32'h0001003f};
                    REG_REQUEST_ACTIVE: read_data <= {96'd0,active_request_snapshot_bus};
                    default: read_data <= 128'd0;
                endcase
            end
        end
    end

    // 像素域同步命令，在帧边界一次性更新使能和参数。
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            command_toggle_meta_pix <= 0; command_toggle_synced_pix <= 0;
            command_seen_pix <= 0;
            command_capture_wait_pix <= 0;
            command_pending_pix <= 0;
            command_type_pix <= 0;
            command_threshold_pix <= DEFAULT_THRESHOLD;
            command_strength_pix <= DEFAULT_STRENGTH;
            command_request_pix <= 0;
            status_toggle_pix <= 0;
            active_enable <= 0;
            active_threshold <= DEFAULT_THRESHOLD;
            active_strength <= DEFAULT_STRENGTH;
            active_generation <= 0;
            active_request <= 0;
        end else begin
            command_toggle_meta_pix <= command_toggle_bus;
            command_toggle_synced_pix <= command_toggle_meta_pix;

            if (!command_capture_wait_pix &&
                command_toggle_sync_pix[1] != command_seen_pix) begin
                command_seen_pix <= command_toggle_sync_pix[1];
                command_capture_wait_pix <= 1'b1;
            end else if (command_capture_wait_pix) begin
                command_type_pix <= command_type_bus;
                command_threshold_pix <= command_threshold_bus;
                command_strength_pix <= command_strength_bus;
                command_request_pix <= command_request_bus;
                command_capture_wait_pix <= 1'b0;
                command_pending_pix <= 1'b1;
            end

            if (frame_commit) begin
                if (command_type_pix == CMD_APPLY) begin
                    active_enable <= 1'b1;
                    active_threshold <= command_threshold_pix;
                    active_strength <= command_strength_pix;
                end else if (command_type_pix == CMD_DISABLE) begin
                    active_enable <= 1'b0;
                end
                active_request <= command_request_pix;
                if (active_generation == 32'hffffffff)
                    active_generation <= 32'd1;
                else
                    active_generation <= active_generation + 1'b1;
                status_toggle_pix <= command_seen_pix;
                command_pending_pix <= 1'b0;
            end
        end
    end
endmodule
