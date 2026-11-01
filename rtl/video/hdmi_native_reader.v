`timescale 1ns/1ps
// 用四个行缓存读取帧槽；切换布局前通过请求、响应、接收握手
// 排空旧 DDR 事务及缓存数据。
module hdmi_native_reader #(
    parameter integer WIDTH = 1920, HEIGHT = 1080,
    parameter [24:0] BASE = 25'd2097152
) (
    input wire rst_n, mem_clk, pix_clk,
    input wire frame_start, display_enable, quad_mode, read_en,
    output wire [15:0] read_data,
    output wire frame_valid,
    output reg [31:0] underflow_frames, missed_requests,
    input wire latest_valid, source_live,
    input wire [1:0] latest_slot,
    input wire [31:0] latest_sequence,
    input wire [31:0] latest_epoch, input wire [63:0] latest_timestamp,
    input wire [31:0] latest_left_crc,
    output [127:0] native_identity,
    output reg [127:0] native_left_check,
    output [63:0] native_left_counts,
    output reg read_owned, output reg [1:0] read_slot,
    output reg rd_req, output reg [9:0] rd_len,
    output reg [24:0] rd_addr,
    input wire rd_valid, rd_finish, rd_fault,
    input wire [255:0] rd_data
);
    localparam integer ROW_BEATS = WIDTH/16;
    localparam integer ROW_BITS = $clog2(ROW_BEATS);
    localparam integer ROW_STRIDE = 1<<ROW_BITS;
    localparam integer FRAME_BEATS = ROW_BEATS*HEIGHT;
    localparam IDLE=0, DRAIN=1, ACCEPT=2, BANK=3, ISSUE=4,
        BURST=5, ADVANCE=6, STOP=7;
    reg [2:0] state;
    (* ASYNC_REG="TRUE" *) reg [2:0] pix_reset, mem_reset;
    wire pix_reset_n = pix_reset[2], mem_reset_n = mem_reset[2];
    always @(posedge pix_clk or negedge rst_n)
        if (!rst_n) pix_reset <= 0; else pix_reset <= {pix_reset[1:0],1'b1};
    always @(posedge mem_clk or negedge rst_n)
        if (!rst_n) mem_reset <= 0; else mem_reset <= {mem_reset[1:0],1'b1};
    reg request_toggle, request_enable, request_quad, accept_toggle;
    reg response_toggle, response_valid, response_quad;
    reg [31:0] response_sequence;
    reg [31:0] response_epoch;
    reg [63:0] response_timestamp;
    reg [31:0] response_left_crc,pixel_expected_crc;
    reg [127:0] pixel_identity;
    assign native_identity=pixel_identity;
    (* ASYNC_REG="TRUE" *) reg [1:0] request_sync, accept_sync, response_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] live_sync, fault_sync;
    reg request_seen, active_quad, active_generation, reader_fault;
    reg waiting, allow_response, pixel_active, pixel_bad, pixel_quad;
    reg [31:0] pixel_sequence;
    reg [12:0] pixel_x, pixel_y;
    reg [1:0] pixel_bank, write_bank;
    reg [3:0] bank_put, bank_get;
    (* ASYNC_REG="TRUE" *) reg [3:0] put_meta, put_sync, get_meta, get_sync;
    // 从发出 put 到收到对应 get，整组描述信息保持不变。
    reg [12:0] bank_row [0:3];
    reg bank_generation [0:3];
    reg [31:0] bank_sequence [0:3];
    // 行缓存步长取 2 的幂，像素 RAM 地址可直接拼接生成。
    reg [255:0] rows [0:4*ROW_STRIDE-1];
    reg [255:0] previous_row [0:ROW_BEATS-1];
    reg [255:0] pixel_word;
    reg [3:0] pixel_lane;
    reg pixel_word_valid;
    wire bank_ready = put_sync[pixel_bank] != bank_get[pixel_bank] &&
        bank_row[pixel_bank] == pixel_y &&
        bank_generation[pixel_bank] == request_toggle &&
        bank_sequence[pixel_bank] == pixel_sequence;
    wire [12:0] output_width = pixel_quad ? WIDTH/2 : WIDTH;
    wire [ROW_BITS+1:0] pixel_row_address={pixel_bank,pixel_x[ROW_BITS+3:4]};
    assign frame_valid = pixel_active && !pixel_bad && live_sync[1] && !fault_sync[1];
    assign read_data = pixel_word_valid ? pixel_word[pixel_lane*16 +: 16] : 16'd0;

    always @(posedge pix_clk or negedge pix_reset_n) begin
        if (!pix_reset_n) begin
            request_toggle <= 0; request_enable <= 0; request_quad <= 0;
            accept_toggle <= 0; response_sync <= 0; live_sync <= 0; fault_sync <= 0;
            put_meta <= 0; put_sync <= 0; bank_get <= 0;
            waiting <= 0; allow_response <= 0; pixel_active <= 0; pixel_bad <= 1; pixel_quad <= 0;
            pixel_sequence <= 0; pixel_x <= 0; pixel_y <= 0; pixel_bank <= 0;
            pixel_identity<=0;
            pixel_expected_crc<=0;
            pixel_word <= 0; pixel_lane <= 0; pixel_word_valid <= 0;
            underflow_frames <= 0; missed_requests <= 0;
        end else begin
            response_sync <= {response_sync[0], response_toggle};
            live_sync <= {live_sync[0], source_live};
            fault_sync <= {fault_sync[0], reader_fault};
            put_meta <= bank_put; put_sync <= put_meta;
            pixel_word_valid <= 0;
            if (waiting || pixel_bad || !pixel_active) bank_get <= put_sync;
            if (pixel_active && (!live_sync[1] || fault_sync[1])) pixel_bad <= 1;
            if (waiting && response_sync[1] == request_toggle) begin
                waiting <= 0; pixel_active <= response_valid && allow_response;
                pixel_bad <= !(response_valid && allow_response); pixel_quad <= response_quad;
                pixel_sequence <= response_sequence;
                pixel_identity<={response_timestamp,response_epoch,response_sequence};
                pixel_expected_crc<=response_left_crc;
                accept_toggle <= request_toggle;
            end
            if (frame_start) begin
                pixel_active <= 0; pixel_bad <= 1;
                pixel_x <= 0; pixel_y <= 0; pixel_bank <= 0;
                if (!waiting && response_sync[1] == request_toggle) begin
                    request_enable <= display_enable; request_quad <= quad_mode;
                    request_toggle <= !request_toggle; waiting <= 1; allow_response <= 1;
                end else begin
                    missed_requests <= missed_requests + 1'b1; allow_response <= 0;
                    // 仍确认迟到响应以排空存储侧，
                    // 但本次跳过的帧不显示这些行。
                    if (waiting && response_sync[1] == request_toggle) pixel_active <= 0;
                end
            end else if (read_en) begin
                if (frame_valid && bank_ready) begin
                    pixel_word <= rows[pixel_row_address];
                    pixel_lane <= pixel_x[3:0]; pixel_word_valid <= 1;
                    if (pixel_x == output_width-1) begin
                        bank_get[pixel_bank] <= put_sync[pixel_bank];
                        pixel_bank <= pixel_bank + 1'b1;
                        pixel_x <= 0; pixel_y <= pixel_y + 1'b1;
                    end else pixel_x <= pixel_x + 1'b1;
                end else begin
                    if (!pixel_bad) underflow_frames <= underflow_frames + 1'b1;
                    pixel_bad <= 1; allow_response <= 0;
                end
            end
        end
    end

    reg [12:0] source_row;
    reg [10:0] column;
    reg [9:0] received, burst_length;
    reg [3:0] burst_skip;
    reg burst_bad;
    // DDR 按对齐的 16 拍突发读取，只保留属于当前行的数据。
    // 1920 像素奇数行从半突发边界开始，
    // 首个突发只取后 8 拍。
    wire take_word = state == BURST && rd_valid && received >= burst_skip &&
        received < {6'd0,burst_skip} + burst_length;
    wire [10:0] word_column = column + received - burst_skip;
    reg average_pending;
    reg [255:0] upper_word, lower_word;
    reg [10:0] average_column;
    wire [ROW_BITS+1:0] write_row_address={write_bank,word_column[ROW_BITS-1:0]};
    wire [ROW_BITS+1:0] average_row_address={write_bank,average_column[ROW_BITS:1]};
    reg [127:0] half_word;
    wire [127:0] average_word;
    genvar p;
    generate for (p=0; p<8; p=p+1) begin: mean_pixels
        wire [15:0] a = upper_word[p*32 +: 16];
        wire [15:0] b = upper_word[p*32+16 +: 16];
        wire [15:0] c = lower_word[p*32 +: 16];
        wire [15:0] d = lower_word[p*32+16 +: 16];
        // 直接在 RGB565 分量精度下求均值，只舍入一次。
        wire [6:0] r = {2'b0,a[15:11]} + {2'b0,b[15:11]} +
                       {2'b0,c[15:11]} + {2'b0,d[15:11]} + 7'd2;
        wire [7:0] g = {2'b0,a[10:5]} + {2'b0,b[10:5]} +
                       {2'b0,c[10:5]} + {2'b0,d[10:5]} + 8'd2;
        wire [6:0] bsum = {2'b0,a[4:0]} + {2'b0,b[4:0]} +
                          {2'b0,c[4:0]} + {2'b0,d[4:0]} + 7'd2;
        assign average_word[p*16 +: 16] = {r[6:2],g[7:2],bsum[6:2]};
    end endgenerate

    // 同步 RAM 延迟一拍后配对当前数据和上一行，
    // 每拍由 16 像素生成 8 个均值像素。
    always @(posedge mem_clk) begin
        if (take_word) begin
            if (active_quad && !source_row[0]) previous_row[word_column] <= rd_data;
            else if (active_quad) begin
                upper_word <= previous_row[word_column];
                lower_word <= rd_data;
            end
        end
        if (average_pending) begin
            if (!average_column[0]) half_word <= average_word;
            else rows[average_row_address] <=
                {average_word,half_word};
        end else if (take_word && !active_quad)
            rows[write_row_address] <= rd_data;
    end
    always @(posedge mem_clk or negedge mem_reset_n) begin
        if (!mem_reset_n) begin average_pending <= 0; average_column <= 0; end
        else begin
            average_pending <= take_word && active_quad && source_row[0];
            if (take_word) average_column <= word_column;
        end
    end

    reg [10:0] next_length;
    wire [24:0] row_offset = source_row*ROW_BEATS + column;
    always @* begin
        next_length = 16 - {7'd0,row_offset[3:0]};
        if (next_length > ROW_BEATS-column) next_length = ROW_BEATS-column;
    end
    integer n;
    always @(posedge mem_clk or negedge mem_reset_n) begin
        if (!mem_reset_n) begin
            request_sync <= 0; accept_sync <= 0; get_meta <= 0; get_sync <= 0;
            state <= IDLE; request_seen <= 0; response_toggle <= 0;
            response_valid <= 0; response_quad <= 0; response_sequence <= 0;
            response_epoch<=0;response_timestamp<=0;
            response_left_crc<=0;
            active_quad <= 0; active_generation <= 0; reader_fault <= 0;
            read_owned <= 0; read_slot <= 0; bank_put <= 0; write_bank <= 0;
            source_row <= 0; column <= 0; received <= 0; burst_length <= 0;
            burst_skip <= 0;
            burst_bad <= 0; rd_req <= 0; rd_len <= 0; rd_addr <= 0;
            for (n=0; n<4; n=n+1) begin
                bank_row[n] <= 0; bank_generation[n] <= 0; bank_sequence[n] <= 0;
            end
        end else begin
            request_sync <= {request_sync[0], request_toggle};
            accept_sync <= {accept_sync[0], accept_toggle};
            get_meta <= bank_get; get_sync <= get_meta;
            // 突发未结束前保持帧槽和模式不变。
            if (request_sync[1] != request_seen && state != BURST) begin
                rd_req <= 0; state <= DRAIN; request_seen <= request_sync[1];
            end else case (state)
                IDLE, STOP: ;
                DRAIN: if (bank_put == get_sync && !average_pending) begin
                    read_owned <= request_enable && latest_valid && source_live && !rd_fault;
                    read_slot <= latest_slot;
                    response_valid <= request_enable && latest_valid && source_live && !rd_fault;
                    response_quad <= request_quad; response_sequence <= latest_sequence;
                    response_epoch<=latest_epoch;response_timestamp<=latest_timestamp;
                    response_left_crc<=latest_left_crc;
                    active_quad <= request_quad; active_generation <= request_seen;
                    response_toggle <= request_seen; reader_fault <= rd_fault;
                    source_row <= 0; column <= 0; write_bank <= 0; state <= ACCEPT;
                end
                ACCEPT: if (accept_sync[1] == request_seen)
                    state <= read_owned ? BANK : IDLE;
                BANK: if (bank_put[write_bank] == get_sync[write_bank]) state <= ISSUE;
                ISSUE: begin
                    rd_addr <= BASE + read_slot*FRAME_BEATS + {row_offset[24:4],4'd0};
                    rd_len <= 10'd16; burst_length <= next_length;
                    burst_skip <= row_offset[3:0];
                    received <= 0; burst_bad <= rd_fault; rd_req <= 1; state <= BURST;
                end
                BURST: begin
                    if (rd_fault) burst_bad <= 1;
                    if (rd_valid) begin
                        rd_req <= 0;
                        if (received < 16) received <= received + 1'b1;
                        else burst_bad <= 1;
                    end
                    if (rd_finish) begin
                        rd_req <= 0;
                        if (burst_bad || rd_fault ||
                            received + (rd_valid ? 1 : 0) != 16) begin
                            reader_fault <= 1; state <= STOP;
                        end else state <= ADVANCE;
                    end
                end
                ADVANCE: if (!average_pending) begin
                    if (column + burst_length < ROW_BEATS) begin
                        column <= column + burst_length; state <= ISSUE;
                    end else begin
                        column <= 0; source_row <= source_row + 1'b1;
                        if (!active_quad || source_row[0]) begin
                            bank_row[write_bank] <= active_quad ? source_row >> 1 : source_row;
                            bank_sequence[write_bank] <= response_sequence;
                            bank_generation[write_bank] <= active_generation;
                            bank_put[write_bank] <= !bank_put[write_bank];
                            write_bank <= write_bank + 1'b1;
                            state <= source_row == HEIGHT-1 ? STOP : BANK;
                        end else state <= ISSUE;
                    end
                end
                default: state <= STOP;
            endcase
        end
    end

    reg left_pending,last_pending;
    reg [31:0] pixel_crc,left_checks,left_mismatches;
    assign native_left_counts={left_mismatches,left_checks};
    function [31:0] crc16;
        input [31:0] previous;
        input [15:0] value;
        reg [31:0] c;
        integer b;
        begin
            c=previous ^ {16'd0,value};
            for(b=0;b<16;b=b+1) c=c[0] ? (c>>1)^32'hedb88320 : c>>1;
            crc16=c;
        end
    endfunction
    wire [31:0] next_pixel_crc=left_pending ? crc16(pixel_crc,read_data) : pixel_crc;
    always @(posedge pix_clk or negedge pix_reset_n) begin
        if(!pix_reset_n) begin
            left_pending<=0;last_pending<=0;pixel_crc<=32'hffffffff;
            native_left_check<=0;left_checks<=0;left_mismatches<=0;
        end else if(frame_start) begin
            left_pending<=0;last_pending<=0;pixel_crc<=32'hffffffff;
        end else begin
            left_pending<=read_en && frame_valid && bank_ready && !pixel_quad && pixel_x<128;
            last_pending<=read_en && frame_valid && bank_ready && !pixel_quad &&
                pixel_x==WIDTH-1 && pixel_y==HEIGHT-1;
            pixel_crc<=next_pixel_crc;
            if(last_pending && frame_valid) begin
                native_left_check<={pixel_expected_crc,~next_pixel_crc,pixel_identity[63:0]};
                left_checks<=left_checks+1'b1;
                if(~next_pixel_crc!=pixel_expected_crc) left_mismatches<=left_mismatches+1'b1;
            end
        end
    end
endmodule
