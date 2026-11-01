`timescale 1ns/1ps
// 清空 FIFO 时两域均须置复位，释放可各自同步完成。
// 队首寄存器缓存 RAM 中的一项，不额外增加 FIFO 容量。
module video_token_fifo_bram #(
    parameter WIDTH = 315,
    parameter ADDR_BITS = 9
) (
    input  wire             wr_clk,
    input  wire             wr_rst_n,
    input  wire             wr_valid,
    input  wire [WIDTH-1:0] wr_data,
    output wire             wr_ready,
    input  wire             rd_clk,
    input  wire             rd_rst_n,
    output wire             rd_valid,
    output wire [WIDTH-1:0] rd_data,
    input  wire             rd_ready,
    output wire [ADDR_BITS:0] rd_count
);
    localparam DEPTH = 1 << ADDR_BITS;
    localparam [ADDR_BITS:0] FULL_MASK = {2'b11, {(ADDR_BITS-1){1'b0}}};

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [WIDTH-1:0] head_data;
    reg head_valid, full;
    reg [ADDR_BITS:0] wr_bin, rd_bin;
    reg [ADDR_BITS:0] wr_gray /* synthesis syn_preserve=1 */;
    reg [ADDR_BITS:0] rd_gray /* synthesis syn_preserve=1 */;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] rd_gray_sync1 /* synthesis syn_preserve=1 */;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] rd_gray_sync2 /* synthesis syn_preserve=1 */;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] wr_gray_sync1 /* synthesis syn_preserve=1 */;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] wr_gray_sync2 /* synthesis syn_preserve=1 */;

    wire push = wr_valid && wr_ready;
    wire pop = rd_valid && rd_ready;
    wire [ADDR_BITS:0] wr_bin_next = wr_bin + push;
    wire [ADDR_BITS:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    wire [ADDR_BITS:0] rd_bin_next = rd_bin + pop;
    wire [ADDR_BITS:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;
    wire full_next = wr_gray_next == (rd_gray_sync2 ^ FULL_MASK);
    wire next_available = rd_gray_next != wr_gray_sync2;
    wire replace_head = !head_valid || rd_ready;
    wire load_head = rd_rst_n && replace_head && next_available;
    reg [ADDR_BITS:0] wr_bin_synced;
    integer k;
    always @* begin
        wr_bin_synced[ADDR_BITS]=wr_gray_sync2[ADDR_BITS];
        for(k=ADDR_BITS-1;k>=0;k=k-1)
            wr_bin_synced[k]=wr_bin_synced[k+1]^wr_gray_sync2[k];
    end
    // 根据同步后的写指针保守计算可读项数。
    assign rd_count=wr_bin_synced-rd_bin;

    assign wr_ready = wr_rst_n && !full;
    assign rd_valid = rd_rst_n && head_valid;
    assign rd_data = head_data;

    // RAM 端口及读数据寄存器不使用异步复位，以便推断块 RAM。
    always @(posedge wr_clk) begin
        if (push)
            mem[wr_bin[ADDR_BITS-1:0]] <= wr_data;
    end

    // 出队时预读下一项，以便连续读取。
    // 反压期间保持 RAM 地址和队首数据不变。
    always @(posedge rd_clk) begin
        if (load_head)
            head_data <= mem[rd_bin_next[ADDR_BITS-1:0]];
    end

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin <= 0;
            wr_gray <= 0;
            full <= 1'b0;
        end else begin
            wr_bin <= wr_bin_next;
            wr_gray <= wr_gray_next;
            full <= full_next;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin <= 0;
            rd_gray <= 0;
            head_valid <= 1'b0;
        end else begin
            rd_bin <= rd_bin_next;
            rd_gray <= rd_gray_next;
            if (replace_head)
                head_valid <= next_available;
        end
    end

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

`ifndef SYNTHESIS
    initial begin
        if (WIDTH < 1 || ADDR_BITS < 1)
            $fatal(1, "video FIFO requires WIDTH >= 1 and ADDR_BITS >= 1");
    end
`endif
endmodule
