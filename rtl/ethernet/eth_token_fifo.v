module eth_token_fifo #(
    parameter WIDTH = 258,
    parameter ADDR_BITS = 7
)(
    input  wire             wr_clk,
    input  wire             wr_rst_n,
    input  wire             wr_valid,
    input  wire [WIDTH-1:0] wr_data,
    output wire             wr_ready,
    input  wire             rd_clk,
    input  wire             rd_rst_n,
    output wire             rd_valid,
    output wire [WIDTH-1:0] rd_data,
    input  wire             rd_ready
);
    localparam DEPTH = (1 << ADDR_BITS);

    reg [WIDTH-1:0] mem [0:DEPTH-1];

    reg [ADDR_BITS:0] wr_bin, rd_bin;
    reg [ADDR_BITS:0] wr_gray /* synthesis syn_preserve=1 */;
    reg [ADDR_BITS:0] rd_gray /* synthesis syn_preserve=1 */;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] rd_gray_sync1 /* synthesis syn_preserve=1 */;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] rd_gray_sync2 /* synthesis syn_preserve=1 */;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] wr_gray_sync1 /* synthesis syn_preserve=1 */;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] wr_gray_sync2 /* synthesis syn_preserve=1 */;
    reg                full;

    wire [ADDR_BITS:0] wr_bin_nxt   = wr_bin + (wr_valid && wr_ready);
    wire [ADDR_BITS:0] wr_gray_nxt  = (wr_bin_nxt >> 1) ^ wr_bin_nxt;
    wire [ADDR_BITS:0] rd_bin_nxt   = rd_bin + (rd_valid && rd_ready);
    wire [ADDR_BITS:0] rd_gray_nxt  = (rd_bin_nxt >> 1) ^ rd_bin_nxt;

    wire [ADDR_BITS:0] full_cmp = {~rd_gray_sync2[ADDR_BITS:ADDR_BITS-1],
                                    rd_gray_sync2[ADDR_BITS-2:0]};
    wire               full_nxt = (wr_gray_nxt == full_cmp);
    wire               empty    = (rd_gray == wr_gray_sync2);

    assign wr_ready = ~full;
    assign rd_valid = ~empty;
    assign rd_data  = mem[rd_bin[ADDR_BITS-1:0]];

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin  <= 0;
            wr_gray <= 0;
            full    <= 1'b0;
        end else begin
            if (wr_valid && wr_ready) begin
                mem[wr_bin[ADDR_BITS-1:0]] <= wr_data;
                wr_bin  <= wr_bin_nxt;
                wr_gray <= wr_gray_nxt;
            end
            full <= full_nxt;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin  <= 0;
            rd_gray <= 0;
        end else if (rd_valid && rd_ready) begin
            rd_bin  <= rd_bin_nxt;
            rd_gray <= rd_gray_nxt;
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
endmodule
