module eth_ch3_writer #(
    parameter integer FRAME_BEATS = 32400,
    parameter [24:0]  SLOT0 = 25'd388800,
    parameter [24:0]  SLOT1 = 25'd421200,
    parameter [24:0]  SLOT2 = 25'd453600,
    parameter [24:0]  SLOT3 = 25'd486000
)(
    input                clk,
    input                rst_n,
    input                token_valid,
    input      [1:0]     token_kind,
    input      [255:0]   token_data,
    input                wr_burst_data_req,
    input                wr_burst_finish,
    input                read_owned,
    input      [1:0]     read_owned_slot,
    input                retiring_valid,
    input      [1:0]     retiring_slot,
    output               token_ready,
    output reg           wr_burst_req,
    output reg  [9:0]    wr_burst_len,
    output reg  [24:0]   wr_burst_addr,
    output reg  [255:0]  wr_burst_data,
    output reg           active,
    output reg  [1:0]    write_slot,
    output reg           latest_valid,
    output reg  [1:0]    latest_slot,
    output reg           frame_committed
);

    localparam [24:0] FRAME_BEATS_W = FRAME_BEATS;

    reg [255:0] mem [0:15];
    reg [4:0]   count;
    reg [4:0]   send_count;
    reg [24:0]  done_count;
    reg         sending;

    reg [24:0]  base_sel;
    reg         avail0, avail1, avail2, avail3;

    always @(*) begin
        avail0 = (latest_valid && latest_slot == 2'd0) ||
                 (read_owned && read_owned_slot == 2'd0) ||
                 (retiring_valid && retiring_slot == 2'd0);
        avail1 = (latest_valid && latest_slot == 2'd1) ||
                 (read_owned && read_owned_slot == 2'd1) ||
                 (retiring_valid && retiring_slot == 2'd1);
        avail2 = (latest_valid && latest_slot == 2'd2) ||
                 (read_owned && read_owned_slot == 2'd2) ||
                 (retiring_valid && retiring_slot == 2'd2);
        avail3 = (latest_valid && latest_slot == 2'd3) ||
                 (read_owned && read_owned_slot == 2'd3) ||
                 (retiring_valid && retiring_slot == 2'd3);

        if (!avail0)      base_sel = SLOT0;
        else if (!avail1) base_sel = SLOT1;
        else if (!avail2) base_sel = SLOT2;
        else              base_sel = SLOT3;
    end

    assign token_ready = (!sending && (count < 5'd16));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_burst_req   <= 1'b0;
            wr_burst_len   <= 10'd16;
            wr_burst_addr  <= 25'd0;
            wr_burst_data  <= 256'd0;
            active         <= 1'b0;
            write_slot     <= 2'd0;
            latest_valid   <= 1'b0;
            latest_slot    <= 2'd0;
            frame_committed<= 1'b0;
            count          <= 5'd0;
            send_count     <= 5'd0;
            done_count     <= 25'd0;
            sending        <= 1'b0;
        end else begin
            frame_committed <= 1'b0;

            if (sending) begin
                // 与 frame_fifo_write 一致，请求预取下一拍的数据，
                // 包含 AXI 主控首拍。
                if (wr_burst_data_req && send_count < 5'd16) begin
                    wr_burst_data <= mem[send_count[3:0]];
                    send_count <= send_count + 5'd1;
                end

                if (wr_burst_finish &&
                    (send_count == 5'd16 ||
                     (send_count == 5'd15 && wr_burst_data_req))) begin
                    sending      <= 1'b0;
                    wr_burst_req <= 1'b0;
                    count        <= 5'd0;
                    send_count   <= 5'd0;
                    done_count   <= done_count + 25'd16;
                    wr_burst_addr <= wr_burst_addr + 25'd16;
                end
            end else if (count == 5'd16) begin
                sending      <= 1'b1;
                wr_burst_req <= 1'b1;
                send_count   <= 5'd0;
            end else if (token_valid && token_ready) begin
                case (token_kind)
                    2'd0: begin
                        active     <= 1'b1;
                        write_slot <= (avail0 ? (avail1 ? (avail2 ? 2'd3 : 2'd2) : 2'd1) : 2'd0);
                        wr_burst_addr <= base_sel;
                        count      <= 5'd0;
                        done_count <= 25'd0;
                    end
                    2'd1: begin
                        if (active) begin
                            if ((done_count + {20'd0, count}) < FRAME_BEATS_W) begin
                                mem[count[3:0]] <= token_data;
                                count <= count + 5'd1;
                            end else begin
                                active <= 1'b0;
                                count  <= 5'd0;
                            end
                        end
                    end
                    2'd2: begin
                        if (active &&
                            done_count == FRAME_BEATS_W &&
                            count == 5'd0) begin
                            latest_valid    <= 1'b1;
                            latest_slot     <= write_slot;
                            frame_committed <= 1'b1;
                        end
                        active <= 1'b0;
                        count  <= 5'd0;
                    end
                    2'd3: begin
                        active <= 1'b0;
                        count  <= 5'd0;
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule
