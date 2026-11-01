module eth_video_decode (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        udp_valid,
    input  wire [7:0]  udp_data,
    input  wire        udp_first,
    input  wire        udp_last,
    output wire        udp_ready,
    input  wire        fault,
    output wire        token_valid,
    output wire [1:0]  token_kind,
    output wire [255:0] token_data,
    input  wire        token_ready
);
    parameter TIMEOUT_CYCLES = 12500000;

    localparam [2:0] RECV0=3'd0, CHECK1=3'd1, START2=3'd2, DATA3=3'd3, END4=3'd4, ABORT5=3'd5;

    reg [2:0]  state;
    reg [7:0]  h [0:39];
    reg [255:0] pixels [0:19];
    reg [9:0]  cnt;
    reg [1:0]  phase;
    reg [8:0]  pixelindex;
    reg [7:0]  pairRG;
    reg [31:0] crc;
    reg        packetbad;
    reg        active;
    reg [31:0] session;
    reg [31:0] frame;
    reg [15:0] nextindex;
    reg [1:0]  column;
    reg        oddrow;
    reg        emitodd;
    reg [4:0]  beat;
    reg        poison;
    reg [23:0] timer;

    wire udp_hs = udp_valid && udp_ready;
    wire token_hs = token_valid && token_ready;
    wire timeout_event = active && (timer >= TIMEOUT_CYCLES[23:0]-24'd1);

    // CRC32 使用反射多项式 0xEDB88320。
    function [31:0] CRCbyte;
        input [31:0] c;
        input [7:0]  d;
        reg [31:0] x;
        integer i;
        begin
            x = c ^ {24'd0, d};
            for (i = 0; i < 8; i = i + 1)
                x = (x >> 1) ^ (32'hEDB88320 & ({32{x[0]}}));
            CRCbyte = x;
        end
    endfunction

    wire [31:0] hdr_index    = {h[16], h[17]};
    wire [31:0] expected_off = ({16'd0, hdr_index[15:0]} << 10) - ({16'd0, hdr_index[15:0]} << 6);
    wire [31:0] framebytes   = {h[32], h[33], h[34], h[35]};
    wire [31:0] hdr_crc      = {h[36], h[37], h[38], h[39]};
    wire        header_valid =
        ({h[0],h[1],h[2],h[3]} == 32'h51535631) &&
        (h[4] == 8'd1) && (h[5] == 8'd1) &&
        ({h[6],h[7]} == 16'd40) &&
        (hdr_index < 32'd3240) &&
        ({h[18],h[19]} == 16'd3240) &&
        ({h[20],h[21],h[22],h[23]} == expected_off) &&
        ({h[24],h[25]} == 16'd960) &&
        ({h[26],h[27]} == 16'd1920) &&
        ({h[28],h[29]} == 16'd1080) &&
        ({h[30],h[31]} == 16'd0) &&
        (framebytes == 32'd3110400) &&
        (~crc == hdr_crc) &&
        (pixelindex == 9'd320) &&
        (!packetbad);

    assign udp_ready   = (state == RECV0);
    assign token_valid = (state >= START2);
    assign token_kind  = (state == START2) ? 2'd0 :
                         (state == DATA3)  ? 2'd1 :
                         (state == END4)   ? 2'd2 : 2'd3;
    assign token_data  = (state == DATA3) ? pixels[beat] : 256'd0;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= RECV0; cnt <= 10'd0; phase <= 2'd0; pixelindex <= 9'd0;
            pairRG <= 8'd0; crc <= 32'hFFFFFFFF; packetbad <= 1'b0; active <= 1'b0;
            session <= 32'd0; frame <= 32'd0; nextindex <= 16'd0; column <= 2'd0;
            oddrow <= 1'b0; emitodd <= 1'b0; beat <= 5'd0; poison <= 1'b0; timer <= 24'd0;
        end else begin
            if (active && !udp_hs) begin
                if (timer < TIMEOUT_CYCLES[23:0]) timer <= timer + 24'd1;
            end else if (!active) begin
                timer <= 24'd0;
            end else begin
                timer <= 24'd0;
            end

            if (state != ABORT5 && (poison || fault || timeout_event) &&
                (state == RECV0 || state == CHECK1 || (token_valid && token_ready))) begin
                state <= ABORT5;
                cnt <= 10'd0;
                active <= 1'b0;
            end else begin
                case (state)
                    RECV0: begin
                        if (udp_hs) begin
                            if (cnt == 10'd0) begin
                                packetbad <= !udp_first;
                                h[0] <= udp_data;
                                crc <= CRCbyte(32'hFFFFFFFF, udp_data);
                                phase <= 2'd0;
                                pixelindex <= 9'd0;
                            end else begin
                                if (cnt < 10'd36 || cnt >= 10'd40)
                                    crc <= CRCbyte(crc, udp_data);
                                if (cnt < 10'd40)
                                    h[cnt[5:0]] <= udp_data;
                            end

                            if (cnt >= 10'd40 && cnt < 10'd1000) begin
                                case (phase)
                                    2'd0: begin
                                        pairRG <= udp_data;
                                        phase <= 2'd1;
                                    end
                                    2'd1: begin
                                        pixels[pixelindex[8:4]][pixelindex[3:0]*16 +: 16] <=
                                            {pairRG[7:4], 1'b0, pairRG[3:0], 2'b0, udp_data[7:4], 1'b0};
                                        pixelindex <= pixelindex + 9'd1;
                                        phase <= 2'd2;
                                    end
                                    default: begin
                                        phase <= 2'd0;
                                    end
                                endcase
                            end else if (cnt < 10'd40) begin
                                phase <= 2'd0;
                            end

                            if (cnt != 10'd0 && udp_first)
                                packetbad <= 1'b1;

                            if (udp_last || cnt == 10'd999) begin
                                if (!udp_last || cnt != 10'd999)
                                    packetbad <= 1'b1;
                                state <= CHECK1;
                                cnt <= 10'd0;
                            end else begin
                                cnt <= cnt + 10'd1;
                            end
                        end
                    end

                    CHECK1: begin
                        if (!poison && header_valid) begin
                            if (hdr_index == 32'd0) begin
                                session <= {h[8],h[9],h[10],h[11]};
                                frame   <= {h[12],h[13],h[14],h[15]};
                                active <= 1'b1;
                                nextindex <= 16'd1;
                                column <= 2'd1;
                                oddrow <= 1'b0;
                                emitodd <= 1'b0;
                                state <= START2;
                                beat <= 5'd0;
                            end else if (active &&
                                         ({h[8],h[9],h[10],h[11]} == session) &&
                                         ({h[12],h[13],h[14],h[15]} == frame) &&
                                         (hdr_index[15:0] == nextindex)) begin
                                nextindex <= nextindex + 16'd1;
                                emitodd <= oddrow;
                                beat <= 5'd0;
                                if (column == 2'd2) begin
                                    column <= 2'd0;
                                    oddrow <= ~oddrow;
                                end else begin
                                    column <= column + 2'd1;
                                end
                                if (hdr_index == 32'd3239) begin
                                    active <= 1'b0;
                                    state <= END4;
                                end else begin
                                    state <= oddrow ? RECV0 : DATA3;
                                end
                            end else if (!active) begin
                                state <= RECV0;
                            end else begin
                                active <= 1'b0;
                                state <= ABORT5;
                            end
                        end else begin
                            active <= 1'b0;
                            state <= ABORT5;
                        end
                    end

                    START2: begin
                        if (token_hs) begin
                            state <= DATA3;
                            beat <= 5'd0;
                        end
                    end

                    DATA3: begin
                        if (token_hs) begin
                            if (beat < 5'd19)
                                beat <= beat + 5'd1;
                            else
                                state <= RECV0;
                        end
                    end

                    END4: begin
                        if (token_hs) begin
                            state <= RECV0;
                        end
                    end

                    ABORT5: begin
                        if (token_hs) begin
                            if (!fault)
                                poison <= 1'b0;
                            state <= RECV0;
                        end
                    end

                    default: state <= RECV0;
                endcase
            end

            if (fault || timeout_event)
                poison <= 1'b1;
        end
    end
endmodule
