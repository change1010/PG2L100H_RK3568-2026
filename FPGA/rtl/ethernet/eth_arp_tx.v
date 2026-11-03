module eth_arp_tx #(
    parameter [47:0] LOCAL_MAC48 = 48'h020000000001,
    parameter [31:0] LOCAL_IP32  = 32'hC0A80164
)(
    input            clk,
    input            rst_n,
    input            request,
    input      [47:0] sender_mac48,
    input      [31:0] sender_ip32,
    output reg        gmii_valid,
    output reg  [7:0] data8
);
    localparam [2:0] IDLE=3'd0, PRE=3'd1, BODY=3'd2, FCS=3'd3, IFG=3'd4;
    reg [2:0]  fsm;
    reg [3:0]  ifg;
    reg [6:0]  count;
    reg [47:0] dst_mac;
    reg [31:0] dst_ip;
    reg [31:0] crc_reg;
    reg [31:0] crc_latch;
    reg [47:0] pre_lat;

    function [7:0] frame_byte;
        input [5:0] pos;
        begin
            case (pos)
                6'd0,6'd1,6'd2,6'd3,6'd4,6'd5:
                    frame_byte = dst_mac[47 - pos*8 -: 8];
                6'd6,6'd7,6'd8,6'd9,6'd10,6'd11:
                    frame_byte = LOCAL_MAC48[47 - (pos-6)*8 -: 8];
                6'd12: frame_byte = 8'h08;
                6'd13: frame_byte = 8'h06;
                6'd14: frame_byte = 8'h00;
                6'd15: frame_byte = 8'h01;
                6'd16: frame_byte = 8'h08;
                6'd17: frame_byte = 8'h00;
                6'd18: frame_byte = 8'h06;
                6'd19: frame_byte = 8'h04;
                6'd20: frame_byte = 8'h00;
                6'd21: frame_byte = 8'h02;
                6'd22,6'd23,6'd24,6'd25,6'd26,6'd27:
                    frame_byte = LOCAL_MAC48[47 - (pos-22)*8 -: 8];
                6'd28,6'd29,6'd30,6'd31:
                    frame_byte = LOCAL_IP32[31 - (pos-28)*8 -: 8];
                6'd32,6'd33,6'd34,6'd35,6'd36,6'd37:
                    frame_byte = dst_mac[47 - (pos-32)*8 -: 8];
                6'd38,6'd39,6'd40,6'd41:
                    frame_byte = dst_ip[31 - (pos-38)*8 -: 8];
                default:
                    frame_byte = 8'h00;
            endcase
        end
    endfunction

    function [31:0] crc_next;
        input [31:0] crc;
        input [7:0]  data;
        integer i;
        reg [31:0] c;
        begin
            c = crc ^ {24'h000000, data};
            for (i = 0; i < 8; i = i + 1) begin
                if (c[0])
                    c = (c >> 1) ^ 32'hEDB88320;
                else
                    c = c >> 1;
            end
            crc_next = c;
        end
    endfunction

    wire [7:0] cur_byte;
    assign cur_byte = frame_byte(count[5:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm        <= IDLE;
            ifg        <= 4'd0;
            count      <= 7'd0;
            dst_mac    <= 48'd0;
            dst_ip     <= 32'd0;
            crc_reg    <= 32'hFFFFFFFF;
            crc_latch  <= 32'd0;
            gmii_valid <= 1'b0;
            data8      <= 8'd0;
            pre_lat    <= 48'h555555555555;
        end else begin
            gmii_valid <= 1'b0;
            case (fsm)
                IDLE: begin
                    if (request) begin
                        dst_mac <= sender_mac48;
                        dst_ip  <= sender_ip32;
                        fsm     <= PRE;
                        count   <= 7'd0;
                    end
                end
                PRE: begin
                    gmii_valid <= 1'b1;
                    data8 <= (count == 7'd7) ? 8'hD5 : 8'h55;
                    if (count == 7'd7) begin
                        fsm     <= BODY;
                        count   <= 7'd0;
                        crc_reg <= 32'hFFFFFFFF;
                    end else begin
                        count <= count + 7'd1;
                    end
                end
                BODY: begin
                    gmii_valid <= 1'b1;
                    data8 <= cur_byte;
                    if (count == 7'd59) begin
                        crc_latch <= ~crc_next(crc_reg, cur_byte);
                        fsm       <= FCS;
                        count     <= 7'd0;
                    end else begin
                        crc_reg <= crc_next(crc_reg, cur_byte);
                        count   <= count + 7'd1;
                    end
                end
                FCS: begin
                    gmii_valid <= 1'b1;
                    data8 <= crc_latch[{count[1:0],3'b000} +: 8];
                    if (count == 7'd3) begin
                        fsm   <= IFG;
                        ifg   <= 4'd12;
                        count <= 7'd0;
                    end else begin
                        count <= count + 7'd1;
                    end
                end
                IFG: begin
                    if (ifg == 4'd0) begin
                        fsm <= IDLE;
                    end else begin
                        ifg <= ifg - 4'd1;
                    end
                end
                default: fsm <= IDLE;
            endcase
        end
    end
endmodule
