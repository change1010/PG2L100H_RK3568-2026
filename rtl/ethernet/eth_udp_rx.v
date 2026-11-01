module eth_udp_rx(
  input clk, rst_n,
  input gmii_valid, gmii_error,
  input [7:0] gmii_data,
  output wire udp_valid,
  output wire [7:0] udp_data,
  output wire udp_first, udp_last,
  input udp_ready,
  output reg fault, arp_request,
  output reg [47:0] sender_mac,
  output reg [31:0] sender_ip
);
  parameter [47:0] LOCAL_MAC = 48'h020000000001;
  parameter [31:0] LOCAL_IP  = 32'hC0A80164;
  parameter [15:0] UDP_PORT  = 16'd5000;

  reg [7:0] RAM [0:1][0:999];
  reg [7:0] h [0:41];
  reg [11:0] count;
  reg [2:0] prefcount;
  reg inframe, drop, err, bank;
  reg [1:0] busy, pending;

  reg [31:0] crc;
  reg [19:0] ip_sum;
  reg [25:0] udp_sum;
  reg [7:0] pair_hi;
  reg [7:0] udp_len_hi, udp_len_lo;

  reg replaying;
  reg playbank;
  reg [9:0] pos;

  function [31:0] crc_byte;
    input [31:0] c;
    input [7:0] d;
    reg [31:0] x;
    integer i;
    begin
      x = c ^ {24'd0, d};
      for (i = 0; i < 8; i = i + 1)
        if (x[0]) x = (x >> 1) ^ 32'hEDB88320;
        else      x = (x >> 1);
      crc_byte = x;
    end
  endfunction

  function [15:0] fold;
    input [31:0] s;
    reg [16:0] t;
    begin
      t = {1'b0, s[15:0]} + {1'b0, s[31:16]};
      t = {1'b0, t[15:0]} + {1'b0, t[16]};
      fold = t[15:0];
    end
  endfunction

  wire [15:0] ip_fold = fold({12'd0, ip_sum});
  // 末尾四个帧校验字节期间 UDP 和已稳定，
  // 利用这段时间流水计算伪首部求和及进位折叠，再判定帧有效性。
  reg [17:0] pseudo_source, pseudo_destination;
  reg [18:0] pseudo_sum;
  reg [26:0] udp_total;
  reg [15:0] udp_fold;
  always @(posedge clk) begin
    pseudo_source <= {2'd0,h[26],h[27]} + {2'd0,h[28],h[29]};
    pseudo_destination <= {2'd0,h[30],h[31]} + {2'd0,h[32],h[33]} + 18'd1025;
    pseudo_sum <= {1'b0,pseudo_source}+{1'b0,pseudo_destination};
    udp_total <= {1'b0,udp_sum}+{8'd0,pseudo_sum};
    udp_fold <= fold({5'd0,udp_total});
  end


  wire dst_mac  = (h[0]==LOCAL_MAC[47:40]) && (h[1]==LOCAL_MAC[39:32]) &&
                  (h[2]==LOCAL_MAC[31:24]) && (h[3]==LOCAL_MAC[23:16]) &&
                  (h[4]==LOCAL_MAC[15:8])  && (h[5]==LOCAL_MAC[7:0]);
  wire dst_ip   = (h[30]==LOCAL_IP[31:24]) && (h[31]==LOCAL_IP[23:16]) &&
                  (h[32]==LOCAL_IP[15:8])  && (h[33]==LOCAL_IP[7:0]);
  wire dst_port = (h[36]==UDP_PORT[15:8]) && (h[37]==UDP_PORT[7:0]);
  wire ip_ok    = (h[14]==8'h45) && (h[16]==8'h04) && (h[17]==8'h04) &&
                  ((h[20]==8'h00)||(h[20]==8'h40)) && (h[21]==8'h00) &&
                  (h[23]==8'h11) && (h[38]==8'h03) && (h[39]==8'hF0);
  wire udp_cksum_ok = (h[40]==8'h00 && h[41]==8'h00) || (udp_fold == 16'hFFFF);
  assign udp_valid = replaying;
  assign udp_data  = RAM[playbank][pos];
  assign udp_first = (pos == 0);
  assign udp_last  = (pos == 999);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      count <= 0; prefcount <= 0; inframe <= 0; drop <= 0; err <= 0;
      bank <= 0; busy <= 0; pending <= 0; crc <= 32'hFFFFFFFF;
      ip_sum <= 0; udp_sum <= 0; pair_hi <= 0; udp_len_hi <= 0; udp_len_lo <= 0;
      replaying <= 0; playbank <= 0; pos <= 0;
         
      fault <= 0; arp_request <= 0; sender_mac <= 0; sender_ip <= 0;
    end else begin
      fault <= 0; arp_request <= 0;

      if (gmii_valid) begin
        if (!inframe) begin
          if (gmii_data == 8'h55) begin
            if (prefcount < 7) prefcount <= prefcount + 1;
          end
          else if (gmii_data == 8'hD5 && prefcount == 7) begin
            prefcount <= 0; inframe <= 1; err <= gmii_error; count <= 0;
            crc <= 32'hFFFFFFFF; ip_sum <= 0; udp_sum <= 0;
            drop <= 0;
            if (!busy[0]) begin bank <= 0; busy[0] <= 1; end
            else if (!busy[1]) begin bank <= 1; busy[1] <= 1; end
            else begin drop <= 1; fault <= 1; end
          end
          else prefcount <= 0;
        end else begin
          err <= err || gmii_error;
          if (count < 2048) begin
            crc <= crc_byte(crc, gmii_data);
            if (count < 42) h[count] <= gmii_data;
            else if (count < 1042 && !drop) RAM[bank][count-42] <= gmii_data;

            if (count >= 14 && count < 34) begin
              if (count[0]) ip_sum <= ip_sum + {4'd0, pair_hi, gmii_data};
              else pair_hi <= gmii_data;
            end
            if (count >= 34 && count < 1042) begin
              if (count[0]) udp_sum <= udp_sum + {10'd0, pair_hi, gmii_data};
              else pair_hi <= gmii_data;
            end
            if (count == 38) udp_len_hi <= gmii_data;
            if (count == 39) udp_len_lo <= gmii_data;
            count <= count + 1;
          end else begin
            count <= count; err <= 1;
          end
        end
      end else begin
        prefcount <= 0;
        if (inframe) begin
          inframe <= 0;
          if (!drop) begin
            if (h[12]==8'h08 && h[13]==8'h00 && dst_mac && dst_ip && dst_port) begin
              if (!err && crc==32'hDEBB20E3 && count==1046 && ip_ok && ip_fold==16'hFFFF && udp_cksum_ok) begin
                pending[bank] <= 1;
              end else begin
                busy[bank] <= 0; fault <= 1;
              end
            end else begin
              busy[bank] <= 0;
              if (h[12]==8'h08 && h[13]==8'h06 && h[14]==8'h00 && h[15]==8'h01 &&
                  h[16]==8'h08 && h[17]==8'h00 && h[18]==8'h06 && h[19]==8'h04 &&
                  h[20]==8'h00 && h[21]==8'h01 &&
                  h[38]==LOCAL_IP[31:24] && h[39]==LOCAL_IP[23:16] &&
                  h[40]==LOCAL_IP[15:8] && h[41]==LOCAL_IP[7:0] &&
                  crc==32'hDEBB20E3 && !err && count>=64) begin
                arp_request <= 1;
                sender_mac <= {h[22],h[23],h[24],h[25],h[26],h[27]};
                sender_ip  <= {h[28],h[29],h[30],h[31]};
              end
            end
          end
        end
      end

      if (!replaying) begin
        if (pending != 2'b00) begin
          replaying <= 1;
          playbank <= pending[0] ? 0 : 1;
          pos <= 0;
           
           
        end else begin
            
        end
      end else begin
        
        
        
        
        if (pos == 999) begin
          if (udp_ready) begin
            busy[playbank] <= 0;
            pending[playbank] <= 0;
            replaying <= 0;  
          end
        end else if (udp_ready) begin
          pos <= pos + 1;
        end
      end
    end
  end
endmodule
