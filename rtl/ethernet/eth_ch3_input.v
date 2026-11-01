// 连接以太网收发、视频解码及 ch3 帧缓存写入。
module eth_ch3_input (
    input  wire        core_clk,
    input  wire        rst_n,
    input  wire        rgmii_rxc,
    input  wire        rgmii_rx_ctl,
    input  wire [3:0]  rgmii_rxd,
    output wire        rgmii_txc,
    output wire        rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd,
    output wire        wr_burst_req,
    output wire [9:0]  len,
    output wire [24:0] addr,
    output wire [255:0] data,
    input  wire        wr_burst_data_req,
    input  wire        wr_burst_finish,
    input  wire        read_req_ack,
    output reg  [1:0]  read_index,
    output reg         display_valid
);

    // RGMII 接口
    wire        eth_clk;
    wire        eth_clk_90p_unused;
    wire        mac_rx_error;
    wire        mac_rx_valid;
    wire [7:0]  mac_rx_data;
    wire        mac_tx_data_valid;
    wire [7:0]  mac_tx_data;

    assign eth_clk_90p_unused = 1'b0;

    rgmii_interface u_rgmii (
        .rst             (~rst_n),
        .rgmii_clk       (eth_clk),
        .rgmii_clk_90p   (eth_clk_90p_unused),
        .mac_tx_data_valid(mac_tx_data_valid),
        .mac_tx_data     (mac_tx_data),
        .mac_rx_error    (mac_rx_error),
        .mac_rx_data_valid(mac_rx_valid),
        .mac_rx_data     (mac_rx_data),
        .rgmii_rxc       (rgmii_rxc),
        .rgmii_rx_ctl    (rgmii_rx_ctl),
        .rgmii_rxd       (rgmii_rxd),
        .rgmii_txc       (rgmii_txc),
        .rgmii_tx_ctl    (rgmii_tx_ctl),
        .rgmii_txd       (rgmii_txd)
    );

    // 以太网时钟域同步释放复位。
    reg [2:0] eth_reset_sync;
    wire      eth_rst_n = eth_reset_sync[2];

    always @(posedge eth_clk or negedge rst_n) begin
        if (!rst_n) eth_reset_sync <= 3'b000;
        else        eth_reset_sync <= {eth_reset_sync[1:0], 1'b1};
    end

    // 接收 UDP
    wire        udp_valid;
    wire [7:0]  udp_data;
    wire        udp_first;
    wire        udp_last;
    wire        udp_ready;
    wire        udp_fault;
    wire        arp_request;
    wire [47:0] sender_mac;
    wire [31:0] sender_ip;

    eth_udp_rx u_udp_rx (
        .clk        (eth_clk),
        .rst_n      (eth_rst_n),
        .gmii_valid (mac_rx_valid),
        .gmii_error (mac_rx_error),
        .gmii_data (mac_rx_data),
        .udp_valid  (udp_valid),
        .udp_data  (udp_data),
        .udp_first  (udp_first),
        .udp_last   (udp_last),
        .udp_ready  (udp_ready),
        .fault      (udp_fault),
        .arp_request(arp_request),
        .sender_mac(sender_mac),
        .sender_ip(sender_ip)
    );

    // 视频协议解码
    wire        token_valid;
    wire [1:0]  token_kind;
    wire [255:0] token_data;
    wire        token_ready;

    eth_video_decode u_decode (
        .clk        (eth_clk),
        .rst_n      (eth_rst_n),
        .udp_valid  (udp_valid),
        .udp_data  (udp_data),
        .udp_first  (udp_first),
        .udp_last   (udp_last),
        .udp_ready  (udp_ready),
        .fault      (udp_fault),
        .token_valid(token_valid),
        .token_kind(token_kind),
        .token_data(token_data),
        .token_ready(token_ready)
    );

    // 发送 ARP 响应
    wire        arp_gmii_valid;
    wire [7:0]  arp_data8;

    eth_arp_tx u_arp_tx (
        .clk        (eth_clk),
        .rst_n      (eth_rst_n),
        .request    (arp_request),
        .sender_mac48(sender_mac),
        .sender_ip32(sender_ip),
        .gmii_valid (arp_gmii_valid),
        .data8      (arp_data8)
    );

    assign mac_tx_data_valid = arp_gmii_valid;
    assign mac_tx_data       = arp_data8;

    // 跨域记录 FIFO
    wire        fifo_wr_ready;
    wire        fifo_rd_valid;
    wire [257:0] fifo_rd_data;
    wire        fifo_rd_ready;

    wire [257:0] fifo_wr_data = {token_kind, token_data};

    eth_token_fifo u_token_fifo (
        .wr_clk    (eth_clk),
        .wr_rst_n  (eth_rst_n),
        .wr_valid  (token_valid),
        .wr_data(fifo_wr_data),
        .wr_ready  (fifo_wr_ready),
        .rd_clk    (core_clk),
        .rd_rst_n  (rst_n),
        .rd_valid  (fifo_rd_valid),
        .rd_data(fifo_rd_data),
        .rd_ready  (fifo_rd_ready)
    );

    assign token_ready = fifo_wr_ready;

    // DDR 帧写入
    wire        w_token_valid = fifo_rd_valid;
    wire [1:0]  w_token_kind  = fifo_rd_data[257:256];
    wire [255:0] w_token_data = fifo_rd_data[255:0];
    wire        w_token_ready;

    wire        w_active;
    wire [1:0]  w_write_slot;
    wire        w_latest_valid;
    wire [1:0]  w_latest_slot;
    wire        w_frame_committed;

    eth_ch3_writer u_writer (
        .clk              (core_clk),
        .rst_n            (rst_n),
        .token_valid      (w_token_valid),
        .token_kind      (w_token_kind),
        .token_data    (w_token_data),
        .token_ready      (w_token_ready),
        .wr_burst_req     (wr_burst_req),
        .wr_burst_len   (len),
        .wr_burst_addr  (addr),
        .wr_burst_data (data),
        .wr_burst_data_req(wr_burst_data_req),
        .wr_burst_finish  (wr_burst_finish),
        .active           (w_active),
        .write_slot      (w_write_slot),
        .latest_valid     (w_latest_valid),
        .latest_slot     (w_latest_slot),
        .frame_committed  (w_frame_committed),
        .read_owned       (1'b1),
        .read_owned_slot (read_index),
        .retiring_valid   (1'b0),
        .retiring_slot   (2'b00)
    );

    assign fifo_rd_ready = w_token_ready;

    // 采样读取槽号和显示有效状态。
    reg ack_d;

    always @(posedge core_clk or negedge rst_n) begin
        if (!rst_n) begin
            read_index    <= 2'b00;
            display_valid <= 1'b0;
            ack_d         <= 1'b0;
        end else begin
            ack_d <= read_req_ack;
            if (read_req_ack && !ack_d) begin
                if (w_latest_valid) begin
                    read_index    <= w_latest_slot;
                    display_valid <= 1'b1;
                end else begin
                    display_valid <= 1'b0;
                end
            end
        end
    end

endmodule
