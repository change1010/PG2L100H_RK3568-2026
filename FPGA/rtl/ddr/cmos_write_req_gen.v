module cmos_write_req_gen(
	input              rst,
	input              pclk,
	input              cmos_vsync,
	output reg         write_req,
	output reg[1:0]    write_addr_index,
	output reg[1:0]    read_addr_index,
	input              write_req_ack
);
reg cmos_vsync_d0;
reg cmos_vsync_d1;
// write_req_ack 来自 core_clk 域（frame_fifo_write），在像素域直接采样属跨时钟域。
// 补两级同步器。该信号是【电平保持】型（跟随 write_req 直到其撤销），
// 多两拍延迟不影响功能（帧槽索引每帧只变一次、帧内稳定），但消除了亚稳态采样。
reg write_req_ack_d0;
reg write_req_ack_d1;
always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		cmos_vsync_d0 <= 1'b0;
		cmos_vsync_d1 <= 1'b0;
	end
	else
	begin
		cmos_vsync_d0 <= cmos_vsync;
		cmos_vsync_d1 <= cmos_vsync_d0;
	end
end
always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		write_req <= 1'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0) //检测场同步信号上升沿
		write_req <= 1'b1;
	else if(write_req_ack_d1 == 1'b1) //写请求应答后拉低（已同步）
		write_req <= 1'b0;
end
always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		write_addr_index <= 2'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0)  //每次写完一帧累加一次,0~3循环
		write_addr_index <= write_addr_index + 2'd1;
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		read_addr_index <= 2'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0)
		read_addr_index <= write_addr_index;
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		write_req_ack_d0 <= 1'b0;
		write_req_ack_d1 <= 1'b0;
	end
	else
	begin
		write_req_ack_d0 <= write_req_ack;
		write_req_ack_d1 <= write_req_ack_d0;
	end
end
endmodule 