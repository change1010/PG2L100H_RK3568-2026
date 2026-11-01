`timescale 1ns/1ps
module frame_fifo_write
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 23,
	parameter BUSRT_BITS             = 10,
	parameter BURST_SIZE             = 16
)               
(
	input                            rst,                  
	input                            mem_clk,                    // 存储控制器用户时钟

	output reg                       wr_burst_req,               // 发起突发写请求
	output reg[BUSRT_BITS - 1:0]     wr_burst_len,               // 突发写长度，单位为数据拍
	output reg[ADDR_BITS - 1:0]      wr_burst_addr,              // 突发写起始地址
	input                            wr_burst_data_req,          // 预取写数据请求，数据在下一拍给出
	input                            wr_burst_finish,            // 本次写突发完成

	input                            write_req,                  // 帧写入请求，保持到 write_req_ack 有效
	output reg                       write_req_ack,              // 帧写入请求应答
	output                           write_finish,               // 整帧写入完成
	input[ADDR_BITS - 1:0]           write_addr_0,               // 写入帧槽 0 的基地址
	input[ADDR_BITS - 1:0]           write_addr_1,               // 写入帧槽 1 的基地址
	input[ADDR_BITS - 1:0]           write_addr_2,               // 写入帧槽 2 的基地址
	input[ADDR_BITS - 1:0]           write_addr_3,               // 写入帧槽 3 的基地址
	input[1:0]                       write_addr_index,           // 按索引选择写帧槽基地址
	input[ADDR_BITS - 1:0]           write_len,                  // 整帧写入长度
	output reg                       fifo_aclr,                  // FIFO 异步清零
	input[15:0]                      rdusedw                     // 写缓存读侧可读字数
);
localparam ONE                       = 256'd1;                   // 全一常量，可按所需位宽取切片
localparam ZERO                      = 256'd0;                   // 全零常量
// 写状态编码
localparam S_IDLE                    = 0;                        // 空闲，等待帧写入请求
localparam S_ACK                     = 1;                        // 写入请求应答
localparam S_CHECK_FIFO              = 2;                        // 确认 FIFO 已有足够数据发起一个写突发
localparam S_WRITE_BURST             = 3;                        // 发起写突发
localparam S_WRITE_BURST_END         = 4;                        // 一个写突发完成
localparam S_END                     = 5;                        // 整帧写入完成

reg                                 write_req_d0;                // 写请求同步到 mem_clk 域的第一级
reg                                 write_req_d1;                // 第二级
reg                                 write_req_d2;                // 第三级；后续状态机使用此级请求
reg[ADDR_BITS - 1:0]                write_len_d0;                // 写入长度跨域采样第一级；请求握手期间须保持稳定
reg[ADDR_BITS - 1:0]                write_len_d1;                // 第二级
reg[ADDR_BITS - 1:0]                write_len_latch;             // 锁存本次帧写入长度
reg[ADDR_BITS - 1:0]                write_cnt;                   // 累计已写入的数据拍数
reg[1:0]                            write_addr_index_d0;
reg[1:0]                            write_addr_index_d1;
reg[3:0]                            state;                       // 状态机
// S_WRITE_BURST 的超时计数（见该状态里的说明）。125MHz 下 20000 拍 = 160us，
// 远大于一次 16 拍突发 + 仲裁的最坏耗时，又远小于一帧 16.7ms。
reg[15:0]                           burst_timer;
localparam [15:0]                   BURST_TIMEOUT = 16'd20000;

assign write_finish = (state == S_END) ? 1'b1 : 1'b0;            // 在 S_END 状态报告整帧写入完成
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		write_req_d0    <=  1'b0;
		write_req_d1    <=  1'b0;
		write_req_d2    <=  1'b0;
		write_len_d0    <=  ZERO[ADDR_BITS - 1:0];              
		write_len_d1    <=  ZERO[ADDR_BITS - 1:0];              
		write_addr_index_d0    <=  2'b00;
		write_addr_index_d1    <=  2'b00;
	end
	else
	begin
		write_req_d0    <=  write_req;
		write_req_d1    <=  write_req_d0;
		write_req_d2    <=  write_req_d1;
		write_len_d0    <=  write_len;
		write_len_d1    <=  write_len_d0;
		write_addr_index_d0 <= write_addr_index;
		write_addr_index_d1 <= write_addr_index_d0;
	end 
end


always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		write_len_latch <= ZERO[ADDR_BITS - 1:0];
		wr_burst_addr <= ZERO[ADDR_BITS - 1:0];
		wr_burst_req <= 1'b0;
		write_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		write_req_ack <= 1'b0;
		wr_burst_len <= ZERO[BUSRT_BITS - 1:0];
		burst_timer  <= 16'd0;
	end
	else
		case(state)
			// 空闲时等待同步后的写请求，随后进入 S_ACK。
			S_IDLE:
			begin
				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				write_req_ack <= 1'b0;
			end
			// 在 S_ACK 应答请求、清空 FIFO，并锁存帧槽地址和长度。
			S_ACK:
			begin
				// 请求撤销后清除应答，进入 FIFO 水位检查。
				if(write_req_d2 == 1'b0)
				begin
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;
					write_req_ack <= 1'b0;
				end
				else
				begin
					// 写请求应答
					write_req_ack <= 1'b1;
					// 清空 FIFO
					fifo_aclr <= 1'b1;
					// 按索引选择写帧槽基地址
					if(write_addr_index_d1 == 2'd0)
						wr_burst_addr <= write_addr_0;
					else if(write_addr_index_d1 == 2'd1)
						wr_burst_addr <= write_addr_1;
					else if(write_addr_index_d1 == 2'd2)
						wr_burst_addr <= write_addr_2;
					else if(write_addr_index_d1 == 2'd3)
						wr_burst_addr <= write_addr_3;
					// 锁存长度
					write_len_latch <= write_len_d1;                    
				end
				// 清零已写拍数
				write_cnt <= ZERO[ADDR_BITS - 1:0];
				burst_timer <= 16'd0;
			end
			S_CHECK_FIFO:
			begin
				// 新帧写请求优先进入 S_ACK。
				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				// FIFO 数据足够时发起一个写突发。
				else if(rdusedw >= BURST_SIZE)
				begin
					state <= S_WRITE_BURST;
					wr_burst_len <= BURST_SIZE[BUSRT_BITS - 1:0];
					wr_burst_req <= 1'b1;
					burst_timer <= 16'd0;
				end
			end
			
			S_WRITE_BURST:
			begin
				// 本次突发完成
				if(wr_burst_finish == 1'b1)
				begin
					wr_burst_req <= 1'b0;
					state <= S_WRITE_BURST_END;
					// 累计本次突发拍数
					write_cnt <= write_cnt + BURST_SIZE[ADDR_BITS - 1:0];
					// 地址前进到下一突发
					wr_burst_addr <= wr_burst_addr + BURST_SIZE[ADDR_BITS - 1:0];
				end
				// 超时自愈。本状态只认 wr_burst_finish，而 finish 来自 mem_write_arbi
				// 的 CHx_END 单拍脉冲：仲裁器看门狗、跨时钟采样、或 mux 的 state2 卡住
				// 都可能让它丢一拍。丢掉之后这里会永久停住、wr_burst_req 永不撤销，
				// 而 vsync 到来只会回 S_ACK 重装地址，也不会解开这个状态 —— 整块帧缓存
				// 从此不再写入，读侧照旧每帧重读旧槽，就是“一帧定格”。
				// 超时按“本突发视作完成”处理：撤 req、前进计数器，最多丢一个突发。
				else if(burst_timer == BURST_TIMEOUT)
				begin
					wr_burst_req <= 1'b0;
					state <= S_WRITE_BURST_END;
					write_cnt <= write_cnt + BURST_SIZE[ADDR_BITS - 1:0];
					wr_burst_addr <= wr_burst_addr + BURST_SIZE[ADDR_BITS - 1:0];
				end
				else
					burst_timer <= burst_timer + 16'd1;
			end
			S_WRITE_BURST_END:
			begin
				// 新帧写请求优先进入 S_ACK。
				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				// 未达到整帧长度则继续写突发，
				// 否则结束本帧写入。
				else if(write_cnt < write_len_latch)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				state <= S_IDLE;
			end
			default:
				state <= S_IDLE;
		endcase
end
endmodule
