`timescale 1ns/1ps

// 模块名称：frame_read_write
// 功能说明：
//   DDR 帧缓存读写桥接模块。
//   - 写路径：将 write_clk 域的窄位宽像素流写入异步 FIFO，转换后在 mem_clk 域
//            发起 DDR 突发写请求。
//   - 读路径：在 mem_clk 域接收 DDR 突发读数据写入异步 FIFO，再在 read_clk 域
//            按需输出窄位宽读数据。

// 架构说明：
//   1) write_buf (afifo_16i_64o_512)：跨时钟 + 位宽转换（写侧16bit -> 存储侧更宽）
//   2) frame_fifo_write：根据 FIFO 水位与请求生成 wr_burst_req/addr/len
//   3) read_buf  (afifo_64i_16o_128)：跨时钟 + 位宽转换（存储侧更宽 -> 读侧16bit）
//   4) frame_fifo_read：根据请求生成 rd_burst_req/addr/len 并管理读完成握手

// 使用建议：
//   - rst 为模块总复位；fifo_aclr 通常由帧边界控制，避免跨帧残留数据。
//   - BURST_SIZE 需与存储控制器效率/仲裁策略配合设置。
//   - 当显示端/采集端分属不同时钟域时，本模块是关键的 CDC（跨时钟域）缓冲桥梁。

module frame_read_write
#(
	parameter MEM_DATA_BITS          = 256,
	parameter READ_DATA_BITS         = 16,
	parameter WRITE_DATA_BITS        = 16,
	parameter ADDR_BITS              = 25,
	parameter BUSRT_BITS             = 10,
	parameter BURST_SIZE             = 16
)
(
	input                            rst,
	input                            mem_clk,                    // DDR 控制器用户时钟域

	// ---------------------------- DDR 读突发接口 -----------------------------
	output                           rd_burst_req,               // 突发读请求
	output[BUSRT_BITS - 1:0]         rd_burst_len,               // 突发读长度（单位：数据拍，不是字节）
	output[ADDR_BITS - 1:0]          rd_burst_addr,              // 突发读起始地址
	input                            rd_burst_data_valid,        // 读数据有效
	input[MEM_DATA_BITS - 1:0]       rd_burst_data,              // 读数据总线
	input                            rd_burst_finish,            // 本次突发读完成

	// ------------------------------ 用户读接口 -------------------------------
	input                            read_clk,                   // 用户读时钟域
	input                            read_req,                   // 读事务请求（保持到 ack）
	output                           read_req_ack,               // 读事务应答
	output                           read_finish,                // 读事务完成
	input[ADDR_BITS - 1:0]           read_addr_0,                // 读基地址0
	input[ADDR_BITS - 1:0]           read_addr_1,                // 读基地址1
	input[ADDR_BITS - 1:0]           read_addr_2,                // 读基地址2
	input[ADDR_BITS - 1:0]           read_addr_3,                // 读基地址3
	input[1:0]                       read_addr_index,            // 读地址索引选择
	input[ADDR_BITS - 1:0]           read_len,                   // 读总长度
	input                            read_en,                    // 读使能（弹出一个 READ_DATA_BITS 数据）
	output[READ_DATA_BITS  - 1:0]    read_data,                  // 读数据输出

	// ---------------------------- DDR 写突发接口 -----------------------------
	output                           wr_burst_req/*synthesis PAP_MARK_DEBUG="1"*/,
	output[BUSRT_BITS - 1:0]         wr_burst_len/*synthesis PAP_MARK_DEBUG="1"*/,
	output[ADDR_BITS - 1:0]          wr_burst_addr/*synthesis PAP_MARK_DEBUG="1"*/,
	input                            wr_burst_data_req/*synthesis PAP_MARK_DEBUG="1"*/, // 控制器请求下一拍写数据
	output[MEM_DATA_BITS - 1:0]      wr_burst_data/*synthesis PAP_MARK_DEBUG="1"*/,
	input                            wr_burst_finish/*synthesis PAP_MARK_DEBUG="1"*/,

	// ------------------------------ 用户写接口 -------------------------------
	input                            write_clk,                  // 用户写时钟域
	input                            write_req,                  // 写事务请求（保持到 ack）
	output                           write_req_ack,              // 写事务应答
	output                           write_finish,               // 写事务完成
	input[ADDR_BITS - 1:0]           write_addr_0,               // 写基地址0
	input[ADDR_BITS - 1:0]           write_addr_1,               // 写基地址1
	input[ADDR_BITS - 1:0]           write_addr_2,               // 写基地址2
	input[ADDR_BITS - 1:0]           write_addr_3,               // 写基地址3
	input[1:0]                       write_addr_index,           // 写地址索引选择
	input[ADDR_BITS - 1:0]           write_len,                  // 写总长度
	input                            write_en,                   // 写入一个 WRITE_DATA_BITS 数据
	input[WRITE_DATA_BITS - 1:0]     write_data                  // 写数据输入
);

// FIFO 水位信号：用于读写控制模块判断是否可以发起突发事务
//   - rdusedw：写路径 FIFO 在 mem_clk 域的可读数据量
//   - wrusedw：读路径 FIFO 在 mem_clk 域的已写入数据量
//   控制模块据此避免 FIFO 欠读/溢出。
wire[15:0]                           wrusedw;                    // read_buf 写侧已使用字数（mem_clk 域）
wire[15:0]                           rdusedw /*synthesis PAP_MARK_DEBUG="1"*/; // write_buf 读侧水位（mem_clk 域）
wire                                 read_fifo_aclr;             // 读 FIFO 异步清零
wire                                 write_fifo_aclr;            // 写 FIFO 异步清零

// 写路径 FIFO：用户写时钟域 -> DDR 时钟域

afifo_16i_64o_512 write_buf(
    .wr_clk			(write_clk		),
    .wr_rst			(write_fifo_aclr),  // 一帧开始或结束时复位，避免跨帧脏数据
    .wr_en			(write_en		),
    .wr_data		(write_data		),
    .wr_full		(				),
    .wr_water_level	(				),
    .almost_full	(				),

    .rd_clk			(mem_clk		),
    .rd_rst			(write_fifo_aclr),
    .rd_en			(wr_burst_data_req),
    .rd_data		(wr_burst_data	),
    .rd_empty		(				),
    .rd_water_level	(rdusedw[8:0]	),
    .almost_empty	(				)
	);

// 写事务控制：根据请求/地址索引/长度以及 FIFO 水位产生 DDR 写突发控制信号
frame_fifo_write
#(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BUSRT_BITS                 (BUSRT_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               )
)
frame_fifo_write_m0
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
	.wr_burst_req               (wr_burst_req             ),
	.wr_burst_len               (wr_burst_len             ),
	.wr_burst_addr              (wr_burst_addr            ),
	.wr_burst_data_req          (wr_burst_data_req        ),
	.wr_burst_finish            (wr_burst_finish          ),
	.write_req                  (write_req                ),
	.write_req_ack              (write_req_ack            ),
	.write_finish               (write_finish             ),
	.write_addr_0               (write_addr_0             ),
	.write_addr_1               (write_addr_1             ),
	.write_addr_2               (write_addr_2             ),
	.write_addr_3               (write_addr_3             ),
	.write_addr_index           (write_addr_index         ),
	.write_len                  (write_len                ),
	.fifo_aclr                  (write_fifo_aclr          ),
	.rdusedw                    (rdusedw                  )
);

// 读路径 FIFO：DDR 时钟域 -> 用户读时钟域

afifo_64i_16o_128 read_buf (
    .wr_clk(mem_clk),
    .wr_rst(read_fifo_aclr),
    .wr_en(rd_burst_data_valid),
    .wr_data(rd_burst_data),
    .wr_full(),
    .wr_water_level(wrusedw[8:0]),
    .almost_full(),
    .rd_clk(read_clk),
    .rd_rst(read_fifo_aclr),
    .rd_en(read_en),
    .rd_data(read_data),
    .rd_empty(),
    .rd_water_level(),
    .almost_empty());

// 读事务控制：根据用户读请求和 FIFO 水位调度 DDR 突发读
frame_fifo_read
#(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BUSRT_BITS                 (BUSRT_BITS               ),
	.FIFO_DEPTH                 (128                      ),
	.BURST_SIZE                 (BURST_SIZE               )
)
frame_fifo_read_m0
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
	.rd_burst_req               (rd_burst_req             ),
	.rd_burst_len               (rd_burst_len             ),
	.rd_burst_addr              (rd_burst_addr            ),
	.rd_burst_data_valid        (rd_burst_data_valid      ),
	.rd_burst_finish            (rd_burst_finish          ),
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_finish                (read_finish              ),
	.read_addr_0                (read_addr_0              ),
	.read_addr_1                (read_addr_1              ),
	.read_addr_2                (read_addr_2              ),
	.read_addr_3                (read_addr_3              ),
	.read_addr_index            (read_addr_index          ),
	.read_len                   (read_len                 ),
	.fifo_aclr                  (read_fifo_aclr           ),
	.wrusedw                    (wrusedw                  )
);

endmodule
