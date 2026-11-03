#ifndef FPGA_ROAD_UAPI_H
#define FPGA_ROAD_UAPI_H
#include <linux/types.h>
#include <linux/ioctl.h>

#define FPGA_ROAD_MAGIC 0x31445246U
#define FPGA_ROAD_RECORD_BYTES 192U
#define FPGA_ROAD_CAP 0x0e00U
#define FPGA_ROAD_STATUS 0x0e10U
#define FPGA_ROAD_DATA 0x0e20U
#define FPGA_ROAD_POP 0x0ee0U
/* All record words are little endian. ioctl prefix fields are native endian.
 * Prefix: magic, ABI, native epoch/sequence, timestamp lo/hi, source, flags.
 * Then lane result (32 dwords), crosswalk result (4), reserved (4).
 * Lane result bits 0..255 are diagnostics; eight 80-bit slots start at bit 256.
 */
struct vdma_fpga_road {
    __u32 version, size, dropped, reserved;
    __u32 record[48];
};
#define VDMA_FPGA_ROAD_READ _IOWR('V', 0x77, struct vdma_fpga_road)
struct vdma_fpga_road_status {
    __u32 version, size, magic, period, record_bytes, abi;
    __u32 head_valid, dropped, consumed, reserved;
};
/* Read-only capability/queue query; never consumes a result or starts DMA. */
#define VDMA_FPGA_ROAD_STATUS _IOWR('V', 0x78, struct vdma_fpga_road_status)
#endif
