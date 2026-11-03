#define _POSIX_C_SOURCE 200809L
#include "fpga_road_uapi.h"
#include "video_dma_rgb_uapi.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s DEVICE (read-only status, no DMA or POP)\n", argv[0]);
        return 2;
    }
    int fd = open(argv[1], O_RDWR | O_CLOEXEC);
    if (fd < 0) { perror("FPGA probe open"); return 1; }
    struct vdma_rgb_caps rgb = {.version = 1, .size = sizeof(rgb)};
    struct vdma_fpga_road_status road = {.version = 1, .size = sizeof(road)};
    int result = 0;
    if (ioctl(fd, VDMA_RGB_CAPS, &rgb)) {
        fprintf(stderr, "RGB_CAPS error=%d %s\n", errno, strerror(errno)); result = 1;
    } else {
        printf("RGB_CAPS abi=%u width=%u height=%u stride=%u bytes=%u format=%u preprocessing=%u slots=%u\n",
            rgb.abi, rgb.width, rgb.height, rgb.stride, rgb.bytes, rgb.format, rgb.preprocessing, rgb.slots);
        if (rgb.abi != 4 || rgb.width != 640 || rgb.height != 360 ||
            rgb.stride != 1920 || rgb.bytes != VDMA_RGB_BYTES ||
            rgb.format != 2 || rgb.preprocessing != 1 || rgb.slots != 3) result = 1;
    }
    if (ioctl(fd, VDMA_FPGA_ROAD_STATUS, &road)) {
        fprintf(stderr, "FPGA_ROAD_STATUS error=%d %s\n", errno, strerror(errno)); result = 1;
    } else {
        printf("FPGA_ROAD_CAP magic=%08x period=%u bytes=%u abi=%u head_valid=%u dropped=%u consumed=%u\n",
            road.magic, road.period, road.record_bytes, road.abi,
            road.head_valid, road.dropped, road.consumed);
        if (road.magic != FPGA_ROAD_MAGIC || road.period < 1 || road.period > 255 ||
            road.record_bytes != FPGA_ROAD_RECORD_BYTES || road.abi != 1) result = 1;
    }
    close(fd);
    return result;
}
