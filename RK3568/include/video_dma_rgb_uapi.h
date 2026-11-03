#ifndef VIDEO_DMA_RGB_UAPI_H
#define VIDEO_DMA_RGB_UAPI_H
#include <linux/types.h>
#include <linux/ioctl.h>

#define VDMA_RGB_BYTES 691200U
#define VDMA_RGB_SLOT_BYTES 692224U
#define VDMA_RGB_SLOTS 3U
#define VDMA_RGB_MAP_BYTES (VDMA_RGB_SLOT_BYTES * VDMA_RGB_SLOTS)
#define VDMA_RGB_MMAP_PAGE 0x20000U

struct vdma_rgb_caps {
    __u32 version, size, abi, width, height, stride, bytes, format;
    __u32 preprocessing, slot_bytes, slots, mmap_page, mmap_bytes, reserved[3];
};
struct vdma_rgb_control {
    __u32 version, size, enable, reserved;
};
struct vdma_rgb_frame {
    __u32 version, size, serial, host_slot, width, height, stride, bytes;
    __u32 format, preprocessing, epoch, sequence, config, layout, reserved[2];
};
struct vdma_rgb_stats {
    __u32 version, size, producer, consumer, dropped, committed;
    __u32 writer_rejected, mailbox_skipped, capture_rejected, memory_status;
    __u32 control, acquired, reserved[4];
};
struct vdma_rgb_export {
    __u32 version, size, slot, reserved;
};
#define VDMA_RGB_CAPS _IOWR('V', 0x70, struct vdma_rgb_caps)
#define VDMA_RGB_CONTROL _IOW('V', 0x71, struct vdma_rgb_control)
#define VDMA_RGB_ACQUIRE _IOWR('V', 0x72, struct vdma_rgb_frame)
#define VDMA_RGB_RELEASE _IOW('V', 0x73, struct vdma_rgb_frame)
#define VDMA_RGB_STATS _IOWR('V', 0x74, struct vdma_rgb_stats)
#define VDMA_RGB_EXPORT _IOW('V', 0x75, struct vdma_rgb_export)

static inline int vdma_rgb_decode(const __u32 words[16], __u32 serial,
                                  struct vdma_rgb_frame *out)
{
    if (words[0] != serial || words[4] != 4 || words[5] != VDMA_RGB_BYTES ||
        words[6] != 4 || words[7] > 4 || words[11] != 1 ||
        words[12] != ((360U << 16) | 640U) || words[13] != 1920 ||
        words[14] != 2 || words[15] >= VDMA_RGB_SLOTS)
        return -1;
    *out = (struct vdma_rgb_frame){
        .version = 1, .size = sizeof(*out), .serial = serial,
        .host_slot = words[15], .width = 640, .height = 360, .stride = 1920,
        .bytes = VDMA_RGB_BYTES, .format = 2, .preprocessing = 1,
        .epoch = words[9], .sequence = words[8], .config = words[10],
        .layout = words[7], .reserved = {0, 0}
    };
    return 0;
}
#endif
