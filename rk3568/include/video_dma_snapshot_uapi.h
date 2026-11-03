#ifndef VIDEO_DMA_SNAPSHOT_UAPI_H
#define VIDEO_DMA_SNAPSHOT_UAPI_H

#include "video_dma_feature_uapi.h"
#include "video_dma_uapi.h"

#define VDMA_SNAPSHOT_VERSION 1U
#define VDMA_SNAPSHOT_COPY_HELD 1U
#define VDMA_SNAPSHOT_FRAME_BYTES 4147200U

struct vdma_snapshot_caps {
    __u32 version, size, capabilities, frame_bytes;
    __u32 format, width, height, stride;
    __u32 reserved[4];
};

/* Application owns destination. On any error discard ALL destination bytes.
 * Success does not acquire/release a frame or advance either video cursor.
 * expected.slot/token must name a held DISPLAY frame. Pixel bytes have no header.
 * Neither an exported fd nor this operation provides an asynchronous DMA fence.
 */
struct vdma_snapshot_copy {
    __u32 version, size;
    __aligned_u64 destination;
    __u32 capacity, flags;
    struct vdma_frame_identity expected;
    struct vdma_frame_identity actual;
    struct vdma_frame_header header;
    __u32 copied_bytes, reserved[3];
};

#define VDMA_SNAPSHOT_CAPS _IOWR('V', 0x4f, struct vdma_snapshot_caps)
#define VDMA_COPY_HELD_FRAME _IOWR('V', 0x50, struct vdma_snapshot_copy)

#endif
