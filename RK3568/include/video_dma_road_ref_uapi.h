#ifndef VIDEO_DMA_ROAD_REF_UAPI_H
#define VIDEO_DMA_ROAD_REF_UAPI_H
#include <linux/types.h>
#include <linux/ioctl.h>
#include "road_ref_wire.h"

#define VDMA_RR_VERSION 1U
/* Independent page offsets, outside native video and legacy plane mappings. */
#define VDMA_RR_MMAP_PAGE 0x10000U
struct vdma_rr_caps {
    __u32 version, size, slots, slot_bytes;
    __u32 payload_bytes, mmap_page, record_bytes, reserved;
};
struct vdma_rr_control {
    __u32 version, size, period, reserved;
    __u32 session, producer, consumer, status;
};
struct vdma_rr_frame {
    __u32 version, size, token, session, slot, reserved[3];
    RoadRefWireRecord record; /* Exact little-endian wire bytes. */
};
/* Bound to the held video slot/token. These are source-native identities,
 * distinct from the 1080p display-canvas identity. */
struct vdma_native_identity {
    __u32 version, size, slot, token;
    __u32 native_sequence, native_epoch, timestamp_lo, timestamp_hi;
};
/* Read-only DMA-BUF; successful ioctl returns its fd. The buffer is stable
 * only while the corresponding RR lease is held by this owner. */
struct vdma_rr_export {
    __u32 version, size, slot, reserved;
};
#define VDMA_RR_CAPS _IOWR('V', 0x70, struct vdma_rr_caps)
#define VDMA_RR_START _IOWR('V', 0x71, struct vdma_rr_control)
#define VDMA_RR_STOP _IO('V', 0x72)
#define VDMA_RR_ACQUIRE _IOWR('V', 0x73, struct vdma_rr_frame)
#define VDMA_RR_RELEASE _IOW('V', 0x74, struct vdma_rr_frame)
#define VDMA_NATIVE_IDENTITY _IOWR('V', 0x75, struct vdma_native_identity)
#define VDMA_RR_EXPORT _IOW('V', 0x76, struct vdma_rr_export)
#endif
