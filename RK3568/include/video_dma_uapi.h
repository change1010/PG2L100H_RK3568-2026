#ifndef VIDEO_DMA_UAPI_H
#define VIDEO_DMA_UAPI_H
#include <linux/types.h>
#include <linux/ioctl.h>
#define VDMA_LAYOUT_CMOS1 0U
#define VDMA_LAYOUT_CMOS2 1U
#define VDMA_LAYOUT_HDMI 2U
#define VDMA_LAYOUT_ETHERNET 3U
#define VDMA_LAYOUT_QUAD 4U
#define VDMA_LAYOUT_MAX VDMA_LAYOUT_QUAD
#define VDMA_START _IO('V', 0x40)
#define VDMA_STOP _IO('V', 0x41)
struct vdma_stats {
    __u32 producer;
    __u32 consumer;
    __u32 captured;
    __u32 dropped;
    __u32 errors;
    __u32 status;
};
#define VDMA_STATS _IOR('V', 0x42, struct vdma_stats)
/* Diagnostic readback: ROAD dense probe at register 0x15f (bar offset 0x15f0):
   bits[26:24]=dense_status bit[23]=dense_busy bits[22:15]=dense_rows_seen
   bits[14:7]=dense_runs_seen bits[6:0]=packet_status.
   Deliberately outside the 0x140..0x15b window so it never collides with the
   packet lease status word the driver validates against 0x3f. */
#define VDMA_DENSE_DEBUG _IOR('V', 0x53, unsigned int)
struct vdma_frame_header {
    __u32 magic;
    __u32 sequence;
    __u32 width;
    __u32 height;
    __u32 stride;
    __u32 format;
    __u32 layout;
    __u32 bytes;
};
struct vdma_mmap_info {
    __u32 version;
    __u32 slots;
    __u32 slot_bytes;
    __u32 frame_bytes;
};
struct vdma_buffer {
    __u32 slot;
    __u32 token;
    struct vdma_frame_header header;
};
struct vdma_release_buffer {
    __u32 slot;
    __u32 token;
};
#define VDMA_MMAP_INFO _IOR('V', 0x43, struct vdma_mmap_info)
#define VDMA_ACQUIRE _IOR('V', 0x44, struct vdma_buffer)
#define VDMA_RELEASE _IOW('V', 0x45, struct vdma_release_buffer)
struct vdma_export {
    __u32 slot;
    __u32 reserved;
};
#define VDMA_EXPORT _IOW('V', 0x46, struct vdma_export)
#define VDMA_ACQUIRE_NEXT _IOR('V', 0x47, struct vdma_buffer)
/* Independent status extension. Existing video structs/ioctl numbers unchanged.
 * flags: source[2:0], process[5:3], run[6], turn[8:7].
 * canvas_sequence is a live display reference, NOT DMA header.sequence.
 */
struct vdma_control_status {
    __u32 version;
    __u32 generation;
    __u32 flags;
    __u32 event_sequence;
    __u32 alarm_reset_sequence;
    __u32 canvas_sequence;
};
#define VDMA_CONTROL_STATUS _IOR('V', 0x48, struct vdma_control_status)
/* Lane plane channel (BAR0 0x127..0x13F). Read-only status mirror of the RTL
 * block added for the ch2 grey-plane path. 0x53 is already taken by
 * VDMA_DENSE_DEBUG, so this uses 0x54. Every field is the raw register word;
 * decoding is left to the caller so the RTL stays the single source of truth.
 * magic == 0x504c414e ("PLAN") is the only reliable presence test: an older
 * bitstream leaves 0x127..0x13F undecoded, where reads land in the legacy PIO
 * RAM and may legitimately read back as zero.
 */
struct vdma_plane_info {
    __u32 version;
    __u32 size;
    __u32 magic;
    __u32 control;    /* bit0 enable, bits[7:4] period_div */
    __u32 status;     /* bit0 busy, bit1 overflow, bit2 src_lost, [15:8] overflow_row */
    __u32 sequence;
    __u32 generation;
    __u32 base_beat;
    __u32 slot_beats;
    __u32 slot_count;
    __u32 geometry;   /* [31:16] height, [15:0] width */
    __u32 format;
    __u32 slot_index;
    __u32 dropped;
    __u32 reserved[2];
};
#define VDMA_PLANE_INFO _IOR('V', 0x54, struct vdma_plane_info)

/*
 * 平面帧租约。ACQUIRE 返回当前可读的帧描述并锁住宿主窗口（期间驱动不推进 CONSUMER），
 * RELEASE 归还并推进 0x138 CONSUMER，FPGA 才允许写下一帧。
 * offset 是相对设备映射起点的字节偏移，用户态用 mmap 的子窗口（vm_pgoff 判据）取像素。
 */
struct vdma_plane_frame {
    __u32 version;    /* 1; preserve this 48-byte ABI and its ioctl numbers */
    __u32 size;
    __u32 sequence;   /* 提交计数，与 0x129 SEQ 一致 */
    __u32 generation;
    __u32 geometry;   /* [31:16] height, [15:0] width */
    __u32 format;
    __u32 offset;     /* 平面窗口在设备映射中的字节偏移 */
    __u32 length;     /* 有效载荷字节数；V3 为 460800 */
    __u32 flags;
    __u32 reserved[3];
};
#define VDMA_PLANE_ACQUIRE _IOWR('V', 0x55, struct vdma_plane_frame)
#define VDMA_PLANE_RELEASE _IOW('V', 0x56, struct vdma_plane_frame)
/* Extended acquisition has a separate command so old probes remain usable.
 * Release a v2 lease with VDMA_PLANE_RELEASE and &descriptor.frame.
 * Source fields are valid only when frame.flags has VDMA_PLANE_SOURCE_ID;
 * version-1 bitstreams still yield pixels but do not supply source identity.
 */
struct vdma_plane_frame_v2 {
    struct vdma_plane_frame frame;
    __u32 source_epoch;
    __u32 source_sequence;
    __u32 source_layout;
    __u32 source_config_generation;
};
#define VDMA_PLANE_SOURCE_ID 0x00000001U
#define VDMA_PLANE_ACQUIRE_V2 _IOWR('V', 0x57, struct vdma_plane_frame_v2)

/*
 * Versioned plane capability/lease ABI for the production YC path.
 *
 * The legacy GRAY8 ABI above is intentionally frozen: old applications may
 * still be paired with the old bitstream.  The current RTL publishes one
 * tightly packed payload containing Y8 followed by C8, so it needs explicit
 * offsets and lengths instead of overloading frame.length.
 */
#define VDMA_PLANE_FORMAT_GRAY8 0U
#define VDMA_PLANE_FORMAT_YC8 1U
#define VDMA_PLANE_YC 0x00000002U
#define VDMA_PLANE_CAPS_VERSION 1U

struct vdma_plane_caps {
    __u32 version;          /* UAPI capability structure version */
    __u32 size;
    __u32 hardware_version; /* BAR 0x133 */
    __u32 format;           /* VDMA_PLANE_FORMAT_* */
    __u32 kind;             /* completion metadata kind */
    __u32 metadata_version; /* completion metadata version */
    __u32 geometry;         /* [31:16] height, [15:0] width */
    __u32 stride;           /* bytes per row in each plane */
    __u32 payload_bytes;    /* complete host payload */
    __u32 y_offset;
    __u32 y_length;
    __u32 c_offset;
    __u32 c_length;
    __u32 mmap_bytes;       /* page-aligned host mapping length */
    __u32 rtl_revision;     /* BAR 0x139 when available */
    __u32 flags;            /* VDMA_PLANE_YC for the YC format */
    __u32 reserved[4];
};
#define VDMA_PLANE_CAPS _IOWR('V', 0x58, struct vdma_plane_caps)

/*
 * V3 descriptor.  The first 48 bytes deliberately remain the legacy frame
 * prefix so VDMA_PLANE_RELEASE can continue to receive &descriptor.frame.
 */
struct vdma_plane_frame_v3 {
    struct vdma_plane_frame frame;
    __u32 y_offset;
    __u32 y_length;
    __u32 c_offset;
    __u32 c_length;
    __u32 stride;
    __u32 metadata_version;
    __u32 kind;
    __u32 rtl_revision;
    __u32 source_epoch;
    __u32 source_sequence;
    __u32 source_layout;
    __u32 source_config_generation;
};
#define VDMA_PLANE_ACQUIRE_V3 _IOWR('V', 0x59, struct vdma_plane_frame_v3)
#endif
