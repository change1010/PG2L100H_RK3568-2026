#ifndef VIDEO_DMA_FEATURE_UAPI_H
#define VIDEO_DMA_FEATURE_UAPI_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* Production ROAD-only transport requires BAR v5 and a matching driver.
 * CONFIG v3 and ROAD packet v3; COLOR constants remain for offline history.
 * No structure or ioctl in video_dma_uapi.h is changed.
 * Linux ioctl structures use native endian; FPGA packet bytes remain LE.
 */
#define VDMA_FEATURE_UAPI_VERSION 1U
#define VDMA_FEATURE_QUAD_CONFIG_VERSION 2U
#define VDMA_FEATURE_SINGLE_PROFILES 15U
#define VDMA_FEATURE_QUAD_PROFILE 16U
#define VDMA_FEATURE_JOINT_CONFIG_VERSION 4U
#define VDMA_FEATURE_JOINT_PROFILE 256U
#define VDMA_FEATURE_READ_BYTES 2048U
#define VDMA_FEATURE_COLOR_BIT 1U
#define VDMA_FEATURE_ROAD_BIT 2U
#define VDMA_FEATURE_ROAD_PACKET_VERSION 5U
#define VDMA_FEATURE_ROAD_PACKET_VERSION_MIN 3U
#define VDMA_FEATURE_ROAD_MAX_PACKET_BYTES 32768U

struct vdma_feature_caps {
    __u32 version, size, feature_mask, max_packet_bytes;
    __u32 max_color_cols, max_color_rows, max_lane_rows, max_lane_per_row;
    __u32 max_zebra_rows, packet_banks, aperture_bytes;
    union { __u32 reserved; __u32 profile_mask; };
};
struct vdma_feature_config {
    /* reserved[0:1]=ROAD min/max width; v4 reserved[2]=yellow min chroma.
     * v3 continues using extract_min_chroma for the ROAD yellow gate. */
    __u32 version, size, request_id, enable_mask;
    __u32 source_id, analysis_period, color_cols, color_rows;
    __u32 color_roi[4], road_roi[4];
    __u32 extract_min_value, extract_min_chroma, white_min_y, white_max_chroma;
    __u32 gx_threshold, gy_threshold, lane_rows, lane_per_row;
    __u32 denoise_enable, active_generation, applied_request_id, reserved[5];
};
struct vdma_frame_identity {
    __u32 version, size, slot, token;
    __u32 stream_epoch, frame_sequence, source_id, layout;
    __u32 config_generation, canvas_width, canvas_height, flags;
};
struct vdma_feature_lease {
    __u32 version, size, lease_id, packet_generation;
    __u32 stream_epoch, frame_sequence, packet_bytes, feature_mask;
    __u32 status_flags, reserved[3];
};
struct vdma_feature_read {
    __u32 version, size, lease_id, packet_generation;
    __u32 stream_epoch, offset, bytes, reserved;
    __u8 data[VDMA_FEATURE_READ_BYTES];
};
#define VDMA_FEATURE_CAPS _IOWR('V', 0x49, struct vdma_feature_caps)
#define VDMA_FEATURE_CONFIG _IOWR('V', 0x4a, struct vdma_feature_config)
#define VDMA_FRAME_IDENTITY _IOWR('V', 0x4b, struct vdma_frame_identity)
#define VDMA_FEATURE_ACQUIRE _IOWR('V', 0x4c, struct vdma_feature_lease)
#define VDMA_FEATURE_READ _IOWR('V', 0x4d, struct vdma_feature_read)
#define VDMA_FEATURE_RELEASE _IOW('V', 0x4e, struct vdma_feature_lease)

#endif
