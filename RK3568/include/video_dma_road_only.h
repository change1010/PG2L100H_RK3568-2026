#ifndef VIDEO_DMA_ROAD_ONLY_H
#define VIDEO_DMA_ROAD_ONLY_H

#include "video_dma_feature_validate.h"

/* Production capabilities; historical COLOR wire parsers remain offline. */
#define VDMA_ROAD_ONLY_TRANSPORT_VERSION 5U

static inline int vdma_road_only_config_valid(const struct vdma_feature_config *config)
{
    /* v3 is still accepted: an operator who flashes the new kernel but not the
     * new bitstream must keep working, and vdma_road_config_valid already pins
     * each version to its own enable_mask and reserved[2] rules. */
    return config && (config->version == 3 || config->version == 4) &&
        (config->enable_mask & VDMA_FEATURE_ROAD_BIT) &&
        !(config->enable_mask & ~((__u32)VDMA_FEATURE_COLOR_BIT | VDMA_FEATURE_ROAD_BIT)) &&
        vdma_road_config_valid(config);
}

static inline int vdma_road_only_type_valid(__u32 requested_type)
{
    return requested_type == 0 || requested_type == VDMA_FEATURE_ROAD_BIT;
}

static inline int vdma_road_only_caps_valid(const struct vdma_feature_caps *caps)
{
    return caps && caps->version == 2 && caps->size == sizeof(*caps) &&
        caps->feature_mask == VDMA_FEATURE_ROAD_BIT &&
        caps->profile_mask == VDMA_FEATURE_SINGLE_PROFILES &&
        caps->max_packet_bytes == VDMA_FEATURE_ROAD_MAX_PACKET_BYTES &&
        !caps->max_color_cols && !caps->max_color_rows && !caps->max_zebra_rows &&
        caps->max_lane_rows == 96 && caps->max_lane_per_row == 13 &&
        caps->packet_banks == 2 && caps->aperture_bytes == VDMA_FEATURE_READ_BYTES;
}

#endif
