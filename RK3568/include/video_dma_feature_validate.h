#ifndef VIDEO_DMA_FEATURE_VALIDATE_H
#define VIDEO_DMA_FEATURE_VALIDATE_H

#include "video_dma_feature_uapi.h"

static inline int vdma_feature_config_valid(const struct vdma_feature_config *config)
{
    unsigned int index;

    if (!config || !(config->version == 1 && config->source_id < 4) ||
        config->size != sizeof(*config) || config->enable_mask > 1 ||
        !config->analysis_period || config->analysis_period > 255 ||
        config->color_cols != 32 || config->color_rows != 18 ||
        config->color_roi[0] >= config->color_roi[2] ||
        config->color_roi[1] >= config->color_roi[3] ||
        config->color_roi[2] > 1920 || config->color_roi[3] > 1080 ||
        config->color_roi[2] - config->color_roi[0] < 32 ||
        config->color_roi[3] - config->color_roi[1] < 18 ||
        config->extract_min_value > 255 || !config->extract_min_chroma ||
        config->extract_min_chroma > 255 || config->white_min_y ||
        config->white_max_chroma || config->gx_threshold || config->gy_threshold ||
        config->lane_rows || config->lane_per_row || config->denoise_enable ||
        config->active_generation || config->applied_request_id)
        return 0;
    for (index = 0; index < 4; index++)
        if (config->road_roi[index])
            return 0;
    for (index = 0; index < 5; index++)
        if (config->reserved[index])
            return 0;
    return 1;
}

/* CONFIG v3 selects ROAD alone; v4 atomically selects COLOR and ROAD. */
static inline int vdma_road_config_valid(const struct vdma_feature_config *config)
{
    unsigned int road_min_width;
    unsigned int road_max_width;
    unsigned int index;

    if (!config || (config->version != 3 && config->version != 4) || config->size != sizeof(*config) ||
        !(config->enable_mask & VDMA_FEATURE_ROAD_BIT) ||
        (config->enable_mask & ~((__u32)VDMA_FEATURE_COLOR_BIT | VDMA_FEATURE_ROAD_BIT)) ||
        config->source_id > 3 || !config->analysis_period || config->analysis_period > 255 ||
        config->color_cols != 32 || config->color_rows != 18 ||
        config->color_roi[0] >= config->color_roi[2] ||
        config->color_roi[1] >= config->color_roi[3] ||
        config->color_roi[2] > 1920 || config->color_roi[3] > 1080 ||
        config->road_roi[0] >= config->road_roi[2] ||
        config->road_roi[1] >= config->road_roi[3] ||
        config->road_roi[2] > 1920 || config->road_roi[3] > 1080 ||
        config->road_roi[3] - config->road_roi[1] < 1 ||
        config->lane_rows < 1 || config->lane_rows > 96 ||
        config->lane_rows > config->road_roi[3] - config->road_roi[1] ||
        config->lane_per_row < 1 || config->lane_per_row > 16 ||
        config->denoise_enable != 0 ||
        config->extract_min_value > 255 || !config->extract_min_chroma ||
        config->extract_min_chroma > 255 || config->white_min_y > 255 ||
        config->white_max_chroma > 255 || config->gx_threshold > 32767 ||
        config->gy_threshold > 32767 ||
        config->active_generation || config->applied_request_id)
        return 0;
    if ((config->version == 3 && config->enable_mask != 2) ||
        (config->version == 4 && config->enable_mask != 3) ||
        config->color_roi[2]-config->color_roi[0] < 32 ||
        config->color_roi[3]-config->color_roi[1] < 18) return 0;
    road_min_width = config->reserved[0];
    road_max_width = config->reserved[1];
    if (road_min_width < 1 || road_max_width < road_min_width || road_max_width > 1920)
        return 0;
    if (config->version == 4 && (!config->reserved[2] || config->reserved[2] > 255)) return 0;
    if (config->version == 3 && config->reserved[2]) return 0;
    for (index = 3; index < 5; index++)
        if (config->reserved[index])
            return 0;
    return 1;
}

static inline int vdma_feature_read_valid(const struct vdma_feature_read *request,
                                         unsigned int packet_bytes)
{
    return request->version == 1 && request->size == sizeof(*request) &&
        !request->reserved && request->bytes && request->bytes <= VDMA_FEATURE_READ_BYTES &&
        request->offset <= packet_bytes && request->bytes <= packet_bytes - request->offset;
}

#endif
