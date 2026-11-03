#ifndef ROAD_FEATURE_TRANSPORT_H
#define ROAD_FEATURE_TRANSPORT_H

#include "road_feature_consumer.h"
#include "video_dma_feature_uapi.h"

struct vdma_road_settings {
    uint32_t request_id, analysis_period, source_id;
    uint32_t road_roi[4];
    uint32_t gx_threshold, gy_threshold;
    uint32_t white_min_y, white_max_chroma, yellow_min_chroma;
    uint32_t lane_rows, lane_per_row, denoise_enable;
    uint32_t min_width, max_width;
};

struct vdma_road_transport {
    int fd, configured, leased;
    struct vdma_feature_lease lease;
    struct vdma_feature_config config;
    unsigned char packet[VDMA_FEATURE_ROAD_MAX_PACKET_BYTES];
};

int vdma_road_transport_init(struct vdma_road_transport *transport, int fd,
                             const struct vdma_road_settings *settings);
int vdma_road_transport_read(struct vdma_road_transport *transport,
                             struct vdma_road_result *result);
int vdma_road_transport_close(struct vdma_road_transport *transport);

#endif
