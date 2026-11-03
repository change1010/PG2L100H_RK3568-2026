#ifndef ROAD_FEATURE_CONSUMER_H
#define ROAD_FEATURE_CONSUMER_H

#include <stddef.h>
#include <stdint.h>
#include "video_dma_features.h"

struct vdma_road_result {
    uint32_t stream_epoch, frame_sequence, source_id, layout;
    uint32_t config_generation, packet_generation, status_flags;
    uint32_t roi[4], sample_rows, candidates_per_row;
    uint32_t total_candidates, retained_candidates, truncated_rows;
    uint32_t analysis_period, config_hash, input_frame_flags;
    uint32_t candidate_count;
    struct vdma_lane_candidate candidates[VDMA_FEATURE_ROAD_MAX_CANDIDATES];
};

/* Decode only a validated ROAD packet. No geometry or lane classification is
 * performed; candidates remain raw FPGA observations.
 *
 * vdma_road_decode() decodes section 0, which is all a v3/v4 packet has and the
 * lane section of a v5 packet. vdma_road_decode_section() also reaches a v5
 * packet's second (dense zebra) section; on v3/v4 anything but index 0 is
 * rejected rather than invented. */
int vdma_road_decode(const void *packet, size_t bytes,
                     struct vdma_road_result *result);
int vdma_road_decode_section(const void *packet, size_t bytes, uint32_t index,
                             struct vdma_road_result *result);

#endif
