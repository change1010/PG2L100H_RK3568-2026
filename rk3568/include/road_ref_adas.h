#ifndef ROAD_REF_ADAS_H
#define ROAD_REF_ADAS_H
#include "adas_result.h"
#include "video_dma_snapshot.h"
#include "../reference/road_ref/road_ref.h"

#define ROAD_REF_ALGORITHM_ID UINT32_C(0x52524631)
#define ROAD_REF_ALGORITHM_VERSION 1U
#define ROAD_REF_PROCESSOR_TYPE 4U
struct road_ref_adas;
struct road_ref_adas_stats {
    uint64_t attempts, processed, published, cancelled, failed, rejected;
    uint64_t total_ns, max_ns, bgr_frames, plane_frames;
    uint64_t committed, expired, input_age_ns, end_to_end_ns, end_to_end_max_ns;
    uint64_t lane_ns, crosswalk_ns, temporal_ns;
    uint64_t last_sequence;
    uint32_t raw_count, confirmed_count, cross_detected, cross_stable, truncated;
};
struct road_ref_adas *road_ref_adas_create(struct adas_store *store);
void road_ref_adas_destroy(struct road_ref_adas *producer);
void road_ref_adas_stats(struct road_ref_adas *producer, struct road_ref_adas_stats *out);
/* Development only: RGB565 loses source color precision. Final input uses planes. */
int road_ref_adas_rgb565_processor(struct road_ref_adas *producer,
                                 struct vdma_snapshot_processor *out);
int road_ref_adas_processor(struct road_ref_adas *producer, struct vdma_snapshot_processor *out);
int road_ref_adas_process_planes(struct road_ref_adas *producer,
    const struct vdma_snapshot_input *identity, const RoadRefPlanesFrame *planes);
int road_ref_adas_copy_geometry(struct road_ref_geometry *out, const RoadRefResult *result);
#endif
