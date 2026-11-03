#ifndef DETECTION_OVERLAY_H
#define DETECTION_OVERLAY_H
#include <stdint.h>
#include "video_dma_feature_uapi.h"
#ifdef __cplusplus
extern "C" {
#endif
#define DETECTION_OVERLAY_CAPACITY 48U
enum detection_overlay_state { DETECTION_NOT_LOADED, DETECTION_WAITING, DETECTION_RUNNING, DETECTION_ERROR };
enum detection_overlay_light_flag {
    DETECTION_OVERLAY_LIGHT_PREDICTED = 1U,
    DETECTION_OVERLAY_LIGHT_PENDING = 2U
};
enum detection_light_color { DETECTION_LIGHT_UNKNOWN, DETECTION_LIGHT_RED,
    DETECTION_LIGHT_YELLOW, DETECTION_LIGHT_GREEN };
/* Owned values only. No C11 atomics, pixels or borrowed handles cross into Qt. */
struct detection_overlay_box {
    int32_t x0, y0, x1, y1;
    uint32_t model_class, confidence_q10000, signal, light_color;
    uint32_t display_held; /* Geometry only; never evidence of the previous color. */
};
struct detection_overlay {
    uint32_t state, valid, count, stream_epoch, source_id, layout, config_generation, frame_sequence;
    uint64_t observed_ns, valid_until_ns;
    double fps, latency_ms;
    struct detection_overlay_box boxes[DETECTION_OVERLAY_CAPACITY];
    uint32_t light_temporal_valid;
    uint32_t track_ids[DETECTION_OVERLAY_CAPACITY], light_flags[DETECTION_OVERLAY_CAPACITY];
    uint64_t light_deadline_ns[DETECTION_OVERLAY_CAPACITY];
};
struct adas_detection;
/* Only for application-owned frozen test pixels; never a live TTL bypass. */
void detection_overlay_from_frozen(struct detection_overlay *out,
    const struct adas_detection *detection, uint64_t now_ns);
struct adas_result;
void detection_overlay_from_result(struct detection_overlay *out, const struct adas_result *result, uint64_t now_ns);
int detection_overlay_visible(const struct detection_overlay *overlay, const struct vdma_frame_identity *frame, uint64_t now_ns);
#ifdef __cplusplus
}
#endif
#endif
