#ifndef LIGHT_TEMPORAL_H
#define LIGHT_TEMPORAL_H
#include "adas_result.h"
#include "traffic_light_color.h"
#include "yolo_decode.h"
#include <stdio.h>

#define LIGHT_ASSOCIATION_MINIMUM .10f
#define LIGHT_TRACK_TTL_NS UINT64_C(650000000)
#define LIGHT_COLOR_GAP_NS UINT64_C(120000000)
#define LIGHT_VISUAL_TTL_NS UINT64_C(150000000)
struct light_track {
    uint32_t id, active, matched, score, template_valid;
    struct adas_box box, measured;
    float vx, vy;
    uint8_t patch[256];
    uint64_t detector_ns, high_ns, last_ns, color_ns, pending_ns;
    uint64_t edge_ns, intermittent_until_ns;
    uint32_t color, pending, pending_count, lit, edges;
};
struct light_temporal {
    struct adas_branch_header context;
    uint32_t initialized, next_id, trace_frames, search_cursor;
    struct light_track tracks[ADAS_MAX_SIGNALS];
    FILE *trace, *roi;
};
/* One inference worker owns state and pixels. No Qt access or extra NPU runs. */
void light_temporal_open_trace(struct light_temporal *state);
void light_temporal_close(struct light_temporal *state);
int light_temporal_update(struct light_temporal *state,
    struct adas_detection *candidate, const struct yolo_detections *detections,
    float high, const void *pixels, size_t bytes, size_t stride,
    struct light_color_workspace *workspace);
#endif
