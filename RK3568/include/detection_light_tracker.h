#ifndef DETECTION_LIGHT_TRACKER_H
#define DETECTION_LIGHT_TRACKER_H
#include "detection_overlay.h"
#ifdef __cplusplus
extern "C" {
#endif
#define DETECTION_LIGHT_HOLD_NS UINT64_C(200000000)
struct detection_light_track {
    struct detection_overlay_box box, raw;
    uint64_t last_seen_ns;
    uint32_t active, seen;
};
struct detection_light_tracker {
    struct detection_overlay identity;
    uint32_t initialized;
    struct detection_light_track tracks[DETECTION_OVERLAY_CAPACITY];
};
/* Qt display only. Input ADAS publications and their colors remain untouched.
 * Call for every render, including clears; duplicate publications do not renew
 * a track's age or apply smoothing again. All timestamps are monotonic. */
void detection_light_tracker_update(struct detection_light_tracker *tracker,
    const struct detection_overlay *input, const struct vdma_frame_identity *frame,
    uint64_t now_ns, int enabled, struct detection_overlay *out);
#ifdef __cplusplus
}
#endif
#endif
