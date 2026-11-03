#ifndef LIGHT_SPECIALIST_H
#define LIGHT_SPECIALIST_H
#include "adas_signal_test.h"
#include "detection_overlay.h"
#include "video_dma_snapshot.h"
#ifdef __cplusplus
extern "C" {
#endif
struct light_specialist_profile {
    int scope[4];
    unsigned layout;
};
const struct light_specialist_profile *light_specialist_profile_at(unsigned index);
/* Serialized frozen-image worker; roi-first may complete after one search.
 * VDMA_LIGHT_SEARCH_POLICY=dual retains the two-search comparison path. */
int light_specialist_analyze(const struct vdma_snapshot_input *input,
    const int scope[4], unsigned layout, struct vdma_snapshot_output *output,
    struct detection_overlay *overlay, struct adas_signal_test_observation *observation,
    const char *dump_prefix);
int light_specialist_sample_color(const unsigned char *pixels, const int box[4],
    unsigned *color, unsigned *evidence);
#ifdef __cplusplus
}
#endif
#endif
