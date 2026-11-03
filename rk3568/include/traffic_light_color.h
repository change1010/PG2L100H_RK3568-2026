#ifndef TRAFFIC_LIGHT_COLOR_H
#define TRAFFIC_LIGHT_COLOR_H

#include <stddef.h>
#include <stdint.h>
#include "adas_result.h"

#define LIGHT_COLOR_SIDE 128U
#define LIGHT_COLOR_PIXELS (LIGHT_COLOR_SIDE * LIGHT_COLOR_SIDE)

enum light_color_reason {
    LIGHT_COLOR_OK, LIGHT_COLOR_TOO_SMALL, LIGHT_COLOR_NO_EVIDENCE,
    LIGHT_COLOR_CONFLICT
};
struct light_color_result {
    uint32_t color, reason, evidence_permille;
};
/* Reuse in one worker; no borrowed pixels or per-frame allocations. */
struct light_color_workspace {
    uint8_t labels[LIGHT_COLOR_PIXELS], value[LIGHT_COLOR_PIXELS];
    uint16_t queue[LIGHT_COLOR_PIXELS];
};

/* Little-endian RGB565, half-open box in the supplied image.
 * UNKNOWN includes dark/white/ambiguous crops; darkness never proves OFF.
 * Evidence is measured pixel support, not a calibrated probability.
 */
int traffic_light_color_rgb565(const void *pixels, size_t bytes, size_t stride,
    uint32_t width, uint32_t height, const struct adas_box *box,
    struct light_color_workspace *work, struct light_color_result *out);

#endif
