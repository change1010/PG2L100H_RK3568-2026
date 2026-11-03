#ifndef ROAD_ZEBRA_DENSE_H
#define ROAD_ZEBRA_DENSE_H

#include <stdint.h>

#include "road_feature_consumer.h"

/* Dense-run zebra producer.
 *
 * The FPGA ROAD stream emits gradient-gated edge fragments (median width 2 px),
 * which road_cv.c:detect_zebra cannot use: it needs filled stripes of at least
 * zebra_min_width. Gate C measured 0/62 recall on that stream.
 *
 * This producer builds the filled runs the detector expects, straight from the
 * RGB565 canvas, and thresholds each sampled row against that row's own median
 * luminance instead of a fixed absolute level. On the 89-image curated set the
 * relative gate recovers 27/62 zebras versus 16/62 for the frozen PC v3
 * detector (recall 43.5% vs 25.8%).
 *
 * Cost is bounded by design: only sample_rows x roi_width pixels are touched,
 * never the full frame. Measured on the host over the 89-image curated set,
 * producer + road_cv_analyze costs 1.6-2.4 ms/frame against 72 ms for the
 * frozen PC v3 Python detector.
 */
struct road_zebra_dense_config {
    uint32_t roi[4];         /* x0, y0, x1, y1 in input pixels; plane pixels for YC */
    uint32_t sample_rows;    /* <= 96, the ROAD transport hard limit */
    uint32_t per_row;        /* <= ROAD_WIRE_MAX_PER_ROW (16) */
    uint32_t max_chroma;     /* white/grey chroma ceiling */
    uint32_t relative_floor; /* absolute lower bound on the row threshold */
    uint32_t relative_margin;/* threshold = row_median + margin */
    uint32_t local_block_width; /* input pixels (plane pixels for YC), 0 disables */
    uint32_t local_margin;      /* conservative threshold raise; 0 disables */
    uint32_t min_width;
    uint32_t max_width;
    /* A row with more runs than per_row keeps the widest ones. When set, the
     * retained candidates carry transport bit 8 and road_cv.c:detect_zebra
     * excludes that row from zebra geometry. This is the default because a
     * partial row must not be treated as a complete stripe pattern. The bit is
     * ignored by lane candidate filtering, so lane fitting remains unchanged.
     * truncated_rows is counted either way for diagnostics. */
    uint32_t mark_truncated;
};

void road_zebra_dense_config_default(struct road_zebra_dense_config *config);

/* Expand an RGB565 canvas into the ROAD candidate set road_cv_analyze expects.
 * Fills identity fields with source_id = layout = 0 and analysis_period = 1;
 * callers that care about stream identity must overwrite those afterwards.
 * Returns 0 on success, -1 on invalid arguments. */
int road_zebra_dense_extract(const uint16_t *rgb565, uint32_t width, uint32_t height,
                             const struct road_zebra_dense_config *config,
                             struct vdma_road_result *result);

/* Same producer for the FPGA's tightly packed Y8/C8 planes.  The ROI is
 * expressed in plane coordinates; output_scale is normally 3 so candidates
 * are returned in the 1920x1080 canvas coordinate system. */
int road_zebra_dense_extract_yc(const uint8_t *y8, const uint8_t *c8,
                                uint32_t width, uint32_t height,
                                const struct road_zebra_dense_config *config,
                                uint32_t output_scale,
                                struct vdma_road_result *result);

#endif
