#ifndef ROAD_ZEBRA_CV_H
#define ROAD_ZEBRA_CV_H

#include <stdint.h>
#include "adas_result.h"

#define ROAD_ZEBRA_CV_WIDTH 640U
#define ROAD_ZEBRA_CV_HEIGHT 360U
#define ROAD_ZEBRA_CV_PIXELS (ROAD_ZEBRA_CV_WIDTH * ROAD_ZEBRA_CV_HEIGHT)
#define ROAD_ZEBRA_CV_MAX_COMPONENTS 192U
#define ROAD_ZEBRA_CV_MAX_STRIPES 32U

struct road_zebra_cv_config {
    uint32_t roi_y; /* plane pixels, not 1920x1080 canvas coordinates */
    uint32_t min_luma, max_chroma;
    uint32_t local_margin, min_contrast, min_relative_contrast;
    uint32_t min_area, min_length, min_stripes;
    uint32_t min_evidence_permille;
};

struct road_zebra_cv_component {
    uint32_t x0, y0, x1, y1, area;
    double cx, cy, ux, uy, length, width;
    double brightness, chroma;
};

/* One workspace per producer; inputs are immutable. Allocate once, off stack.
 * The queue and masks cover exactly one 640x360 image. */
struct road_zebra_cv_workspace {
    uint32_t integral[(ROAD_ZEBRA_CV_WIDTH + 1U) * (ROAD_ZEBRA_CV_HEIGHT + 1U)];
    uint32_t queue[ROAD_ZEBRA_CV_PIXELS];
    uint8_t mask[ROAD_ZEBRA_CV_PIXELS];
    struct road_zebra_cv_component components[ROAD_ZEBRA_CV_MAX_COMPONENTS];
};

struct road_zebra_cv_diagnostics {
    uint32_t candidate_pixels, components, retained_components;
    uint32_t rejected_scale, rejected_contrast, rejected_edge_pair;
    uint32_t rejected_geometry, capacity_dropped, hypotheses;
    uint32_t best_stripes, best_support, best_scale;
    uint32_t threshold_min, threshold_max;
};

struct road_zebra_cv_output {
    struct adas_collection meta;
    struct adas_zebra zebra; /* 1920x1080 canvas coordinates */
};

void road_zebra_cv_config_default(struct road_zebra_cv_config *config);
int road_zebra_cv_analyze(struct road_zebra_cv_workspace *workspace,
                          const uint8_t *y, const uint8_t *c,
                          uint32_t width, uint32_t height, uint32_t stride,
                          const struct road_zebra_cv_config *config,
                          struct road_zebra_cv_output *output,
                          struct road_zebra_cv_diagnostics *diagnostics);
#endif
