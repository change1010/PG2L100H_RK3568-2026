#ifndef ADAS_LANE_TEST_H
#define ADAS_LANE_TEST_H

#include "adas_decision.h"

#ifdef __cplusplus
extern "C" {
#endif

struct road_ref_geometry;
enum adas_lane_test_reason {
    ADAS_LANE_TEST_OK, ADAS_LANE_TEST_MISSING, ADAS_LANE_TEST_AMBIGUOUS,
    ADAS_LANE_TEST_GEOMETRY, ADAS_LANE_TEST_CONTEXT
};
/* Limited front-view prior, in native 1920x1080 coordinates. It is not a
 * camera calibration or a guarantee of ego association on arbitrary roads. */
struct adas_lane_test_config {
    /* minimum_span applies to the associated boundary, not each paint dash.
     * ambiguity_px also bounds endpoint residuals (half) and extrapolation. */
    int32_t reference_x, reference_y, minimum_span, maximum_width, ambiguity_px;
};
struct adas_lane_test_side {
    uint32_t line_type, reason;
    int32_t x, x0, y0, x1, y1;
    uint32_t segment_count, support_px, span_px;
};
struct adas_lane_test_observation {
    struct adas_drive_frame frame;
    uint64_t observed_ns, valid_until_ns;
    struct adas_lane_test_side sides[2];
};

struct adas_lane_test_config adas_lane_test_default_config(void);
int adas_lane_test_select(const struct road_ref_geometry *geometry,
                          const struct adas_lane_test_config *config,
                          struct adas_lane_test_side sides[2]);
int adas_lane_test_same_frame(const struct adas_drive_frame *a,
                              const struct adas_drive_frame *b);
/* Only CHANGE_LEFT/CHANGE_RIGHT are accepted. Other output metadata is owned
 * by the one-shot caller. There is no signal, lane-selection or STOP input. */
void adas_lane_test_apply(const struct adas_lane_test_observation *observation,
                          const struct adas_drive_frame *frame, uint32_t command,
                          struct adas_drive_output *output);

#ifdef __cplusplus
}
#endif
#endif
