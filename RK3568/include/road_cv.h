#ifndef ROAD_CV_H
#define ROAD_CV_H

#include <stdint.h>
#include "adas_result.h"
#include "road_feature_consumer.h"

/* Candidate-level ROAD semantic producer.
 *
 * FPGA supplies row-wise white/yellow gradient segments. This module applies
 * conservative geometry only; it does not claim pixel-level segmentation or
 * neural inference. A result is UNKNOWN when the candidate evidence is too
 * sparse or ambiguous.
 */
#define ROAD_CV_MAX_POINTS ADAS_MAX_LANE_POINTS
#define ROAD_CV_MAX_ZEBRAS ADAS_MAX_ZEBRAS
/* Per-row staging inside detect_zebra(). The v3 wire allows per_row <= 8 and
 * the copy loop wrote into a fixed eight-entry array with no bound check; the
 * v4 wire allows 16 (8 Sobel fragments + 8 dense runs). Size the staging for
 * the wire maximum and bound the copy, so a larger per_row can only mark the
 * row truncated, never write past the array. */
#define ROAD_CV_MAX_SEGMENTS 16U
#define ROAD_CV_ALGORITHM_ID 2U
#define ROAD_CV_ALGORITHM_VERSION 3U

struct road_cv_config {
    uint32_t min_lane_points;
    uint32_t min_lane_evidence_permille;
    uint32_t max_lane_fit_error;
    uint32_t min_lane_y_span;
    /* A lane candidate is clustered on its own inliers, then accepted on two
     * local measures instead of the old "one line must explain 60% of every
     * sampled row" gate, which no single painted line can satisfy once left
     * edge, right edge and dashes share the row budget.
     *
     * continuity: inlier rows as a fraction (per-mille) of the distinct sample
     *   rows inside the cluster's own y-span. A solid line is dense; a scatter
     *   or a broken dash train is not.
     * slope: |dx/dy| in per-mille. A painted lane sweeps sideways toward the
     *   camera; a near-vertical mast, pole or building edge keeps a constant x
     *   over many rows and must not be reported as a lane. */
    uint32_t min_lane_continuity_permille;
    uint32_t min_lane_slope_permille;
    uint32_t max_lane_width;
    uint32_t zebra_min_width;
    uint32_t zebra_min_stripes;
    uint32_t zebra_y_tolerance;
    uint32_t zebra_min_evidence_permille;
    uint32_t zebra_max_width;
    uint32_t zebra_min_rows;
    uint32_t zebra_min_x_span;
    uint32_t zebra_min_y_span;
    /* A crosswalk a vehicle can drive over lies on the road surface, so it
     * projects below the horizon: the band must start at least this far into the
     * ROI, in per-mille of the ROI height. Horizon clutter (sky, tree line,
     * distant signage) is regular in the same way and only this prior separates
     * it from the paint. */
    uint32_t zebra_min_band_permille;
};

struct road_cv_output {
    uint32_t state, reason, supported_tasks; /* capabilities, not detections */
    struct adas_lane left, right;
    struct adas_collection zebras_meta;
    struct adas_zebra zebras[ROAD_CV_MAX_ZEBRAS];
};

/* Optional zebra-only diagnostics. The existing road_cv_analyze() ABI remains
 * unchanged; callers that need explainability can use the extended entry
 * point. Lane fitting does not update these counters. */
struct road_cv_diagnostics {
    uint32_t candidate_rows;
    uint32_t complete_rows;
    uint32_t truncated_rows;
    uint32_t pattern_rows;
    uint32_t rejected_truncated_rows;
    uint32_t rejected_pattern_rows;
    uint32_t rejected_geometry_rows; /* rejected chain extensions, not unique rows */
    uint32_t rejected_evidence;
    uint32_t best_chain_rows;
    uint32_t best_chain_stripes;
    /* Bounds are canvas pixels; zero when the corresponding row count is zero.
     * Extension counters can overlap and are not frame-level miss counts. */
    uint32_t candidate_y_min, candidate_y_max;
    uint32_t pattern_y_min, pattern_y_max;
    uint32_t band_floor;
    uint32_t rejected_band_rows;
    uint32_t rejected_scale_chains;
    uint32_t rejected_gap_chains;
    uint32_t rejected_capacity_rows;
    uint32_t rejected_candidate_scale;
};

void road_cv_config_default(struct road_cv_config *config);

/* Per-stream temporal filter. A single frame cannot tell a crosswalk from a
 * band of paint one sample row lower, so the reported band hops between
 * neighbouring rows as the car moves. Holding the box still requires state
 * across frames; road_cv_analyze() stays pure and allocation-free, and callers
 * that want stable output keep one filter per stream.
 *
 * The filter never invents geometry: it only holds a box that the current
 * frame's own evidence still supports, and drops it as soon as a frame
 * disagrees beyond the tolerance. */
#define ROAD_CV_TRACK_HISTORY 4U

struct road_cv_track {
    uint32_t valid;
    uint32_t age;
    int32_t box[4];
    uint32_t stripe_count;
    uint32_t evidence_permille;
    /* A jump away from the held box is not believed on its first sighting: the
     * per-frame jitter between neighbouring bands looks exactly like a new
     * crosswalk. A challenger has to appear twice in a row before it wins. */
    int32_t pending[4];
    uint32_t pending_hits;
};

void road_cv_track_reset(struct road_cv_track *track);

/* Apply the filter to one frame's output. `iou_permille` is how much the
 * incoming box must overlap the held box to be treated as the same crosswalk;
 * 0 disables smoothing. Returns 1 when the output was rewritten. */
int road_cv_track_update(struct road_cv_track *track,
                         struct road_cv_output *output,
                         uint32_t iou_permille);

/* Pure, allocation-free conversion. Returns 0 for a well-formed input,
 * including UNKNOWN output states; returns -1 for invalid arguments, clearing
 * output first. UNKNOWN tasks own no drawable geometry or evidence scores.
 * Raw candidate absence does not establish semantic absence. */
int road_cv_analyze(const struct vdma_road_result *input,
                    const struct road_cv_config *config,
                    struct road_cv_output *output);

int road_cv_analyze_diagnostics(const struct vdma_road_result *input,
                                const struct road_cv_config *config,
                                struct road_cv_output *output,
                                struct road_cv_diagnostics *diagnostics);

#endif
