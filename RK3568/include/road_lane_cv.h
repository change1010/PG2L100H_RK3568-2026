#ifndef ROAD_LANE_CV_H
#define ROAD_LANE_CV_H

#include <stdint.h>

#include "adas_result.h"

/* Board-side forward-view lane detector.
 *
 * This is a C11 port of scripts/road_roi_lane.py (algorithm pc-forward-lane-v1),
 * with deterministic bounded candidates and fitting. Historical offline scores
 * do not establish accuracy on the current FPGA plane or live video.
 *
 * The offline entry accepts the packed 1920x1080 RGB565 canvas
 * (little endian, stride = 2 * canvas_width), i.e. exactly the buffer
 * road_zebra_adas.c hands to road_zebra_dense_extract(). No ROI/RGA/driver
 * dependency, no allocation inside the call.
 *
 * Differences from the Python reference, all deliberate:
 *   - Hough: a deterministic rho/theta accumulator replaces cv2.HoughLinesP.
 *     Seeds are equivalent in meaning (a segment through candidate centres with
 *     at least `threshold` votes, min length, bounded gap) but not bit-identical.
 *   - Area downscale works on the linear grey sum, not per-channel uint8 means,
 *     so a pixel can differ by ~1 LSB from cv2 INTER_AREA.
 * The GRAY8 production version also balances slope directions, deduplicates
 * before consuming fit/track budgets, and jointly selects compatible sides.
 * Its default forward-road ROI is a camera profile rather than a universal
 * road boundary. Verified continuous near support also prevents rough
 * competing associations from winning solely through the camera prior.
 * Reassociation, robust fitting and canvas reprojection retain
 * the reference equations; historical Python scores do not apply to it.
 */

#define ROAD_LANE_MAX_WORK_WIDTH 960U
#define ROAD_LANE_MAX_WORK_HEIGHT 540U
#define ROAD_LANE_RHO_RADIUS 800
#define ROAD_LANE_THETA_BINS 360
#define ROAD_LANE_MAX_POINTS (ROAD_LANE_MAX_WORK_HEIGHT * 64U)

enum road_lane_rejection {
    ROAD_LANE_ACCEPTED, ROAD_LANE_SEED_SHORT, ROAD_LANE_SEED_SLOPE,
    ROAD_LANE_SUPPORT_LOW, ROAD_LANE_FIT_DEGENERATE, ROAD_LANE_SPAN_SHORT,
    ROAD_LANE_POSITION_RESIDUAL, ROAD_LANE_WIDTH_RESIDUAL,
    ROAD_LANE_OUTSIDE_CORRIDOR, ROAD_LANE_WRONG_SIDE, ROAD_LANE_TRACK_BUDGET,
    ROAD_LANE_FRAGMENT_SUPPORT, ROAD_LANE_TRANSVERSE_PATTERN,
    ROAD_LANE_REJECTION_COUNT
};

/* Optional offline diagnostics. Coordinates are in the work plane. Production
 * entry points never allocate or fill this potentially large structure. */
struct road_lane_diag_point { float x; int32_t y; };
struct road_lane_diag_support {
    float x;
    int32_t y;
    int32_t width;
    float strength;
    float contrast_left;
    float contrast_right;
};
/* Work-plane evidence; these flags describe contradictions, not object classes. */
enum road_lane_fragment_flag {
    ROAD_LANE_FRAGMENT_ROUGH = 1U,
    ROAD_LANE_FRAGMENT_WIDTH = 2U,
    ROAD_LANE_FRAGMENT_DIRECTION = 4U,
    ROAD_LANE_FRAGMENT_OFFSET = 8U,
    ROAD_LANE_FRAGMENT_REMOVED = 16U,
    ROAD_LANE_FRAGMENT_REFERENCE = 32U,
    ROAD_LANE_FRAGMENT_ASYMMETRY = 64U
};
struct road_lane_diag_fragment {
    int32_t first_y, last_y;
    uint32_t count, flags;
    float slope, intercept, residual;
    float width_slope, width_intercept, width_residual;
    float strength, contrast_left, contrast_right;
};
struct road_lane_diag_track {
    int32_t seed[4];
    uint32_t side, reason, count, edge_asymmetry_count;
    float a, b, residual, width_residual;
    struct road_lane_diag_point points[ROAD_LANE_MAX_WORK_HEIGHT];
    /*
     * Per-point candidate provenance for offline vehicle-edge diagnosis.
     * This is filled only by the debug entry point and never affects the
     * production lane result.
     */
    struct road_lane_diag_support support[ROAD_LANE_MAX_WORK_HEIGHT];
    uint32_t fragment_count, removed_rows;
    struct road_lane_diag_fragment fragments[ROAD_LANE_MAX_WORK_HEIGHT / 3U];
};
struct road_lane_diagnostics {
    uint32_t seed_count, attempt_count, track_count;
    int32_t selected[2];
    int32_t seeds[512][4];
    struct road_lane_diag_track attempts[512], tracks[16];
    /* Final support after pair clipping, before public point decimation. */
    struct road_lane_diag_track output[2];
};

struct road_lane_config {
    uint32_t canvas_width;
    uint32_t canvas_height;
    uint32_t work_width;              /* 160..960; work_height derives from content */
    int32_t content[4];               /* x0, y0, x1, y1 in canvas pixels */
    int32_t polygon[8];               /* 4 convex canvas-space points, x/y pairs */
    uint32_t max_row_candidates;      /* default 48 */
    uint32_t max_hypotheses;          /* default 96 */
    uint32_t max_tracks;              /* default 8 */
    uint32_t min_gray;                /* default 20 */
    int32_t min_contrast_q8;          /* default 7.0 -> 1792 */
    uint32_t vehicle_edge_filter_enable; /* default 0; experimental local gate */
    uint32_t fragment_filter_enable; /* default 1; 0 reproduces the pre-cleanup path */
    int32_t vehicle_edge_min_strength_q8; /* default 15.0 when enabled */
    int32_t vehicle_edge_asymmetry_q8; /* default 4.0 ratio when enabled */
    int32_t min_outward_slope_q8;     /* default 0.15 -> 38 */
    int32_t max_width_residual_q8;    /* default 2.5 -> 640 */
    uint32_t top_permille;            /* default 550 */
    uint32_t bottom_permille;         /* default 940 */
    uint32_t anchor_x_permille;       /* default 500 */
    uint32_t anchor_y_permille;       /* default 900 */
};

struct road_lane_stats {
    uint32_t work_width;
    uint32_t work_height;
    uint32_t mask_pixels;
    uint32_t candidates;
    uint32_t seeds;
    uint32_t tracks;
    uint32_t dropped_row_candidates;
    uint32_t dropped_hypotheses;
    uint32_t dropped_tracks;
    uint32_t vehicle_edge_filtered;
    uint32_t fragment_rows_removed, fragment_tracks_cleaned;
    uint32_t row_candidates_seen, row_capacity_drops;
    uint32_t fit_attempts[2];
    /* One final gate result per attempted fit; ACCEPTED includes duplicates. */
    uint32_t rejected[ROAD_LANE_REJECTION_COUNT];
    /* Stage timings in nanoseconds. Zero when the build has no clock source.
     * ns_gray is the 1080p->work downsample + grey conversion; it is the only
     * stage that touches every source pixel and therefore the only one worth
     * moving to hardware. */
    uint64_t ns_gray;
    uint64_t ns_cand;
    uint64_t ns_hough;
    uint64_t ns_track;
};

struct road_lane_cand {
    float center;
    int32_t y;
    float strength;
    int32_t width;
    float contrast_left;
    float contrast_right;
};

/* Caller-owned scratch. ~1.4 MB; keep it static in the producer thread rather
 * than allocating per frame. Never shared between threads. */
struct road_lane_workspace {
    float gray_row[ROAD_LANE_MAX_WORK_WIDTH];
    float gray_prefix[ROAD_LANE_MAX_WORK_WIDTH + 1];
    int32_t bg_prefix[ROAD_LANE_MAX_WORK_WIDTH + 1];
    uint8_t mask_row[ROAD_LANE_MAX_WORK_WIDTH];
    uint16_t rank[ROAD_LANE_MAX_POINTS];
    uint16_t px[ROAD_LANE_MAX_POINTS];
    uint16_t py[ROAD_LANE_MAX_POINTS];
    int16_t hough[ROAD_LANE_THETA_BINS * (2 * ROAD_LANE_RHO_RADIUS + 1)];
    int32_t bucket_head[2 * ROAD_LANE_RHO_RADIUS + 1];
    int32_t bucket_next[ROAD_LANE_MAX_POINTS];
    struct road_lane_cand cand[ROAD_LANE_MAX_WORK_HEIGHT][48];
    uint16_t cand_count[ROAD_LANE_MAX_WORK_HEIGHT];
};

void road_lane_config_default(struct road_lane_config *config);

/* Fills left/right on success (state ADAS_UNKNOWN when no side passes).
 * Returns 0 on success, -1 on invalid arguments. */
int road_lane_analyze(struct road_lane_workspace *workspace,
                      const uint16_t *rgb565,
                      const struct road_lane_config *config,
                      struct adas_lane *left,
                      struct adas_lane *right,
                      struct road_lane_stats *stats);

/* Same pipeline fed by the FPGA ch2 grey plane instead of canvas pixels.
 * `gray` is width*height GRAY8 (row-major, stride == width) and width/height
 * must equal the work geometry derived from config, i.e. 640x360 for the
 * default config. content must cover the full canvas; use polygon and the row
 * band to restrict the road without shifting plane coordinates. This skips the
 * area-average + luma stage that costs 25.5 ms/frame on the board.
 * Returns 0 on success, -1 on invalid arguments, -2 when width/height do not
 * match the work geometry. Report invalid input; do not silently substitute RGB
 * when validating the FPGA plane path. */
int road_lane_analyze_gray(struct road_lane_workspace *workspace,
                           const uint8_t *gray,
                           uint32_t width,
                           uint32_t height,
                           const struct road_lane_config *config,
                           struct adas_lane *left,
                           struct adas_lane *right,
                           struct road_lane_stats *stats);

int road_lane_analyze_gray_debug(struct road_lane_workspace *workspace,
                                 const uint8_t *gray, uint32_t width, uint32_t height,
                                 const struct road_lane_config *config,
                                 struct adas_lane *left, struct adas_lane *right,
                                 struct road_lane_stats *stats,
                                 struct road_lane_diagnostics *diagnostics);

#endif
