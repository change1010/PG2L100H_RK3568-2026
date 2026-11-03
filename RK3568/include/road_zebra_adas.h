#ifndef ROAD_ZEBRA_ADAS_H
#define ROAD_ZEBRA_ADAS_H

#include <stdint.h>

#include "adas_result.h"
#include "road_cv.h"
#include "road_zebra_dense.h"
#include "road_zebra_cv.h"
#include "video_dma_snapshot.h"

/* Snapshot processor that publishes one ADAS ROAD snapshot per sampled frame.
 *
 * The FPGA ROAD candidate stream cannot feed road_cv.c:detect_zebra directly:
 * it carries gradient-gated edge fragments (measured width median 4-5 px,
 * 97-99% of the per-row capacity already truncated) and no luminance, so the
 * filled stripes and white/gap contrast the detector needs are absent. Gate C
 * measured 0/62 recall on that stream.
 *
 * Zebra extraction uses paired FPGA Y/C when present; the RGB565 entry remains
 * for old GRAY8 hardware and offline regressions.
 * Lanes use only the validated FPGA Y plane; missing/foreign planes produce
 * UNKNOWN lanes without an RGB fallback. Both publish through the existing
 * ADAS ROAD branch.
 */
#define ROAD_ZEBRA_PROCESSOR_TYPE 2U
#define ROAD_ZEBRA_PROCESSOR_VERSION 3U
enum road_zebra_detector {
    ROAD_ZEBRA_LEGACY = 0,
    ROAD_ZEBRA_PIXEL = 1 /* experimental; paired YC required */
};

struct road_zebra_adas_stats {
    uint64_t attempts;       /* process() calls */
    uint64_t extracted;      /* dense extraction succeeded */
    uint64_t published;      /* ADAS publish accepted */
    uint64_t busy;           /* token/publication returned ADAS_BUSY */
    uint64_t stale;          /* token/publication returned ADAS_REJECTED_STALE */
    uint64_t invalid;        /* canvas shape or arguments rejected */
    uint64_t extract_failed;
    uint64_t max_ns;
    uint64_t lane_gray, lane_missing, lane_ns, lane_max_ns;
    uint64_t zebra_yc, zebra_rgb;
    uint64_t token_rejected, publication_rejected;
    uint64_t token_busy, token_stale, token_invalid;
    uint64_t publish_busy, publish_stale, publish_invalid;
    uint64_t analyze_failed;
    int32_t last_token_status, last_publication_status;
    /* Read back from the store after each publication so the caller can tell a
     * quiet road (ADAS_UNKNOWN / ADAS_ABSENT) from a detected crosswalk. */
    uint64_t observed;       /* read-backs that produced a result */
    uint64_t state_valid;    /* zebra task state; aggregate ROAD in lane-only mode */
    uint64_t state_absent;   /* ADAS_ABSENT */
    uint64_t state_unknown;  /* ADAS_UNKNOWN */
    uint64_t state_failed;   /* ADAS_FAILED or anything else */
    uint64_t zebra_total;    /* sum of reported regions */
    uint32_t last_state, last_reason, last_zebras;
    uint32_t last_box[4];    /* x0, y0, x1, y1 of the first region, 0 when none */
    uint32_t last_stripes;   /* stripes in the first region */
    uint32_t last_evidence;  /* evidence_permille of the first region */
    uint64_t zebra_candidate_rows;
    uint64_t zebra_complete_rows;
    uint64_t zebra_truncated_rows;
    uint64_t zebra_pattern_rows;
    uint64_t zebra_rejected_truncated_rows;
    uint64_t zebra_rejected_pattern_rows;
    uint64_t zebra_rejected_geometry_rows;
    uint64_t zebra_rejected_evidence;
    /* Per-frame raw detector state, before the temporal hold. */
    uint32_t raw_zebra_state, raw_zebras, raw_zebra_evidence;
    uint64_t zebra_held_frames;
    struct road_cv_diagnostics last_diagnostics;
    uint32_t detector;
    struct road_zebra_cv_diagnostics pixel_diagnostics;
};

struct road_zebra_adas;

/* The store must outlive the producer; the producer must outlive the snapshot
 * worker, whose processor context points at it. */
struct road_zebra_adas *road_zebra_adas_create(struct adas_store *store);
/* Select LEFT|RIGHT, optionally ZEBRA, before starting the snapshot worker. */
struct road_zebra_adas *road_zebra_adas_create_tasks(struct adas_store *store, uint32_t tasks);
void road_zebra_adas_destroy(struct road_zebra_adas *producer);
/* Configure before starting the snapshot worker. Zero removes the extra
 * within-ROI floor; the extraction ROI itself is unchanged. */
int road_zebra_adas_set_band_floor(struct road_zebra_adas *producer, uint32_t permille);
/* Startup only. Legacy stays default until independent accuracy acceptance.
 * Pixel workspace is allocated once; failure leaves the previous mode intact. */
int road_zebra_adas_set_detector(struct road_zebra_adas *producer,
                                enum road_zebra_detector detector);
/* Copies a descriptor for vdma_snapshot_init_processor. */
int road_zebra_adas_processor(struct road_zebra_adas *producer,
                              struct vdma_snapshot_processor *processor);
void road_zebra_adas_stats(const struct road_zebra_adas *producer,
                           struct road_zebra_adas_stats *stats);
/* Diagnostic: dump the next processed canvas as raw RGB565 so the detection can
 * be checked against the exact pixels that produced it. Writes once, then stops. */
int road_zebra_adas_dump(struct road_zebra_adas *producer, const char *path);

#endif
