#ifndef PERCEPTION_RESULT_H
#define PERCEPTION_RESULT_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

/* Application-only C11 API. Serialize named fields, not this host structure.
 * No DRM handles, borrowed pointers or DMA slot ownership crosses this API.
 */
#define PERCEPTION_VERSION 3U
#define PERCEPTION_ALGORITHM_VERSION 1U
#define PERCEPTION_MAX_COLORS 32U
#define PERCEPTION_MAX_ZEBRAS 8U
#define PERCEPTION_MAX_LANE_POINTS 96U
#define PERCEPTION_TTL_NS UINT64_C(200000000)
#define PERCEPTION_COARSE 1U
#define PERCEPTION_REFINED 2U
#define PERCEPTION_TRUNCATED 4U

enum perception_state { PERCEPTION_DISABLED, PERCEPTION_UNKNOWN, PERCEPTION_ABSENT, PERCEPTION_VALID };
enum perception_reason {
    PERCEPTION_REASON_NONE, PERCEPTION_REASON_SOURCE, PERCEPTION_REASON_CONFIG,
    PERCEPTION_REASON_INPUT_LOST, PERCEPTION_REASON_RESET, PERCEPTION_REASON_EXPIRED,
    PERCEPTION_REASON_FEATURE_MISSING, PERCEPTION_REASON_INSUFFICIENT
};
enum perception_line { PERCEPTION_LINE_UNKNOWN, PERCEPTION_LINE_SOLID, PERCEPTION_LINE_DASHED };
enum perception_return { PERCEPTION_INVALID = -1, PERCEPTION_OK, PERCEPTION_NO_NEW, PERCEPTION_STALE, PERCEPTION_BUSY };
enum perception_event_kind { PERCEPTION_EVENT_FRAME, PERCEPTION_EVENT_CONTROL };

struct perception_identity {
    uint32_t stream_epoch, frame_sequence, source_id, layout, config_generation;
    uint32_t canvas_width, canvas_height;
};
struct perception_point { uint32_t x, y; };
struct perception_box { uint32_t x0, y0, x1, y1; };
struct perception_region {
    struct perception_box box;
    struct perception_point center;
    uint32_t color, pixel_area, evidence_score, flags; /* score 0..1000, not probability */
    uint32_t region_index, region_source_id;
};
struct perception_lane {
    uint32_t state, reason, flags, line_type, evidence_score, point_count;
    struct perception_point points[PERCEPTION_MAX_LANE_POINTS]; /* increasing y */
};
struct perception_zebra {
    struct perception_box box;
    uint32_t stripe_count, evidence_score, flags, proximity_band; /* band 0=unknown,1..3=image zones */
};
struct perception_result {
    uint32_t version, size;
    struct perception_identity identity;
    uint64_t result_sequence;
    uint64_t frame_observed_ns, feature_received_ns, generated_ns; /* RK CLOCK_MONOTONIC */
    uint32_t algorithm_version, merge_config_generation;
    uint32_t batch_id, required_region_mask, received_region_mask;
    uint64_t session_generation, changed_ns;
    uint32_t event_kind, identity_present;
    uint32_t color_state, color_reason, color_count;
    uint32_t zebra_state, zebra_reason, zebra_count;
    struct perception_region colors[PERCEPTION_MAX_COLORS];
    struct perception_lane left, right;
    struct perception_zebra zebras[PERCEPTION_MAX_ZEBRAS];
};
struct perception_cursor { uint64_t sequence; bool expired; };
struct perception_store {
    atomic_bool busy;
    bool available;
    uint64_t sequence;
    uint64_t active_session, last_generated_ns, frame_observed_watermark;
    uint32_t active_merge, frame_watermark;
    bool bound, frame_watermark_valid, colors_enabled;
    struct perception_identity target;
    struct perception_result latest;
};

int perception_result_valid(const struct perception_result *result);
int perception_store_init(struct perception_store *store);
int perception_publish(struct perception_store *store, const struct perception_result *snapshot);
int perception_begin_session(struct perception_store *store, uint32_t merge_generation,
                              uint64_t now_ns, uint32_t reason, bool colors_enabled);
int perception_bind_identity(struct perception_store *store, const struct perception_identity *identity);
int perception_invalidate_current(struct perception_store *store,
                                  const struct perception_identity *failed_identity,
                                  uint64_t now_ns, uint32_t reason);
int perception_try_get_latest(struct perception_store *store, struct perception_cursor *cursor,
                              uint64_t now_ns, uint64_t ttl_ns,
                              struct perception_result *owned_copy);
int perception_invalidate(struct perception_store *store, const struct perception_identity *identity,
                          uint64_t now_ns, uint32_t reason);
int perception_invalidate_config(struct perception_store *store,
                                 const struct perception_identity *identity,
                                 uint32_t merge_config_generation,
                                 uint64_t now_ns, uint32_t reason);

#endif
