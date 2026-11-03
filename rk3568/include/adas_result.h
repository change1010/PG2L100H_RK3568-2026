#ifndef ADAS_RESULT_H
#define ADAS_RESULT_H

#include <stdatomic.h>
#include <stdint.h>
#include "road_ref_geometry.h"

/* Application-only C11 API, not a wire/ioctl ABI. Zero-initialize inputs.
 * Payloads own all values: no pixels, borrowed pointers or hardware handles.
 * Serialize named fields for IPC; never transmit native padding/atomics.
 */
#define ADAS_VERSION 3U
#define ADAS_WIDTH 1920U
#define ADAS_HEIGHT 1080U
#define ADAS_MAX_OBJECTS 32U
#define ADAS_MAX_SIGNALS 16U
#define ADAS_MAX_LANE_POINTS 96U
#define ADAS_MAX_ZEBRAS 8U
#define ADAS_TRUNCATED 1U
#define ADAS_LEFT 1U
#define ADAS_STRAIGHT 2U
#define ADAS_RIGHT 4U
#define ADAS_DIRECTIONS 7U
#define ADAS_CLASS_VEHICLE 1U
#define ADAS_CLASS_PEDESTRIAN 2U
#define ADAS_CLASS_OBSTACLE 4U
#define ADAS_CLASS_OTHER 8U
#define ADAS_CLASSES 15U
#define ADAS_SIGNAL_LOCATION 1U
#define ADAS_SIGNAL_DIRECTION 2U
#define ADAS_SIGNAL_COLOR 4U
#define ADAS_ROAD_LEFT 1U
#define ADAS_ROAD_RIGHT 2U
#define ADAS_ROAD_ZEBRA 4U
#define ADAS_DETECTION_TTL_NS UINT64_C(500000000)
#define ADAS_ROAD_TTL_NS UINT64_C(200000000)
#define ADAS_FUSION_WINDOW_NS UINT64_C(100000000)

enum adas_state {
    ADAS_DISABLED, ADAS_UNKNOWN, ADAS_VALID, ADAS_ABSENT,
    ADAS_FAILED, ADAS_EXPIRED, ADAS_CANCELLED
};
enum adas_reason {
    ADAS_REASON_NONE, ADAS_NOT_ENABLED, ADAS_UNSUPPORTED_TASK,
    ADAS_UNSUPPORTED_LAYOUT, ADAS_WAIT_INPUT, ADAS_INSUFFICIENT,
    ADAS_AMBIGUOUS, ADAS_INPUT_MISSING, ADAS_SOURCE_CHANGED,
    ADAS_EPOCH_CHANGED, ADAS_FPGA_CONFIG_CHANGED, ADAS_POLICY_CHANGED,
    ADAS_STOPPED, ADAS_MODEL_RELOAD, ADAS_INPUT_LOST, ADAS_REASON_EXPIRED,
    ADAS_PROCESSOR_ERROR, ADAS_PROTOCOL_ERROR, ADAS_REASON_CANCELLED
};
enum adas_origin { ADAS_ORIGIN_NONE, ADAS_YOLO, ADAS_ROAD_CV, ADAS_ROI_CV, ADAS_FUSED, ADAS_MOCK, ADAS_ROAD_FPGA };
enum adas_class { ADAS_CLASS_UNKNOWN, ADAS_VEHICLE, ADAS_PEDESTRIAN, ADAS_OBSTACLE, ADAS_OTHER };
enum adas_shape { ADAS_SHAPE_UNKNOWN, ADAS_ARROW, ADAS_ROUND };
enum adas_color { ADAS_COLOR_UNKNOWN, ADAS_RED, ADAS_YELLOW, ADAS_GREEN, ADAS_OFF };
enum adas_association { ADAS_UNRESOLVED, ADAS_EGO_PATH, ADAS_OTHER_PATH };
enum adas_association_reason { ADAS_ASSOC_NONE, ADAS_ASSOC_MODEL, ADAS_ASSOC_SCENE_RULE, ADAS_ASSOC_MOCK };
enum adas_line { ADAS_LINE_UNKNOWN, ADAS_SOLID, ADAS_DASHED };
enum adas_branch { ADAS_DETECTION, ADAS_ROAD };
enum adas_return {
    ADAS_INVALID = -1, ADAS_OK_NEW, ADAS_OK_SAME, ADAS_NO_DATA,
    ADAS_BUSY, ADAS_REJECTED_STALE
};

struct adas_context {
    uint32_t stream_epoch, source_id, layout, config_generation, input_profile_generation;
};
struct adas_identity {
    uint32_t stream_epoch, frame_sequence, source_id, layout, config_generation;
    uint32_t canvas_width, canvas_height;
};
struct adas_box { int32_t x0, y0, x1, y1; };
struct adas_point { int32_t x, y; };
struct adas_collection {
    uint32_t state, reason, count, observed_count, total_count_known, flags;
};
struct adas_branch_header {
    uint32_t state, reason, origin, flags, identity_present;
    struct adas_identity identity;
    uint32_t input_profile_generation;
    uint64_t application_session, producer_session;
    uint32_t algorithm_id, algorithm_version, algorithm_config_generation, error_errno;
    uint64_t observed_ns, received_ns, finished_ns, state_changed_ns, ttl_ns;
};
struct adas_object {
    uint32_t object_id, semantic_class, model_class_id, class_map_version;
    struct adas_box box;
    uint32_t confidence_present, confidence_q10000, origin, flags, original_index;
};
struct adas_signal {
    uint32_t signal_id;
    struct adas_box box;
    uint32_t direction_mask, shape, color, direction_valid, color_valid;
    uint32_t association, association_reason, association_rule_version;
    uint64_t association_control_generation;
    uint32_t localization_confidence_present, localization_confidence_q10000;
    uint32_t direction_confidence_present, direction_confidence_q10000;
    uint32_t color_confidence_present, color_confidence_q10000;
    uint32_t color_evidence_present, color_evidence_permille;
    uint32_t origin, flags, original_index;
};
/* Display evidence only; the decision engine continues to read raw signals.
 * Dark/ambiguous pixels never establish OFF or renew an actionable color. */
#define LIGHT_VISUAL_PREDICTED 1U
#define LIGHT_VISUAL_PENDING 2U
#define LIGHT_VISUAL_INTERMITTENT 4U
struct adas_light_visual {
    struct adas_box box;
    uint32_t track_id, color, instant_color, reason, evidence_permille;
    uint32_t confidence_q10000, flags;
    uint64_t detector_ns, color_ns;
};
struct adas_lane {
    uint32_t state, reason, line_type, evidence_present, evidence_permille;
    uint32_t point_count, flags;
    struct adas_point points[ADAS_MAX_LANE_POINTS];
    /* Bit i certifies continuous measured support between sampled points
     * i-1 and i. Zero retains the adjacent-row-only rendering contract.
     * This is application metadata, never part of the FPGA/DMA wire format. */
    uint32_t measured_connection[(ADAS_MAX_LANE_POINTS + 31U) / 32U];
};
struct adas_zebra {
    uint32_t zebra_id;
    struct adas_box box;
    uint32_t stripe_count, evidence_present, evidence_permille, origin, flags, original_index;
};
struct adas_detection {
    struct adas_branch_header header;
    uint32_t class_map_version, semantic_supported_mask;
    uint32_t signal_capability_mask, signals_supported_direction_mask;
    struct adas_collection objects_meta, signals_meta;
    struct adas_object objects[ADAS_MAX_OBJECTS];
    struct adas_signal signals[ADAS_MAX_SIGNALS];
    uint32_t light_visual_valid, light_visual_count;
    struct adas_light_visual light_visual[ADAS_MAX_SIGNALS];
};
struct adas_road {
    struct adas_branch_header header;
    uint32_t supported_tasks;
    struct adas_lane left, right;
    struct adas_collection zebras_meta;
    struct adas_zebra zebras[ADAS_MAX_ZEBRAS];
    struct road_ref_geometry reference;
};
struct adas_result {
    uint32_t version, size;
    uint64_t publication_sequence, application_session;
    uint32_t policy_generation, context_present;
    struct adas_context context;
    uint32_t canvas_width, canvas_height;
    uint64_t changed_ns, read_ns;
    struct adas_detection detection;
    struct adas_road road;
};

/* Registered by the control thread, fixed throughout one application session.
 * producer_session is a local ID, explicitly bound to application_session.
 */
struct adas_producer {
    uint32_t enabled, origin;
    uint64_t producer_session;
    uint32_t algorithm_id, algorithm_version, algorithm_config_generation;
    uint32_t class_map_version, semantic_supported_mask;
    uint32_t signal_capability_mask, signals_supported_direction_mask, road_supported_tasks;
};
struct adas_setup {
    uint32_t policy_generation, context_present, simulation_mode;
    struct adas_context context;
    uint64_t detection_ttl_ns, road_ttl_ns, fusion_window_ns;
    struct adas_producer detection, road;
};
struct adas_token {
    uint32_t version, branch, policy_generation;
    struct adas_branch_header header;
};
struct adas_cursor { uint64_t sequence; };
struct adas_watermark { uint32_t present, frame_sequence; uint64_t observed_ns; };

/* Internal storage. Do not access fields while live; exposed for static
 * allocation. No heap allocation, hardware access or blocking wait in this API.
 * Lifecycle control is serialized by the application's owner thread.
 * Stop/invalidate before joining producers; all readers exit before destroy.
 */
struct adas_store {
    atomic_bool busy;
    uint32_t initialized, available, terminal;
    uint64_t session, sequence, last_now_ns, session_started_ns;
    struct adas_setup setup;
    struct adas_watermark watermarks[2];
    struct adas_result latest;
    void (*publish_notify)(void *context);
    void *publish_notify_context;
    void (*publish_observer)(void *context, const struct adas_result *snapshot,
                             uint32_t branch, uint64_t now_ns);
    void *publish_observer_context;
};

int adas_detection_valid(const struct adas_detection *result, uint64_t now_ns);
int adas_road_valid(const struct adas_road *result, uint64_t now_ns);
int adas_result_valid(const struct adas_result *result, uint64_t now_ns);
int adas_store_init(struct adas_store *store);
/* Lifecycle only: set before starting producers, clear after they all join.
 * Successful commits call this outside the store lock. Callback must not block;
 * it is a wake request, with no borrowed result or guarantee it remains latest. */
int adas_store_set_publish_notify(struct adas_store *store,
                                 void (*notify)(void *context), void *context);
/* Lifecycle as above. Borrowed deep snapshot of this successful commit only.
 * Called outside the lock; bounded callback must copy anything it retains. */
int adas_store_set_publish_observer(struct adas_store *store,
    void (*observer)(void *, const struct adas_result *, uint32_t, uint64_t), void *context);
int adas_begin_session(struct adas_store *store, const struct adas_setup *setup,
                       uint64_t now_ns, uint32_t reason);
int adas_invalidate(struct adas_store *store, uint64_t now_ns, uint32_t reason);
int adas_stop(struct adas_store *store, uint64_t now_ns);
int adas_get_token(struct adas_store *store, uint32_t branch,
                   const struct adas_identity *identity, uint64_t observed_ns,
                   uint64_t now_ns, struct adas_token *out);
/* Initialize a zeroed candidate with registered metadata and UNKNOWN tasks.
 * The producer must set received/finished, state and supported subtask results.
 */
int adas_prepare_detection(struct adas_store *store, const struct adas_token *token,
                           struct adas_detection *out);
int adas_prepare_road(struct adas_store *store, const struct adas_token *token,
                      struct adas_road *out);
int adas_publish_detection(struct adas_store *store, const struct adas_token *token,
                           const struct adas_detection *candidate, uint64_t now_ns);
int adas_publish_road(struct adas_store *store, const struct adas_token *token,
                      const struct adas_road *candidate, uint64_t now_ns);
int adas_tick(struct adas_store *store, uint64_t now_ns);
/* OK_NEW/OK_SAME always copy a coherent snapshot. Other returns zero out.
 * Reading the same result never renews TTL. Each consumer owns its cursor.
 */
int adas_get_latest(struct adas_store *store, uint64_t now_ns,
                    struct adas_cursor *cursor, struct adas_result *out);
/* Pure checks for an owned snapshot before use: caller still compares current
 * application session/policy immediately before final display/decision commit.
 * Branch readiness checks MOCK against simulation_mode and task freshness.
 */
int adas_branch_ready(const struct adas_result *result, uint32_t branch,
                      uint64_t now_ns, uint32_t simulation_mode);
int adas_can_fuse(const struct adas_result *result, uint64_t now_ns,
                  uint64_t window_ns, uint32_t simulation_mode);
int adas_same_frame(const struct adas_result *result);
int adas_signal_actionable(const struct adas_signal *signal, uint32_t direction,
                           uint64_t control_generation);
/* Caller guarantees no concurrent producers/readers. Zeroes storage; never
 * joins threads or releases external contexts, which belong to the caller.
 */
void adas_store_destroy(struct adas_store *store);

#endif
