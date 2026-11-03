#ifndef ADAS_SIGNAL_TEST_H
#define ADAS_SIGNAL_TEST_H

#include "adas_decision.h"
#include "adas_lane_test.h"

#ifdef __cplusplus
extern "C" {
#endif

enum adas_test_readiness { ADAS_TEST_RECOGNIZING, ADAS_TEST_READY, ADAS_TEST_INSUFFICIENT };
#define ADAS_SIGNAL_TEST_RULE_VERSION 5U
enum adas_test_layout {
    ADAS_TEST_LAYOUT_UNKNOWN,
    ADAS_TEST_TWO_ARROW_CIRCLE,
    ADAS_TEST_TWO_CIRCLES,
    ADAS_TEST_THREE_ARROW_CIRCLES
};
enum adas_test_reason {
    ADAS_TEST_NONE, ADAS_TEST_NO_MODE, ADAS_TEST_NO_FRAME, ADAS_TEST_NO_LANE,
    ADAS_TEST_GROUP_UNKNOWN, ADAS_TEST_DIRECTION_UNDEFINED, ADAS_TEST_COLOR_UNKNOWN,
    ADAS_TEST_YELLOW, ADAS_TEST_COLOR_CONFLICT, ADAS_TEST_BLINKING,
    ADAS_TEST_NOT_STABLE, ADAS_TEST_TIMEOUT, ADAS_TEST_UNSUPPORTED,
    ADAS_TEST_FAILED, ADAS_TEST_LAYOUT_UNSUPPORTED, ADAS_TEST_GROUP_CHANGED,
    ADAS_TEST_STRAIGHT_UNKNOWN, ADAS_TEST_STRAIGHT_CONFLICT, ADAS_TEST_BAD_TIME
};
struct adas_test_group {
    uint64_t id;
    uint32_t identity_valid, color_valid, color;
};
struct adas_signal_test_config {
    uint32_t stable_tasks;
    uint64_t stable_span_ns, maximum_gap_ns, wait_ns;
};
/* Group formation belongs to the perception adapter. A raw detector box is
 * never sufficient to set groups_valid. The adapter certifies the supported
 * layout, scope, complete ordered identities and retained occluded identities.
 * Missing colors do not remove/reindex groups. sequence names a completed task. */
struct adas_signal_test_observation {
    uint64_t context, selection, sequence, started_ns, finished_ns;
    uint64_t perception;
    uint64_t group_signature;
    uint32_t groups_valid, layout, group_count, reason;
    struct adas_test_group groups[3];
    /* Frozen picture identity shared by every lane in this perception batch. */
    struct adas_drive_frame frame;
};
struct adas_test_direction_evidence {
    uint64_t first_ns, previous_ns, group_signature, group_id;
    uint32_t readiness, reason, stable_count, color;
};
struct adas_signal_test {
    struct adas_signal_test_config config;
    struct adas_drive_frame frame;
    struct adas_drive_control control;
    struct adas_drive_output output;
    struct adas_lane_test_observation road;
    uint64_t context, selection, started_ns, last_now_ns, last_task;
    uint64_t perception;
    uint32_t completed_tasks;
    struct adas_test_direction_evidence directions[2];
    uint64_t first_ns, previous_ns, group_signature, group_id;
    /* Pinned for this frozen context; lane/clear/config cannot reindex it. */
    uint64_t layout_signature, group_ids[3];
    uint32_t layout, group_count;
    uint32_t active, readiness, reason, result_reason, stable_count, color;
};

void adas_signal_test_init(struct adas_signal_test *state);
int adas_signal_test_set_config(struct adas_signal_test *state,
                                const struct adas_signal_test_config *config, uint64_t now);
int adas_signal_test_enter(struct adas_signal_test *state,
                           const struct adas_drive_frame *frame, uint64_t now);
void adas_signal_test_exit(struct adas_signal_test *state, uint64_t now);
void adas_signal_test_clear(struct adas_signal_test *state, uint64_t now);
int adas_signal_test_configure(struct adas_signal_test *state,
                               uint32_t scene, uint32_t lane, uint64_t now);
int adas_signal_test_observe(struct adas_signal_test *state,
                             const struct adas_signal_test_observation *observation, uint64_t now);
int adas_signal_test_observe_road(struct adas_signal_test *state,
                                 const struct adas_lane_test_observation *observation, uint64_t now);
void adas_signal_test_tick(struct adas_signal_test *state, uint64_t now);
/* Immediate evaluation of the saved evidence. No pending command/queue. */
int adas_signal_test_click(struct adas_signal_test *state, uint32_t command, uint64_t now);

#ifdef __cplusplus
}
#endif
#endif
