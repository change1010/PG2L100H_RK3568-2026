#ifndef ADAS_PEDESTRIAN_TEST_H
#define ADAS_PEDESTRIAN_TEST_H

#include "adas_decision.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADAS_PED_MAX_OBJECTS 32U
#define ADAS_PED_RULE_VERSION 1U
enum adas_ped_risk { ADAS_PED_UNKNOWN, ADAS_PED_CLEAR, ADAS_PED_NEARBY, ADAS_PED_CONFLICT };
enum adas_ped_reason {
    ADAS_PED_WAIT = 1U, ADAS_PED_BAD_ID = 2U, ADAS_PED_BAD_TIME = 4U,
    ADAS_PED_COLLECTION = 8U, ADAS_PED_POSITION = 16U, ADAS_PED_BOUNDARY = 32U,
    ADAS_PED_OUT_OF_RANGE = 64U, ADAS_PED_PATH = 128U, ADAS_PED_FAILED = 256U
};
/* Coordinates in thousandths of the full front image. A fixed-view
 * development profile, never a per-picture answer or a metric safety claim. */
struct adas_ped_path {
    uint32_t version, source_id, layout, config_generation;
    int32_t top_y, bottom_y, top_left, top_right, bottom_left, bottom_right;
    int32_t boundary, nearby;
    uint32_t minimum_confidence, minimum_height, minimum_width;
};
struct adas_ped_object {
    int32_t x0, y0, x1, y1;
    uint32_t confidence, position_unknown, original_index;
};
struct adas_ped_observation {
    struct adas_drive_frame frame;
    uint64_t observed_ns, finished_ns, ttl_ns;
    uint32_t valid, complete, count;
    uint32_t algorithm_id, algorithm_version, model_generation, input_profile_generation;
    struct adas_ped_object objects[ADAS_PED_MAX_OBJECTS];
};
struct adas_ped_test {
    struct adas_drive_frame frame;
    struct adas_ped_path path;
    struct adas_ped_observation observation;
    struct adas_drive_control control;
    struct adas_drive_output output;
    uint64_t context, started_ns, last_now_ns;
    uint32_t active, received, risk, reason, recommended_action;
};
struct adas_detection;
struct adas_result;
struct adas_ped_path adas_ped_default_path(uint32_t config_generation);
void adas_ped_from_detection(struct adas_ped_observation *out,
                              const struct adas_detection *detection);
void adas_ped_from_result(struct adas_ped_observation *out, const struct adas_result *result);
void adas_ped_test_init(struct adas_ped_test *state);
int adas_ped_test_enter(struct adas_ped_test *state, const struct adas_drive_frame *frame,
                        const struct adas_ped_path *path, uint64_t now);
void adas_ped_test_exit(struct adas_ped_test *state, uint64_t now);
void adas_ped_test_clear(struct adas_ped_test *state, uint64_t now);
/* frozen_task is allowed only for one inference on this context's owned
 * image; live cached publications must pass their original TTL at binding. */
int adas_ped_test_observe(struct adas_ped_test *state, uint64_t context,
                          const struct adas_ped_observation *observation,
                          uint64_t image_observed_ns, int frozen_task, uint64_t now);
void adas_ped_test_tick(struct adas_ped_test *state, uint64_t now);
int adas_ped_test_click(struct adas_ped_test *state, uint32_t command, uint64_t now);

#ifdef __cplusplus
}
#endif
#endif
