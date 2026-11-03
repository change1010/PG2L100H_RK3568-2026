#ifndef PEDESTRIAN_ALERT_H
#define PEDESTRIAN_ALERT_H

#ifdef __cplusplus
#include <atomic>
/* The C store is never instantiated here; Qt borrows immutable payloads. */
typedef std::atomic_bool atomic_bool;
extern "C" {
#endif
#include "adas_result.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define PED_ALERT_HISTORY 32U
#define PED_ALERT_CONFIG_VERSION 1U
enum ped_alert_level { PED_ALERT_NONE, PED_ALERT_CAUTION, PED_ALERT_YIELD };
enum ped_alert_status {
    PED_ALERT_OFF, PED_ALERT_RECOGNITION_OFF, PED_ALERT_UNSUPPORTED,
    PED_ALERT_WAITING, PED_ALERT_READY, PED_ALERT_UNAVAILABLE
};
enum ped_alert_reason {
    PED_ALERT_ROI = 1U, PED_ALERT_ZEBRA = 2U, PED_ALERT_NEAR = 4U,
    PED_ALERT_POSITION = 8U, PED_ALERT_INCOMPLETE = 16U, PED_ALERT_ROAD_WAIT = 32U,
    PED_ALERT_STALE = 64U, PED_ALERT_FAILURE = 128U, PED_ALERT_PENDING = 256U
};
/* Thousandths of the full image; development front-view profile, not a lane. */
struct ped_alert_roi {
    int32_t top_y, bottom_y, top_left, top_right, bottom_left, bottom_right;
};
struct ped_alert_config {
    uint32_t version, confidence, height, near_height, width;
    uint32_t foot_height, foot_width, overlap, boundary;
    struct ped_alert_roi attention, relevant, near;
    uint64_t release_ns;
};
struct ped_alert_output {
    uint64_t generation, revision, observed_ns, valid_until_ns, evaluations, paired, dropped, rejected;
    struct adas_identity identity;
    uint32_t enabled, status, level, reason, complete, road_available;
};
struct ped_alert_people {
    struct adas_branch_header header;
    struct adas_collection meta;
    uint32_t valid, count, pending, processed;
    struct adas_object objects[ADAS_MAX_OBJECTS];
};
struct ped_alert_road {
    struct adas_branch_header header;
    struct adas_collection meta;
    uint32_t valid;
    struct adas_zebra zebras[ADAS_MAX_ZEBRAS];
};
/* Serialized by the caller. No UI, hardware, pixels or worker ownership. */
struct pedestrian_alert {
    struct ped_alert_config config;
    struct adas_identity context;
    struct ped_alert_output output;
    uint64_t accept_after_ns, last_now_ns, last_observed_ns, negative_since_ns;
    uint64_t application_session, producer_session;
    uint32_t input_profile, algorithm_id, algorithm_version, model_generation;
    uint32_t running, people_next, road_next, votes, vote_count, negative_count;
    struct ped_alert_people people[PED_ALERT_HISTORY];
    struct ped_alert_road roads[PED_ALERT_HISTORY];
};

struct ped_alert_config ped_alert_default_config(void);
void ped_alert_init(struct pedestrian_alert *state);
/* status is OFF/RECOGNITION_OFF/UNSUPPORTED/WAITING; WAITING means eligible. */
void ped_alert_control(struct pedestrian_alert *state, uint32_t enabled, uint32_t status,
                       const struct adas_identity *context, uint64_t now);
void ped_alert_gap(struct pedestrian_alert *state);
int ped_alert_publish(struct pedestrian_alert *state, const struct adas_result *result,
                       uint32_t branch, uint64_t now);
void ped_alert_tick(struct pedestrian_alert *state, uint64_t now, uint32_t budget);

#ifdef __cplusplus
}
#endif
#endif
