#ifndef ADAS_DECISION_H
#define ADAS_DECISION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct adas_result;

#define ADAS_DRIVE_QUEUE_CAPACITY 16U
#define ADAS_DECISION_SIGNAL 1U
#define ADAS_DECISION_LINE 2U
#define ADAS_DECISION_HAZARD 4U

/* Preserve the dashboard's four button codes; zero is an explicit STOP,
 * never the initial state. ADVANCE follows ego_lane at an intersection. */
enum adas_drive_command {
    ADAS_DRIVE_STOP, ADAS_DRIVE_CHANGE_LEFT, ADAS_DRIVE_ADVANCE,
    ADAS_DRIVE_CHANGE_RIGHT, ADAS_DRIVE_UNKNOWN
};
enum adas_drive_scene { ADAS_SCENE_UNKNOWN, ADAS_SCENE_ROAD, ADAS_SCENE_INTERSECTION };
enum adas_drive_permission { ADAS_DRIVE_UNASSESSED, ADAS_DRIVE_ALLOW, ADAS_DRIVE_BLOCK, ADAS_DRIVE_INSUFFICIENT };
enum adas_drive_action { ADAS_ACTION_UNKNOWN, ADAS_ACTION_STOP, ADAS_ACTION_ADVANCE,
                         ADAS_ACTION_CHANGE_LEFT, ADAS_ACTION_CHANGE_RIGHT, ADAS_ACTION_KEEP_LANE };
enum adas_drive_view { ADAS_VIEW_UNKNOWN, ADAS_VIEW_FRONT, ADAS_VIEW_LEFT, ADAS_VIEW_RIGHT };
enum adas_drive_reason {
    ADAS_DRIVE_NO_INPUT = 1U, ADAS_DRIVE_USER_STOP = 2U,
    ADAS_DRIVE_NO_SCENE = 4U, ADAS_DRIVE_NO_LANE = 8U,
    ADAS_DRIVE_NO_EVIDENCE = 16U, ADAS_DRIVE_BAD_CONTEXT = 32U,
    ADAS_DRIVE_SIGNAL_UNKNOWN = 64U, ADAS_DRIVE_RED = 128U,
    ADAS_DRIVE_YELLOW = 256U, ADAS_DRIVE_SIGNAL_CONFLICT = 512U,
    ADAS_DRIVE_LINE_UNKNOWN = 1024U, ADAS_DRIVE_SOLID = 2048U,
    ADAS_DRIVE_HAZARD_UNKNOWN = 4096U, ADAS_DRIVE_PEDESTRIAN = 8192U,
    ADAS_DRIVE_VEHICLE = 16384U, ADAS_DRIVE_QUEUE_LOST = 32768U,
    ADAS_DRIVE_TIME_ERROR = 65536U, ADAS_DRIVE_PEDESTRIAN_NEARBY = 131072U
};
#define ADAS_DRIVE_ALARM_REASONS (ADAS_DRIVE_RED | ADAS_DRIVE_SOLID | \
                                ADAS_DRIVE_PEDESTRIAN | ADAS_DRIVE_VEHICLE)

struct adas_drive_control {
    uint32_t command, scene, ego_lane;
    uint64_t generation, association_generation, observed_ns;
};
struct adas_drive_frame {
    uint32_t valid, stream_epoch, source_id, layout, config_generation, frame_sequence;
};
/* Explicit, reviewed scene calibration, never inferred from source_id.
 * A region certifies only image-space coverage, not physical collision safety.
 * path[0..2] correspond to ADVANCE, CHANGE_LEFT, CHANGE_RIGHT. */
struct adas_drive_path {
    uint32_t configured, source_id, layout, config_generation, scene, ego_lane, view;
    uint32_t version;
    int32_t x0, y0, x1, y1;
};
struct adas_decision_config {
    uint32_t simulation_mode, hazard_required;
    struct adas_drive_path path[3];
};
struct adas_drive_output {
    uint32_t evaluated, permission, action, path_direction, reasons, required_tasks;
    uint32_t alarm_active, alarm_unresolved, alarm_acknowledged, alarm_history, alarm_checked;
    uint32_t simulation_mode;
    uint64_t control_generation, association_generation;
    uint64_t application_session, publication_sequence, evaluated_ns, valid_until_ns;
};
struct adas_drive_alarm_scope {
    uint32_t scene, ego_lane, command;
    struct adas_drive_frame frame;
    uint64_t application_session;
};
/* One serialized owner; GUI operations and owner steps must share a mutex.
 * The queue preserves MOVE->STOP between video ticks. Overflow fails closed. */
struct adas_decision {
    struct adas_drive_control selected, applied;
    struct adas_drive_control queue[ADAS_DRIVE_QUEUE_CAPACITY];
    uint32_t head, count, queue_lost;
    uint64_t last_step_ns, alarm_revision;
    struct adas_drive_alarm_scope alarm_scope[4];
    struct adas_drive_output output;
};

void adas_decision_init(struct adas_decision *state);
int adas_decision_select(struct adas_decision *state, uint32_t command, uint64_t now_ns);
int adas_decision_configure(struct adas_decision *state, uint32_t scene,
                            uint32_t ego_lane, uint64_t now_ns);
void adas_decision_reset_input(struct adas_decision *state, uint64_t now_ns);
void adas_decision_acknowledge(struct adas_decision *state);
void adas_decision_evaluate(const struct adas_drive_control *control,
                            const struct adas_decision_config *config,
                            const struct adas_drive_frame *frame,
                            const struct adas_result *result, uint64_t now_ns,
                            struct adas_drive_output *out);
int adas_decision_step(struct adas_decision *state,
                       const struct adas_decision_config *config,
                       const struct adas_drive_frame *frame,
                       const struct adas_result *result, uint64_t now_ns);

#ifdef __cplusplus
}
#endif
#endif
