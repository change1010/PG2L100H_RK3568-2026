#include "adas_signal_test.h"
#include "adas_result.h"
#include <string.h>

static int lane_valid(uint32_t lane)
{
    return lane == ADAS_LEFT || lane == ADAS_STRAIGHT || lane == ADAS_RIGHT;
}

static void reset_layout(struct adas_signal_test *s)
{
    s->layout_signature = 0;
    s->layout = s->group_count = 0;
    memset(s->group_ids, 0, sizeof(s->group_ids));
}

static int trusted_color(const struct adas_test_group *g)
{
    return g->color_valid == 1 &&
        (g->color == ADAS_RED || g->color == ADAS_YELLOW || g->color == ADAS_GREEN);
}

static uint32_t certify_layout(struct adas_signal_test *s,
                                const struct adas_signal_test_observation *o)
{
    const uint32_t count = o->layout == ADAS_TEST_THREE_ARROW_CIRCLES ? 3 :
        (o->layout == ADAS_TEST_TWO_ARROW_CIRCLE || o->layout == ADAS_TEST_TWO_CIRCLES) ? 2 : 0;
    if (!count) return ADAS_TEST_LAYOUT_UNSUPPORTED;
    if (o->groups_valid != 1 || !o->group_signature || o->group_count != count)
        return ADAS_TEST_GROUP_UNKNOWN;
    for (uint32_t i = 0; i < count; ++i) {
        const struct adas_test_group *g = &o->groups[i];
        if (g->identity_valid != 1 || !g->id || g->color_valid > 1)
            return ADAS_TEST_GROUP_UNKNOWN;
        for (uint32_t j = 0; j < i; ++j)
            if (o->groups[j].id == g->id) return ADAS_TEST_GROUP_UNKNOWN;
    }
    /* An absent detector box must never turn a known straight lamp into a
     * left lamp. Only a new frozen context can establish new ordered IDs. */
    if (s->layout_signature) {
        if (s->layout_signature != o->group_signature || s->layout != o->layout ||
            s->group_count != count) return ADAS_TEST_GROUP_CHANGED;
        for (uint32_t i = 0; i < count; ++i)
            if (s->group_ids[i] != o->groups[i].id) return ADAS_TEST_GROUP_CHANGED;
    } else {
        s->layout_signature = o->group_signature;
        s->layout = o->layout; s->group_count = count;
        for (uint32_t i = 0; i < count; ++i) s->group_ids[i] = o->groups[i].id;
    }
    return ADAS_TEST_NONE;
}

static uint32_t select_signal(const struct adas_signal_test_observation *o, uint32_t index,
                              uint32_t *color, uint64_t *group_id)
{
    const struct adas_test_group *a = &o->groups[index];
    *group_id = a->id;
    if (index == 1 && o->group_count == 3) {
        const struct adas_test_group *b = &o->groups[2];
        const int a_valid = trusted_color(a), b_valid = trusted_color(b);
        if (a_valid && b_valid && a->color != b->color) return ADAS_TEST_STRAIGHT_CONFLICT;
        if (!a_valid && !b_valid) return ADAS_TEST_STRAIGHT_UNKNOWN;
        *color = a_valid ? a->color : b->color;
    } else {
        if (!trusted_color(a)) return ADAS_TEST_COLOR_UNKNOWN;
        *color = a->color;
    }
    return ADAS_TEST_NONE;
}

static void reset_result(struct adas_signal_test *s)
{
    s->control.command = ADAS_DRIVE_UNKNOWN;
    s->control.generation++;
    memset(&s->output, 0, sizeof(s->output));
    s->output.reasons = ADAS_DRIVE_NO_INPUT;
    s->result_reason = ADAS_TEST_NONE;
}

static void reset_evidence(struct adas_signal_test *s, uint64_t now, uint32_t reason)
{
    s->readiness = reason == ADAS_TEST_NOT_STABLE ? ADAS_TEST_RECOGNIZING : ADAS_TEST_INSUFFICIENT;
    s->reason = reason;
    s->stable_count = s->color = 0;
    s->first_ns = s->previous_ns = s->group_signature = s->group_id = 0;
    (void)now;
}

static void reset_direction(struct adas_test_direction_evidence *d, uint32_t reason)
{
    memset(d, 0, sizeof(*d));
    d->readiness = reason == ADAS_TEST_NOT_STABLE ? ADAS_TEST_RECOGNIZING : ADAS_TEST_INSUFFICIENT;
    d->reason = reason;
}

static void select_evidence(struct adas_signal_test *s)
{
    if (!s->active) { reset_evidence(s, s->last_now_ns, ADAS_TEST_NO_MODE); return; }
    if (!lane_valid(s->control.ego_lane)) {
        reset_evidence(s, s->last_now_ns, ADAS_TEST_NO_LANE); return;
    }
    if (s->control.scene != ADAS_SCENE_INTERSECTION) {
        reset_evidence(s, s->last_now_ns, ADAS_TEST_UNSUPPORTED); return;
    }
    if (s->control.ego_lane == ADAS_RIGHT) {
        reset_evidence(s, s->last_now_ns, ADAS_TEST_DIRECTION_UNDEFINED); return;
    }
    const struct adas_test_direction_evidence *d =
        &s->directions[s->control.ego_lane == ADAS_LEFT ? 0 : 1];
    s->readiness = d->readiness; s->reason = d->reason;
    s->stable_count = d->stable_count; s->color = d->color;
    s->first_ns = d->first_ns; s->previous_ns = d->previous_ns;
    s->group_signature = d->group_signature; s->group_id = d->group_id;
}

static void restart_perception(struct adas_signal_test *s, uint64_t now)
{
    s->perception++;
    s->completed_tasks = 0;
    s->started_ns = now;
    for (unsigned i = 0; i < 2; ++i) reset_direction(&s->directions[i], ADAS_TEST_NOT_STABLE);
    select_evidence(s);
}

void adas_signal_test_init(struct adas_signal_test *s)
{
    memset(s, 0, sizeof(*s));
    s->config = (struct adas_signal_test_config){5, UINT64_C(300000000),
        UINT64_C(500000000), UINT64_C(3000000000)};
    s->control.command = ADAS_DRIVE_UNKNOWN;
    s->output.reasons = ADAS_DRIVE_NO_INPUT;
    s->reason = ADAS_TEST_NO_MODE;
}

int adas_signal_test_set_config(struct adas_signal_test *s,
                                const struct adas_signal_test_config *config, uint64_t now)
{
    if (!config || !config->stable_tasks || !config->maximum_gap_ns || !config->wait_ns ||
        !now || now < s->last_now_ns) return -1;
    s->config = *config;
    s->last_now_ns = now;
    s->selection++;
    reset_result(s);
    restart_perception(s, now);
    return 0;
}

int adas_signal_test_enter(struct adas_signal_test *s,
                           const struct adas_drive_frame *frame, uint64_t now)
{
    if (!now || now < s->last_now_ns) return -1;
    if (!frame || !frame->valid || frame->source_id >= 4 || frame->layout != frame->source_id) {
        if (!s->active) s->reason = ADAS_TEST_NO_FRAME;
        return -1;
    }
    if (s->active) return -1;
    s->frame = *frame;
    memset(&s->road, 0, sizeof(s->road));
    reset_layout(s);
    s->context++;
    s->selection++;
    s->active = 1;
    s->last_task = 0;
    s->last_now_ns = now;
    s->control.scene = ADAS_SCENE_UNKNOWN;
    s->control.ego_lane = 0;
    reset_result(s);
    restart_perception(s, now);
    return 0;
}

void adas_signal_test_exit(struct adas_signal_test *s, uint64_t now)
{
    if (now < s->last_now_ns) now = s->last_now_ns;
    s->last_now_ns = now;
    s->context++;
    s->selection++;
    s->perception++;
    s->completed_tasks = 0;
    memset(s->directions, 0, sizeof(s->directions));
    s->active = 0;
    s->control.scene = ADAS_SCENE_UNKNOWN;
    s->control.ego_lane = 0;
    memset(&s->frame, 0, sizeof(s->frame));
    memset(&s->road, 0, sizeof(s->road));
    reset_layout(s);
    reset_result(s);
    reset_evidence(s, now, ADAS_TEST_NO_MODE);
}

void adas_signal_test_clear(struct adas_signal_test *s, uint64_t now)
{
    if (now < s->last_now_ns) now = s->last_now_ns;
    s->last_now_ns = now;
    s->selection++;
    s->control.scene = ADAS_SCENE_UNKNOWN;
    s->control.ego_lane = 0;
    reset_result(s);
    select_evidence(s);
}

int adas_signal_test_configure(struct adas_signal_test *s,
                               uint32_t scene, uint32_t lane, uint64_t now)
{
    if (!s->active || !now || now < s->last_now_ns || scene > ADAS_SCENE_INTERSECTION ||
        (lane && !lane_valid(lane))) return -1;
    if (scene == s->control.scene && lane == s->control.ego_lane) return 0;
    s->selection++;
    s->last_now_ns = now;
    s->control.scene = scene;
    s->control.ego_lane = lane;
    reset_result(s);
    select_evidence(s);
    return 1;
}

void adas_signal_test_tick(struct adas_signal_test *s, uint64_t now)
{
    if (!now || now < s->last_now_ns) {
        s->selection++;
        s->perception++;
        for (unsigned i = 0; i < 2; ++i) reset_direction(&s->directions[i], ADAS_TEST_BAD_TIME);
        select_evidence(s);
        return;
    }
    s->last_now_ns = now;
    if (!s->active) return;
    for (unsigned i = 0; i < 2; ++i) {
        struct adas_test_direction_evidence *d = &s->directions[i];
        if (d->readiness == ADAS_TEST_READY) continue;
        if (s->completed_tasks < s->config.stable_tasks && d->previous_ns &&
            now - d->previous_ns > s->config.maximum_gap_ns)
            reset_direction(d, ADAS_TEST_NOT_STABLE);
        if (d->reason == ADAS_TEST_NOT_STABLE && now - s->started_ns >= s->config.wait_ns) {
            d->reason = ADAS_TEST_TIMEOUT;
            d->readiness = ADAS_TEST_INSUFFICIENT;
        }
    }
    select_evidence(s);
}

int adas_signal_test_observe(struct adas_signal_test *s,
                             const struct adas_signal_test_observation *o, uint64_t now)
{
    /* Lane selection cannot invalidate a task from the same physical batch. */
    if (!o || !s->active || o->context != s->context || o->perception != s->perception ||
        !o->selection || o->selection > s->selection ||
        !adas_lane_test_same_frame(&o->frame, &s->frame) ||
        !o->sequence || o->sequence <= s->last_task) return 0;
    if (!now || now < s->last_now_ns) {
        adas_signal_test_tick(s, now);
        return -1;
    }
    s->last_now_ns = now;
    s->last_task = o->sequence;
    if (s->completed_tasks < s->config.stable_tasks) s->completed_tasks++;
    uint32_t reason = o->reason;
    if (reason > ADAS_TEST_BAD_TIME) reason = ADAS_TEST_FAILED;
    if (!o->started_ns || o->started_ns < s->started_ns ||
        o->finished_ns < o->started_ns || o->finished_ns > now) reason = ADAS_TEST_BAD_TIME;
    if (!reason) reason = certify_layout(s, o);
    for (unsigned i = 0; i < 2; ++i) {
        struct adas_test_direction_evidence *d = &s->directions[i];
        uint32_t color = ADAS_COLOR_UNKNOWN;
        uint64_t group_id = 0;
        uint32_t direction_reason = reason ? reason : select_signal(o, i, &color, &group_id);
        if (direction_reason) { reset_direction(d, direction_reason); continue; }
        if (!d->previous_ns || o->group_signature != d->group_signature ||
            group_id != d->group_id || color != d->color || o->finished_ns < d->previous_ns ||
            (d->readiness != ADAS_TEST_READY &&
             o->finished_ns - d->previous_ns > s->config.maximum_gap_ns)) {
            reset_direction(d, ADAS_TEST_NOT_STABLE);
            d->first_ns = o->finished_ns;
        }
        d->group_signature = o->group_signature; d->group_id = group_id; d->color = color;
        d->previous_ns = o->finished_ns;
        if (d->stable_count < s->config.stable_tasks) d->stable_count++;
        if (d->stable_count >= s->config.stable_tasks &&
            d->previous_ns - d->first_ns >= s->config.stable_span_ns) {
            d->readiness = ADAS_TEST_READY;
            d->reason = ADAS_TEST_NONE;
        } else if (s->completed_tasks >= s->config.stable_tasks) {
            d->readiness = ADAS_TEST_INSUFFICIENT;
        }
    }
    select_evidence(s);
    return 1;
}

int adas_signal_test_observe_road(struct adas_signal_test *s,
                                 const struct adas_lane_test_observation *o, uint64_t now)
{
    if (!s->active || !o || !adas_lane_test_same_frame(&s->frame, &o->frame) ||
        !now || now < s->last_now_ns || !o->observed_ns ||
        o->observed_ns > now || o->valid_until_ns <= now ||
        s->road.observed_ns) return 0;
    s->road = *o;
    return 1;
}

int adas_signal_test_click(struct adas_signal_test *s, uint32_t command, uint64_t now)
{
    if (!s->active || command > ADAS_DRIVE_CHANGE_RIGHT || !now || now < s->last_now_ns) return -1;
    adas_signal_test_tick(s, now);
    s->control.command = command;
    s->control.generation++;
    s->control.observed_ns = now;
    struct adas_drive_output *out = &s->output;
    memset(out, 0, sizeof(*out));
    out->evaluated = 1;
    out->control_generation = s->control.generation;
    out->evaluated_ns = now;
    out->path_direction = s->control.ego_lane;
    out->required_tasks = ADAS_DECISION_SIGNAL;
    out->application_session = s->context;
    out->publication_sequence = s->last_task;
    out->permission = ADAS_DRIVE_INSUFFICIENT;
    s->result_reason = s->reason;
    if (command == ADAS_DRIVE_CHANGE_LEFT || command == ADAS_DRIVE_CHANGE_RIGHT) {
        adas_lane_test_apply(&s->road, &s->frame, command, out);
        out->publication_sequence = s->road.frame.frame_sequence;
        s->result_reason = ADAS_TEST_NONE;
        return 1;
    }
    /* The user-defined simulation exempts the right lane from signal gating. */
    if (s->control.scene == ADAS_SCENE_INTERSECTION && s->control.ego_lane == ADAS_RIGHT) {
        out->required_tasks = 0;
        out->simulation_mode = 1;
        s->result_reason = ADAS_TEST_NONE;
        if (command == ADAS_DRIVE_STOP) {
            out->permission = ADAS_DRIVE_BLOCK;
            out->action = ADAS_ACTION_STOP;
            out->reasons = ADAS_DRIVE_USER_STOP;
        } else {
            out->permission = ADAS_DRIVE_ALLOW;
            out->action = ADAS_ACTION_ADVANCE;
        }
        return 1;
    }
    if (s->control.scene != ADAS_SCENE_INTERSECTION) out->reasons |= ADAS_DRIVE_NO_SCENE;
    if (!lane_valid(s->control.ego_lane)) out->reasons |= ADAS_DRIVE_NO_LANE;
    if (s->readiness != ADAS_TEST_READY) out->reasons |= ADAS_DRIVE_SIGNAL_UNKNOWN;
    if (command == ADAS_DRIVE_STOP) {
        out->action = ADAS_ACTION_STOP;
        out->reasons |= ADAS_DRIVE_USER_STOP;
        if (s->readiness == ADAS_TEST_READY && !(out->reasons & (ADAS_DRIVE_NO_SCENE | ADAS_DRIVE_NO_LANE)))
            out->permission = ADAS_DRIVE_BLOCK;
    } else if (command != ADAS_DRIVE_ADVANCE) {
        out->reasons |= ADAS_DRIVE_LINE_UNKNOWN;
    } else if (!out->reasons) {
        if (s->color == ADAS_RED) {
            out->permission = ADAS_DRIVE_BLOCK;
            out->action = ADAS_ACTION_STOP;
            out->reasons = out->alarm_active = ADAS_DRIVE_RED;
        } else {
            out->permission = ADAS_DRIVE_ALLOW;
            out->action = ADAS_ACTION_ADVANCE;
        }
    }
    return 1;
}
