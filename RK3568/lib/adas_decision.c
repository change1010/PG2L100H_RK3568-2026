#include "adas_decision.h"
#include "adas_result.h"
#include <string.h>

static int direction_valid(uint32_t lane)
{
    return lane == ADAS_LEFT || lane == ADAS_STRAIGHT || lane == ADAS_RIGHT;
}

void adas_decision_init(struct adas_decision *s)
{
    memset(s, 0, sizeof(*s));
    s->selected.command = s->applied.command = ADAS_DRIVE_UNKNOWN;
    s->selected.association_generation = s->applied.association_generation = 1;
    s->output.reasons = ADAS_DRIVE_NO_INPUT;
}

void adas_decision_reset_input(struct adas_decision *s, uint64_t now)
{
    s->selected.command = ADAS_DRIVE_UNKNOWN;
    s->selected.observed_ns = now;
    s->selected.generation++;
    s->selected.association_generation++;
    s->applied = s->selected;
    s->head = s->count = s->queue_lost = 0;
    s->output.evaluated = 0;
    s->output.permission = ADAS_DRIVE_UNASSESSED;
    s->output.action = ADAS_ACTION_UNKNOWN;
    s->output.reasons = ADAS_DRIVE_NO_INPUT;
    s->output.valid_until_ns = 0;
    s->output.control_generation = s->selected.generation;
    s->output.association_generation = s->selected.association_generation;
    s->output.path_direction = s->selected.ego_lane;
    s->output.alarm_checked = 0;
    s->output.alarm_unresolved |= s->output.alarm_active;
    s->output.alarm_active = 0;
}

int adas_decision_configure(struct adas_decision *s, uint32_t scene,
                            uint32_t lane, uint64_t now)
{
    if (scene > ADAS_SCENE_INTERSECTION || (lane && !direction_valid(lane)) ||
        !now || now < s->selected.observed_ns) return -1;
    if (scene == s->selected.scene && lane == s->selected.ego_lane) return 0;
    s->selected.scene = scene;
    s->selected.ego_lane = lane;
    adas_decision_reset_input(s, now);
    return 1;
}

int adas_decision_select(struct adas_decision *s, uint32_t command, uint64_t now)
{
    if (command > ADAS_DRIVE_UNKNOWN || !now || now < s->selected.observed_ns) return -1;
    if (command == ADAS_DRIVE_UNKNOWN) {
        adas_decision_reset_input(s, now);
        return 1;
    }
    if (s->queue_lost) return -1;
    if (s->count == ADAS_DRIVE_QUEUE_CAPACITY) {
        adas_decision_reset_input(s, now);
        s->queue_lost = 1;
        s->output.reasons |= ADAS_DRIVE_QUEUE_LOST;
        return -1;
    }
    /* Repeated clicks are new attempts, even when the held command is equal. */
    s->selected.command = command;
    s->selected.observed_ns = now;
    s->selected.generation++;
    s->queue[(s->head + s->count) % ADAS_DRIVE_QUEUE_CAPACITY] = s->selected;
    s->count++;
    s->output.permission = ADAS_DRIVE_UNASSESSED;
    s->output.action = ADAS_ACTION_UNKNOWN;
    s->output.valid_until_ns = 0;
    return 1;
}

void adas_decision_acknowledge(struct adas_decision *s)
{
    s->output.alarm_acknowledged |= s->output.alarm_active | s->output.alarm_unresolved;
}

static int frame_matches(const struct adas_result *r, const struct adas_drive_frame *f)
{
    return f && f->valid && r->context_present &&
        f->source_id < 4 && f->layout == f->source_id &&
        r->context.stream_epoch == f->stream_epoch &&
        r->context.source_id == f->source_id && r->context.layout == f->layout &&
        r->context.config_generation == f->config_generation;
}

static int branch_available(const struct adas_result *r, uint32_t branch,
                             const struct adas_decision_config *cfg,
                             const struct adas_drive_frame *frame, uint64_t now,
                             uint64_t cutoff)
{
    if (!r || !frame_matches(r, frame) ||
        !adas_branch_ready(r, branch, now, cfg->simulation_mode)) return 0;
    const struct adas_branch_header *h = branch == ADAS_DETECTION ?
        &r->detection.header : &r->road.header;
    const uint32_t age = frame->frame_sequence - h->identity.frame_sequence;
    /* An event may only use observations made at/before that event. Later
     * observations can assess a still-held command on the continuous step. */
    return age < UINT32_C(0x80000000) && h->observed_ns <= cutoff;
}

static void limit_validity(struct adas_drive_output *out, const struct adas_branch_header *h)
{
    const uint64_t expiry = UINT64_MAX - h->observed_ns < h->ttl_ns ?
        UINT64_MAX : h->observed_ns + h->ttl_ns;
    if (!out->valid_until_ns || expiry < out->valid_until_ns) out->valid_until_ns = expiry;
}

static uint32_t signal_rule(const struct adas_result *r,
                            const struct adas_drive_control *c, uint32_t *blocked)
{
    const struct adas_detection *d = &r->detection;
    const uint32_t caps = ADAS_SIGNAL_LOCATION | ADAS_SIGNAL_DIRECTION | ADAS_SIGNAL_COLOR;
    uint32_t reasons = 0, colors = 0, found = 0;
    if ((d->signal_capability_mask & caps) != caps ||
        !(d->signals_supported_direction_mask & c->ego_lane))
        return ADAS_DRIVE_SIGNAL_UNKNOWN;
    if (d->signals_meta.state != ADAS_VALID) return ADAS_DRIVE_SIGNAL_UNKNOWN;
    if (d->signals_meta.flags || !d->signals_meta.total_count_known)
        reasons |= ADAS_DRIVE_SIGNAL_UNKNOWN;
    for (uint32_t i = 0; i < d->signals_meta.count; i++) {
        const struct adas_signal *s = &d->signals[i];
        if (s->direction_valid && !(s->direction_mask & c->ego_lane)) continue;
        if (s->association != ADAS_UNRESOLVED &&
            s->association_control_generation != c->association_generation) {
            reasons |= ADAS_DRIVE_SIGNAL_UNKNOWN;
            continue;
        }
        if (s->association == ADAS_OTHER_PATH) continue;
        if (!adas_signal_actionable(s, c->ego_lane, c->association_generation) ||
            s->shape == ADAS_SHAPE_UNKNOWN) {
            reasons |= ADAS_DRIVE_SIGNAL_UNKNOWN;
            continue;
        }
        found++;
        colors |= 1U << s->color;
        if (s->color == ADAS_RED) reasons |= ADAS_DRIVE_RED;
        else if (s->color == ADAS_YELLOW) reasons |= ADAS_DRIVE_YELLOW;
        else if (s->color != ADAS_GREEN) reasons |= ADAS_DRIVE_SIGNAL_UNKNOWN;
    }
    if (!found) reasons |= ADAS_DRIVE_SIGNAL_UNKNOWN;
    if (colors && (colors & (colors - 1U))) reasons |= ADAS_DRIVE_SIGNAL_CONFLICT;
    if (reasons & (ADAS_DRIVE_RED | ADAS_DRIVE_YELLOW)) *blocked = 1;
    return reasons;
}

static int path_matches(const struct adas_drive_path *p, const struct adas_drive_control *c,
                         const struct adas_drive_frame *f)
{
    const uint32_t view = c->command == ADAS_DRIVE_CHANGE_LEFT ? ADAS_VIEW_LEFT :
        c->command == ADAS_DRIVE_CHANGE_RIGHT ? ADAS_VIEW_RIGHT : ADAS_VIEW_FRONT;
    return p->configured == 1 && p->version && p->source_id == f->source_id &&
        p->layout == f->layout && p->config_generation == f->config_generation &&
        p->scene == c->scene && p->ego_lane == c->ego_lane && p->view == view &&
        p->x0 >= 0 && p->y0 >= 0 && p->x1 > p->x0 && p->y1 > p->y0 &&
        p->x1 <= (int32_t)ADAS_WIDTH && p->y1 <= (int32_t)ADAS_HEIGHT;
}

static uint32_t hazard_rule(const struct adas_result *r, const struct adas_drive_control *c,
                            const struct adas_decision_config *cfg,
                            const struct adas_drive_frame *f, uint32_t *blocked)
{
    const uint32_t index = c->command == ADAS_DRIVE_CHANGE_LEFT ? 1 :
        c->command == ADAS_DRIVE_CHANGE_RIGHT ? 2 : 0;
    const struct adas_drive_path *p = &cfg->path[index];
    if (!path_matches(p, c, f)) return ADAS_DRIVE_HAZARD_UNKNOWN;
    const struct adas_detection *d = &r->detection;
    const uint32_t classes = ADAS_CLASS_VEHICLE | ADAS_CLASS_PEDESTRIAN;
    uint32_t reasons = 0;
    if ((d->semantic_supported_mask & classes) != classes ||
        (d->objects_meta.state != ADAS_VALID && d->objects_meta.state != ADAS_ABSENT) ||
        d->objects_meta.flags || !d->objects_meta.total_count_known)
        reasons |= ADAS_DRIVE_HAZARD_UNKNOWN;
    for (uint32_t i = 0; i < d->objects_meta.count; i++) {
        const struct adas_object *o = &d->objects[i];
        const struct adas_box *b = &o->box;
        if (o->semantic_class == ADAS_CLASS_UNKNOWN) {
            reasons |= ADAS_DRIVE_HAZARD_UNKNOWN;
        } else if (o->semantic_class == ADAS_PEDESTRIAN) {
            /* Proposed footpoint approximation. Clipped feet cannot certify
             * clearance; the inclusive region boundary is conservative. */
            const int32_t x = b->x0 + (b->x1 - b->x0) / 2, y = b->y1 - 1;
            if (x >= p->x0 && x <= p->x1 && y >= p->y0 && y <= p->y1)
                reasons |= ADAS_DRIVE_PEDESTRIAN;
            if (b->y1 == (int32_t)ADAS_HEIGHT || b->x0 == 0 || b->x1 == (int32_t)ADAS_WIDTH)
                reasons |= ADAS_DRIVE_HAZARD_UNKNOWN;
        } else if (o->semantic_class == ADAS_VEHICLE &&
                   b->x0 <= p->x1 && b->x1 >= p->x0 && b->y0 <= p->y1 && b->y1 >= p->y0) {
            reasons |= ADAS_DRIVE_VEHICLE;
        }
    }
    if (reasons & (ADAS_DRIVE_PEDESTRIAN | ADAS_DRIVE_VEHICLE)) *blocked = 1;
    return reasons;
}

static void evaluate_at(const struct adas_drive_control *c, const struct adas_decision_config *cfg,
                         const struct adas_drive_frame *f, const struct adas_result *r,
                         uint64_t now, uint64_t cutoff, struct adas_drive_output *out)
{
    memset(out, 0, sizeof(*out));
    out->evaluated_ns = now;
    out->control_generation = c->generation;
    out->association_generation = c->association_generation;
    out->path_direction = c->ego_lane;
    out->simulation_mode = cfg->simulation_mode;
    if (c->command == ADAS_DRIVE_UNKNOWN) {
        out->reasons = ADAS_DRIVE_NO_INPUT;
        return;
    }
    out->evaluated = 1;
    out->permission = ADAS_DRIVE_INSUFFICIENT;
    if (c->command > ADAS_DRIVE_UNKNOWN || !now || cfg->simulation_mode > 1 ||
        cfg->hazard_required > 1 || !c->observed_ns || c->observed_ns > now) {
        out->reasons = ADAS_DRIVE_TIME_ERROR;
        return;
    }
    if (c->command == ADAS_DRIVE_STOP) {
        out->permission = ADAS_DRIVE_BLOCK;
        out->action = ADAS_ACTION_STOP;
        out->reasons = ADAS_DRIVE_USER_STOP;
        return;
    }
    if (c->scene != ADAS_SCENE_ROAD && c->scene != ADAS_SCENE_INTERSECTION)
        out->reasons |= ADAS_DRIVE_NO_SCENE;
    if (c->scene == ADAS_SCENE_INTERSECTION && !direction_valid(c->ego_lane))
        out->reasons |= ADAS_DRIVE_NO_LANE;
    if (c->scene == ADAS_SCENE_INTERSECTION) out->required_tasks |= ADAS_DECISION_SIGNAL;
    if (c->command == ADAS_DRIVE_CHANGE_LEFT || c->command == ADAS_DRIVE_CHANGE_RIGHT)
        out->required_tasks |= ADAS_DECISION_LINE;
    if (cfg->hazard_required) out->required_tasks |= ADAS_DECISION_HAZARD;
    if (!r || !adas_result_valid(r, now)) out->reasons |= ADAS_DRIVE_NO_EVIDENCE;
    else if (!frame_matches(r, f)) out->reasons |= ADAS_DRIVE_BAD_CONTEXT;
    if (out->reasons) return;
    out->application_session = r->application_session;
    out->publication_sequence = r->publication_sequence;
    uint32_t blocked = 0;
    if (out->required_tasks & ADAS_DECISION_SIGNAL) {
        if (!branch_available(r, ADAS_DETECTION, cfg, f, now, cutoff))
            out->reasons |= ADAS_DRIVE_SIGNAL_UNKNOWN;
        else {
            limit_validity(out, &r->detection.header);
            const uint32_t reasons = signal_rule(r, c, &blocked);
            out->reasons |= reasons;
            if (!(reasons & (ADAS_DRIVE_SIGNAL_UNKNOWN | ADAS_DRIVE_SIGNAL_CONFLICT)))
                out->alarm_checked |= ADAS_DRIVE_RED;
        }
    }
    if (out->required_tasks & ADAS_DECISION_LINE) {
        if (!branch_available(r, ADAS_ROAD, cfg, f, now, cutoff))
            out->reasons |= ADAS_DRIVE_LINE_UNKNOWN;
        else {
            const struct adas_lane *lane = c->command == ADAS_DRIVE_CHANGE_LEFT ? &r->road.left : &r->road.right;
            const uint32_t task = c->command == ADAS_DRIVE_CHANGE_LEFT ? ADAS_ROAD_LEFT : ADAS_ROAD_RIGHT;
            limit_validity(out, &r->road.header);
            if (!(r->road.supported_tasks & task) || lane->state != ADAS_VALID ||
                lane->flags || lane->line_type == ADAS_LINE_UNKNOWN)
                out->reasons |= ADAS_DRIVE_LINE_UNKNOWN;
            else if (lane->line_type == ADAS_SOLID) {
                blocked = 1;
                out->reasons |= ADAS_DRIVE_SOLID;
            }
            if (!(out->reasons & ADAS_DRIVE_LINE_UNKNOWN))
                out->alarm_checked |= ADAS_DRIVE_SOLID;
        }
    }
    if (out->required_tasks & ADAS_DECISION_HAZARD) {
        if (!branch_available(r, ADAS_DETECTION, cfg, f, now, cutoff))
            out->reasons |= ADAS_DRIVE_HAZARD_UNKNOWN;
        else {
            limit_validity(out, &r->detection.header);
            const uint32_t reasons = hazard_rule(r, c, cfg, f, &blocked);
            out->reasons |= reasons;
            if (!(reasons & ADAS_DRIVE_HAZARD_UNKNOWN))
                out->alarm_checked |= ADAS_DRIVE_PEDESTRIAN | ADAS_DRIVE_VEHICLE;
        }
    }
    /* Each branch can independently prohibit. Combining clearances from
     * different frames requires additional temporal alignment. */
    if ((out->required_tasks & ADAS_DECISION_LINE) &&
        (out->required_tasks & (ADAS_DECISION_SIGNAL | ADAS_DECISION_HAZARD)) &&
        !adas_can_fuse(r, now, ADAS_FUSION_WINDOW_NS, cfg->simulation_mode))
        out->reasons |= ADAS_DRIVE_NO_EVIDENCE;
    out->permission = blocked ? ADAS_DRIVE_BLOCK :
        out->reasons ? ADAS_DRIVE_INSUFFICIENT : ADAS_DRIVE_ALLOW;
    if (out->permission == ADAS_DRIVE_ALLOW)
        out->action = c->command == ADAS_DRIVE_CHANGE_LEFT ? ADAS_ACTION_CHANGE_LEFT :
            c->command == ADAS_DRIVE_CHANGE_RIGHT ? ADAS_ACTION_CHANGE_RIGHT : ADAS_ACTION_ADVANCE;
    else if (blocked)
        out->action = out->reasons == ADAS_DRIVE_SOLID ? ADAS_ACTION_KEEP_LANE : ADAS_ACTION_STOP;
    out->alarm_active = out->reasons & ADAS_DRIVE_ALARM_REASONS;
    if (out->permission != ADAS_DRIVE_ALLOW) out->valid_until_ns = 0;
}

void adas_decision_evaluate(const struct adas_drive_control *c,
                            const struct adas_decision_config *cfg,
                            const struct adas_drive_frame *f, const struct adas_result *r,
                            uint64_t now, struct adas_drive_output *out)
{
    evaluate_at(c, cfg, f, r, now, now, out);
}

static void commit(struct adas_decision *s, struct adas_drive_output *next,
                    const struct adas_drive_control *c, const struct adas_drive_frame *f)
{
    const struct adas_drive_output *old = &s->output;
    next->alarm_unresolved = old->alarm_active | old->alarm_unresolved;
    const uint32_t bits[] = {ADAS_DRIVE_RED, ADAS_DRIVE_SOLID, ADAS_DRIVE_PEDESTRIAN, ADAS_DRIVE_VEHICLE};
    for (uint32_t i = 0; i < 4; i++) {
        struct adas_drive_alarm_scope *scope = &s->alarm_scope[i];
        if (next->alarm_active & bits[i]) {
            *scope = (struct adas_drive_alarm_scope){c->scene,c->ego_lane,c->command,*f,next->application_session};
            next->alarm_unresolved &= ~bits[i];
        } else if ((next->alarm_checked & bits[i]) &&
            scope->scene == c->scene && scope->ego_lane == c->ego_lane &&
            (i == 0 || scope->command == c->command) &&
            scope->application_session == next->application_session &&
            scope->frame.source_id == f->source_id && scope->frame.layout == f->layout &&
            scope->frame.stream_epoch == f->stream_epoch &&
            scope->frame.config_generation == f->config_generation) {
            next->alarm_unresolved &= ~bits[i];
        }
    }
    next->alarm_history = old->alarm_history | next->alarm_active | next->alarm_unresolved;
    next->alarm_acknowledged = old->alarm_acknowledged &
        (next->alarm_active | next->alarm_unresolved);
    if (next->alarm_active != old->alarm_active ||
        next->alarm_unresolved != old->alarm_unresolved) s->alarm_revision++;
    s->output = *next;
}

int adas_decision_step(struct adas_decision *s, const struct adas_decision_config *cfg,
                       const struct adas_drive_frame *f, const struct adas_result *r, uint64_t now)
{
    if (!now || now < s->last_step_ns) {
        adas_decision_reset_input(s, s->last_step_ns);
        s->queue_lost = 1;
        s->output.reasons |= ADAS_DRIVE_TIME_ERROR;
        return -1;
    }
    s->last_step_ns = now;
    if (s->queue_lost) {
        s->output.reasons |= ADAS_DRIVE_QUEUE_LOST;
        return -1;
    }
    while (s->count) {
        s->applied = s->queue[s->head];
        s->head = (s->head + 1U) % ADAS_DRIVE_QUEUE_CAPACITY;
        s->count--;
        struct adas_drive_output next;
        evaluate_at(&s->applied, cfg, f, r, now, s->applied.observed_ns, &next);
        commit(s, &next, &s->applied, f);
    }
    struct adas_drive_output next;
    adas_decision_evaluate(&s->applied, cfg, f, r, now, &next);
    commit(s, &next, &s->applied, f);
    return 0;
}
