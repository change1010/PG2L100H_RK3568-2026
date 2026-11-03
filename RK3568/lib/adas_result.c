#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "adas_result.h"

static int zero_bytes(const void *data, size_t size)
{
    const unsigned char *p = data;
    for (size_t i = 0; i < size; i++)
        if (p[i]) return 0;
    return 1;
}

static int context_valid(const struct adas_context *c)
{
    return c->source_id <= 4 && c->source_id == c->layout && c->input_profile_generation;
}

static int identity_valid(const struct adas_identity *id)
{
    return id->source_id < 4 && id->source_id == id->layout &&
        id->canvas_width == ADAS_WIDTH && id->canvas_height == ADAS_HEIGHT;
}

static int identity_matches(const struct adas_identity *id, const struct adas_context *c)
{
    return id->stream_epoch == c->stream_epoch && id->source_id == c->source_id &&
        id->layout == c->layout && id->config_generation == c->config_generation;
}

static int same_identity(const struct adas_identity *a, const struct adas_identity *b)
{
    return a->stream_epoch == b->stream_epoch && a->frame_sequence == b->frame_sequence &&
        a->source_id == b->source_id && a->layout == b->layout &&
        a->config_generation == b->config_generation &&
        a->canvas_width == b->canvas_width && a->canvas_height == b->canvas_height;
}

static int state_reason(uint32_t state, uint32_t reason)
{
    switch (state) {
    case ADAS_VALID: case ADAS_ABSENT: return reason == ADAS_REASON_NONE;
    case ADAS_DISABLED:
        return reason == ADAS_NOT_ENABLED || reason == ADAS_UNSUPPORTED_TASK ||
            reason == ADAS_UNSUPPORTED_LAYOUT || reason == ADAS_STOPPED;
    case ADAS_UNKNOWN:
        return (reason >= ADAS_WAIT_INPUT && reason <= ADAS_INPUT_MISSING) ||
            (reason >= ADAS_SOURCE_CHANGED && reason <= ADAS_INPUT_LOST);
    case ADAS_FAILED: return reason == ADAS_PROCESSOR_ERROR || reason == ADAS_PROTOCOL_ERROR;
    case ADAS_EXPIRED: return reason == ADAS_REASON_EXPIRED;
    case ADAS_CANCELLED: return reason == ADAS_REASON_CANCELLED;
    default: return 0;
    }
}

static int score_valid(uint32_t present, uint32_t value, uint32_t maximum)
{
    return present <= 1 && value <= maximum && (present || !value);
}

static int box_valid(const struct adas_box *b)
{
    return b->x0 >= 0 && b->y0 >= 0 && b->x0 < b->x1 && b->y0 < b->y1 &&
        b->x1 <= (int32_t)ADAS_WIDTH && b->y1 <= (int32_t)ADAS_HEIGHT;
}

static int collection_valid(const struct adas_collection *m, uint32_t capacity)
{
    if (!state_reason(m->state, m->reason) || m->count > capacity ||
        m->total_count_known > 1 || (m->flags & ~ADAS_TRUNCATED)) return 0;
    if (m->state != ADAS_VALID && m->state != ADAS_ABSENT)
        return !m->count && !m->observed_count && !m->total_count_known && !m->flags;
    if (m->state == ADAS_ABSENT)
        return !m->count && !m->observed_count && m->total_count_known && !m->flags;
    if (!m->count) return 0;
    if (!m->total_count_known) return !m->observed_count && m->flags == ADAS_TRUNCATED;
    if (m->observed_count < m->count) return 0;
    return (m->observed_count > m->count) == !!(m->flags & ADAS_TRUNCATED);
}

static int subtask_valid(uint32_t state, uint32_t reason,
                         const struct adas_branch_header *h, int supported)
{
    if (!state_reason(state, reason)) return 0;
    if (!supported) return state == ADAS_DISABLED && reason == ADAS_UNSUPPORTED_TASK;
    if (h->state != ADAS_VALID) return state == h->state && reason == h->reason;
    return state == ADAS_VALID || state == ADAS_ABSENT || state == ADAS_UNKNOWN;
}

static int branch_header_valid(const struct adas_branch_header *h, uint64_t now)
{
    if (!now || h->state == ADAS_ABSENT || !state_reason(h->state, h->reason) ||
        h->origin > ADAS_ROAD_FPGA || (h->flags & ~ADAS_TRUNCATED) ||
        h->identity_present > 1 || !h->application_session || !h->ttl_ns ||
        !h->state_changed_ns || h->state_changed_ns > now ||
        (h->state == ADAS_FAILED ? !h->error_errno : !!h->error_errno)) return 0;
    if (h->state != ADAS_VALID && h->flags) return 0;
    if (!h->identity_present)
        return h->state != ADAS_VALID && zero_bytes(&h->identity, sizeof(h->identity)) &&
            !h->observed_ns && !h->received_ns && !h->finished_ns;
    return identity_valid(&h->identity) && h->origin != ADAS_ORIGIN_NONE &&
        h->producer_session && h->input_profile_generation && h->algorithm_id &&
        h->algorithm_version && h->algorithm_config_generation && h->observed_ns &&
        h->observed_ns <= h->received_ns && h->received_ns <= h->finished_ns &&
        h->finished_ns <= h->state_changed_ns;
}

static int detection_capabilities(uint32_t classes, uint32_t caps, uint32_t directions)
{
    return !(classes & ~ADAS_CLASSES) && !(caps & ~7U) && !(directions & ~ADAS_DIRECTIONS) &&
        (!(caps & (ADAS_SIGNAL_DIRECTION | ADAS_SIGNAL_COLOR)) || (caps & ADAS_SIGNAL_LOCATION)) &&
        ((caps & ADAS_SIGNAL_DIRECTION) ? !!directions : !directions);
}

static int object_valid(const struct adas_object *o, const struct adas_detection *d)
{
    if (!o->object_id || o->semantic_class > ADAS_OTHER ||
        o->class_map_version != d->class_map_version || !box_valid(&o->box) ||
        !score_valid(o->confidence_present, o->confidence_q10000, 10000) ||
        o->origin < ADAS_YOLO || o->origin > ADAS_MOCK || o->flags) return 0;
    if (o->origin != d->header.origin) return 0;
    return o->semantic_class == ADAS_CLASS_UNKNOWN ||
        (d->semantic_supported_mask & (1U << (o->semantic_class - 1)));
}

static int signal_valid(const struct adas_signal *s, const struct adas_detection *d)
{
    if (!s->signal_id || !box_valid(&s->box) || s->shape > ADAS_ROUND ||
        s->color > ADAS_OFF || s->direction_valid > 1 || s->color_valid > 1 ||
        (s->direction_mask & ~ADAS_DIRECTIONS) ||
        s->direction_valid != !!s->direction_mask ||
        s->color_valid != (s->color != ADAS_COLOR_UNKNOWN) ||
        s->association > ADAS_OTHER_PATH || s->association_reason > ADAS_ASSOC_MOCK ||
        s->origin != d->header.origin || s->flags ||
        !score_valid(s->localization_confidence_present, s->localization_confidence_q10000, 10000) ||
        !score_valid(s->direction_confidence_present, s->direction_confidence_q10000, 10000) ||
        !score_valid(s->color_confidence_present, s->color_confidence_q10000, 10000) ||
        !score_valid(s->color_evidence_present, s->color_evidence_permille, 1000)) return 0;
    if (s->direction_valid && (!(d->signal_capability_mask & ADAS_SIGNAL_DIRECTION) ||
        (s->direction_mask & ~d->signals_supported_direction_mask))) return 0;
    if (s->color_valid && !(d->signal_capability_mask & ADAS_SIGNAL_COLOR)) return 0;
    if ((!s->direction_valid && s->direction_confidence_present) ||
        (!s->color_valid && (s->color_confidence_present || s->color_evidence_present))) return 0;
    if (s->association == ADAS_UNRESOLVED) {
        if (s->association_reason || s->association_rule_version || s->association_control_generation)
            return 0;
    } else if (!s->direction_valid || !s->association_reason || !s->association_rule_version ||
               !s->association_control_generation) return 0;
    if (s->shape == ADAS_ROUND && s->direction_valid &&
        s->association_reason != ADAS_ASSOC_SCENE_RULE && s->association_reason != ADAS_ASSOC_MOCK)
        return 0;
    if (s->association_reason == ADAS_ASSOC_MOCK && s->origin != ADAS_MOCK) return 0;
    return 1;
}

int adas_detection_valid(const struct adas_detection *d, uint64_t now)
{
    if (!d || !branch_header_valid(&d->header, now) ||
        !detection_capabilities(d->semantic_supported_mask, d->signal_capability_mask,
                               d->signals_supported_direction_mask) ||
        (d->semantic_supported_mask && !d->class_map_version) ||
        !collection_valid(&d->objects_meta, ADAS_MAX_OBJECTS) ||
        !collection_valid(&d->signals_meta, ADAS_MAX_SIGNALS) ||
        !subtask_valid(d->objects_meta.state, d->objects_meta.reason, &d->header,
                       d->semantic_supported_mask != 0) ||
        !subtask_valid(d->signals_meta.state, d->signals_meta.reason, &d->header,
                       (d->signal_capability_mask & ADAS_SIGNAL_LOCATION) != 0) ||
        d->header.flags != (d->objects_meta.flags | d->signals_meta.flags) ||
        d->light_visual_valid > 1 || d->light_visual_count > ADAS_MAX_SIGNALS ||
        (!d->light_visual_valid && d->light_visual_count) ||
        (d->light_visual_valid && d->header.state != ADAS_VALID)) return 0;
    for (uint32_t i = 0; i < d->light_visual_count; ++i) {
        const struct adas_light_visual *v = &d->light_visual[i];
        if (!box_valid(&v->box) || !v->track_id || v->color > ADAS_GREEN ||
            v->instant_color > ADAS_GREEN || v->reason > 3 || v->evidence_permille > 1000 ||
            v->confidence_q10000 > 10000 || (v->flags & ~7U) ||
            v->detector_ns > d->header.observed_ns || v->color_ns > d->header.observed_ns ||
            d->header.observed_ns-v->detector_ns >= UINT64_C(650000000) ||
            (v->color && (!v->color_ns ||
             d->header.observed_ns-v->color_ns > UINT64_C(120000000)))) return 0;
        for (uint32_t j = 0; j < i; ++j)
            if (v->track_id == d->light_visual[j].track_id) return 0;
    }
    if (!zero_bytes(d->light_visual+d->light_visual_count,
        (ADAS_MAX_SIGNALS-d->light_visual_count)*sizeof(d->light_visual[0]))) return 0;
    for (uint32_t i = 0; i < d->objects_meta.count; i++) {
        if (!object_valid(&d->objects[i], d)) return 0;
        for (uint32_t j = 0; j < i; j++)
            if (d->objects[j].object_id == d->objects[i].object_id) return 0;
    }
    for (uint32_t i = 0; i < d->signals_meta.count; i++) {
        if (!signal_valid(&d->signals[i], d)) return 0;
        for (uint32_t j = 0; j < i; j++)
            if (d->signals[j].signal_id == d->signals[i].signal_id) return 0;
    }
    return zero_bytes(d->objects + d->objects_meta.count,
                      (ADAS_MAX_OBJECTS - d->objects_meta.count) * sizeof(d->objects[0])) &&
        zero_bytes(d->signals + d->signals_meta.count,
                   (ADAS_MAX_SIGNALS - d->signals_meta.count) * sizeof(d->signals[0]));
}

static int lane_valid(const struct adas_lane *l, const struct adas_branch_header *h, int supported)
{
    if (!subtask_valid(l->state, l->reason, h, supported) || l->line_type > ADAS_DASHED ||
        l->point_count > ADAS_MAX_LANE_POINTS || (l->flags & ~ADAS_TRUNCATED) ||
        !score_valid(l->evidence_present, l->evidence_permille, 1000)) return 0;
    if (l->state != ADAS_VALID)
        return !l->point_count && !l->line_type && !l->evidence_present &&
            !l->flags && zero_bytes(l->points, sizeof(l->points)) &&
            zero_bytes(l->measured_connection, sizeof(l->measured_connection));
    if (l->point_count < 2) return 0;
    for (uint32_t i = 0; i < l->point_count; i++) {
        const struct adas_point *p = &l->points[i];
        if (p->x < 0 || p->x >= (int32_t)ADAS_WIDTH || p->y < 0 || p->y >= (int32_t)ADAS_HEIGHT ||
            (i && p->y <= l->points[i - 1].y)) return 0;
    }
    for (uint32_t i = 0; i < ADAS_MAX_LANE_POINTS; i++) {
        if (!(l->measured_connection[i / 32U] & (UINT32_C(1) << (i % 32U)))) continue;
        if (!i || i >= l->point_count) return 0;
        int32_t dy = l->points[i].y - l->points[i - 1].y;
        int32_t dx = l->points[i].x - l->points[i - 1].x;
        if (dy <= 0 || dy > 18 || dx < -3 * dy - 3 || dx > 3 * dy + 3) return 0;
    }
    return zero_bytes(l->points + l->point_count,
                      (ADAS_MAX_LANE_POINTS - l->point_count) * sizeof(l->points[0]));
}

static int reference_valid(const struct road_ref_geometry *g, uint32_t state)
{
    if (!g->present) return zero_bytes(g, sizeof(*g));
    if (g->present != 1 || state != ADAS_VALID || g->truncated > 1 ||
        g->raw_count > ROAD_REF_MAX_SEGMENTS || g->confirmed_count > ROAD_REF_MAX_SEGMENTS ||
        g->point_count > ROAD_REF_MAX_POINTS || g->raw_count > g->raw_observed ||
        g->confirmed_count > g->confirmed_observed ||
        g->truncated != (uint32_t)(g->raw_count != g->raw_observed ||
                                  g->confirmed_count != g->confirmed_observed) ||
        g->cross_detected > 1 || g->cross_stable > 1) return 0;
    if (g->cross_detected && (g->cross_y_begin < 0 ||
        g->cross_y_end <= g->cross_y_begin || g->cross_y_end > (int32_t)ADAS_HEIGHT ||
        g->cross_stripes < 2)) return 0;
    uint32_t used = 0;
    for (unsigned int group = 0; group < 2; ++group) {
        const struct road_ref_polygon *p = group ? g->confirmed : g->raw;
        uint32_t count = group ? g->confirmed_count : g->raw_count;
        for (uint32_t i = 0; i < count; ++i) {
            if ((p[i].line_type != ADAS_SOLID && p[i].line_type != ADAS_DASHED) ||
                p[i].point_count < 3 || p[i].first_point != used ||
                p[i].point_count > g->point_count - used ||
                !(p[i].area_px >= 0 && p[i].area_px <= 1920. * 1080.) ||
                !(p[i].visible_span_px >= 0 && p[i].visible_span_px <= 1080.)) return 0;
            used += p[i].point_count;
        }
        if (!zero_bytes(p + count, (ROAD_REF_MAX_SEGMENTS - count) * sizeof(*p))) return 0;
    }
    if (used != g->point_count || !(g->cross_score >= 0 && g->cross_score <= 1e12) ||
        !(g->cross_pitch >= 0 && g->cross_pitch <= 1e12)) return 0;
    for (uint32_t i = 0; i < used; ++i)
        if (g->points[i].x < 0 || g->points[i].x >= (int32_t)ADAS_WIDTH ||
            g->points[i].y < 0 || g->points[i].y >= (int32_t)ADAS_HEIGHT) return 0;
    return zero_bytes(g->points + used, (ROAD_REF_MAX_POINTS - used) * sizeof(g->points[0]));
}

int adas_road_valid(const struct adas_road *r, uint64_t now)
{
    if (!r || !branch_header_valid(&r->header, now) || (r->supported_tasks & ~7U) ||
        !reference_valid(&r->reference, r->header.state) ||
        !lane_valid(&r->left, &r->header, r->supported_tasks & ADAS_ROAD_LEFT) ||
        !lane_valid(&r->right, &r->header, r->supported_tasks & ADAS_ROAD_RIGHT) ||
        !collection_valid(&r->zebras_meta, ADAS_MAX_ZEBRAS) ||
        !subtask_valid(r->zebras_meta.state, r->zebras_meta.reason, &r->header,
                       r->supported_tasks & ADAS_ROAD_ZEBRA) ||
        r->header.flags != (r->left.flags | r->right.flags | r->zebras_meta.flags)) return 0;
    for (uint32_t i = 0; i < r->zebras_meta.count; i++) {
        const struct adas_zebra *z = &r->zebras[i];
        if (!z->zebra_id || !box_valid(&z->box) || z->stripe_count < 2 || z->flags ||
            z->origin != r->header.origin ||
            !score_valid(z->evidence_present, z->evidence_permille, 1000)) return 0;
        for (uint32_t j = 0; j < i; j++)
            if (r->zebras[j].zebra_id == z->zebra_id) return 0;
    }
    return zero_bytes(r->zebras + r->zebras_meta.count,
                      (ADAS_MAX_ZEBRAS - r->zebras_meta.count) * sizeof(r->zebras[0]));
}

static int result_branch_matches(const struct adas_result *r, const struct adas_branch_header *h)
{
    if (h->application_session != r->application_session) return 0;
    if (!h->identity_present) return 1;
    return r->context_present && identity_matches(&h->identity, &r->context) &&
        h->input_profile_generation == r->context.input_profile_generation;
}

int adas_result_valid(const struct adas_result *r, uint64_t now)
{
    if (!r || !now || r->version != ADAS_VERSION || r->size != sizeof(*r) ||
        !r->publication_sequence || !r->application_session || !r->policy_generation ||
        r->context_present > 1 || r->canvas_width != ADAS_WIDTH || r->canvas_height != ADAS_HEIGHT ||
        !r->changed_ns || r->changed_ns > r->read_ns || r->read_ns > now ||
        (r->context_present ? !context_valid(&r->context) : !zero_bytes(&r->context, sizeof(r->context))) ||
        !adas_detection_valid(&r->detection, now) || !adas_road_valid(&r->road, now) ||
        !result_branch_matches(r, &r->detection.header) ||
        !result_branch_matches(r, &r->road.header) ||
        r->detection.header.state_changed_ns > r->changed_ns ||
        r->road.header.state_changed_ns > r->changed_ns) return 0;
    if ((!r->context_present || r->context.source_id == 4) &&
        (r->detection.header.state == ADAS_VALID || r->road.header.state == ADAS_VALID)) return 0;
    return 1;
}

int adas_branch_ready(const struct adas_result *r, uint32_t branch, uint64_t now, uint32_t simulation)
{
    if (branch > ADAS_ROAD || simulation > 1 || !adas_result_valid(r, now)) return 0;
    const struct adas_branch_header *h = branch == ADAS_DETECTION ? &r->detection.header : &r->road.header;
    return h->state == ADAS_VALID && h->identity_present && now >= h->observed_ns &&
        now - h->observed_ns < h->ttl_ns && (simulation || h->origin != ADAS_MOCK);
}

int adas_can_fuse(const struct adas_result *r, uint64_t now, uint64_t window, uint32_t simulation)
{
    if (!window || !adas_branch_ready(r, ADAS_DETECTION, now, simulation) ||
        !adas_branch_ready(r, ADAS_ROAD, now, simulation)) return 0;
    uint64_t a = r->detection.header.observed_ns, b = r->road.header.observed_ns;
    return (a >= b ? a - b : b - a) <= window;
}

int adas_same_frame(const struct adas_result *r)
{
    if (!r || !r->detection.header.identity_present || !r->road.header.identity_present ||
        !adas_result_valid(r, r->read_ns)) return 0;
    return same_identity(&r->detection.header.identity, &r->road.header.identity);
}

int adas_signal_actionable(const struct adas_signal *s, uint32_t direction, uint64_t generation)
{
    /* Attribute gate only. Caller checks whole-branch freshness, conflicts,
     * truncation, capabilities and vehicle risks before authorizing behavior.
     * This function does not itself mean permission to drive. */
    return s && (direction == ADAS_LEFT || direction == ADAS_STRAIGHT || direction == ADAS_RIGHT) &&
        generation && s->direction_valid == 1 && (s->direction_mask & direction) &&
        s->color_valid == 1 && s->color >= ADAS_RED && s->color <= ADAS_OFF &&
        s->association == ADAS_EGO_PATH && s->association_control_generation == generation;
}

static int producer_valid(const struct adas_producer *p, uint32_t branch)
{
    if (p->enabled > 1) return 0;
    if (!p->enabled) return zero_bytes(p, sizeof(*p));
    if (!p->producer_session || !p->algorithm_id || !p->algorithm_version ||
        !p->algorithm_config_generation || p->origin < ADAS_YOLO || p->origin > ADAS_ROAD_FPGA)
        return 0;
    if (branch == ADAS_ROAD)
        return (p->origin == ADAS_ROAD_CV || p->origin == ADAS_ROAD_FPGA ||
                p->origin == ADAS_FUSED || p->origin == ADAS_MOCK) &&
            !p->class_map_version && !p->semantic_supported_mask && !p->signal_capability_mask &&
            !p->signals_supported_direction_mask && p->road_supported_tasks &&
            !(p->road_supported_tasks & ~7U);
    return p->origin != ADAS_ROAD_CV && p->origin != ADAS_ROAD_FPGA && !p->road_supported_tasks &&
        detection_capabilities(p->semantic_supported_mask, p->signal_capability_mask,
                               p->signals_supported_direction_mask) &&
        (p->semantic_supported_mask || p->signal_capability_mask) &&
        (p->semantic_supported_mask ? !!p->class_map_version : !p->class_map_version);
}

static int setup_valid(const struct adas_setup *s)
{
    return s && s->policy_generation && s->context_present <= 1 && s->simulation_mode <= 1 &&
        s->detection_ttl_ns && s->road_ttl_ns && s->fusion_window_ns &&
        (s->context_present ? context_valid(&s->context) : zero_bytes(&s->context, sizeof(s->context))) &&
        producer_valid(&s->detection, ADAS_DETECTION) && producer_valid(&s->road, ADAS_ROAD);
}

static int lock_store(struct adas_store *s)
{
    if (!s || !s->initialized) return ADAS_INVALID;
    if (atomic_exchange_explicit(&s->busy, true, memory_order_acquire)) return ADAS_BUSY;
    if (s->terminal) {
        atomic_store_explicit(&s->busy, false, memory_order_release);
        return ADAS_INVALID;
    }
    return ADAS_OK_NEW;
}

static void unlock_store(struct adas_store *s)
{
    atomic_store_explicit(&s->busy, false, memory_order_release);
}

static int time_valid(const struct adas_store *s, uint64_t now)
{
    return now && now >= s->last_now_ns;
}

static int changed(struct adas_store *s, uint64_t now)
{
    if (s->sequence == UINT64_MAX) {
        s->terminal = 1;
        s->available = 0;
        memset(&s->latest, 0, sizeof(s->latest));
        return ADAS_INVALID;
    }
    s->latest.publication_sequence = ++s->sequence;
    s->latest.changed_ns = now;
    s->latest.read_ns = now;
    s->available = 1;
    return ADAS_OK_NEW;
}

static const struct adas_producer *producer(const struct adas_store *s, uint32_t branch)
{
    return branch == ADAS_DETECTION ? &s->setup.detection : &s->setup.road;
}

static void collection_state(struct adas_collection *m, uint32_t state, uint32_t reason, int supported)
{
    memset(m, 0, sizeof(*m));
    m->state = supported ? state : ADAS_DISABLED;
    m->reason = supported ? reason : ADAS_UNSUPPORTED_TASK;
}

static void lane_state(struct adas_lane *l, uint32_t state, uint32_t reason, int supported)
{
    memset(l, 0, sizeof(*l));
    l->state = supported ? state : ADAS_DISABLED;
    l->reason = supported ? reason : ADAS_UNSUPPORTED_TASK;
}

static void detection_clear(struct adas_detection *d, uint32_t state, uint32_t reason, uint64_t now)
{
    d->header.state = state;
    d->header.reason = reason;
    d->header.flags = 0;
    d->header.error_errno = 0;
    d->header.state_changed_ns = now;
    collection_state(&d->objects_meta, state, reason, d->semantic_supported_mask != 0);
    collection_state(&d->signals_meta, state, reason, d->signal_capability_mask & ADAS_SIGNAL_LOCATION);
    memset(d->objects, 0, sizeof(d->objects));
    memset(d->signals, 0, sizeof(d->signals));
    d->light_visual_valid = d->light_visual_count = 0;
    memset(d->light_visual,0,sizeof(d->light_visual));
}

static void road_clear(struct adas_road *r, uint32_t state, uint32_t reason, uint64_t now)
{
    r->header.state = state;
    r->header.reason = reason;
    r->header.flags = 0;
    r->header.error_errno = 0;
    r->header.state_changed_ns = now;
    lane_state(&r->left, state, reason, r->supported_tasks & ADAS_ROAD_LEFT);
    lane_state(&r->right, state, reason, r->supported_tasks & ADAS_ROAD_RIGHT);
    collection_state(&r->zebras_meta, state, reason, r->supported_tasks & ADAS_ROAD_ZEBRA);
    memset(r->zebras, 0, sizeof(r->zebras));
    memset(&r->reference, 0, sizeof(r->reference));
}

static void registered_header(const struct adas_store *s, uint32_t branch,
                               struct adas_branch_header *h)
{
    const struct adas_producer *p = producer(s, branch);
    memset(h, 0, sizeof(*h));
    h->origin = p->origin;
    h->application_session = s->session;
    h->producer_session = p->producer_session;
    h->algorithm_id = p->algorithm_id;
    h->algorithm_version = p->algorithm_version;
    h->algorithm_config_generation = p->algorithm_config_generation;
    h->input_profile_generation = s->setup.context_present ? s->setup.context.input_profile_generation : 0;
    h->ttl_ns = branch == ADAS_DETECTION ? s->setup.detection_ttl_ns : s->setup.road_ttl_ns;
}

static void detection_metadata(const struct adas_store *s, struct adas_detection *d)
{
    d->class_map_version = s->setup.detection.class_map_version;
    d->semantic_supported_mask = s->setup.detection.semantic_supported_mask;
    d->signal_capability_mask = s->setup.detection.signal_capability_mask;
    d->signals_supported_direction_mask = s->setup.detection.signals_supported_direction_mask;
}

static void control_result(struct adas_store *s, uint64_t now, uint32_t reason)
{
    memset(&s->latest, 0, sizeof(s->latest));
    s->latest.version = ADAS_VERSION;
    s->latest.size = sizeof(s->latest);
    s->latest.application_session = s->session;
    s->latest.policy_generation = s->setup.policy_generation;
    s->latest.context_present = s->setup.context_present;
    s->latest.context = s->setup.context;
    s->latest.canvas_width = ADAS_WIDTH;
    s->latest.canvas_height = ADAS_HEIGHT;
    registered_header(s, ADAS_DETECTION, &s->latest.detection.header);
    registered_header(s, ADAS_ROAD, &s->latest.road.header);
    detection_metadata(s, &s->latest.detection);
    s->latest.road.supported_tasks = s->setup.road.road_supported_tasks;
    for (uint32_t b = 0; b < 2; b++) {
        uint32_t state = ADAS_UNKNOWN, why = reason;
        if (s->setup.context_present && s->setup.context.source_id == 4) {
            state = ADAS_DISABLED;
            why = ADAS_UNSUPPORTED_LAYOUT;
        } else if (!producer(s, b)->enabled) {
            state = ADAS_DISABLED;
            why = reason == ADAS_STOPPED ? ADAS_STOPPED : ADAS_NOT_ENABLED;
        }
        if (b == ADAS_DETECTION) detection_clear(&s->latest.detection, state, why, now);
        else road_clear(&s->latest.road, state, why, now);
    }
}

int adas_store_init(struct adas_store *s)
{
    if (!s) return ADAS_INVALID;
    memset(s, 0, sizeof(*s));
    atomic_init(&s->busy, false);
    if (!atomic_is_lock_free(&s->busy)) return ADAS_INVALID;
    s->initialized = 1;
    s->session = 1;
    return ADAS_OK_NEW;
}

static int begin_locked(struct adas_store *s, const struct adas_setup *setup,
                         uint64_t now, uint32_t reason)
{
    if (s->session == UINT64_MAX || s->sequence == UINT64_MAX) {
        s->terminal = 1;
        s->available = 0;
        memset(&s->latest, 0, sizeof(s->latest));
        return ADAS_INVALID;
    }
    s->setup = *setup;
    s->session++;
    s->session_started_ns = now;
    s->last_now_ns = now;
    memset(s->watermarks, 0, sizeof(s->watermarks));
    control_result(s, now, reason);
    return changed(s, now);
}

int adas_begin_session(struct adas_store *s, const struct adas_setup *setup,
                       uint64_t now, uint32_t reason)
{
    if (!setup_valid(setup) || !state_reason(ADAS_UNKNOWN, reason)) return ADAS_INVALID;
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!time_valid(s, now)) rc = ADAS_INVALID;
    else if (s->setup.policy_generation &&
        (uint32_t)(setup->policy_generation - s->setup.policy_generation) >= UINT32_C(0x80000000))
        rc = ADAS_INVALID;
    else if (s->setup.policy_generation == setup->policy_generation &&
        (s->setup.detection_ttl_ns != setup->detection_ttl_ns ||
         s->setup.road_ttl_ns != setup->road_ttl_ns ||
         s->setup.fusion_window_ns != setup->fusion_window_ns ||
         s->setup.simulation_mode != setup->simulation_mode ||
         s->setup.detection.enabled != setup->detection.enabled ||
         s->setup.road.enabled != setup->road.enabled)) rc = ADAS_INVALID;
    else rc = begin_locked(s, setup, now, reason);
    unlock_store(s);
    return rc;
}

int adas_invalidate(struct adas_store *s, uint64_t now, uint32_t reason)
{
    if (!state_reason(ADAS_UNKNOWN, reason)) return ADAS_INVALID;
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!s->setup.policy_generation || !time_valid(s, now)) rc = ADAS_INVALID;
    else {
        struct adas_setup setup = s->setup;
        setup.context_present = 0;
        memset(&setup.context, 0, sizeof(setup.context));
        rc = begin_locked(s, &setup, now, reason);
    }
    unlock_store(s);
    return rc;
}

int adas_stop(struct adas_store *s, uint64_t now)
{
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!s->setup.policy_generation || !time_valid(s, now)) rc = ADAS_INVALID;
    else {
        struct adas_setup setup = s->setup;
        setup.context_present = 0;
        memset(&setup.context, 0, sizeof(setup.context));
        memset(&setup.detection, 0, sizeof(setup.detection));
        memset(&setup.road, 0, sizeof(setup.road));
        rc = begin_locked(s, &setup, now, ADAS_STOPPED);
    }
    unlock_store(s);
    return rc;
}

static int token_matches(const struct adas_store *s, const struct adas_token *t, uint32_t branch)
{
    if (!t || t->version != ADAS_VERSION || t->branch != branch ||
        t->policy_generation != s->setup.policy_generation || !producer(s, branch)->enabled ||
        !s->setup.context_present || s->setup.context.source_id == 4) return 0;
    struct adas_branch_header expected;
    registered_header(s, branch, &expected);
    const struct adas_branch_header *h = &t->header;
    return h->state == ADAS_UNKNOWN && h->reason == ADAS_WAIT_INPUT && !h->flags &&
        h->identity_present == 1 && identity_valid(&h->identity) &&
        identity_matches(&h->identity, &s->setup.context) &&
        h->application_session == expected.application_session &&
        h->producer_session == expected.producer_session && h->origin == expected.origin &&
        h->input_profile_generation == expected.input_profile_generation &&
        h->algorithm_id == expected.algorithm_id && h->algorithm_version == expected.algorithm_version &&
        h->algorithm_config_generation == expected.algorithm_config_generation && h->ttl_ns == expected.ttl_ns &&
        !h->error_errno && !h->received_ns && !h->finished_ns &&
        h->observed_ns >= s->session_started_ns && h->observed_ns <= h->state_changed_ns &&
        h->state_changed_ns <= s->last_now_ns;
}

int adas_get_token(struct adas_store *s, uint32_t branch, const struct adas_identity *id,
                   uint64_t observed, uint64_t now, struct adas_token *out)
{
    if (out) memset(out, 0, sizeof(*out));
    if (!out || branch > ADAS_ROAD || !id || !identity_valid(id)) return ADAS_INVALID;
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!time_valid(s, now) || !observed || observed > now) rc = ADAS_INVALID;
    else if (!producer(s, branch)->enabled || !s->setup.context_present ||
        s->setup.context.source_id == 4 || !identity_matches(id, &s->setup.context) ||
        observed < s->session_started_ns) rc = ADAS_REJECTED_STALE;
    else {
        s->last_now_ns = now;
        out->version = ADAS_VERSION;
        out->branch = branch;
        out->policy_generation = s->setup.policy_generation;
        registered_header(s, branch, &out->header);
        out->header.state = ADAS_UNKNOWN;
        out->header.reason = ADAS_WAIT_INPUT;
        out->header.identity_present = 1;
        out->header.identity = *id;
        out->header.observed_ns = observed;
        out->header.state_changed_ns = now;
    }
    unlock_store(s);
    return rc;
}

int adas_prepare_detection(struct adas_store *s, const struct adas_token *t, struct adas_detection *out)
{
    if (out) memset(out, 0, sizeof(*out));
    if (!out) return ADAS_INVALID;
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!token_matches(s, t, ADAS_DETECTION)) rc = ADAS_REJECTED_STALE;
    else {
        out->header = t->header;
        detection_metadata(s, out);
        detection_clear(out, ADAS_UNKNOWN, ADAS_WAIT_INPUT, t->header.state_changed_ns);
    }
    unlock_store(s);
    return rc;
}

int adas_prepare_road(struct adas_store *s, const struct adas_token *t, struct adas_road *out)
{
    if (out) memset(out, 0, sizeof(*out));
    if (!out) return ADAS_INVALID;
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!token_matches(s, t, ADAS_ROAD)) rc = ADAS_REJECTED_STALE;
    else {
        out->header = t->header;
        out->supported_tasks = s->setup.road.road_supported_tasks;
        road_clear(out, ADAS_UNKNOWN, ADAS_WAIT_INPUT, t->header.state_changed_ns);
    }
    unlock_store(s);
    return rc;
}

static int candidate_header_matches(const struct adas_branch_header *h, const struct adas_token *t)
{
    const struct adas_branch_header *e = &t->header;
    return h->identity_present == 1 && same_identity(&h->identity, &e->identity) &&
        h->input_profile_generation == e->input_profile_generation &&
        h->application_session == e->application_session && h->producer_session == e->producer_session &&
        h->algorithm_id == e->algorithm_id && h->algorithm_version == e->algorithm_version &&
        h->algorithm_config_generation == e->algorithm_config_generation && h->origin == e->origin &&
        h->observed_ns == e->observed_ns && h->ttl_ns == e->ttl_ns;
}

static int forward_input(const struct adas_store *s, const struct adas_token *t, uint32_t branch)
{
    const struct adas_watermark *w = &s->watermarks[branch];
    uint32_t delta = t->header.identity.frame_sequence - w->frame_sequence;
    return !w->present || (delta && delta < UINT32_C(0x80000000) && t->header.observed_ns >= w->observed_ns);
}

static void commit_watermark(struct adas_store *s, const struct adas_token *t, uint32_t branch, uint64_t now)
{
    s->watermarks[branch] = (struct adas_watermark){1, t->header.identity.frame_sequence, t->header.observed_ns};
    s->last_now_ns = now;
}

int adas_store_set_publish_notify(struct adas_store *s,
                                  void (*notify)(void *context), void *context)
{
    if (!s || !s->initialized || (!notify && context)) return ADAS_INVALID;
    s->publish_notify = notify;
    s->publish_notify_context = context;
    return ADAS_OK_NEW;
}

int adas_store_set_publish_observer(struct adas_store *s,
    void (*observer)(void *, const struct adas_result *, uint32_t, uint64_t), void *context)
{
    if (!s || !s->initialized || (!observer && context)) return ADAS_INVALID;
    s->publish_observer = observer;
    s->publish_observer_context = context;
    return ADAS_OK_NEW;
}

int adas_publish_detection(struct adas_store *s, const struct adas_token *t,
                           const struct adas_detection *candidate, uint64_t now)
{
    if (!t || !candidate) return ADAS_INVALID;
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!time_valid(s, now)) rc = ADAS_INVALID;
    else if (!token_matches(s, t, ADAS_DETECTION) || !forward_input(s, t, ADAS_DETECTION))
        rc = ADAS_REJECTED_STALE;
    else {
        struct adas_detection d = *candidate;
        d.header.state_changed_ns = now; /* store owns state-change time */
        if (!candidate_header_matches(&d.header, t) || !adas_detection_valid(&d, now) ||
            d.header.state == ADAS_DISABLED || d.header.state == ADAS_EXPIRED ||
            d.class_map_version != s->setup.detection.class_map_version ||
            d.semantic_supported_mask != s->setup.detection.semantic_supported_mask ||
            d.signal_capability_mask != s->setup.detection.signal_capability_mask ||
            d.signals_supported_direction_mask != s->setup.detection.signals_supported_direction_mask)
            rc = ADAS_INVALID;
        else {
            if ((d.header.state == ADAS_VALID || d.header.state == ADAS_UNKNOWN) &&
                now - d.header.observed_ns >= d.header.ttl_ns)
                detection_clear(&d, ADAS_EXPIRED, ADAS_REASON_EXPIRED, now);
            s->latest.detection = d;
            commit_watermark(s, t, ADAS_DETECTION, now);
            rc = changed(s, now);
        }
    }
    void (*observer)(void *, const struct adas_result *, uint32_t, uint64_t) =
        rc == ADAS_OK_NEW ? s->publish_observer : NULL;
    struct adas_result snapshot;
    if (observer) snapshot = s->latest;
    void *observer_context = s->publish_observer_context;
    void (*notify)(void *) = rc == ADAS_OK_NEW ? s->publish_notify : NULL;
    void *notify_context = s->publish_notify_context;
    unlock_store(s);
    if (observer) observer(observer_context, &snapshot, ADAS_DETECTION, now);
    if (notify) notify(notify_context);
    return rc;
}

int adas_publish_road(struct adas_store *s, const struct adas_token *t,
                      const struct adas_road *candidate, uint64_t now)
{
    if (!t || !candidate) return ADAS_INVALID;
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!time_valid(s, now)) rc = ADAS_INVALID;
    else if (!token_matches(s, t, ADAS_ROAD) || !forward_input(s, t, ADAS_ROAD)) rc = ADAS_REJECTED_STALE;
    else {
        struct adas_road r = *candidate;
        r.header.state_changed_ns = now;
        if (!candidate_header_matches(&r.header, t) || !adas_road_valid(&r, now) ||
            r.header.state == ADAS_DISABLED || r.header.state == ADAS_EXPIRED ||
            r.supported_tasks != s->setup.road.road_supported_tasks) rc = ADAS_INVALID;
        else {
            if ((r.header.state == ADAS_VALID || r.header.state == ADAS_UNKNOWN) &&
                now - r.header.observed_ns >= r.header.ttl_ns)
                road_clear(&r, ADAS_EXPIRED, ADAS_REASON_EXPIRED, now);
            s->latest.road = r;
            commit_watermark(s, t, ADAS_ROAD, now);
            rc = changed(s, now);
        }
    }
    void (*observer)(void *, const struct adas_result *, uint32_t, uint64_t) =
        rc == ADAS_OK_NEW ? s->publish_observer : NULL;
    struct adas_result snapshot;
    if (observer) snapshot = s->latest;
    void *observer_context = s->publish_observer_context;
    void (*notify)(void *) = rc == ADAS_OK_NEW ? s->publish_notify : NULL;
    void *notify_context = s->publish_notify_context;
    unlock_store(s);
    if (observer) observer(observer_context, &snapshot, ADAS_ROAD, now);
    if (notify) notify(notify_context);
    return rc;
}

static int expire_locked(struct adas_store *s, uint64_t now)
{
    int expired = 0;
    struct adas_branch_header *h = &s->latest.detection.header;
    if ((h->state == ADAS_VALID || h->state == ADAS_UNKNOWN) && h->identity_present &&
        now - h->observed_ns >= h->ttl_ns) {
        detection_clear(&s->latest.detection, ADAS_EXPIRED, ADAS_REASON_EXPIRED, now);
        expired = 1;
    }
    h = &s->latest.road.header;
    if ((h->state == ADAS_VALID || h->state == ADAS_UNKNOWN) && h->identity_present &&
        now - h->observed_ns >= h->ttl_ns) {
        road_clear(&s->latest.road, ADAS_EXPIRED, ADAS_REASON_EXPIRED, now);
        expired = 1;
    }
    s->last_now_ns = now;
    return expired ? changed(s, now) : ADAS_OK_SAME;
}

int adas_tick(struct adas_store *s, uint64_t now)
{
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!time_valid(s, now)) rc = ADAS_INVALID;
    else if (!s->available) { s->last_now_ns = now; rc = ADAS_NO_DATA; }
    else rc = expire_locked(s, now);
    unlock_store(s);
    return rc;
}

int adas_get_latest(struct adas_store *s, uint64_t now, struct adas_cursor *cursor, struct adas_result *out)
{
    if (out) memset(out, 0, sizeof(*out));
    if (!out || !cursor) return ADAS_INVALID;
    int rc = lock_store(s);
    if (rc != ADAS_OK_NEW) return rc;
    if (!time_valid(s, now)) rc = ADAS_INVALID;
    else if (!s->available) { s->last_now_ns = now; rc = ADAS_NO_DATA; }
    else if (expire_locked(s, now) == ADAS_INVALID) rc = ADAS_INVALID;
    else {
        *out = s->latest;
        out->read_ns = now;
        rc = cursor->sequence == s->sequence ? ADAS_OK_SAME : ADAS_OK_NEW;
        cursor->sequence = s->sequence;
    }
    unlock_store(s);
    return rc;
}

void adas_store_destroy(struct adas_store *s)
{
    if (!s) return;
    /* All callers/threads must already have exited. */
    memset(s, 0, sizeof(*s));
    atomic_init(&s->busy, false);
}
