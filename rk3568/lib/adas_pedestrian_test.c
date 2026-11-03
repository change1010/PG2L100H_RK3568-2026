#include "adas_pedestrian_test.h"
#include "adas_result.h"
#include <math.h>
#include <string.h>

#define PED_WAIT_NS UINT64_C(3000000000)

static int same_frame(const struct adas_drive_frame *a, const struct adas_drive_frame *b)
{
    return a->valid == 1 && b->valid == 1 && a->stream_epoch == b->stream_epoch &&
        a->source_id == b->source_id && a->layout == b->layout &&
        a->config_generation == b->config_generation && a->frame_sequence == b->frame_sequence;
}

struct adas_ped_path adas_ped_default_path(uint32_t generation)
{
    return (struct adas_ped_path){ADAS_PED_RULE_VERSION,2,2,generation,
        450,1000,430,570,180,820,12,130,3500,32,10};
}

void adas_ped_from_result(struct adas_ped_observation *o, const struct adas_result *result)
{
    adas_ped_from_detection(o,result ? &result->detection : NULL);
}

void adas_ped_from_detection(struct adas_ped_observation *o, const struct adas_detection *d)
{
    memset(o,0,sizeof(*o));
    if (!d) return;
    const struct adas_branch_header *h = &d->header;
    const struct adas_identity *id = &h->identity;
    o->frame = (struct adas_drive_frame){h->identity_present,id->stream_epoch,
        id->source_id,id->layout,id->config_generation,id->frame_sequence};
    o->observed_ns = h->observed_ns; o->finished_ns = h->finished_ns; o->ttl_ns = h->ttl_ns;
    o->algorithm_id = h->algorithm_id; o->algorithm_version = h->algorithm_version;
    o->model_generation = h->algorithm_config_generation;
    o->input_profile_generation = h->input_profile_generation;
    o->valid = h->state == ADAS_VALID && h->origin == ADAS_YOLO &&
        id->canvas_width == 1920 && id->canvas_height == 1080 &&
        (d->semantic_supported_mask & ADAS_CLASS_PEDESTRIAN) && !h->error_errno &&
        o->algorithm_id && o->algorithm_version && o->model_generation &&
        o->input_profile_generation && d->class_map_version == 1;
    const struct adas_collection *m = &d->objects_meta;
    o->complete = (m->state == ADAS_VALID || m->state == ADAS_ABSENT) &&
        m->total_count_known == 1 && !m->flags && m->count <= ADAS_MAX_OBJECTS &&
        m->observed_count == m->count &&
        ((m->state == ADAS_ABSENT) == (m->count == 0));
    for (uint32_t i = 0; i < m->count && i < ADAS_MAX_OBJECTS; ++i) {
        const struct adas_object *b = &d->objects[i];
        if (b->semantic_class != ADAS_PEDESTRIAN) continue;
        struct adas_ped_object *p = &o->objects[o->count++];
        *p = (struct adas_ped_object){b->box.x0,b->box.y0,b->box.x1,b->box.y1,
            b->confidence_present == 1 ? b->confidence_q10000 : 0,
            b->flags || b->origin != ADAS_YOLO || b->class_map_version != 1 ||
                b->model_class_id != 0, b->original_index};
    }
}

static int path_valid(const struct adas_ped_path *p)
{
    return p->version && p->source_id == 2 && p->layout == 2 &&
        p->top_y >= 0 && p->top_y < p->bottom_y && p->bottom_y <= 1000 &&
        p->bottom_left >= 0 && p->bottom_left <= p->top_left &&
        p->top_left < p->top_right && p->top_right <= p->bottom_right &&
        p->bottom_right <= 1000 && p->boundary > 0 && p->boundary < 100 &&
        p->nearby > p->boundary && p->nearby < 500 &&
        p->minimum_confidence > 0 && p->minimum_confidence <= 10000 &&
        p->minimum_height > 0 && p->minimum_width > 0;
}

/* Minimum signed distance to the four inward half planes of a convex path. */
static double distance_to_path(const struct adas_ped_path *p, double x, double y)
{
    const double xs[] = {p->top_left,p->top_right,p->bottom_right,p->bottom_left};
    const double ys[] = {p->top_y,p->top_y,p->bottom_y,p->bottom_y};
    double nearest = 2000;
    for (unsigned i = 0; i < 4; ++i) {
        unsigned j = (i+1)%4;
        double dx = xs[j]-xs[i], dy = ys[j]-ys[i];
        double d = (dx*(y-ys[i])-dy*(x-xs[i]))/sqrt(dx*dx+dy*dy);
        if (d < nearest) nearest = d;
    }
    return nearest;
}

static uint32_t object_risk(const struct adas_ped_path *p,
                            const struct adas_ped_object *b, uint32_t *reason)
{
    if (b->position_unknown || b->confidence < p->minimum_confidence || b->confidence > 10000 ||
        b->x0 <= 2 || b->y0 <= 2 || b->x1 >= 1918 || b->y1 >= 1078 ||
        b->x1 > 1920 || b->y1 > 1080 || b->x1-b->x0 < (int)p->minimum_width ||
        b->y1-b->y0 < (int)p->minimum_height) {
        *reason |= ADAS_PED_POSITION; return ADAS_PED_UNKNOWN;
    }
    const double x = (b->x0+b->x1)*500.0/1920;
    const double y = b->y1*1000.0/1080;
    const double half = fmin((b->x1-b->x0)*0.15,20)*1000.0/1920;
    const double height = fmin((b->y1-b->y0)*0.05,10)*1000.0/1080;
    const double points[4][2] = {{x-half,y-height},{x+half,y-height},{x+half,y},{x-half,y}};
    int inside = 1, outside = 0;
    for (unsigned i = 0; i < 4; ++i) {
        double d = distance_to_path(p,points[i][0],points[i][1]);
        if (d <= p->boundary) inside = 0;
        if (d >= -p->boundary) outside = 1;
    }
    if (inside) return ADAS_PED_CONFLICT;
    /* Also catch an edge cutting the foot band with every corner outside. */
    if (outside || distance_to_path(p,x,y-height/2) >= -p->boundary) {
        *reason |= ADAS_PED_BOUNDARY; return ADAS_PED_UNKNOWN;
    }
    if (y < p->top_y) {
        *reason |= ADAS_PED_OUT_OF_RANGE; return ADAS_PED_UNKNOWN;
    }
    return distance_to_path(p,x,y) >= -p->nearby ? ADAS_PED_NEARBY : ADAS_PED_CLEAR;
}

static void assess(struct adas_ped_test *s)
{
    const struct adas_ped_observation *o = &s->observation;
    s->risk = ADAS_PED_UNKNOWN; s->reason = 0; s->recommended_action = ADAS_ACTION_UNKNOWN;
    if (!o->valid) { s->reason = ADAS_PED_FAILED; return; }
    int unknown = !o->complete || o->count > ADAS_PED_MAX_OBJECTS, nearby = 0, conflict = 0;
    if (unknown) s->reason |= ADAS_PED_COLLECTION;
    for (unsigned i = 0; i < o->count && i < ADAS_PED_MAX_OBJECTS; ++i) {
        uint32_t risk = object_risk(&s->path,&o->objects[i],&s->reason);
        conflict |= risk == ADAS_PED_CONFLICT;
        unknown |= risk == ADAS_PED_UNKNOWN;
        nearby |= risk == ADAS_PED_NEARBY;
    }
    s->risk = conflict ? ADAS_PED_CONFLICT : unknown ? ADAS_PED_UNKNOWN :
        nearby ? ADAS_PED_NEARBY : ADAS_PED_CLEAR;
    s->recommended_action = conflict ? ADAS_ACTION_STOP :
        unknown ? ADAS_ACTION_UNKNOWN : ADAS_ACTION_ADVANCE;
}

void adas_ped_test_init(struct adas_ped_test *s)
{
    memset(s,0,sizeof(*s));
    s->control.command = ADAS_DRIVE_UNKNOWN;
    s->risk = ADAS_PED_UNKNOWN; s->reason = ADAS_PED_WAIT;
}

void adas_ped_test_clear(struct adas_ped_test *s, uint64_t now)
{
    s->control.command = ADAS_DRIVE_UNKNOWN; s->control.generation++;
    s->control.observed_ns = now;
    memset(&s->output,0,sizeof(s->output));
}

int adas_ped_test_enter(struct adas_ped_test *s, const struct adas_drive_frame *f,
                        const struct adas_ped_path *p, uint64_t now)
{
    if (!f || !p || !path_valid(p) || f->valid != 1 || !f->stream_epoch || !now ||
        f->source_id != p->source_id || f->layout != p->layout ||
        f->config_generation != p->config_generation) return -1;
    uint64_t context = s->context+1, generation = s->control.generation;
    adas_ped_test_init(s); s->context = context; s->control.generation = generation;
    s->active = 1; s->frame = *f; s->path = *p;
    s->started_ns = s->last_now_ns = now;
    s->control.scene = ADAS_SCENE_ROAD;
    adas_ped_test_clear(s,now);
    return 0;
}

void adas_ped_test_exit(struct adas_ped_test *s, uint64_t now)
{
    uint64_t context = s->context+1, generation = s->control.generation;
    adas_ped_test_init(s); s->context = context; s->control.generation = generation;
    adas_ped_test_clear(s,now);
}

int adas_ped_test_observe(struct adas_ped_test *s, uint64_t context,
                          const struct adas_ped_observation *o,
                          uint64_t image_ns, int frozen, uint64_t now)
{
    if (!s->active || s->received || context != s->context || !o ||
        !same_frame(&s->frame,&o->frame) || !image_ns || o->observed_ns != image_ns ||
        now < s->last_now_ns || now < s->started_ns ||
        now-s->started_ns >= PED_WAIT_NS || o->finished_ns > now ||
        o->finished_ns < o->observed_ns || !o->algorithm_id || !o->algorithm_version ||
        !o->model_generation || !o->input_profile_generation) return 0;
    if (frozen ? o->finished_ns < s->started_ns :
        (!o->ttl_ns || now < o->observed_ns || now-o->observed_ns >= o->ttl_ns)) return 0;
    s->last_now_ns = now; s->received = 1; s->observation = *o;
    assess(s);
    /* A late observation never re-evaluates an earlier click. */
    return 1;
}

void adas_ped_test_tick(struct adas_ped_test *s, uint64_t now)
{
    if (!s->active) return;
    if (now < s->last_now_ns) {
        s->received = 1; s->risk = ADAS_PED_UNKNOWN; s->reason = ADAS_PED_BAD_TIME;
        s->recommended_action = ADAS_ACTION_UNKNOWN;
        adas_ped_test_clear(s,now);
    } else if (!s->received && now-s->started_ns >= PED_WAIT_NS) {
        s->risk = ADAS_PED_UNKNOWN; s->reason = ADAS_PED_BAD_TIME;
    }
    s->last_now_ns = now;
}

int adas_ped_test_click(struct adas_ped_test *s, uint32_t command, uint64_t now)
{
    if (!s->active || (command != ADAS_DRIVE_STOP && command != ADAS_DRIVE_ADVANCE)) return -1;
    adas_ped_test_tick(s,now);
    s->control.command = command; s->control.generation++; s->control.observed_ns = now;
    struct adas_drive_output *o = &s->output;
    memset(o,0,sizeof(*o));
    o->evaluated = 1; o->control_generation = s->control.generation;
    o->evaluated_ns = now; o->simulation_mode = 1; o->required_tasks = ADAS_DECISION_HAZARD;
    o->reasons = s->risk == ADAS_PED_CONFLICT ? ADAS_DRIVE_PEDESTRIAN :
        s->risk == ADAS_PED_UNKNOWN ? ADAS_DRIVE_HAZARD_UNKNOWN :
        s->risk == ADAS_PED_NEARBY ? ADAS_DRIVE_PEDESTRIAN_NEARBY : 0;
    if (command == ADAS_DRIVE_STOP) {
        o->permission = ADAS_DRIVE_BLOCK; o->action = ADAS_ACTION_STOP;
        o->reasons |= ADAS_DRIVE_USER_STOP;
    } else if (s->risk == ADAS_PED_CONFLICT) {
        o->permission = ADAS_DRIVE_BLOCK; o->action = ADAS_ACTION_STOP;
        o->alarm_active = ADAS_DRIVE_PEDESTRIAN;
    } else if (s->risk == ADAS_PED_UNKNOWN) {
        o->permission = ADAS_DRIVE_INSUFFICIENT; o->action = ADAS_ACTION_UNKNOWN;
    } else {
        o->permission = ADAS_DRIVE_ALLOW; o->action = ADAS_ACTION_ADVANCE;
        o->path_direction = 2;
    }
    return 1;
}
