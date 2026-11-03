#include "pedestrian_alert.h"
#include <string.h>

static int same_context(const struct adas_identity *a, const struct adas_identity *b)
{
    return a->stream_epoch == b->stream_epoch && a->source_id == b->source_id &&
        a->layout == b->layout && a->config_generation == b->config_generation &&
        a->canvas_width == b->canvas_width && a->canvas_height == b->canvas_height;
}

static int same_frame(const struct adas_identity *a, const struct adas_identity *b)
{
    return same_context(a,b) && a->frame_sequence == b->frame_sequence;
}

struct ped_alert_config ped_alert_default_config(void)
{
    return (struct ped_alert_config){
        .version = PED_ALERT_CONFIG_VERSION, .confidence = 3500, .height = 80,
        .near_height = 250, .width = 10, .foot_height = 100, .foot_width = 500,
        .overlap = 600, .boundary = 12, .release_ns = UINT64_C(300000000),
        .attention = {380,1000,250,750,50,950},
        .relevant = {450,1000,430,570,250,750},
        .near = {700,1000,450,550,350,650}
    };
}

void ped_alert_init(struct pedestrian_alert *s)
{
    memset(s,0,sizeof(*s));
    s->config = ped_alert_default_config();
}

void ped_alert_gap(struct pedestrian_alert *s)
{
    s->votes = s->vote_count = s->negative_count = 0;
    s->negative_since_ns = 0;
}

void ped_alert_control(struct pedestrian_alert *s, uint32_t enabled, uint32_t status,
                       const struct adas_identity *id, uint64_t now)
{
    const uint32_t running = enabled && status == PED_ALERT_WAITING && id &&
        id->stream_epoch && id->source_id == 2 && id->layout == 2 &&
        id->canvas_width == 1920 && id->canvas_height == 1080;
    if (!enabled) status = PED_ALERT_OFF;
    else if (status == PED_ALERT_WAITING && !running) status = PED_ALERT_UNSUPPORTED;
    if (s->output.enabled == enabled && s->running == running &&
        (running ? same_context(&s->context,id) : s->output.status == status)) return;
    struct ped_alert_config config = s->config;
    struct ped_alert_output old = s->output;
    memset(s,0,sizeof(*s));
    s->config = config;
    s->output.generation = old.generation+1;
    s->output.revision = old.revision+1;
    s->output.evaluations = old.evaluations; s->output.paired = old.paired;
    s->output.dropped = old.dropped; s->output.rejected = old.rejected;
    s->output.enabled = enabled; s->output.status = status;
    s->running = running; s->accept_after_ns = s->last_now_ns = now;
    if (id) s->context = *id;
}

static int fresh(const struct adas_branch_header *h, uint64_t now)
{
    return h->observed_ns && h->observed_ns <= now && h->ttl_ns &&
        now-h->observed_ns < h->ttl_ns && h->finished_ns >= h->observed_ns &&
        h->finished_ns <= now;
}

static int complete(const struct adas_collection *m)
{
    return (m->state == ADAS_VALID || m->state == ADAS_ABSENT) &&
        m->total_count_known == 1 && !m->flags && m->observed_count == m->count &&
        ((m->state == ADAS_ABSENT) == !m->count);
}

int ped_alert_publish(struct pedestrian_alert *s, const struct adas_result *r,
                       uint32_t branch, uint64_t now)
{
    if (!s->running || !r || branch > ADAS_ROAD) return 0;
    const struct adas_branch_header *h = branch == ADAS_DETECTION ?
        &r->detection.header : &r->road.header;
    if (!h->identity_present || !same_context(&h->identity,&s->context) ||
        h->observed_ns <= s->accept_after_ns || h->observed_ns < s->last_observed_ns ||
        !fresh(h,now) ||
        !h->application_session || !h->producer_session || !h->input_profile_generation ||
        !h->algorithm_id || !h->algorithm_version || !h->algorithm_config_generation) {
        s->output.rejected++; return 0;
    }
    if (branch == ADAS_DETECTION) {
        const struct adas_detection *d = &r->detection;
        for (unsigned i = 0; i < PED_ALERT_HISTORY; ++i)
            if (s->people[i].header.application_session == h->application_session &&
                same_frame(&s->people[i].header.identity,&h->identity)) return 0;
        /* A producer/model/session transition retires pending cross-version evidence. */
        if (s->application_session &&
            (s->application_session != h->application_session ||
             s->producer_session != h->producer_session || s->input_profile != h->input_profile_generation ||
             s->algorithm_id != h->algorithm_id || s->algorithm_version != h->algorithm_version ||
             s->model_generation != h->algorithm_config_generation)) {
            memset(s->people,0,sizeof(s->people));
            for (unsigned i = 0; i < PED_ALERT_HISTORY; ++i)
                if (s->roads[i].header.application_session != h->application_session ||
                    s->roads[i].header.input_profile_generation != h->input_profile_generation)
                    memset(&s->roads[i],0,sizeof(s->roads[i]));
            ped_alert_gap(s); s->last_observed_ns = 0;
            s->output.level = PED_ALERT_NONE; s->output.status = PED_ALERT_WAITING;
            s->output.reason = PED_ALERT_PENDING; s->output.valid_until_ns = 0;
            s->output.revision++;
        }
        s->application_session = h->application_session; s->producer_session = h->producer_session;
        s->input_profile = h->input_profile_generation; s->algorithm_id = h->algorithm_id;
        s->algorithm_version = h->algorithm_version; s->model_generation = h->algorithm_config_generation;
        struct ped_alert_people *p = &s->people[s->people_next++ % PED_ALERT_HISTORY];
        if (p->pending) { s->output.dropped++; ped_alert_gap(s); }
        memset(p,0,sizeof(*p)); p->header = *h; p->meta = d->objects_meta;
        p->pending = 1;
        p->valid = adas_detection_valid(d,now) && h->state == ADAS_VALID &&
            h->origin == ADAS_YOLO && !h->error_errno && d->class_map_version == 1 &&
            (d->semantic_supported_mask & ADAS_CLASS_PEDESTRIAN);
        for (unsigned i = 0; i < d->objects_meta.count && i < ADAS_MAX_OBJECTS; ++i)
            if (d->objects[i].semantic_class == ADAS_PEDESTRIAN)
                p->objects[p->count++] = d->objects[i];
    } else {
        const struct adas_road *road = &r->road;
        for (unsigned i = 0; i < PED_ALERT_HISTORY; ++i)
            if (s->roads[i].header.application_session == h->application_session &&
                same_frame(&s->roads[i].header.identity,&h->identity)) return 0;
        struct ped_alert_road *z = &s->roads[s->road_next++ % PED_ALERT_HISTORY];
        if (fresh(&z->header,now)) { s->output.dropped++; ped_alert_gap(s); }
        memset(z,0,sizeof(*z)); z->header = *h; z->meta = road->zebras_meta;
        z->valid = adas_road_valid(road,now) && h->state == ADAS_VALID &&
            h->origin == ADAS_ROAD_FPGA && !h->error_errno &&
            (road->supported_tasks & ADAS_ROAD_ZEBRA);
        for (unsigned i = 0; i < z->meta.count && i < ADAS_MAX_ZEBRAS; ++i)
            z->zebras[i] = road->zebras[i];
        for (unsigned i = 0; i < PED_ALERT_HISTORY; ++i)
            if (same_frame(&s->people[i].header.identity,&h->identity))
                s->people[i].pending = 1;
    }
    return 1;
}

static int in_roi(const struct ped_alert_roi *r, double x, double y, double margin)
{
    if (y < r->top_y+margin || y > r->bottom_y-margin) return 0;
    double t = (y-r->top_y)/(r->bottom_y-r->top_y);
    double left = r->top_left+t*(r->bottom_left-r->top_left);
    double right = r->top_right+t*(r->bottom_right-r->top_right);
    return x >= left+margin && x <= right-margin;
}

static int box_valid(const struct adas_box *b)
{
    return b->x0 >= 0 && b->y0 >= 0 && b->x0 < b->x1 && b->y0 < b->y1 &&
        b->x1 <= 1920 && b->y1 <= 1080;
}

static double maxd(double a, double b) { return a > b ? a : b; }
static double mind(double a, double b) { return a < b ? a : b; }

static uint32_t assess(const struct pedestrian_alert *s, const struct ped_alert_people *p,
                       const struct ped_alert_road *z, uint32_t *reason, int *known)
{
    const struct ped_alert_config *c = &s->config;
    uint32_t level = PED_ALERT_NONE;
    *known = complete(&p->meta);
    *reason = *known ? 0 : PED_ALERT_INCOMPLETE;
    if (!z || !complete(&z->meta)) *reason |= PED_ALERT_ROAD_WAIT;
    for (unsigned i = 0; i < p->count; ++i) {
        const struct adas_object *o = &p->objects[i];
        const struct adas_box *b = &o->box;
        if (!box_valid(b) || o->origin != ADAS_YOLO || o->model_class_id != 0 ||
            o->class_map_version != 1 || o->confidence_present != 1 || o->flags ||
            o->confidence_q10000 < c->confidence || o->confidence_q10000 > 10000) {
            *known = 0; *reason |= PED_ALERT_POSITION; continue;
        }
        double w = b->x1-b->x0, h = b->y1-b->y0;
        double x = (b->x0+b->x1)*500.0/1920, y = b->y1*1000.0/1080;
        if (w < c->width || h*1000 < c->height*1080 ||
            !in_roi(&c->attention,x,y,0)) continue;
        uint32_t candidate = PED_ALERT_CAUTION, why = PED_ALERT_ROI;
        const int foot_known = b->x0 > 2 && b->y0 > 2 && b->x1 < 1918 &&
            b->y1 < 1078 && w/h >= .12 && w/h <= .9;
        if (!foot_known) { *reason |= PED_ALERT_POSITION; *known = 0; }
        const double x0 = (b->x0+b->x1-w*c->foot_width/1000.)/2;
        const double x1 = (b->x0+b->x1+w*c->foot_width/1000.)/2;
        const double y0 = b->y1-h*c->foot_height/1000.;
        const double y1 = b->y1;
        if (foot_known && h*1000 >= c->near_height*1080 &&
            in_roi(&c->near,x0*1000/1920,y0*1000/1080,c->boundary) &&
            in_roi(&c->near,x1*1000/1920,y1*1000/1080,c->boundary)) {
            candidate = PED_ALERT_YIELD; why = PED_ALERT_NEAR;
        }
        if (foot_known && z && in_roi(&c->relevant,x,y,c->boundary)) {
            for (unsigned k = 0; k < z->meta.count && k < ADAS_MAX_ZEBRAS; ++k) {
                const struct adas_zebra *stripe = &z->zebras[k];
                const struct adas_box *b2 = &stripe->box;
                if (!box_valid(b2) || stripe->flags || stripe->origin != ADAS_ROAD_FPGA ||
                    stripe->stripe_count < 2) continue;
                double intersection = maxd(0,mind(x1,b2->x1)-maxd(x0,b2->x0)) *
                    maxd(0,mind(y1,b2->y1)-maxd(y0,b2->y0));
                if (x*1920/1000 > b2->x0+2 && x*1920/1000 < b2->x1-2 &&
                    y1 > b2->y0+2 && y1 < b2->y1-2 &&
                    intersection*1000 >= (x1-x0)*(y1-y0)*c->overlap) {
                    candidate = PED_ALERT_YIELD; why |= PED_ALERT_ZEBRA;
                }
            }
        }
        if (candidate > level) { level = candidate; *reason &= ~(PED_ALERT_ROI|PED_ALERT_NEAR|PED_ALERT_ZEBRA); }
        if (candidate == level) *reason |= why;
    }
    return level;
}

static void evaluate(struct pedestrian_alert *s, struct ped_alert_people *p, uint64_t now)
{
    const struct adas_branch_header *h = &p->header;
    if (!fresh(h,now) || h->observed_ns < s->last_observed_ns) return;
    const int is_new = !p->processed && h->observed_ns > s->last_observed_ns;
    const struct ped_alert_road *z = NULL;
    for (unsigned i = 0; i < PED_ALERT_HISTORY; ++i) {
        const struct ped_alert_road *r = &s->roads[i];
        if (r->valid && fresh(&r->header,now) &&
            r->header.application_session == h->application_session &&
            r->header.input_profile_generation == h->input_profile_generation &&
            same_frame(&r->header.identity,&h->identity)) { z = r; break; }
    }
    if (z) s->output.paired++;
    uint32_t reason = 0;
    int known = 0;
    uint32_t level = p->valid ? assess(s,p,z,&reason,&known) : PED_ALERT_NONE;
    s->output.evaluations++; p->processed = 1;
    if (is_new) {
        s->last_observed_ns = h->observed_ns;
        s->votes = ((s->votes << 1) | (level >= PED_ALERT_CAUTION)) & 7U;
        if (s->vote_count < 3) s->vote_count++;
    }
    if (!p->valid || (!known && !level)) {
        s->output.status = PED_ALERT_UNAVAILABLE; s->output.level = PED_ALERT_NONE;
        s->output.reason = p->valid ? PED_ALERT_INCOMPLETE : PED_ALERT_FAILURE;
        s->output.valid_until_ns = 0; ped_alert_gap(s);
        return;
    }
    const unsigned votes = (s->votes & 1U)+((s->votes >> 1)&1U)+((s->votes >> 2)&1U);
    uint32_t accepted = level == PED_ALERT_CAUTION && votes < 2 ? PED_ALERT_NONE : level;
    uint64_t until = h->observed_ns+h->ttl_ns;
    if (accepted == PED_ALERT_YIELD && (reason & PED_ALERT_ZEBRA) &&
        !(reason & PED_ALERT_NEAR) && z)
        until = mind(until,z->header.observed_ns+z->header.ttl_ns);
    if (accepted >= s->output.level) {
        s->output.level = accepted; s->output.reason = reason;
        s->output.valid_until_ns = until;
        s->negative_count = 0; s->negative_since_ns = 0;
    } else {
        /* Missing zebra is not a negative observation of a previous zebra relation. */
        int negative = known && ((z && complete(&z->meta)) || !p->count ||
            !(s->output.reason & PED_ALERT_ZEBRA));
        if (is_new && negative) {
            if (!s->negative_count) s->negative_since_ns = h->observed_ns;
            s->negative_count++;
        } else if (is_new) { s->negative_count = 0; s->negative_since_ns = 0; }
        if (negative && s->negative_count >= 5 &&
            h->observed_ns-s->negative_since_ns >= s->config.release_ns) {
            s->output.level = accepted; s->output.reason = reason;
            s->output.valid_until_ns = until;
        } else if (now >= s->output.valid_until_ns) {
            s->output.level = accepted; s->output.reason = reason|PED_ALERT_PENDING;
            s->output.valid_until_ns = until;
        } else s->output.reason |= PED_ALERT_PENDING;
    }
    s->output.observed_ns = h->observed_ns; s->output.identity = h->identity;
    s->output.complete = known; s->output.road_available = z != NULL;
    s->output.status = !known && !s->output.level ? PED_ALERT_UNAVAILABLE : PED_ALERT_READY;
}

void ped_alert_tick(struct pedestrian_alert *s, uint64_t now, uint32_t budget)
{
    if (!s->running) return;
    struct ped_alert_output before = s->output;
    if (now < s->last_now_ns) {
        const struct adas_identity context = s->context;
        ped_alert_control(s,0,PED_ALERT_OFF,NULL,now);
        ped_alert_control(s,1,PED_ALERT_WAITING,&context,now);
        s->output.status = PED_ALERT_UNAVAILABLE; s->output.reason = PED_ALERT_STALE;
        return;
    }
    s->last_now_ns = now;
    if (!s->last_observed_ns && now-s->accept_after_ns >= ADAS_DETECTION_TTL_NS) {
        s->output.status = PED_ALERT_UNAVAILABLE; s->output.reason = PED_ALERT_STALE;
    }
    /* Retire expired votes before accepting a new batch after an owner stall. */
    if (s->output.valid_until_ns && now >= s->output.valid_until_ns) {
        s->output.status = PED_ALERT_UNAVAILABLE; s->output.level = PED_ALERT_NONE;
        s->output.reason = PED_ALERT_STALE; s->output.valid_until_ns = 0;
        s->output.road_available = 0; ped_alert_gap(s);
    }
    while (budget--) {
        struct ped_alert_people *next = NULL;
        for (unsigned i = 0; i < PED_ALERT_HISTORY; ++i)
            if (s->people[i].pending && (!next ||
                s->people[i].header.observed_ns < next->header.observed_ns)) next = &s->people[i];
        if (!next) break;
        next->pending = 0;
        evaluate(s,next,now);
    }
    if (s->output.level != before.level || s->output.status != before.status ||
        s->output.reason != before.reason || s->output.road_available != before.road_available ||
        s->output.complete != before.complete) s->output.revision++;
}
