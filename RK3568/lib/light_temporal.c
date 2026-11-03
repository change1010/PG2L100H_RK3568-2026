#include "light_temporal.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MS UINT64_C(1000000)

static int same_context(const struct adas_branch_header *a, const struct adas_branch_header *b)
{
    return a->application_session == b->application_session &&
        a->producer_session == b->producer_session &&
        a->input_profile_generation == b->input_profile_generation &&
        a->algorithm_config_generation == b->algorithm_config_generation &&
        a->identity.stream_epoch == b->identity.stream_epoch &&
        a->identity.source_id == b->identity.source_id &&
        a->identity.layout == b->identity.layout &&
        a->identity.config_generation == b->identity.config_generation;
}

static struct adas_box from_yolo(const struct yolo_box *b)
{
    return (struct adas_box){(int32_t)b->x0, (int32_t)b->y0, (int32_t)b->x1, (int32_t)b->y1};
}

static int sane(const struct adas_box *b)
{
    return b->x0 >= 0 && b->y0 >= 0 && b->x1 <= 1920 && b->y1 <= 1080 &&
        b->x1 > b->x0 && b->y1 > b->y0;
}

static struct adas_box shift(struct adas_box b, int x, int y)
{
    if (x < -b.x0) x = -b.x0;
    if (x > 1920-b.x1) x = 1920-b.x1;
    if (y < -b.y0) y = -b.y0;
    if (y > 1080-b.y1) y = 1080-b.y1;
    b.x0 += x; b.x1 += x; b.y0 += y; b.y1 += y;
    return b;
}

static struct adas_box predict(const struct light_track *t, uint64_t now)
{
    const float dt = (float)(now-t->last_ns)/1e9f;
    return shift(t->box, (int)lroundf(t->vx*dt), (int)lroundf(t->vy*dt));
}

static float association(const struct adas_box *a, const struct adas_box *b)
{
    const float aw = (float)(a->x1-a->x0), ah = (float)(a->y1-a->y0);
    const float bw = (float)(b->x1-b->x0), bh = (float)(b->y1-b->y0);
    if (bw < aw*.5f || bw > aw*2 || bh < ah*.5f || bh > ah*2) return 0;
    float dx = fabsf((float)(a->x0+a->x1-b->x0-b->x1)*.5f)/(fmaxf(aw,bw)+6);
    float dy = fabsf((float)(a->y0+a->y1-b->y0-b->y1)*.5f)/(fmaxf(ah,bh)+6);
    if (dx > .65f || dy > .65f) return 0;
    float w = fmaxf(0, fminf((float)a->x1,(float)b->x1)-fmaxf((float)a->x0,(float)b->x0));
    float h = fmaxf(0, fminf((float)a->y1,(float)b->y1)-fmaxf((float)a->y0,(float)b->y0));
    return w*h/(aw*ah+bw*bh-w*h) + .35f*(1-dx)*(1-dy);
}

static int gray(const uint8_t *pixels, size_t stride, int x, int y)
{
    if (x < 0) x = 0;
    if (x > 1919) x = 1919;
    if (y < 0) y = 0;
    if (y > 1079) y = 1079;
    const uint8_t *p = pixels+(size_t)y*stride+(size_t)x*2;
    unsigned v = p[0] | ((unsigned)p[1]<<8);
    unsigned r = ((v>>11)&31)*255/31, g = ((v>>5)&63)*255/63, b = (v&31)*255/31;
    int result = (int)((77*r+150*g+29*b)>>8);
    return result > 128 ? 128 : result;
}

/* Sparse housing+context patch, with brightness clipped to reduce lamp-phase
 * sensitivity. Flat/ambiguous patches cannot certify visual motion. */
static int patch(const uint8_t *pixels, size_t stride, struct adas_box b, uint8_t out[256])
{
    int w = b.x1-b.x0, h = b.y1-b.y0, low = 255, high = 0;
    for (int y = 0; y < 16; ++y) for (int x = 0; x < 16; ++x) {
        int v = gray(pixels,stride,b.x0-w/4+(2*x+1)*w*3/64,
                     b.y0-h/4+(2*y+1)*h*3/64);
        out[y*16+x] = (uint8_t)v;
        if (v < low) low = v;
        if (v > high) high = v;
    }
    return high-low >= 20;
}

static int locate(const struct light_track *t, const uint8_t *pixels, size_t stride,
                  struct adas_box predicted, struct adas_box *out)
{
    if (!t->template_valid) return 0;
    unsigned best = UINT32_MAX, second = UINT32_MAX, scores[81];
    int best_x = 0, best_y = 0;
    for (int y = -8; y <= 8; y += 2) for (int x = -8; x <= 8; x += 2) {
        uint8_t current[256];
        struct adas_box b = shift(predicted,x,y);
        scores[((y+8)/2)*9+(x+8)/2] = UINT32_MAX;
        if (!patch(pixels,stride,b,current)) continue;
        unsigned sad = 0;
        for (unsigned k = 0; k < 256; ++k) sad += (unsigned)abs((int)current[k]-t->patch[k]);
        scores[((y+8)/2)*9+(x+8)/2] = sad;
        if (sad < best || (sad == best && abs(x)+abs(y) < abs(best_x)+abs(best_y))) {
            best = sad; best_x = x; best_y = y;
        }
    }
    for (int y = -8; y <= 8; y += 2) for (int x = -8; x <= 8; x += 2)
        if (abs(x-best_x)+abs(y-best_y) >= 6 &&
            scores[((y+8)/2)*9+(x+8)/2] < second)
            second = scores[((y+8)/2)*9+(x+8)/2];
    if (best > 18*256 || second == UINT32_MAX || second < best+128) return 0;
    *out = shift(predicted,best_x,best_y);
    return 1;
}

static int smooth(int old, int next)
{
    int d = next-old;
    return abs(d) <= 1 ? old : old+(d > 0 ? (d+1)/2 : (d-1)/2);
}

static void temporal_color(struct light_track *t, const struct light_color_result *c,
                           uint64_t now, uint32_t *flags)
{
    const uint32_t color = c->color;
    if (c->reason == LIGHT_COLOR_OK || c->reason == LIGHT_COLOR_NO_EVIDENCE) {
        uint32_t lit = color != ADAS_COLOR_UNKNOWN;
        if (t->edge_ns && lit != t->lit) {
            uint64_t gap = now-t->edge_ns;
            if (gap >= 80*MS && gap <= 1200*MS) ++t->edges;
            else t->edges = 0;
            t->edge_ns = now;
            if (t->edges >= 4) t->intermittent_until_ns = now+1500*MS;
        } else if (!t->edge_ns) t->edge_ns = now;
        if (now-t->edge_ns > 1500*MS) { t->edges = 0; t->edge_ns = now; }
        t->lit = lit;
    }
    if (color != ADAS_COLOR_UNKNOWN) {
        if (t->pending != color) {
            t->pending = color; t->pending_count = 1; t->pending_ns = now;
        } else ++t->pending_count;
        /* Red/yellow replace green immediately. A new green needs two
         * independent frames over at least 33ms; raw ADAS is unmodified. */
        if (color != ADAS_GREEN || (t->pending_count >= 2 && now-t->pending_ns >= 33*MS) ||
            t->color == ADAS_GREEN) {
            t->color = color; t->color_ns = now;
        } else {
            t->color = ADAS_COLOR_UNKNOWN;
            *flags |= LIGHT_VISUAL_PENDING;
        }
    } else {
        t->pending = t->pending_count = 0;
        if (c->reason == LIGHT_COLOR_CONFLICT || c->reason == LIGHT_COLOR_TOO_SMALL ||
            !t->color_ns || now-t->color_ns > LIGHT_COLOR_GAP_NS)
            t->color = ADAS_COLOR_UNKNOWN;
        else *flags |= LIGHT_VISUAL_PENDING;
    }
    if (now < t->intermittent_until_ns) *flags |= LIGHT_VISUAL_INTERMITTENT;
}

void light_temporal_open_trace(struct light_temporal *s)
{
    const char *directory = getenv("LIGHT_TRACE_DIR");
    char path[1024];
    if (!directory || !*directory || strlen(directory) > 900) return;
    snprintf(path,sizeof(path),"%s/sequence.jsonl",directory);
    s->trace = fopen(path,"wx");
    snprintf(path,sizeof(path),"%s/roi.rgb565",directory);
    if (s->trace) s->roi = fopen(path,"wbx");
    if (s->trace) setvbuf(s->trace,NULL,_IOFBF,65536);
}

void light_temporal_close(struct light_temporal *s)
{
    if (s->trace) fclose(s->trace);
    if (s->roi) fclose(s->roi);
    s->trace = s->roi = NULL;
}

static void trace_roi(struct light_temporal *s, const uint8_t *pixels, size_t stride,
                      const struct adas_light_visual *v, uint32_t sequence, uint64_t now,
                      int sampled, int appearance, const struct adas_box *roi_box)
{
    if (!s->trace) return;
    long offset = s->roi ? ftell(s->roi) : -1;
    int w = roi_box->x1-roi_box->x0, h = roi_box->y1-roi_box->y0;
    int rw = w > 128 ? 128 : w, rh = h > 128 ? 128 : h;
    if (s->roi) {
        uint8_t row[256];
        for (int y = 0; y < rh; ++y) {
            for (int x = 0; x < rw; ++x) {
                const uint8_t *p = pixels+(size_t)(roi_box->y0+y*h/rh)*stride+
                    (size_t)(roi_box->x0+x*w/rw)*2;
                row[2*x] = p[0]; row[2*x+1] = p[1];
            }
            if (fwrite(row,2,(size_t)rw,s->roi) != (size_t)rw) { offset = -1; break; }
        }
    }
    fprintf(s->trace,"{\"type\":\"%s\",\"seq\":%u,\"ns\":%llu,\"id\":%u,"
        "\"box\":[%d,%d,%d,%d],\"score\":%u,\"instant\":%u,\"display\":%u,"
        "\"reason\":%u,\"evidence\":%u,\"flags\":%u,\"detector_ns\":%llu,"
        "\"sampled\":%d,\"appearance\":%d,\"sample_box\":[%d,%d,%d,%d],"
        "\"roi_offset\":%ld,\"roi_width\":%d,\"roi_height\":%d}\n",
        v->track_id ? "track" : "candidate_roi",
        sequence,(unsigned long long)now,v->track_id,v->box.x0,v->box.y0,v->box.x1,v->box.y1,
        v->confidence_q10000,v->instant_color,v->color,v->reason,v->evidence_permille,
        v->flags,(unsigned long long)v->detector_ns,sampled,appearance,
        roi_box->x0,roi_box->y0,roi_box->x1,roi_box->y1,offset,rw,rh);
}

static void trace_context(struct light_temporal *s, const uint8_t *pixels,
                          size_t stride, uint32_t sequence)
{
    if (!s->trace || !s->roi || s->trace_frames%15) return;
    long offset = ftell(s->roi);
    for (unsigned y = 0; y < 270; ++y) {
        uint8_t row[960];
        for (unsigned x = 0; x < 480; ++x) {
            const uint8_t *p = pixels+(size_t)y*4*stride+x*8;
            row[x*2] = p[0]; row[x*2+1] = p[1];
        }
        if (fwrite(row,1,sizeof(row),s->roi) != sizeof(row)) { offset = -1; break; }
    }
    fprintf(s->trace,"{\"type\":\"context\",\"seq\":%u,\"roi_offset\":%ld,"
        "\"roi_width\":480,\"roi_height\":270}\n",sequence,offset);
}

int light_temporal_update(struct light_temporal *s, struct adas_detection *d,
    const struct yolo_detections *detections, float high, const void *pixels,
    size_t bytes, size_t stride, struct light_color_workspace *workspace)
{
    uint64_t now = d->header.observed_ns;
    if (!pixels || !workspace || stride < 3840 || bytes < stride*1080 ||
        !detections || detections->count > YOLO_MAX_CANDIDATES) return -EINVAL;
    const int context_same = s->initialized && same_context(&s->context,&d->header);
    if (context_same && (now <= s->context.observed_ns ||
        (uint32_t)(d->header.identity.frame_sequence-s->context.identity.frame_sequence) >= 0x80000000U))
        return -ESTALE;
    if (!context_same || now-s->context.observed_ns > LIGHT_TRACK_TTL_NS) {
        memset(s->tracks,0,sizeof(s->tracks));
        s->initialized = 0;
    }
    s->context = d->header; s->initialized = 1;
    uint8_t used[YOLO_MAX_CANDIDATES] = {0};
    struct adas_box predicted[ADAS_MAX_SIGNALS];
    for (unsigned i = 0; i < ADAS_MAX_SIGNALS; ++i) {
        struct light_track *t = &s->tracks[i];
        if (t->active && (now-t->detector_ns >= LIGHT_TRACK_TTL_NS ||
            now-t->high_ns >= 1000*MS)) t->active = 0;
        t->matched = 0;
        predicted[i] = t->active ? predict(t,now) : (struct adas_box){0};
    }
    /* Global one-to-one assignment: strong observations first, then weak
     * observations restricted to existing tracks. Reject ambiguous matches. */
    for (unsigned pass = 0; pass < 2; ++pass) for (;;) {
        float best = .15f;
        int bi = -1, bj = -1;
        for (unsigned i = 0; i < ADAS_MAX_SIGNALS; ++i) {
            struct light_track *t = &s->tracks[i];
            if (!t->active || t->matched) continue;
            for (size_t j = 0; j < detections->count; ++j) {
                const struct yolo_box *b = &detections->boxes[j];
                if (used[j] || b->class_id != 9 || b->score <= fminf(high,LIGHT_ASSOCIATION_MINIMUM) ||
                    (pass == 0 ? b->score <= high : b->score > high)) continue;
                struct adas_box box = from_yolo(b);
                if (!sane(&box)) continue;
                float a = association(&predicted[i],&box);
                /* A close alternative track makes the identity uncertain. */
                for (unsigned k = 0; k < ADAS_MAX_SIGNALS; ++k)
                    if (k != i && s->tracks[k].active && !s->tracks[k].matched &&
                        association(&predicted[k],&box) >= a-.08f) { a = 0; break; }
                if (a > best) { best = a; bi = (int)i; bj = (int)j; }
            }
        }
        if (bi < 0) break;
        struct light_track *t = &s->tracks[bi];
        const struct yolo_box *b = &detections->boxes[bj];
        struct adas_box box = from_yolo(b);
        float dt = (float)(now-t->detector_ns)/1e9f;
        if (dt > .001f) {
            float vx = (box.x0+box.x1-t->measured.x0-t->measured.x1)*.5f/dt;
            float vy = (box.y0+box.y1-t->measured.y0-t->measured.y1)*.5f/dt;
            t->vx = fmaxf(-400,fminf(400,.5f*t->vx+.5f*vx));
            t->vy = fmaxf(-400,fminf(400,.5f*t->vy+.5f*vy));
        }
        t->measured = box;
        t->box = (struct adas_box){smooth(predicted[bi].x0,box.x0),smooth(predicted[bi].y0,box.y0),
            smooth(predicted[bi].x1,box.x1),smooth(predicted[bi].y1,box.y1)};
        t->detector_ns = now; t->score = (uint32_t)lroundf(b->score*10000);
        if (b->score > high) t->high_ns = now;
        t->matched = 1; used[bj] = 1;
    }
    for (size_t j = 0; j < detections->count; ++j) {
        const struct yolo_box *b = &detections->boxes[j];
        if (used[j] || b->class_id != 9 || b->score <= high) continue;
        struct adas_box box = from_yolo(b);
        if (!sane(&box)) continue;
        int near = 0;
        for (unsigned i = 0; i < ADAS_MAX_SIGNALS; ++i)
            if (s->tracks[i].active && association(&s->tracks[i].box,&box) > .15f) near = 1;
        if (near) continue;
        for (unsigned i = 0; i < ADAS_MAX_SIGNALS; ++i) if (!s->tracks[i].active) {
            struct light_track *t = &s->tracks[i];
            memset(t,0,sizeof(*t));
            if (!++s->next_id) ++s->next_id;
            t->id = s->next_id; t->active = t->matched = 1;
            t->box = t->measured = box; t->detector_ns = t->high_ns = now;
            t->score = (uint32_t)lroundf(b->score*10000);
            break;
        }
    }
    d->light_visual_valid = 1; d->light_visual_count = 0;
    if (s->trace) {
        trace_context(s,pixels,stride,d->header.identity.frame_sequence);
        const struct adas_identity *id = &d->header.identity;
        fprintf(s->trace,"{\"type\":\"frame\",\"seq\":%u,\"ns\":%llu,\"epoch\":%u,"
            "\"source\":%u,\"layout\":%u,\"config\":%u,\"session\":%llu,\"raw_signals\":%u}\n",
            id->frame_sequence,(unsigned long long)now,id->stream_epoch,id->source_id,
            id->layout,id->config_generation,(unsigned long long)d->header.application_session,d->signals_meta.count);
        unsigned captured = 0;
        for (size_t j = 0; j < detections->count; ++j) if (detections->boxes[j].class_id == 9) {
            const struct yolo_box *b = &detections->boxes[j];
            fprintf(s->trace,"{\"type\":\"candidate\",\"seq\":%u,\"score\":%.6f,"
                "\"box\":[%.0f,%.0f,%.0f,%.0f],\"associated\":%u}\n",
                id->frame_sequence,b->score,b->x0,b->y0,b->x1,b->y1,used[j]);
            if (captured++ < 16) {
                struct light_color_result c = {0};
                struct adas_light_visual v = {.box = from_yolo(b),
                    .confidence_q10000 = (uint32_t)lroundf(b->score*10000),.detector_ns = now};
                if (!traffic_light_color_rgb565(pixels,bytes,stride,1920,1080,&v.box,workspace,&c)) {
                    v.instant_color = c.color; v.reason = c.reason; v.evidence_permille = c.evidence_permille;
                    trace_roi(s,pixels,stride,&v,id->frame_sequence,now,1,0,&v.box);
                }
            }
        }
    }
    uint8_t search[ADAS_MAX_SIGNALS] = {0};
    unsigned searches = 0;
    for (unsigned n = 0; n < ADAS_MAX_SIGNALS && searches < 2; ++n) {
        unsigned i = (s->search_cursor+n)%ADAS_MAX_SIGNALS;
        if (s->tracks[i].active && !s->tracks[i].matched && s->tracks[i].template_valid &&
            now-s->tracks[i].detector_ns > 100*MS) {
            search[i] = 1; ++searches;
        }
    }
    for (unsigned i = 0; i < ADAS_MAX_SIGNALS; ++i) {
        struct light_track *t = &s->tracks[i];
        if (!t->active) continue;
        uint32_t flags = t->matched ? 0 : LIGHT_VISUAL_PREDICTED;
        int supported = t->matched;
        if (!t->matched) {
            if (search[i]) {
                supported = locate(t,pixels,stride,predicted[i],&t->box);
            }
            if (!supported) t->box = predicted[i];
            t->vx *= .8f; t->vy *= .8f;
        }
        struct light_color_result c = {ADAS_COLOR_UNKNOWN,LIGHT_COLOR_NO_EVIDENCE,0};
        /* Prediction without texture support may bridge at most 150ms. */
        int sampled = supported || now-t->detector_ns <= 150*MS;
        /* Display smoothing must not move a freshly measured lamp out of its
         * color ROI. Missing detections use the validated predicted geometry. */
        struct adas_box sample_box = t->matched ? t->measured : t->box;
        if (sampled) {
            int rc = traffic_light_color_rgb565(pixels,bytes,stride,1920,1080,&sample_box,workspace,&c);
            if (rc) return rc;
            temporal_color(t,&c,now,&flags);
        } else {
            t->color = t->pending = t->pending_count = 0;
        }
        if (t->matched) t->template_valid = (uint32_t)patch(pixels,stride,t->box,t->patch);
        t->last_ns = now;
        struct adas_light_visual *v = &d->light_visual[d->light_visual_count++];
        *v = (struct adas_light_visual){.box = t->box,.track_id = t->id,
            .color = t->color,.instant_color = c.color,.reason = c.reason,
            .evidence_permille = c.evidence_permille,.confidence_q10000 = t->matched ? t->score : 0,
            .flags = flags,.detector_ns = t->detector_ns,.color_ns = t->color_ns};
        trace_roi(s,pixels,stride,v,d->header.identity.frame_sequence,now,sampled,
            supported && !t->matched,&sample_box);
    }
    s->search_cursor = (s->search_cursor+2)%ADAS_MAX_SIGNALS;
    if (s->trace && (++s->trace_frames >= 300 ||
        (s->roi && ftell(s->roi) >= 128L*1024*1024))) light_temporal_close(s);
    return 0;
}
