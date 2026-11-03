#include "detection_overlay.h"
#include "adas_result.h"
_Static_assert(DETECTION_OVERLAY_LIGHT_PREDICTED == LIGHT_VISUAL_PREDICTED &&
    DETECTION_OVERLAY_LIGHT_PENDING == LIGHT_VISUAL_PENDING, "overlay light flags");
#include <string.h>

static void from_detection(struct detection_overlay *out, const struct adas_detection *d, uint64_t now_ns)
{
    memset(out, 0, sizeof(*out));
    out->state = DETECTION_WAITING;
    const struct adas_branch_header *h = &d->header;
    if (h->origin != ADAS_YOLO || h->identity.layout > 3 || h->observed_ns > now_ns ||
        h->ttl_ns > UINT64_MAX - h->observed_ns) return;
    out->state = DETECTION_RUNNING; out->valid = 1;
    out->stream_epoch = h->identity.stream_epoch; out->source_id = h->identity.source_id;
    out->layout = h->identity.layout; out->config_generation = h->identity.config_generation;
    out->frame_sequence = h->identity.frame_sequence; out->observed_ns = h->observed_ns;
    out->valid_until_ns = h->observed_ns + h->ttl_ns;
    out->latency_ms = (h->finished_ns - h->observed_ns) / 1e6;
    for (uint32_t i = 0; i < d->objects_meta.count && i < ADAS_MAX_OBJECTS; ++i) {
        const struct adas_object *o = &d->objects[i];
        out->boxes[out->count++] = (struct detection_overlay_box){o->box.x0,o->box.y0,o->box.x1,o->box.y1,
            o->model_class_id,o->confidence_q10000,0,DETECTION_LIGHT_UNKNOWN,0};
    }
    out->light_temporal_valid = d->light_visual_valid;
    if (d->light_visual_valid) {
        for (uint32_t i = 0; i < d->light_visual_count && i < ADAS_MAX_SIGNALS; ++i) {
            const struct adas_light_visual *v = &d->light_visual[i];
            const uint32_t j = out->count++;
            out->boxes[j] = (struct detection_overlay_box){v->box.x0,v->box.y0,v->box.x1,v->box.y1,
                9,v->confidence_q10000,1,v->color,0};
            out->track_ids[j] = v->track_id; out->light_flags[j] = v->flags;
            out->light_deadline_ns[j] = v->color_ns+UINT64_C(120000000);
        }
    }
    for (uint32_t i = 0; !d->light_visual_valid && i < d->signals_meta.count && i < ADAS_MAX_SIGNALS; ++i) {
        const struct adas_signal *s = &d->signals[i];
        uint32_t color = DETECTION_LIGHT_UNKNOWN;
        if (s->color_valid) {
            if (s->color == ADAS_RED) color = DETECTION_LIGHT_RED;
            else if (s->color == ADAS_YELLOW) color = DETECTION_LIGHT_YELLOW;
            else if (s->color == ADAS_GREEN) color = DETECTION_LIGHT_GREEN;
        }
        out->boxes[out->count++] = (struct detection_overlay_box){s->box.x0,s->box.y0,s->box.x1,s->box.y1,
            9,s->localization_confidence_q10000,1,color,0};
    }
}

void detection_overlay_from_result(struct detection_overlay *out, const struct adas_result *result, uint64_t now_ns)
{
    memset(out, 0, sizeof(*out));
    out->state = DETECTION_WAITING;
    if (result && adas_branch_ready(result, ADAS_DETECTION, now_ns, 0))
        from_detection(out, &result->detection, now_ns);
}

void detection_overlay_from_frozen(struct detection_overlay *out,
    const struct adas_detection *d, uint64_t now_ns)
{
    memset(out, 0, sizeof(*out));
    out->state = DETECTION_ERROR;
    if (!d || !adas_detection_valid(d, now_ns) || d->header.state != ADAS_VALID ||
        d->header.origin != ADAS_YOLO) return;
    from_detection(out, d, now_ns);
    if (out->valid) out->valid_until_ns = UINT64_MAX;
}

int detection_overlay_visible(const struct detection_overlay *o, const struct vdma_frame_identity *f, uint64_t now_ns)
{
    return o && f && o->valid && o->state == DETECTION_RUNNING && o->count <= DETECTION_OVERLAY_CAPACITY &&
        o->observed_ns <= now_ns && now_ns < o->valid_until_ns && f->layout < 4 &&
        o->stream_epoch == f->stream_epoch && o->source_id == f->source_id && o->layout == f->layout &&
        o->config_generation == f->config_generation && (uint32_t)(f->frame_sequence - o->frame_sequence) < 0x80000000U;
}
