#include "detection_light_tracker.h"
#include <string.h>

static int same_context(const struct detection_overlay *a, const struct detection_overlay *b)
{
    return a->stream_epoch == b->stream_epoch && a->source_id == b->source_id &&
        a->layout == b->layout && a->config_generation == b->config_generation;
}

static int fresh(const struct detection_light_track *t, uint64_t now)
{
    return t->active && t->last_seen_ns <= now && now - t->last_seen_ns < DETECTION_LIGHT_HOLD_NS;
}

static int sane(const struct detection_overlay_box *b)
{
    return b->x0 >= 0 && b->y0 >= 0 && b->x0 < b->x1 && b->y0 < b->y1 &&
        b->x1 <= 1920 && b->y1 <= 1080;
}

static uint32_t overlap(const struct detection_overlay_box *a, const struct detection_overlay_box *b)
{
    const int32_t x0 = a->x0 > b->x0 ? a->x0 : b->x0, y0 = a->y0 > b->y0 ? a->y0 : b->y0;
    const int32_t x1 = a->x1 < b->x1 ? a->x1 : b->x1, y1 = a->y1 < b->y1 ? a->y1 : b->y1;
    if (x1 <= x0 || y1 <= y0) return 0;
    const uint64_t intersection = (uint64_t)(x1-x0)*(y1-y0);
    const uint64_t area = (uint64_t)(a->x1-a->x0)*(a->y1-a->y0) +
        (uint64_t)(b->x1-b->x0)*(b->y1-b->y0) - intersection;
    return (uint32_t)(intersection*1000/area);
}

static int32_t smooth(int32_t old, int32_t next)
{
    const int32_t delta = next-old;
    return old + (delta >= 0 ? (delta+1)/2 : (delta-1)/2);
}

void detection_light_tracker_update(struct detection_light_tracker *tracker,
    const struct detection_overlay *input, const struct vdma_frame_identity *frame,
    uint64_t now_ns, int enabled, struct detection_overlay *out)
{
    *out = input ? *input : (struct detection_overlay){0};
    if (!enabled || !detection_overlay_visible(input,frame,now_ns)) {
        memset(tracker,0,sizeof(*tracker));
        out->valid = 0;
        return;
    }
    if (input->light_temporal_valid) {
        memset(tracker,0,sizeof(*tracker));
        /* Worker owns tracking. Repaints cannot extend geometry or old color. */
        out->count = 0;
        for (uint32_t i = 0; i < input->count; ++i) {
            if (input->boxes[i].signal && now_ns-input->observed_ns >= UINT64_C(150000000)) continue;
            uint32_t j = out->count++;
            out->boxes[j] = input->boxes[i];
            out->track_ids[j] = input->track_ids[i];
            out->light_flags[j] = input->light_flags[i];
            out->light_deadline_ns[j] = input->light_deadline_ns[i];
            if (out->boxes[j].signal && now_ns >= input->light_deadline_ns[i])
                out->boxes[j].light_color = DETECTION_LIGHT_UNKNOWN;
        }
        return;
    }
    if (tracker->initialized && !same_context(&tracker->identity,input))
        memset(tracker,0,sizeof(*tracker));
    const int duplicate = tracker->initialized &&
        input->frame_sequence == tracker->identity.frame_sequence &&
        input->observed_ns == tracker->identity.observed_ns;
    if (tracker->initialized && !duplicate &&
        ((uint32_t)(input->frame_sequence-tracker->identity.frame_sequence) >= 0x80000000U ||
         input->observed_ns <= tracker->identity.observed_ns)) {
        /* An older publication must not resurrect or recolor a current track. */
        out->valid = 0;
        return;
    }
    for (uint32_t i = 0; i < DETECTION_OVERLAY_CAPACITY; ++i)
        if (!fresh(&tracker->tracks[i],now_ns)) tracker->tracks[i].active = 0;
    if (!duplicate) {
        int matches[DETECTION_OVERLAY_CAPACITY];
        for (uint32_t i = 0; i < DETECTION_OVERLAY_CAPACITY; ++i) {
            tracker->tracks[i].seen = 0;
            matches[i] = -1;
        }
        /* Greedy global IoU matching is one-to-one and independent of detector
         * output order. Match raw geometry so smoothing cannot amplify lag. */
        for (;;) {
            int best_track = -1, best_box = -1;
            uint32_t best = 249;
            for (uint32_t i = 0; i < DETECTION_OVERLAY_CAPACITY; ++i) {
                const struct detection_light_track *t = &tracker->tracks[i];
                if (!t->active || t->seen) continue;
                for (uint32_t j = 0; j < input->count; ++j) {
                    const struct detection_overlay_box *b = &input->boxes[j];
                    if (!b->signal || !sane(b) || matches[j] >= 0 || b->model_class != t->raw.model_class) continue;
                    const uint32_t score = overlap(&t->raw,b);
                    if (score > best) { best = score; best_track = (int)i; best_box = (int)j; }
                }
            }
            if (best_track < 0) break;
            tracker->tracks[best_track].seen = 1;
            matches[best_box] = best_track;
        }
        for (uint32_t j = 0; j < input->count; ++j) {
            const struct detection_overlay_box *b = &input->boxes[j];
            if (!b->signal || !sane(b)) continue;
            int index = matches[j];
            if (index < 0) {
                for (uint32_t i = 0; i < DETECTION_OVERLAY_CAPACITY; ++i)
                    if (!tracker->tracks[i].active) { index = (int)i; break; }
                if (index < 0) {
                    for (uint32_t i = 0; i < DETECTION_OVERLAY_CAPACITY; ++i)
                        if (!tracker->tracks[i].seen && (index < 0 ||
                            tracker->tracks[i].last_seen_ns < tracker->tracks[index].last_seen_ns))
                            index = (int)i;
                    if (index >= 0) tracker->tracks[index].active = 0;
                }
            }
            if (index < 0) continue;
            struct detection_light_track *t = &tracker->tracks[index];
            struct detection_overlay_box box = *b;
            if (t->active && overlap(&t->raw,b) >= 600) {
                box.x0 = smooth(t->box.x0,b->x0); box.y0 = smooth(t->box.y0,b->y0);
                box.x1 = smooth(t->box.x1,b->x1); box.y1 = smooth(t->box.y1,b->y1);
            }
            box.display_held = 0;
            t->box = box; t->raw = *b; t->last_seen_ns = input->observed_ns;
            t->active = t->seen = 1;
        }
        tracker->identity = *input;
        tracker->initialized = 1;
    }
    out->count = 0;
    for (uint32_t j = 0; j < input->count; ++j)
        if (!input->boxes[j].signal) out->boxes[out->count++] = input->boxes[j];
    /* Observed lights take precedence over held geometry at the output limit. */
    for (int seen = 1; seen >= 0; --seen) {
        for (uint32_t i = 0; i < DETECTION_OVERLAY_CAPACITY && out->count < DETECTION_OVERLAY_CAPACITY; ++i) {
            const struct detection_light_track *t = &tracker->tracks[i];
            if (!fresh(t,now_ns) || t->seen != (uint32_t)seen) continue;
            struct detection_overlay_box box = t->box;
            if (!seen) {
                box.light_color = DETECTION_LIGHT_UNKNOWN;
                box.confidence_q10000 = 0;
                box.display_held = 1;
            }
            out->boxes[out->count++] = box;
        }
    }
}
