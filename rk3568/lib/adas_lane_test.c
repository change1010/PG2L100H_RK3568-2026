#include "adas_lane_test.h"
#include "adas_result.h"
#include <stdlib.h>
#include <string.h>

struct adas_lane_test_config adas_lane_test_default_config(void)
{
    return (struct adas_lane_test_config){960, 900, 120, 96, 48};
}

int adas_lane_test_same_frame(const struct adas_drive_frame *a,
                              const struct adas_drive_frame *b)
{
    return a && b && a->valid && b->valid &&
        a->stream_epoch == b->stream_epoch && a->source_id == b->source_id &&
        a->layout == b->layout && a->config_generation == b->config_generation &&
        a->frame_sequence == b->frame_sequence;
}

#define MAX_LANE_CANDIDATES 128U

struct lane_fragment {
    double top, bottom;
    int x0, x1, y0, y1, width;
    uint32_t type;
};
struct lane_candidate {
    uint64_t members;
    double slope, x;
    int width;
    struct adas_lane_test_side side;
};

static double magnitude(double value) { return value < 0 ? -value : value; }

/* All endpoint sums use native integer/half-integer coordinates. Re-fitting the
 * complete set makes the result independent of which pair seeded the group. */
static void fit_candidate(const struct lane_fragment *f, unsigned n, uint64_t mask,
    const struct adas_lane_test_config *c, struct lane_candidate *out)
{
    double sy = 0, sx = 0, syy = 0, syx = 0, samples = 0;
    memset(out, 0, sizeof(*out));
    out->members = mask;
    out->side.x0 = 1920; out->side.y0 = 1080;
    uint32_t type = 0;
    int conflict = 0, overlap = 0, bad = 0, largest_gap = 0;
    for (unsigned i = 0; i < n; ++i) {
        if (!(mask & (UINT64_C(1) << i))) continue;
        const struct lane_fragment *p = &f[i];
        const double ys[] = {p->y0 - c->reference_y, p->y1 - c->reference_y};
        const double xs[] = {p->top, p->bottom};
        for (unsigned j = 0; j < 2; ++j) {
            sy += ys[j]; sx += xs[j]; syy += ys[j]*ys[j]; syx += ys[j]*xs[j];
            samples++;
        }
        if (p->x0 < out->side.x0) out->side.x0 = p->x0;
        if (p->x1 > out->side.x1) out->side.x1 = p->x1;
        if (p->y0 < out->side.y0) out->side.y0 = p->y0;
        if (p->y1 > out->side.y1) out->side.y1 = p->y1;
        if (p->width > out->width) out->width = p->width;
        out->side.segment_count++;
        out->side.support_px += (uint32_t)(p->y1 - p->y0);
        if (type && type != p->type) conflict = 1;
        if (!p->type || (p->type != ADAS_SOLID && p->type != ADAS_DASHED)) bad = 1;
        type = p->type;
        if (p->width > c->maximum_width) bad = 1;
        int prior_end = -1;
        for (unsigned j = 0; j < n; ++j) {
            if (i == j || !(mask & (UINT64_C(1) << j))) continue;
            if (f[j].y0 < p->y1 && f[j].y1 > p->y0) overlap = 1;
            if (f[j].y1 <= p->y0 && f[j].y1 > prior_end) prior_end = f[j].y1;
        }
        if (prior_end >= 0 && p->y0 - prior_end > largest_gap)
            largest_gap = p->y0 - prior_end;
    }
    const double denominator = samples*syy - sy*sy;
    out->slope = (samples*syx - sy*sx) / denominator;
    out->x = (sx - out->slope*sy) / samples;
    out->side.x = (int32_t)(out->x + 0.5);
    out->side.span_px = (uint32_t)(out->side.y1 - out->side.y0);
    out->side.line_type = type;
    out->side.reason = ADAS_LANE_TEST_OK;
    for (unsigned i = 0; i < n; ++i) {
        if (!(mask & (UINT64_C(1) << i))) continue;
        if (magnitude(f[i].top - (out->x + out->slope*(f[i].y0-c->reference_y))) >
                c->ambiguity_px/2.0 ||
            magnitude(f[i].bottom - (out->x + out->slope*(f[i].y1-c->reference_y))) >
                c->ambiguity_px/2.0) conflict = 1;
    }
    const int left = out->x < c->reference_x;
    if (out->side.span_px < (uint32_t)c->minimum_span ||
        out->side.support_px < (uint32_t)((c->minimum_span + 2)/3) ||
        largest_gap > c->minimum_span + c->ambiguity_px ||
        magnitude(out->slope) > 3.0 ||
        (left ? out->slope > 0.25 : out->slope < -0.25) ||
        out->x < 0 || out->x >= 1920 || bad)
        out->side.reason = ADAS_LANE_TEST_GEOMETRY;
    if (conflict || overlap) out->side.reason = ADAS_LANE_TEST_AMBIGUOUS;
    if (out->side.reason != ADAS_LANE_TEST_OK) out->side.line_type = ADAS_LINE_UNKNOWN;
}

static int add_candidate(struct lane_candidate *out, unsigned *count,
    const struct lane_fragment *f, unsigned n, uint64_t mask,
    const struct adas_lane_test_config *c)
{
    for (unsigned i = 0; i < *count; ++i)
        if (out[i].members == mask) return 0;
    struct lane_candidate candidate;
    fit_candidate(f,n,mask,c,&candidate);
    /* A small extrapolation admits a dash ending near the reference region;
     * distant fragments cannot be promoted to the ego boundary. */
    if (c->reference_y < candidate.side.y0 - c->ambiguity_px ||
        c->reference_y > candidate.side.y1 + c->ambiguity_px) return 0;
    if (*count == MAX_LANE_CANDIDATES) return -1;
    out[(*count)++] = candidate;
    return 0;
}

int adas_lane_test_select(const struct road_ref_geometry *g,
                          const struct adas_lane_test_config *c,
                          struct adas_lane_test_side sides[2])
{
    if (!sides) return -1;
    memset(sides, 0, 2 * sizeof(*sides));
    sides[0].reason = sides[1].reason = ADAS_LANE_TEST_MISSING;
    if (!g || !c || !g->present || g->truncated ||
        g->confirmed_count > ROAD_REF_MAX_SEGMENTS || g->point_count > ROAD_REF_MAX_POINTS ||
        c->reference_x <= 0 || c->reference_x >= 1920 ||
        c->reference_y <= 0 || c->reference_y >= 1080 ||
        c->minimum_span <= 0 || c->minimum_span > 1080 ||
        c->maximum_width <= 0 || c->maximum_width > 1920 ||
        c->ambiguity_px < 2 || c->ambiguity_px > 1920) return -1;
    struct lane_fragment fragments[ROAD_REF_MAX_SEGMENTS];
    const unsigned n = g->confirmed_count;
    for (unsigned i = 0; i < n; ++i) {
        const struct road_ref_polygon *polygon = &g->confirmed[i];
        if (polygon->point_count != 4 || polygon->first_point > g->point_count ||
            g->point_count - polygon->first_point < 4) goto malformed;
        const struct road_ref_point *p = &g->points[polygon->first_point];
        for (unsigned j = 0; j < 4; ++j)
            if (p[j].x < 0 || p[j].x >= 1920 || p[j].y < 0 || p[j].y >= 1080)
                goto malformed;
        if (p[0].y != p[1].y || p[2].y != p[3].y ||
            p[0].y >= p[2].y || p[0].x >= p[1].x || p[3].x >= p[2].x)
            goto malformed;
        fragments[i] = (struct lane_fragment){
            .top = (p[0].x+p[1].x)/2.0, .bottom = (p[2].x+p[3].x)/2.0,
            .x0 = p[0].x < p[3].x ? p[0].x : p[3].x,
            .x1 = p[1].x > p[2].x ? p[1].x : p[2].x,
            .y0 = p[0].y, .y1 = p[2].y,
            .width = p[1].x-p[0].x > p[2].x-p[3].x ? p[1].x-p[0].x : p[2].x-p[3].x,
            .type = polygon->line_type};
    }
    struct lane_candidate candidates[MAX_LANE_CANDIDATES];
    unsigned count = 0;
    for (unsigned i = 0; i < n; ++i) {
        if (add_candidate(candidates,&count,fragments,n,UINT64_C(1)<<i,c)) goto malformed;
        for (unsigned j = i+1; j < n; ++j) {
            const struct lane_fragment *a = &fragments[i], *b = &fragments[j];
            if (a->y0 < b->y1 && a->y1 > b->y0) continue;
            const double ay = (a->y0+a->y1)/2.0, by = (b->y0+b->y1)/2.0;
            if (magnitude(by-ay) < c->minimum_span/2.0) continue;
            const double ax = (a->top+a->bottom)/2.0, bx = (b->top+b->bottom)/2.0;
            const double slope = (bx-ax)/(by-ay);
            const double reference = ax+slope*(c->reference_y-ay);
            if (magnitude(slope) > 3.0 ||
                (reference < c->reference_x ? slope > 0.25 : slope < -0.25)) continue;
            uint64_t mask = 0;
            for (unsigned k = 0; k < n; ++k) {
                const struct lane_fragment *p = &fragments[k];
                const double local_slope = (p->bottom-p->top)/(p->y1-p->y0);
                if (magnitude(local_slope-slope) <= 0.35+2.0/(p->y1-p->y0) &&
                    magnitude(p->top-(ax+slope*(p->y0-ay))) <= c->ambiguity_px/2.0 &&
                    magnitude(p->bottom-(ax+slope*(p->y1-ay))) <= c->ambiguity_px/2.0)
                    mask |= UINT64_C(1)<<k;
            }
            if (!(mask & (UINT64_C(1)<<i)) || !(mask & (UINT64_C(1)<<j))) continue;
            if (add_candidate(candidates,&count,fragments,n,mask,c)) goto malformed;
        }
    }
    int best[2] = {-1,-1}, second[2] = {-1,-1}, anchor_blocked = 0;
    for (unsigned i = 0; i < count; ++i) {
        const struct lane_candidate *p = &candidates[i];
        int subset = 0;
        for (unsigned j = 0; j < count; ++j) {
            if (i != j && p->members != candidates[j].members &&
                (p->members & candidates[j].members) == p->members &&
                magnitude(p->x-candidates[j].x) <= c->ambiguity_px/2.0) subset = 1;
        }
        if (subset) continue;
        const int x = p->side.x, distance = abs(x-c->reference_x);
        if (distance <= (p->width+1)/2) anchor_blocked = 1;
        const unsigned side = x < c->reference_x ? 0 : 1;
        if (best[side] < 0 || distance < abs(candidates[best[side]].side.x-c->reference_x)) {
            second[side] = best[side]; best[side] = (int)i;
        } else if (second[side] < 0 ||
            distance < abs(candidates[second[side]].side.x-c->reference_x)) second[side] = (int)i;
    }
    for (unsigned side = 0; side < 2; ++side) {
        if (best[side] >= 0) {
            sides[side] = candidates[best[side]].side;
            if (abs(sides[side].x-c->reference_x) < c->ambiguity_px) {
                sides[side].line_type = ADAS_LINE_UNKNOWN;
                sides[side].reason = ADAS_LANE_TEST_GEOMETRY;
            }
        }
        if ((second[side] >= 0 &&
            abs(candidates[second[side]].side.x-c->reference_x) -
                abs(sides[side].x-c->reference_x) < c->ambiguity_px) || anchor_blocked) {
            sides[side].line_type = ADAS_LINE_UNKNOWN;
            sides[side].reason = ADAS_LANE_TEST_AMBIGUOUS;
        }
    }
    return 0;
malformed:
    memset(sides, 0, 2 * sizeof(*sides));
    sides[0].reason = sides[1].reason = ADAS_LANE_TEST_GEOMETRY;
    return -1;
}

void adas_lane_test_apply(const struct adas_lane_test_observation *o,
                          const struct adas_drive_frame *frame, uint32_t command,
                          struct adas_drive_output *out)
{
    out->required_tasks = ADAS_DECISION_LINE;
    out->permission = ADAS_DRIVE_INSUFFICIENT;
    out->action = ADAS_ACTION_UNKNOWN;
    out->alarm_active = 0;
    out->path_direction = command == ADAS_DRIVE_CHANGE_LEFT ? ADAS_LEFT : ADAS_RIGHT;
    out->reasons = ADAS_DRIVE_LINE_UNKNOWN;
    if ((command != ADAS_DRIVE_CHANGE_LEFT && command != ADAS_DRIVE_CHANGE_RIGHT) ||
        !o || !adas_lane_test_same_frame(&o->frame, frame) || !o->observed_ns ||
        o->valid_until_ns <= o->observed_ns) return;
    const struct adas_lane_test_side *side = &o->sides[command == ADAS_DRIVE_CHANGE_LEFT ? 0 : 1];
    if (side->reason != ADAS_LANE_TEST_OK) return;
    if (side->line_type == ADAS_SOLID) {
        out->permission = ADAS_DRIVE_BLOCK;
        out->action = ADAS_ACTION_KEEP_LANE;
        out->reasons = out->alarm_active = ADAS_DRIVE_SOLID;
    } else if (side->line_type == ADAS_DASHED) {
        out->permission = ADAS_DRIVE_ALLOW;
        out->action = command == ADAS_DRIVE_CHANGE_LEFT ?
            ADAS_ACTION_CHANGE_LEFT : ADAS_ACTION_CHANGE_RIGHT;
        out->reasons = 0;
    }
}
