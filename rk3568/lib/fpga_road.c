#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "fpga_road.h"

#define VIDEO_HISTORY 128U
#define RESULT_HISTORY 32U
struct binding {
    int valid;
    struct vdma_frame_identity display;
    struct vdma_native_identity native;
    uint64_t observed;
};
struct pending_result {
    int valid;
    uint64_t received, order;
    struct vdma_fpga_road packet;
};
struct fpga_road {
    struct adas_store *store;
    struct binding video[VIDEO_HISTORY];
    struct pending_result pending[RESULT_HISTORY];
    unsigned int video_next, pending_next;
    uint64_t next_order;
    struct adas_road candidate;
    uint64_t published, unmatched, rejected;
    uint64_t received;
    uint32_t dropped, last_epoch, last_sequence;
};

static uint32_t word(const struct vdma_fpga_road *p, unsigned int i)
{
    const unsigned char *b = (const unsigned char *)&p->record[i];
    return (uint32_t)b[0] | (uint32_t)b[1] << 8 |
        (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
}

static uint32_t bits(const struct vdma_fpga_road *p, unsigned int start, unsigned int n)
{
    uint32_t value = word(p, 8 + start / 32) >> (start % 32);
    if (start % 32 + n > 32)
        value |= word(p, 9 + start / 32) << (32 - start % 32);
    return value & ((UINT32_C(1) << n) - 1);
}

static void lane_set(struct adas_lane *lane, int x0, int x1, int y0, int y1, uint32_t dashed)
{
    memset(lane, 0, sizeof(*lane));
    lane->state = ADAS_VALID; lane->reason = ADAS_REASON_NONE;
    lane->line_type = dashed ? ADAS_DASHED : ADAS_SOLID;
    lane->evidence_present = 1; lane->evidence_permille = 500;
    lane->point_count = 2;
    lane->points[0] = (struct adas_point){x0, y0};
    lane->points[1] = (struct adas_point){x1, y1};
}

struct adas_producer fpga_road_registration(void)
{
    return (struct adas_producer){.enabled = 1, .origin = ADAS_ROAD_FPGA,
        .producer_session = 1, .algorithm_id = FPGA_ROAD_MAGIC,
        .algorithm_version = 1, .algorithm_config_generation = 1,
        .road_supported_tasks = ADAS_ROAD_LEFT | ADAS_ROAD_RIGHT | ADAS_ROAD_ZEBRA};
}

/* Keep the prepared header and UNKNOWN ego-side association. A detector slot
 * is a measured paint region; it does not establish a lane's ego association. */
int fpga_road_decode(const struct vdma_fpga_road *p, struct adas_road *out)
{
    if (!p || !out || p->version != 1 || p->size != sizeof(*p) || p->reserved ||
        word(p, 0) != FPGA_ROAD_MAGIC || word(p, 1) != 1 ||
        word(p, 6) != 2 || word(p, 7) != 3 || bits(p, 1008, 16) != 0x4c32)
        return -1;
    for (unsigned int i = 44; i < 48; ++i) if (word(p, i)) return -1;
    memset(&out->reference, 0, sizeof(out->reference));
    memset(out->zebras, 0, sizeof(out->zebras));
    out->header.state = ADAS_VALID; out->header.reason = ADAS_REASON_NONE;
    out->reference.present = 1;
    out->supported_tasks = ADAS_ROAD_LEFT | ADAS_ROAD_RIGHT | ADAS_ROAD_ZEBRA;
    out->left.state = ADAS_ABSENT; out->left.reason = ADAS_REASON_NONE;
    out->right.state = ADAS_ABSENT; out->right.reason = ADAS_REASON_NONE;
    out->zebras_meta = (struct adas_collection){.state = ADAS_ABSENT,
        .reason = ADAS_REASON_NONE, .total_count_known = 1};
    for (unsigned int i = 0; i < 8; ++i) {
        unsigned int base = 256 + i * 80;
        if (!bits(p, base + 79, 1)) continue;
        int x0 = (int)bits(p, base + 66, 12), x1 = (int)bits(p, base + 54, 12);
        int bottom0 = (int)bits(p, base + 42, 12), bottom1 = (int)bits(p, base + 30, 12);
        int y0 = (int)bits(p, base + 18, 12), y1 = (int)bits(p, base + 6, 12);
        const int top0 = x0, top1 = x1;
        if (top0 >= top1 || bottom0 >= bottom1)
            return -1;
        if (bottom0 < x0) x0 = bottom0;
        if (bottom1 > x1) x1 = bottom1;
        if (x0 < 0 || x1 >= 1920 || y0 < 0 || y1 >= 1080 || x0 >= x1 || y0 >= y1)
            return -1;
        struct road_ref_geometry *g = &out->reference;
        struct road_ref_polygon *polygon = &g->confirmed[g->confirmed_count++];
        *polygon = (struct road_ref_polygon){.line_type = bits(p, base + 78, 1) ? ADAS_DASHED : ADAS_SOLID,
            .first_point = g->point_count, .point_count = 4,
            .area_px = 0.5 * (top1 - top0 + bottom1 - bottom0) * (y1 - y0),
            .visible_span_px = y1 - y0};
        g->points[g->point_count++] = (struct road_ref_point){top0, y0};
        g->points[g->point_count++] = (struct road_ref_point){top1, y0};
        g->points[g->point_count++] = (struct road_ref_point){bottom1, y1};
        g->points[g->point_count++] = (struct road_ref_point){bottom0, y1};
        struct adas_lane *lane = ((x0 + x1) / 2 < 960) ? &out->left : &out->right;
        if (lane->state == ADAS_ABSENT)
            lane_set(lane, x0, x1, y0, y1, bits(p, base + 78, 1));
    }
    out->reference.confirmed_observed = out->reference.confirmed_count;
    if (word(p, 43) & 1) {
        uint32_t xx = word(p, 42), yy = word(p, 41);
        struct adas_box b = {(int)(xx & 0xffff), (int)(yy & 0xffff),
            (int)(xx >> 16), (int)(yy >> 16)};
        uint32_t stripes = (word(p, 43) >> 8) & 0xff;
        if (b.x0 >= b.x1 || b.y0 >= b.y1 || b.x1 > 1920 || b.y1 > 1080 || stripes < 2)
            return -1;
        out->zebras_meta.state = ADAS_VALID;
        out->zebras_meta.count = out->zebras_meta.observed_count = 1;
        out->zebras_meta.total_count_known = 1;
        out->zebras[0] = (struct adas_zebra){.zebra_id = 1, .box = b,
            .stripe_count = stripes, .origin = ADAS_ROAD_FPGA};
    }
    return 0;
}

struct fpga_road *fpga_road_create(struct adas_store *store)
{
    struct fpga_road *r = calloc(1, sizeof(*r));
    if (r) r->store = store;
    return r;
}
void fpga_road_destroy(struct fpga_road *r) { free(r); }
void fpga_road_get_stats(const struct fpga_road *r, struct fpga_road_stats *s)
{
    *s = (struct fpga_road_stats){r->received, r->published, r->unmatched,
        r->rejected, r->dropped, r->last_epoch, r->last_sequence};
}
void fpga_road_clear(struct fpga_road *r)
{
    if (!r) return;
    memset(r->video, 0, sizeof(r->video));
    memset(r->pending, 0, sizeof(r->pending));
}

int fpga_road_bind(struct fpga_road *r, const struct vdma_frame_identity *d,
    const struct vdma_native_identity *n, uint64_t observed)
{
    if (!r || !d || !n || !observed || d->source_id != 2 || d->layout != 2 ||
        d->canvas_width != 1920 || d->canvas_height != 1080 ||
        n->version != 1 || n->size != sizeof(*n) || n->slot != d->slot || n->token != d->token)
        return -1;
    for (unsigned int i = 0; i < VIDEO_HISTORY; ++i) {
        const struct binding *old = &r->video[i];
        if (old->valid && (old->display.stream_epoch != d->stream_epoch ||
            old->display.source_id != d->source_id || old->display.layout != d->layout ||
            old->display.config_generation != d->config_generation)) {
            fpga_road_clear(r);
            break;
        }
    }
    struct binding *b = &r->video[r->video_next++ % VIDEO_HISTORY];
    *b = (struct binding){.valid = 1, .display = *d, .native = *n, .observed = observed};
    return 0;
}

int fpga_road_accept(struct fpga_road *r, const struct vdma_fpga_road *p, uint64_t now)
{
    if (!r || !p || !now || fpga_road_decode(p, &r->candidate)) {
        errno = EPROTO; return -1;
    }
    r->received++;
    r->dropped = p->dropped;
    r->last_epoch = word(p, 2); r->last_sequence = word(p, 3);
    struct pending_result *v = &r->pending[r->pending_next++ % RESULT_HISTORY];
    if (v->valid) ++r->unmatched;
    *v = (struct pending_result){.valid = 1, .received = now,
        .order = ++r->next_order, .packet = *p};
    return 0;
}

int fpga_road_flush(struct fpga_road *r, uint64_t now)
{
    if (!r || !now) return -1;
    for (;;) {
        struct pending_result *p = NULL;
        struct binding *matched = NULL;
        /*
         * Do not let an old result with no display binding block a newer
         * result that already has an exact native/display join. Unmatched
         * entries remain bounded by RESULT_HISTORY and are retired by TTL.
         */
        for (unsigned int k = 0; k < RESULT_HISTORY; ++k) {
            struct pending_result *candidate = &r->pending[k];
            if (!candidate->valid) continue;
            if (now < candidate->received ||
                now - candidate->received >= ADAS_ROAD_TTL_NS) {
                candidate->valid = 0; ++r->unmatched; continue;
            }
            struct binding *candidate_binding = NULL;
            for (unsigned int j = 0; j < VIDEO_HISTORY; ++j) {
                struct binding *b = &r->video[j];
                if (!b->valid || b->native.native_epoch != word(&candidate->packet, 2) ||
                    b->native.native_sequence != word(&candidate->packet, 3) ||
                    b->native.timestamp_lo != word(&candidate->packet, 4) ||
                    b->native.timestamp_hi != word(&candidate->packet, 5)) continue;
                candidate_binding = b;
                break;
            }
            if (candidate_binding &&
                (!p || candidate->order < p->order)) {
                p = candidate;
                matched = candidate_binding;
            }
        }
        if (!matched) break;
        {
            struct adas_identity id = {matched->display.stream_epoch, matched->display.frame_sequence,
                matched->display.source_id, matched->display.layout, matched->display.config_generation, 1920, 1080};
            struct adas_token token;
            int rc = adas_get_token(r->store, ADAS_ROAD, &id, matched->observed, now, &token);
            if (rc == ADAS_BUSY) return 0;
            if (rc == ADAS_OK_NEW) rc = adas_prepare_road(r->store, &token, &r->candidate);
            if (rc == ADAS_BUSY) return 0;
            if (rc == ADAS_OK_NEW) {
                if (fpga_road_decode(&p->packet, &r->candidate)) return -1;
                r->candidate.header.received_ns = p->received > matched->observed ? p->received : matched->observed;
                r->candidate.header.finished_ns = now;
                rc = adas_publish_road(r->store, &token, &r->candidate, now);
            }
            if (rc == ADAS_BUSY) return 0;
            if (rc == ADAS_OK_NEW) ++r->published; else ++r->rejected;
            p->valid = 0;
            matched->valid = 0;
        }
    }
    return 0;
}

