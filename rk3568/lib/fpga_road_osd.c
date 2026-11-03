#include <string.h>
#include "fpga_road_osd.h"
#include "adas_result.h"

static int same_frame(const struct road_overlay_geometry *r,
    const struct vdma_frame_identity *f)
{
    return r && r->valid && r->stream_epoch == f->stream_epoch &&
        r->source_id == f->source_id && r->layout == f->layout &&
        r->config_generation == f->config_generation && r->frame_sequence == f->frame_sequence;
}

static int same_detection(const struct detection_overlay *d,
    const struct vdma_frame_identity *f)
{
    return d && d->valid && d->stream_epoch == f->stream_epoch &&
        d->source_id == f->source_id && d->layout == f->layout &&
        d->config_generation == f->config_generation && d->frame_sequence == f->frame_sequence;
}

static void pixel(uint16_t *p, uint32_t stride, uint32_t w, uint32_t h,
                  int32_t x, int32_t y, uint16_t color)
{
    if (x >= 0 && y >= 0 && (uint32_t)x < w && (uint32_t)y < h)
        p[(size_t)y * stride + (uint32_t)x] = color;
}

static int box(uint16_t *p, uint32_t stride, uint32_t w, uint32_t h,
    int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > (int32_t)w) x1 = (int32_t)w;
    if (y1 > (int32_t)h) y1 = (int32_t)h;
    if (x1 - x0 < 4 || y1 - y0 < 4) return 0;
    for (int32_t x = x0; x < x1; ++x) {
        pixel(p,stride,w,h,x,y0,color); pixel(p,stride,w,h,x,y0+1,color);
        pixel(p,stride,w,h,x,y1-1,color); pixel(p,stride,w,h,x,y1-2,color);
    }
    for (int32_t y = y0; y < y1; ++y) {
        pixel(p,stride,w,h,x0,y,color); pixel(p,stride,w,h,x0+1,y,color);
        pixel(p,stride,w,h,x1-1,y,color); pixel(p,stride,w,h,x1-2,y,color);
    }
    return 1;
}

unsigned int fpga_road_osd_rgb565(uint16_t *pixels, uint32_t stride, uint32_t width,
    uint32_t height, const struct vdma_frame_identity *id,
    const struct road_overlay_geometry *road, const struct detection_overlay *detection)
{
    if (!pixels || !id || !width || !height || stride < width ||
        (size_t)height > SIZE_MAX / stride / sizeof(*pixels)) return 0;
    unsigned int painted = 0;
    if (same_frame(road,id)) {
        const struct road_ref_geometry *g = &road->reference;
        for (uint32_t i = 0; g->present && i < g->confirmed_count &&
             i < ROAD_REF_MAX_SEGMENTS; ++i) {
            const struct road_ref_polygon *r = &g->confirmed[i];
            if (r->point_count < 2 || r->first_point >= g->point_count ||
                g->point_count > ROAD_REF_MAX_POINTS ||
                r->point_count > g->point_count - r->first_point) continue;
            int32_t x0 = (int32_t)width, y0 = (int32_t)height, x1 = -1, y1 = -1;
            for (uint32_t j = 0; j < r->point_count; ++j) {
                const struct road_ref_point *p = &g->points[r->first_point+j];
                if (p->x < x0) x0 = p->x;
                if (p->x > x1) x1 = p->x;
                if (p->y < y0) y0 = p->y;
                if (p->y > y1) y1 = p->y;
            }
            if (x1 < 0 || y1 < 0 || x1 >= (int32_t)width || y1 >= (int32_t)height)
                continue;
            const uint16_t color = r->line_type == ADAS_DASHED ? UINT16_C(0x07e0) :
                r->line_type == ADAS_SOLID ? UINT16_C(0xffe0) : UINT16_C(0x8410);
            painted |= box(pixels,stride,width,height,x0,y0,x1+1,y1+1,color);
        }
        for (unsigned int side = 0; !g->present && side < 2; ++side) {
            const struct road_overlay_lane *lane = &road->lanes[side];
            if (lane->count < 2) continue;
            int32_t x0 = (int32_t)width, y0 = (int32_t)height, x1 = 0, y1 = 0;
            uint32_t count = lane->count < ROAD_OVERLAY_MAX_POINTS ?
                lane->count : ROAD_OVERLAY_MAX_POINTS;
            for (uint32_t i = 0; i < count; ++i) {
                const int32_t x = lane->points[i].x, y = lane->points[i].y;
                if (x < 0 || y < 0 || x >= (int32_t)width || y >= (int32_t)height) continue;
                if (x < x0) x0 = x;
                if (x > x1) x1 = x;
                if (y < y0) y0 = y;
                if (y > y1) y1 = y;
            }
            if (x1 > x0 && y1 > y0)
                painted |= box(pixels,stride,width,height,x0,y0,x1+1,y1+1,
                    side ? UINT16_C(0xffe0) : UINT16_C(0x07e0));
        }
        for (uint32_t i = 0; i < road->zebra_count && i < ROAD_OVERLAY_MAX_ZEBRAS; ++i) {
            const struct road_overlay_box *b = &road->zebras[i];
            painted |= box(pixels,stride,width,height,b->x0,b->y0,b->x1,b->y1,
                           UINT16_C(0xffe0));
        }
    }
    if (same_detection(detection,id)) {
        uint32_t count = detection->count < DETECTION_OVERLAY_CAPACITY ?
            detection->count : DETECTION_OVERLAY_CAPACITY;
        for (uint32_t i = 0; i < count; ++i) {
            const struct detection_overlay_box *b = &detection->boxes[i];
            if (b->display_held || (detection->light_temporal_valid &&
                (detection->light_flags[i] & (LIGHT_VISUAL_PREDICTED | LIGHT_VISUAL_PENDING))))
                continue;
            painted |= 2U * box(pixels,stride,width,height,b->x0,b->y0,b->x1,b->y1,
                                UINT16_C(0xf800));
        }
    }
    return painted;
}
