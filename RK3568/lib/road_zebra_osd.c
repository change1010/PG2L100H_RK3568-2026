#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "road_zebra_osd.h"

static void fill_span(uint32_t *row, uint32_t x0, uint32_t x1, uint32_t color)
{
    for (uint32_t x = x0; x < x1; x++) row[x] = color;
}

static void draw_row_band(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height,
                          uint32_t x0, uint32_t x1, uint32_t y0, uint32_t y1, uint32_t color)
{
    if (y1 > height) y1 = height;
    if (x1 > width) x1 = width;
    for (uint32_t y = y0; y < y1; y++) fill_span(argb + (size_t)y * pitch, x0, x1, color);
}

static void draw_col_band(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height,
                          uint32_t x0, uint32_t x1, uint32_t y0, uint32_t y1, uint32_t color)
{
    if (y1 > height) y1 = height;
    if (x1 > width) x1 = width;
    for (uint32_t y = y0; y < y1; y++) {
        uint32_t *row = argb + (size_t)y * pitch;
        fill_span(row, x0, x1, color);
    }
}

void road_zebra_osd_clear(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height)
{
    if (!argb || pitch < width || !width || !height ||
        (size_t)height > SIZE_MAX / pitch / sizeof(uint32_t)) return;
    if (pitch == width) { memset(argb, 0, (size_t)width * height * sizeof(uint32_t)); return; }
    for (uint32_t y = 0; y < height; y++) memset(argb + (size_t)y * pitch, 0, (size_t)width * sizeof(uint32_t));
}

/* Outline: ink band then colour band, so the frame stays readable on a bright
 * road surface without covering the stripes being detected. */
static void draw_box(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height,
                     int32_t bx0, int32_t by0, int32_t bx1, int32_t by1, uint32_t thickness)
{
    if (bx1 <= 0 || by1 <= 0 || bx0 >= (int32_t)width || by0 >= (int32_t)height) return;
    uint32_t x0 = bx0 < 0 ? 0U : (uint32_t)bx0;
    uint32_t y0 = by0 < 0 ? 0U : (uint32_t)by0;
    uint32_t x1 = (uint32_t)bx1 > width ? width : (uint32_t)bx1;
    uint32_t y1 = (uint32_t)by1 > height ? height : (uint32_t)by1;
    if (x1 <= x0 || y1 <= y0) return;
    if (thickness < 1U) thickness = 1U;
    if (thickness * 2U >= x1 - x0) thickness = (x1 - x0) / 2U;
    if (thickness * 2U >= y1 - y0) thickness = (y1 - y0) / 2U;
    if (!thickness) return;
    uint32_t ink = thickness + 1U < 6U ? thickness + 1U : 6U;
    /* Ink first: 1 px wider on every side, then the visible colour band. */
    draw_row_band(argb, pitch, width, height, x0, x1, y0, y0 + ink, ROAD_ZEBRA_OSD_INK_COLOR);
    draw_row_band(argb, pitch, width, height, x0, x1, y1 - ink, y1, ROAD_ZEBRA_OSD_INK_COLOR);
    draw_col_band(argb, pitch, width, height, x0, x0 + ink, y0, y1, ROAD_ZEBRA_OSD_INK_COLOR);
    draw_col_band(argb, pitch, width, height, x1 - ink, x1, y0, y1, ROAD_ZEBRA_OSD_INK_COLOR);
    draw_row_band(argb, pitch, width, height, x0 + 1U, x1 - 1U, y0 + 1U, y0 + thickness, ROAD_ZEBRA_OSD_BOX_COLOR);
    draw_row_band(argb, pitch, width, height, x0 + 1U, x1 - 1U, y1 - thickness, y1 - 1U, ROAD_ZEBRA_OSD_BOX_COLOR);
    draw_col_band(argb, pitch, width, height, x0 + 1U, x0 + thickness, y0 + 1U, y1 - 1U, ROAD_ZEBRA_OSD_BOX_COLOR);
    draw_col_band(argb, pitch, width, height, x1 - thickness, x1 - 1U, y0 + 1U, y1 - 1U, ROAD_ZEBRA_OSD_BOX_COLOR);
}

void road_zebra_osd_geometry(struct road_overlay_geometry *out, const struct adas_road *road)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!road || road->header.state != ADAS_VALID) return;
    out->valid = 1;
    out->stream_epoch = road->header.identity.stream_epoch;
    out->source_id = road->header.identity.source_id;
    out->layout = road->header.identity.layout;
    out->config_generation = road->header.identity.config_generation;
    out->frame_sequence = road->header.identity.frame_sequence;
    out->observed_ns = road->header.observed_ns;
    out->finished_ns = road->header.finished_ns;
    out->reference = road->reference;
    if (road->header.observed_ns <= UINT64_MAX - road->header.ttl_ns)
        out->valid_until_ns = road->header.observed_ns + road->header.ttl_ns;
    const struct adas_lane *lanes[] = {&road->left, &road->right};
    for (unsigned int s = 0; s < 2; s++) {
        if (lanes[s]->state != ADAS_VALID) continue;
        uint32_t count = lanes[s]->point_count;
        if (count > ROAD_OVERLAY_MAX_POINTS) count = ROAD_OVERLAY_MAX_POINTS;
        out->lanes[s].count = count;
        memcpy(out->lanes[s].measured_connection, lanes[s]->measured_connection,
               sizeof(out->lanes[s].measured_connection));
        for (uint32_t i = 0; i < count; i++) {
            out->lanes[s].points[i].x = lanes[s]->points[i].x;
            out->lanes[s].points[i].y = lanes[s]->points[i].y;
        }
    }
    if (road->zebras_meta.state != ADAS_VALID) return;
    out->zebra_count = road->zebras_meta.count;
    if (out->zebra_count > ROAD_OVERLAY_MAX_ZEBRAS) out->zebra_count = ROAD_OVERLAY_MAX_ZEBRAS;
    for (uint32_t i = 0; i < out->zebra_count; i++) {
        const struct adas_box *b = &road->zebras[i].box;
        out->zebras[i] = (struct road_overlay_box){b->x0, b->y0, b->x1, b->y1};
    }
}

static int point_valid(const struct road_overlay_point *p, uint32_t width, uint32_t height)
{
    return p->x >= 0 && p->y >= 0 && (uint32_t)p->x < width && (uint32_t)p->y < height;
}

static void draw_dot(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height,
                      int32_t x, int32_t y, uint32_t thickness, uint32_t color)
{
    int32_t radius = (int32_t)thickness / 2;
    for (int64_t yy = (int64_t)y - radius; yy <= (int64_t)y + radius; yy++)
        if (yy >= 0 && (uint32_t)yy < height)
            for (int64_t xx = (int64_t)x - radius; xx <= (int64_t)x + radius; xx++)
                if (xx >= 0 && (uint32_t)xx < width) argb[(size_t)yy * pitch + (uint32_t)xx] = color;
}

static int draw_lane(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height,
                       const struct road_overlay_lane *lane, uint32_t thickness, uint32_t color)
{
    int drawn = 0;
    for (uint32_t i = 0; i < lane->count; i++) {
        const struct road_overlay_point *p = &lane->points[i];
        if (!point_valid(p, width, height)) continue;
        draw_dot(argb, pitch, width, height, p->x, p->y, thickness, color); drawn = 1;
        if (!i) continue;
        const struct road_overlay_point *a = &lane->points[i - 1];
        if (!point_valid(a, width, height)) continue;
        int32_t dy = p->y - a->y, dx = p->x - a->x;
        /* Decimation can omit measured rows inside one contiguous fragment.
         * Only explicit support metadata permits that connection; real dash
         * gaps and occlusions still have no connection bit. */
        int measured = !!(lane->measured_connection[i / 32U] & (UINT32_C(1) << (i % 32U)));
        int32_t max_dy = measured ? 18 : 3;
        int32_t max_dx = measured ? 3 * dy + 3 : 12;
        if (dy <= 0 || dy > max_dy || dx < -max_dx || dx > max_dx) continue;
        int32_t steps = dx < 0 ? -dx : dx;
        if (steps < dy) steps = dy;
        for (int32_t k = 1; k < steps; k++)
            draw_dot(argb, pitch, width, height, a->x + dx * k / steps,
                     a->y + dy * k / steps, thickness, color);
    }
    return drawn;
}

int road_zebra_osd_draw(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height,
                        const struct adas_road *road, uint32_t thickness)
{
    if (!argb || pitch < width || !width || !height || width > INT32_MAX || height > INT32_MAX ||
        (size_t)height > SIZE_MAX / pitch / sizeof(uint32_t)) { errno = EINVAL; return -1; }
    road_zebra_osd_clear(argb, pitch, width, height);
    struct road_overlay_geometry geometry;
    road_zebra_osd_geometry(&geometry, road);
    if (!geometry.valid) return 0;
    if (thickness < 1) thickness = 1;
    if (thickness > 6) thickness = 6;
    uint32_t drawn = (uint32_t)draw_lane(argb, pitch, width, height, &geometry.lanes[0], thickness, ROAD_OVERLAY_LEFT_COLOR);
    drawn += (uint32_t)draw_lane(argb, pitch, width, height, &geometry.lanes[1], thickness, ROAD_OVERLAY_RIGHT_COLOR);
    for (uint32_t i = 0; i < geometry.zebra_count; i++) {
        const struct road_overlay_box *box = &geometry.zebras[i];
        if (box->x1 <= box->x0 || box->y1 <= box->y0) continue;
        draw_box(argb, pitch, width, height, box->x0, box->y0, box->x1, box->y1, thickness);
        drawn++;
    }
    return (int)drawn;
}

int road_zebra_osd_dump(const uint32_t *argb, uint32_t pitch, uint32_t height, const char *path)
{
    if (!argb || !pitch || !height || !path) { errno = EINVAL; return -1; }
    FILE *file = fopen(path, "wb");
    if (!file) return -1;
    int result = 0;
    for (uint32_t y = 0; y < height; y++)
        if (fwrite(argb + (size_t)y * pitch, sizeof(uint32_t), pitch, file) != pitch) { result = -1; break; }
    if (fclose(file)) result = -1;
    return result;
}
