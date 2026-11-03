#include "traffic_light_color.h"
#include <errno.h>
#include <string.h>

static unsigned maximum(unsigned a, unsigned b) { return a > b ? a : b; }
static unsigned minimum(unsigned a, unsigned b) { return a < b ? a : b; }

static uint8_t pixel_color(unsigned p, uint8_t *value)
{
    int r = (int)((p >> 11) & 31), g = (int)((p >> 5) & 63), b = (int)(p & 31);
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
    int hi = (int)maximum(maximum((unsigned)r, (unsigned)g), (unsigned)b);
    int lo = (int)minimum(minimum((unsigned)r, (unsigned)g), (unsigned)b);
    int delta = hi - lo, hue;
    *value = (uint8_t)hi;
    if (hi < 100 || delta * 255 < hi * 80) return 0;
    if (hi == r) hue = 60 * (g - b) / delta;
    else if (hi == g) hue = 120 + 60 * (b - r) / delta;
    else hue = 240 + 60 * (r - g) / delta;
    if (hue < 0) hue += 360;
    if (hue <= 20 || hue >= 320) return ADAS_RED;
    if (hue >= 30 && hue <= 70) return ADAS_YELLOW;
    if (hue >= 80 && hue <= 180) return ADAS_GREEN;
    return 0;
}

static unsigned component(struct light_color_workspace *w, unsigned start,
    unsigned width, unsigned height, unsigned min_area)
{
    unsigned head = 0, tail = 1, sum = 0, color = w->labels[start];
    unsigned x0 = start % width, x1 = x0, y0 = start / width, y1 = y0;
    w->queue[0] = (uint16_t)start;
    w->labels[start] |= 0x80;
    while (head < tail) {
        unsigned index = w->queue[head++], x = index % width, y = index / width;
        x0 = minimum(x0, x); x1 = maximum(x1, x);
        y0 = minimum(y0, y); y1 = maximum(y1, y);
        sum += w->value[index];
        for (unsigned ny = y ? y - 1 : 0; ny <= minimum(y + 1, height - 1); ++ny) {
            for (unsigned nx = x ? x - 1 : 0; nx <= minimum(x + 1, width - 1); ++nx) {
                unsigned next = ny * width + nx;
                if (w->labels[next] != color) continue;
                w->labels[next] |= 0x80;
                w->queue[tail++] = (uint16_t)next;
            }
        }
    }
    unsigned bw = x1 - x0 + 1, bh = y1 - y0 + 1;
    if (tail < min_area || bw < 2 || bh < 2 ||
        maximum(bw, bh) > 6 * minimum(bw, bh) || tail * 8 < bw * bh ||
        sum < tail * 120) return 0;
    /* A bright colored patch needs darker local surroundings. Uniform paint,
     * isolated pixels and a low-saturation white bloom do not prove a lit lamp.
     */
    unsigned ring_sum = 0, ring_count = 0;
    for (unsigned y = y0 > 2 ? y0 - 2 : 0; y <= minimum(y1 + 2, height - 1); ++y) {
        for (unsigned x = x0 > 2 ? x0 - 2 : 0; x <= minimum(x1 + 2, width - 1); ++x) {
            if (x >= x0 && x <= x1 && y >= y0 && y <= y1) continue;
            unsigned index = y * width + x;
            if (w->labels[index]) continue;
            ring_sum += w->value[index]; ++ring_count;
        }
    }
    if (ring_count < 4 || (uint64_t)sum * ring_count <
        (uint64_t)ring_sum * tail + (uint64_t)24 * tail * ring_count) return 0;
    return tail;
}

int traffic_light_color_rgb565(const void *pixels, size_t bytes, size_t stride,
    uint32_t width, uint32_t height, const struct adas_box *box,
    struct light_color_workspace *work, struct light_color_result *out)
{
    if (!out) return -EINVAL;
    *out = (struct light_color_result){.reason = LIGHT_COLOR_NO_EVIDENCE};
    if (!pixels || !box || !work || !width || !height ||
        width > ADAS_WIDTH || height > ADAS_HEIGHT ||
        stride < (size_t)width * 2 || stride > SIZE_MAX / height ||
        bytes < stride * height || box->x0 < 0 || box->y0 < 0 ||
        box->x1 <= box->x0 || box->y1 <= box->y0 ||
        (uint32_t)box->x1 > width || (uint32_t)box->y1 > height) return -EINVAL;
    unsigned bw = (unsigned)(box->x1 - box->x0), bh = (unsigned)(box->y1 - box->y0);
    unsigned scale = (maximum(bw, bh) + LIGHT_COLOR_SIDE - 1) / LIGHT_COLOR_SIDE;
    unsigned sw = bw / scale, sh = bh / scale;
    if (sw < 6 || sh < 6) {
        out->reason = LIGHT_COLOR_TOO_SMALL;
        return 0;
    }
    const uint8_t *src = pixels;
    for (unsigned y = 0; y < sh; ++y) {
        unsigned sy = (unsigned)box->y0 + (2 * y + 1) * bh / (2 * sh);
        for (unsigned x = 0; x < sw; ++x) {
            unsigned sx = (unsigned)box->x0 + (2 * x + 1) * bw / (2 * sw);
            const uint8_t *p = src + sy * stride + sx * 2;
            unsigned index = y * sw + x;
            work->labels[index] = pixel_color(p[0] | ((unsigned)p[1] << 8), &work->value[index]);
        }
    }
    unsigned support[4] = {0}, colors = 0, selected = 0;
    unsigned min_area = maximum(4, (sw * sh + 99) / 100);
    for (unsigned i = 0; i < sw * sh; ++i) {
        unsigned color = work->labels[i];
        if (color && !(color & 0x80))
            support[color] += component(work, i, sw, sh, min_area);
    }
    for (unsigned color = ADAS_RED; color <= ADAS_GREEN; ++color) {
        if (support[color]) { ++colors; selected = color; }
    }
    if (colors == 1) {
        out->color = selected;
        out->reason = LIGHT_COLOR_OK;
        out->evidence_permille = support[selected] * 1000 / (sw * sh);
    } else if (colors > 1) out->reason = LIGHT_COLOR_CONFLICT;
    return 0;
}
