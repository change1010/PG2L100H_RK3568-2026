#include "yolo_preprocess.h"
#include <errno.h>
#include <string.h>

int yolo_preprocess_small_rgb(const void *pixels, size_t bytes, size_t stride,
                              uint8_t *rgb, size_t rgb_bytes,
                              yolo_cancel_fn cancelled, void *opaque)
{
    if (!pixels || !rgb || bytes != 640u * 360u * 3u || stride != 1920 ||
        rgb_bytes != YOLO_INPUT_BYTES) return -EINVAL;
    if (cancelled && cancelled(opaque)) return -ECANCELED;
    size_t pad = YOLO_PAD_TOP * 1920u;
    memset(rgb, 114, pad);
    memcpy(rgb + pad, pixels, bytes);
    memset(rgb + pad + bytes, 114, rgb_bytes - pad - bytes);
    return cancelled && cancelled(opaque) ? -ECANCELED : 0;
}

int yolo_preprocess_rgb565(const void *pixels, size_t bytes, size_t stride,
                           uint8_t *rgb, size_t rgb_bytes,
                           yolo_cancel_fn cancelled, void *opaque)
{
    const uint8_t *src = pixels;
    if (!pixels || !rgb || stride != 3840 || bytes != 3840u * 1080u ||
        rgb_bytes != YOLO_INPUT_BYTES) return -EINVAL;
    if (cancelled && cancelled(opaque)) return -ECANCELED;
    memset(rgb, 114, rgb_bytes);
    /* For the exact 3:1 shrink, half-pixel bilinear samples are integer centers.
     * Expand before sampling, matching cv2 RGB565-expand -> INTER_LINEAR. */
    for (size_t y = 0; y < 360; ++y) {
        const uint8_t *row = src + (y * 3 + 1) * stride;
        uint8_t *out = rgb + (y + YOLO_PAD_TOP) * 640 * 3;
        if (cancelled && cancelled(opaque)) return -ECANCELED;
        for (size_t x = 0; x < 640; ++x) {
            size_t offset = (x * 3 + 1) * 2;
            unsigned p = row[offset] | ((unsigned)row[offset + 1] << 8);
            unsigned r = (p >> 11) & 31, g = (p >> 5) & 63, b = p & 31;
            out[x * 3] = (uint8_t)((r << 3) | (r >> 2));
            out[x * 3 + 1] = (uint8_t)((g << 2) | (g >> 4));
            out[x * 3 + 2] = (uint8_t)((b << 3) | (b >> 2));
        }
    }
    return cancelled && cancelled(opaque) ? -ECANCELED : 0;
}
