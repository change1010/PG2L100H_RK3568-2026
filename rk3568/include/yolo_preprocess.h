#ifndef YOLO_PREPROCESS_H
#define YOLO_PREPROCESS_H
#include "yolo_decode.h"

/* Fixed production canvas contract: 1920x1080 little-endian RGB565.
 * Caller owns the input for the whole synchronous call and the output buffer. */
int yolo_preprocess_rgb565(const void *pixels, size_t bytes, size_t stride,
                           uint8_t *rgb, size_t rgb_bytes,
                           yolo_cancel_fn cancelled, void *opaque);
int yolo_preprocess_small_rgb(const void *pixels, size_t bytes, size_t stride,
                              uint8_t *rgb, size_t rgb_bytes,
                              yolo_cancel_fn cancelled, void *opaque);
#endif
