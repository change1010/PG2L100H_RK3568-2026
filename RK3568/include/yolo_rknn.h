#ifndef YOLO_RKNN_H
#define YOLO_RKNN_H
#include "yolo_decode.h"
#include <stdio.h>

struct yolo_rknn;
struct yolo_timing { double inputs_ms, run_ms, outputs_ms; };
/* Context has one owner: never run, close or query it concurrently. */
/* Supports decoded [1,84,N]/[1,N,84], or separate channel-first
 * [1,4,N] boxes and [1,80,N] scores. Split outputs are assembled into
 * the same caller-owned float tensor; tensor attributes choose their roles.
 * Also accepts three NCHW INT8 DFL(64)/sigmoid-score(80) head pairs at
 * widths 80/40/20 and heights derived from the compiled input shape.
 * DFL is reconstructed before the common decoder. */
int yolo_rknn_open(struct yolo_rknn **out, const char *path, FILE *diagnostics);
int yolo_rknn_infer(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
                    float *output, size_t elements, struct yolo_timing *timing);
/* DFL6 only: sparse tensor for yolo_decode at this confidence or higher.
 * Rejected rows are zero; retained rows contain only their winning class.
 * The full tensor API above remains available for numerical diagnostics. */
int yolo_rknn_infer_for_decode(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
                               float *output, size_t elements, float confidence,
                               struct yolo_timing *timing);
int yolo_rknn_channel_first(const struct yolo_rknn *ctx);
/* DFL6 only: INT8 argmax/DFL directly to common NMS, no float tensor. */
int yolo_rknn_detect(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
                     float confidence, float nms_iou, struct yolo_decode_workspace *workspace,
                     struct yolo_detections *result, struct yolo_timing *timing,
                     yolo_cancel_fn cancelled, void *opaque);
void yolo_rknn_close(struct yolo_rknn *ctx);
/* DFL6: weak class-9 observations for association only. */
int yolo_rknn_detect_lights(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
    float confidence, float light_minimum, float nms_iou,
    struct yolo_decode_workspace *workspace, struct yolo_detections *result,
    struct yolo_timing *timing, yolo_cancel_fn cancelled, void *opaque);
#endif
