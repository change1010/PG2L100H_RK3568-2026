#ifndef YOLO_DECODE_H
#define YOLO_DECODE_H
#include <stddef.h>
#include <stdint.h>

#define YOLO_INPUT_WIDTH 640
#ifndef YOLO_INPUT_HEIGHT
#define YOLO_INPUT_HEIGHT 384
#endif
#if YOLO_INPUT_HEIGHT != 384 && YOLO_INPUT_HEIGHT != 640
#error "This isolated experiment supports only 640x384 and its 640x640 baseline"
#endif
#define YOLO_INPUT_BYTES (640u * YOLO_INPUT_HEIGHT * 3u)
#define YOLO_CANDIDATE_COUNT (80u * (YOLO_INPUT_HEIGHT / 8u) + \
                             40u * (YOLO_INPUT_HEIGHT / 16u) + \
                             20u * (YOLO_INPUT_HEIGHT / 32u))
#define YOLO_PAD_TOP ((YOLO_INPUT_HEIGHT - 360u) / 2u)
#define YOLO_OUTPUT_ELEMENTS (84u * YOLO_CANDIDATE_COUNT)
#define YOLO_MAX_CANDIDATES 1000

typedef int (*yolo_cancel_fn)(void *opaque);
int yolo_class_supported(unsigned id);

struct yolo_box {
    float x0, y0, x1, y1, score;
    uint32_t class_id, candidate_index;
};

struct yolo_detections {
    size_t count;
    struct yolo_box boxes[YOLO_MAX_CANDIDATES];
};

struct yolo_decode_workspace {
    struct yolo_box candidates[YOLO_MAX_CANDIDATES];
};
int yolo_finish_candidates(struct yolo_decode_workspace *workspace, size_t count,
                           float nms_iou, struct yolo_detections *result,
                           yolo_cancel_fn cancelled, void *opaque);

/* channel_first selects [1,84,N], otherwise [1,N,84].
 * Results are clipped integer-valued canvas coordinates, not tracking IDs.
 * Returns a negative errno; failures never expose partial detections. */
int yolo_decode(const float *tensor, size_t elements, int channel_first,
                float confidence, float nms_iou,
                struct yolo_decode_workspace *workspace,
                struct yolo_detections *result,
                yolo_cancel_fn cancelled, void *opaque);
#endif
