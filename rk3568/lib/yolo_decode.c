#include "yolo_decode.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

int yolo_class_supported(unsigned id)
{
    return id == 0 || id == 1 || id == 2 || id == 3 || id == 5 ||
           id == 7 || id == 9 || id == 11;
}

static float value(const float *a, int first, size_t row, size_t col)
{
    return a[first ? col * YOLO_CANDIDATE_COUNT + row : row * 84 + col];
}

/* Scan contiguous channel slices while retaining the scalar row-order argmax.
 * A 64-row tile also keeps cancellation at the original candidate boundaries. */
static void channel_scores(const float *tensor, size_t base, size_t rows,
                           float *scores, unsigned *classes, int *finite)
{
#if defined(__ARM_NEON)
    const uint32x4_t exponent = vdupq_n_u32(0x7f800000u);
    for (size_t i = 0; i < rows; i += 4) {
        vst1q_f32(scores + i, vld1q_f32(tensor + 4 * YOLO_CANDIDATE_COUNT + base + i));
        vst1q_u32(classes + i, vdupq_n_u32(0));
        vst1q_s32(finite + i, vdupq_n_s32(-1));
    }
    /* Both supported candidate counts and the tile are divisible by four. */
    for (unsigned col = 0; col < 84; ++col) {
        const float *channel = tensor + col * YOLO_CANDIDATE_COUNT + base;
        for (size_t i = 0; i < rows; i += 4) {
            float32x4_t x = vld1q_f32(channel + i);
            uint32x4_t bad = vceqq_u32(vandq_u32(vreinterpretq_u32_f32(x), exponent), exponent);
            vst1q_s32(finite + i, vbicq_s32(vld1q_s32(finite + i), vreinterpretq_s32_u32(bad)));
            if (col > 4) {
                float32x4_t previous = vld1q_f32(scores + i);
                uint32x4_t greater = vcgtq_f32(x, previous);
                vst1q_f32(scores + i, vbslq_f32(greater, x, previous));
                vst1q_u32(classes + i, vbslq_u32(greater, vdupq_n_u32(col - 4),
                                                vld1q_u32(classes + i)));
            }
        }
    }
#else
    for (size_t i = 0; i < rows; ++i) {
        scores[i] = tensor[4 * YOLO_CANDIDATE_COUNT + base + i];
        classes[i] = 0;
        finite[i] = 1;
    }
    for (unsigned col = 0; col < 84; ++col) {
        const float *channel = tensor + col * YOLO_CANDIDATE_COUNT + base;
        for (size_t i = 0; i < rows; ++i) {
            float x = channel[i];
            finite[i] &= isfinite(x) != 0;
            if (col > 4 && x > scores[i]) {
                scores[i] = x;
                classes[i] = col - 4;
            }
        }
    }
#endif
}

static int order(const void *left, const void *right)
{
    const struct yolo_box *a = left, *b = right;
    if (a->score != b->score) return a->score > b->score ? -1 : 1;
    return (a->candidate_index > b->candidate_index) -
           (a->candidate_index < b->candidate_index);
}

static float iou(const struct yolo_box *a, const struct yolo_box *b)
{
    float w = fmaxf(0, fminf(a->x1, b->x1) - fmaxf(a->x0, b->x0));
    float h = fmaxf(0, fminf(a->y1, b->y1) - fmaxf(a->y0, b->y0));
    float area_a = (a->x1 - a->x0) * (a->y1 - a->y0);
    float area_b = (b->x1 - b->x0) * (b->y1 - b->y0);
    return w * h / (area_a + area_b - w * h + 1e-9f);
}

static float clip(float x, float max)
{
    return fminf(max, fmaxf(0, x));
}

int yolo_decode(const float *tensor, size_t elements, int channel_first,
                float confidence, float nms_iou,
                struct yolo_decode_workspace *workspace,
                struct yolo_detections *result,
                yolo_cancel_fn cancelled, void *opaque)
{
    size_t count = 0, threshold_count = 0;
    if (!result) return -EINVAL;
    result->count = 0;
    if (!tensor || !workspace || elements != YOLO_OUTPUT_ELEMENTS ||
        (channel_first != 0 && channel_first != 1) ||
        !isfinite(confidence) || confidence < 0 || confidence > 1 ||
        !isfinite(nms_iou) || nms_iou < 0 || nms_iou > 1) return -EINVAL;
    for (size_t base = 0; base < YOLO_CANDIDATE_COUNT; base += 64) {
        float scores[64];
        unsigned classes[64];
        int valid[64];
        size_t rows = YOLO_CANDIDATE_COUNT - base < 64 ? YOLO_CANDIDATE_COUNT - base : 64;
        if (cancelled && cancelled(opaque)) return -ECANCELED;
        if (channel_first) channel_scores(tensor, base, rows, scores, classes, valid);
        for (size_t k = 0; k < rows; ++k) {
            size_t i = base + k;
            float row[84], score;
            unsigned id = 0;
            int finite = 1;
            for (size_t j = 0; j < (channel_first ? 4u : 84u); ++j) {
                row[j] = value(tensor, channel_first, i, j);
                finite &= isfinite(row[j]) != 0;
            }
            if (channel_first) finite = valid[k];
            if (!finite || row[2] <= 0 || row[3] <= 0) continue;
            if (channel_first) {
                score = scores[k]; id = classes[k];
            } else {
                score = row[4];
                for (unsigned j = 1; j < 80; ++j) {
                    if (row[j + 4] > score) { score = row[j + 4]; id = j; }
                }
            }
            if (score <= confidence || score > 1) continue;
            if (++threshold_count > YOLO_MAX_CANDIDATES) return -EOVERFLOW;
            if (!yolo_class_supported(id)) continue;
            struct yolo_box box = {row[0] - row[2] / 2, row[1] - row[3] / 2,
                                   row[0] + row[2] / 2, row[1] + row[3] / 2,
                                   score, id, (uint32_t)i};
            if (!isfinite(box.x0) || !isfinite(box.y0) ||
                !isfinite(box.x1) || !isfinite(box.y1)) continue;
            workspace->candidates[count++] = box;
        }
    }
    return yolo_finish_candidates(workspace, count, nms_iou, result, cancelled, opaque);
}

int yolo_finish_candidates(struct yolo_decode_workspace *workspace, size_t count,
                           float nms_iou, struct yolo_detections *result,
                           yolo_cancel_fn cancelled, void *opaque)
{
    size_t kept = 0;
    if (!result) return -EINVAL;
    result->count = 0;
    if (!workspace || count > YOLO_MAX_CANDIDATES || !isfinite(nms_iou) ||
        nms_iou < 0 || nms_iou > 1) return -EINVAL;
    qsort(workspace->candidates, count, sizeof(struct yolo_box), order);
    /* Keep model-space boxes through NMS; clip and inverse-letterbox only once. */
    for (size_t i = 0; i < count; ++i) {
        struct yolo_box box = workspace->candidates[i];
        int suppressed = 0;
        if (cancelled && cancelled(opaque)) return -ECANCELED;
        for (size_t j = 0; j < kept; ++j) {
            if (box.class_id == result->boxes[j].class_id &&
                iou(&box, &result->boxes[j]) > nms_iou) {
                suppressed = 1;
                break;
            }
        }
        if (!suppressed) result->boxes[kept++] = box;
    }
    count = 0;
    for (size_t i = 0; i < kept; ++i) {
        struct yolo_box box = result->boxes[i];
        box.x0 = clip(floorf(box.x0 * 3), 1920);
        box.y0 = clip(floorf((box.y0 - YOLO_PAD_TOP) * 3), 1080);
        box.x1 = clip(ceilf(box.x1 * 3), 1920);
        box.y1 = clip(ceilf((box.y1 - YOLO_PAD_TOP) * 3), 1080);
        if (box.x1 > box.x0 && box.y1 > box.y0) result->boxes[count++] = box;
    }
    if (cancelled && cancelled(opaque)) return -ECANCELED;
    result->count = count;
    return 0;
}
