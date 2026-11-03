#define _POSIX_C_SOURCE 200809L
#include "yolo_rknn.h"
#include "rknn_api.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

struct yolo_rknn {
    rknn_context handle;
    int channel_first;
    uint32_t outputs;
    size_t output_offset[6], output_elements[6];
    unsigned head_index[3][2];
    int8_t *head_data[6];
    float head_lut[6][256];
    float light_minimum;
};

static void head_coordinates(const struct yolo_rknn *ctx, unsigned reg,
                              unsigned grid, unsigned grid_h,
                              size_t cell, float *output)
{
    size_t cells = grid * grid_h;
    float stride = 640.0f / grid, distances[4];
    for (unsigned side = 0; side < 4; ++side) {
        const int8_t *data = ctx->head_data[reg] + side * 16 * cells + cell;
        int maximum = -128;
        for (unsigned bin = 0; bin < 16; ++bin)
            if (data[bin * cells] > maximum) maximum = data[bin * cells];
        float sum = 0, weighted = 0;
        for (unsigned bin = 0; bin < 16; ++bin) {
            float weight = ctx->head_lut[reg][maximum - data[bin * cells]];
            sum += weight;
            weighted += bin * weight;
        }
        distances[side] = weighted / sum;
    }
    float left = (float)(cell % grid) + .5f - distances[0];
    float top = (float)(cell / grid) + .5f - distances[1];
    float right = (float)(cell % grid) + .5f + distances[2];
    float bottom = (float)(cell / grid) + .5f + distances[3];
    output[0] = (left + right) * .5f * stride;
    output[1] = (top + bottom) * .5f * stride;
    output[2] = (right - left) * stride;
    output[3] = (bottom - top) * stride;
}

static int assemble_heads(const struct yolo_rknn *ctx, float *output, float minimum,
                          struct yolo_decode_workspace *workspace, size_t *count,
                          yolo_cancel_fn cancelled, void *opaque)
{
    size_t base = 0, threshold_count = 0, weak_count = 0;
    struct yolo_box weak[128];
    if (count) *count = 0;
    if (minimum >= 0 && output) memset(output, 0, YOLO_OUTPUT_ELEMENTS * sizeof(float));
    for (unsigned head = 0, grid = 80; head < 3; ++head, grid /= 2) {
        unsigned reg = ctx->head_index[head][0], cls = ctx->head_index[head][1];
        unsigned grid_h = YOLO_INPUT_HEIGHT / (8u << head);
        size_t cells = grid * grid_h;
        if (minimum >= 0) {
            /* Keep the all-class argmax/tie order and all above-threshold
             * candidates, including unsupported classes for overflow checks. */
            for (size_t start = 0; start < cells; start += 64) {
                if (cancelled && cancelled(opaque)) return -ECANCELED;
                size_t rows = cells - start < 64 ? cells - start : 64;
                int8_t best[64];
                uint8_t classes[64] = {0};
                memcpy(best, ctx->head_data[cls] + start, rows);
                for (unsigned channel = 1; channel < 80; ++channel) {
                    const int8_t *data = ctx->head_data[cls] + channel * cells + start;
#if defined(__ARM_NEON)
                    for (size_t row = 0; row < rows; row += 16) {
                        int8x16_t q = vld1q_s8(data + row), previous = vld1q_s8(best + row);
                        uint8x16_t greater = vcgtq_s8(q, previous);
                        vst1q_s8(best + row, vmaxq_s8(q, previous));
                        vst1q_u8(classes + row, vbslq_u8(
                            greater, vdupq_n_u8(channel), vld1q_u8(classes + row)));
                    }
#else
                    for (size_t row = 0; row < rows; ++row)
                        if (data[row] > best[row]) {
                            best[row] = data[row]; classes[row] = (uint8_t)channel;
                        }
#endif
                }
                for (size_t row = 0; row < rows; ++row) {
                    float score = ctx->head_lut[cls][best[row] + 128];
                    float threshold = workspace && classes[row] == 9 && ctx->light_minimum > 0 ?
                        fminf(minimum,ctx->light_minimum) : minimum;
                    if (score <= threshold || score > 1) continue;
                    int is_weak = workspace && classes[row] == 9 && score <= minimum;
                    if (is_weak && weak_count == 128) continue;
                    size_t cell = start + row;
                    float xywh[4];
                    head_coordinates(ctx, reg, grid, grid_h, cell, xywh);
                    if (output) {
                        output[(4u + classes[row]) * YOLO_CANDIDATE_COUNT + base + cell] = score;
                        for (unsigned side = 0; side < 4; ++side)
                            output[side * YOLO_CANDIDATE_COUNT + base + cell] = xywh[side];
                    } else {
                        if (!isfinite(xywh[0]) || !isfinite(xywh[1]) ||
                            !isfinite(xywh[2]) || !isfinite(xywh[3]) ||
                            xywh[2] <= 0 || xywh[3] <= 0) continue;
                        if (!is_weak && ++threshold_count > YOLO_MAX_CANDIDATES) return -EOVERFLOW;
                        if (!yolo_class_supported(classes[row])) continue;
                        struct yolo_box box = {xywh[0] - xywh[2] / 2,
                            xywh[1] - xywh[3] / 2, xywh[0] + xywh[2] / 2,
                            xywh[1] + xywh[3] / 2, score, classes[row], (uint32_t)(base + cell)};
                        if (isfinite(box.x0) && isfinite(box.y0) &&
                            isfinite(box.x1) && isfinite(box.y1)) {
                            if (is_weak) weak[weak_count++] = box;
                            else workspace->candidates[(*count)++] = box;
                        }
                    }
                }
            }
            base += cells;
            continue;
        }
        for (unsigned channel = 0; channel < 80; ++channel)
            for (size_t cell = 0; cell < cells; ++cell)
                output[(4 + channel) * YOLO_CANDIDATE_COUNT + base + cell] =
                    ctx->head_lut[cls][ctx->head_data[cls][channel * cells + cell] + 128];
        for (size_t cell = 0; cell < cells; ++cell) {
            float xywh[4];
            head_coordinates(ctx, reg, grid, grid_h, cell, xywh);
            for (unsigned side = 0; side < 4; ++side)
                output[side * YOLO_CANDIDATE_COUNT + base + cell] = xywh[side];
        }
        base += cells;
    }
    if (workspace && count)
        for (size_t i = 0; i < weak_count && *count < YOLO_MAX_CANDIDATES; ++i)
            workspace->candidates[(*count)++] = weak[i];
    return 0;
}

static double clock_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void print_attr(FILE *f, const char *kind, const rknn_tensor_attr *a)
{
    fprintf(f, "%s index=%u name=%.*s dims=", kind, a->index, RKNN_MAX_NAME_LEN, a->name);
    for (unsigned i = 0; i < a->n_dims && i < RKNN_MAX_DIMS; ++i)
        fprintf(f, "%s%u", i ? "," : "", a->dims[i]);
    fprintf(f, " fmt=%d type=%d qnt=%d zp=%d scale=%.9g size=%u elems=%u w_stride=%u h_stride=%u size_with_stride=%u\n",
            a->fmt, a->type, a->qnt_type, a->zp, (double)a->scale, a->size, a->n_elems,
            a->w_stride, a->h_stride, a->size_with_stride);
}

int yolo_rknn_open(struct yolo_rknn **out, const char *path, FILE *diagnostics)
{
    struct yolo_rknn *ctx;
    rknn_input_output_num count = {0};
    rknn_tensor_attr input = {0}, output[6] = {{0}};
    rknn_sdk_version version = {0};
    rknn_mem_size memory = {0};
    int ret;
    if (!out || !path) return -EINVAL;
    *out = NULL;
    ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return -ENOMEM;
    ret = rknn_init(&ctx->handle, (void *)path, 0, 0, NULL);
    if (ret) { free(ctx); return -EIO; }
    if (rknn_query(ctx->handle, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count)) ||
        count.n_input != 1 || (count.n_output != 1 && count.n_output != 2 && count.n_output != 6) ||
        rknn_query(ctx->handle, RKNN_QUERY_INPUT_ATTR, &input, sizeof(input))) {
        ret = -EPROTO; goto fail;
    }
    ctx->outputs = count.n_output;
    for (unsigned i = 0; i < ctx->outputs; ++i) {
        output[i].index = i;
        if (rknn_query(ctx->handle, RKNN_QUERY_OUTPUT_ATTR, &output[i], sizeof(output[i]))) {
            ret = -EPROTO; goto fail;
        }
    }
    if (diagnostics) {
        rknn_tensor_attr native = {0};
        print_attr(diagnostics, "input", &input);
        for (unsigned i = 0; i < ctx->outputs; ++i) print_attr(diagnostics, "output", &output[i]);
        int native_ret = rknn_query(ctx->handle, RKNN_QUERY_NATIVE_INPUT_ATTR, &native, sizeof(native));
        fprintf(diagnostics, "native_input_query=%d\n", native_ret);
        if (!native_ret) print_attr(diagnostics, "native_input", &native);
        memset(&native, 0, sizeof(native));
        native_ret = rknn_query(ctx->handle, RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR, &native, sizeof(native));
        fprintf(diagnostics, "native_nhwc_input_query=%d\n", native_ret);
        if (!native_ret) print_attr(diagnostics, "native_nhwc_input", &native);
    }
    if (input.n_dims != 4 || input.dims[0] != 1 || input.n_elems != YOLO_INPUT_BYTES ||
        !((input.fmt == RKNN_TENSOR_NCHW && input.dims[1] == 3 && input.dims[2] == YOLO_INPUT_HEIGHT && input.dims[3] == 640) ||
          (input.fmt == RKNN_TENSOR_NHWC && input.dims[1] == YOLO_INPUT_HEIGHT && input.dims[2] == 640 && input.dims[3] == 3))) {
        ret = -EPROTO; goto fail;
    }
    if (ctx->outputs == 1) {
        const rknn_tensor_attr *a = &output[0];
        if (a->n_dims != 3 || a->dims[0] != 1 || a->n_elems != YOLO_OUTPUT_ELEMENTS ||
            !((a->dims[1] == 84 && a->dims[2] == YOLO_CANDIDATE_COUNT) ||
              (a->dims[1] == YOLO_CANDIDATE_COUNT && a->dims[2] == 84))) { ret = -EPROTO; goto fail; }
        ctx->channel_first = a->dims[1] == 84;
        ctx->output_elements[0] = YOLO_OUTPUT_ELEMENTS;
    } else if (ctx->outputs == 2) {
        unsigned roles = 0;
        for (unsigned i = 0; i < ctx->outputs; ++i) {
            const rknn_tensor_attr *a = &output[i];
            unsigned role = a->dims[1] == 4 ? 1 : a->dims[1] == 80 ? 2 : 0;
            if (!role || (roles & role) || a->n_dims != 3 || a->dims[0] != 1 ||
                a->dims[2] != YOLO_CANDIDATE_COUNT || a->n_elems != a->dims[1] * YOLO_CANDIDATE_COUNT) {
                ret = -EPROTO; goto fail;
            }
            roles |= role;
            ctx->output_offset[i] = role == 1 ? 0 : 4u * YOLO_CANDIDATE_COUNT;
            ctx->output_elements[i] = a->n_elems;
        }
        ctx->channel_first = 1;
    } else {
        unsigned roles = 0;
        for (unsigned i = 0; i < 6; ++i) {
            const rknn_tensor_attr *a = &output[i];
            unsigned head = a->dims[3] == 80 ? 0 : a->dims[3] == 40 ? 1 : 2;
            unsigned role = a->dims[1] == 64 ? 0 : 1;
            unsigned bit = 1u << (head * 2 + role);
            if (a->n_dims != 4 || a->dims[0] != 1 || a->fmt != RKNN_TENSOR_NCHW ||
                (a->dims[1] != 64 && a->dims[1] != 80) ||
                (a->dims[3] != 80 && a->dims[3] != 40 && a->dims[3] != 20) ||
                a->dims[2] != YOLO_INPUT_HEIGHT / (8u << head) || a->type != RKNN_TENSOR_INT8 ||
                a->qnt_type != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC ||
                !isfinite(a->scale) || a->scale <= 0 || a->zp < -128 || a->zp > 127 ||
                a->n_elems != a->dims[1] * a->dims[2] * a->dims[3] ||
                a->size != a->n_elems || (roles & bit)) { ret = -EPROTO; goto fail; }
            roles |= bit;
            ctx->head_index[head][role] = i;
            ctx->output_elements[i] = a->n_elems;
            ctx->head_data[i] = malloc(a->n_elems);
            if (!ctx->head_data[i]) { ret = -ENOMEM; goto fail; }
            for (unsigned q = 0; q < 256; ++q) {
                float v = role ? ((int)q - 128 - a->zp) * a->scale : -(float)q * a->scale;
                ctx->head_lut[i][q] = role ? v : expf(v);
                if (!isfinite(ctx->head_lut[i][q])) { ret = -EPROTO; goto fail; }
            }
        }
        ctx->channel_first = 1;
    }
    if (diagnostics) fprintf(diagnostics, "backend=%s outputs=%u channel_first=%d\n",
                              ctx->outputs == 6 ? "yolov8_dfl6_int8" :
                              ctx->outputs == 1 ? "decoded80_single" : "decoded80_split",
                              ctx->outputs, ctx->channel_first);
    if (rknn_query(ctx->handle, RKNN_QUERY_SDK_VERSION, &version, sizeof(version))) { ret = -EIO; goto fail; }
    if (diagnostics) {
        fprintf(diagnostics, "api=%.*s driver=%.*s\n", 256, version.api_version, 256, version.drv_version);
        ret = rknn_query(ctx->handle, RKNN_QUERY_MEM_SIZE, &memory, sizeof(memory));
        fprintf(diagnostics, "mem_query=%d weights=%u internal=%u dma=%llu sram=%u free_sram=%u\n",
                ret, memory.total_weight_size, memory.total_internal_size,
                (unsigned long long)memory.total_dma_allocated_size, memory.total_sram_size, memory.free_sram_size);
    }
    *out = ctx;
    return 0;
fail:
    yolo_rknn_close(ctx);
    return ret;
}

static int infer(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
                 float *output, size_t elements, float minimum, struct yolo_timing *timing,
                 struct yolo_decode_workspace *workspace, size_t *count,
                 yolo_cancel_fn cancelled, void *opaque)
{
    rknn_input input = {0};
    rknn_output result[6] = {{0}};
    rknn_run_extend run = {0};
    double start, after_inputs, after_run, finish;
    int ret;
    if (!ctx || !rgb || bytes != YOLO_INPUT_BYTES ||
        (output ? elements != YOLO_OUTPUT_ELEMENTS :
         (!workspace || !count || ctx->outputs != 6))) return -EINVAL;
    if (cancelled && cancelled(opaque)) return -ECANCELED;
    input.buf = rgb; input.size = (uint32_t)bytes;
    input.type = RKNN_TENSOR_UINT8; input.fmt = RKNN_TENSOR_NHWC;
    input.pass_through = 0;
    for (unsigned i = 0; i < ctx->outputs; ++i) {
        result[i].index = i; result[i].want_float = ctx->outputs != 6; result[i].is_prealloc = 1;
        result[i].buf = ctx->outputs == 6 ? (void *)ctx->head_data[i] :
                        (void *)(output + ctx->output_offset[i]);
        result[i].size = (uint32_t)(ctx->output_elements[i] *
                                   (ctx->outputs == 6 ? 1 : sizeof(float)));
    }
    run.timeout_ms = 500; run.fence_fd = -1;
    start = clock_ms();
    if (rknn_inputs_set(ctx->handle, 1, &input)) return -EIO;
    after_inputs = clock_ms();
    if (rknn_run(ctx->handle, &run)) return -EIO;
    after_run = clock_ms();
    if (rknn_outputs_get(ctx->handle, ctx->outputs, result, NULL)) return -EIO;
    ret = 0;
    for (unsigned i = 0; i < ctx->outputs; ++i) {
        void *expected = ctx->outputs == 6 ? (void *)ctx->head_data[i] :
                         (void *)(output + ctx->output_offset[i]);
        if (result[i].buf != expected ||
            result[i].size != ctx->output_elements[i] *
                (ctx->outputs == 6 ? 1 : sizeof(float))) ret = -EPROTO;
    }
    if (!ret && ctx->outputs == 6)
        ret = assemble_heads(ctx, output, minimum, workspace, count, cancelled, opaque);
    finish = clock_ms();
    if (rknn_outputs_release(ctx->handle, ctx->outputs, result)) ret = -EIO;
    if (timing) {
        timing->inputs_ms = after_inputs - start; timing->run_ms = after_run - after_inputs;
        timing->outputs_ms = finish - after_run;
    }
    return ret;
}

int yolo_rknn_infer(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
                    float *output, size_t elements, struct yolo_timing *timing)
{
    if (!output) return -EINVAL;
    return infer(ctx, rgb, bytes, output, elements, -1, timing, NULL, NULL, NULL, NULL);
}

int yolo_rknn_infer_for_decode(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
                               float *output, size_t elements, float confidence,
                               struct yolo_timing *timing)
{
    if (!output || !isfinite(confidence) || confidence < 0 || confidence > 1) return -EINVAL;
    return infer(ctx, rgb, bytes, output, elements, confidence, timing, NULL, NULL, NULL, NULL);
}

int yolo_rknn_detect(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
                     float confidence, float nms_iou, struct yolo_decode_workspace *workspace,
                     struct yolo_detections *result, struct yolo_timing *timing,
                     yolo_cancel_fn cancelled, void *opaque)
{
    size_t count = 0;
    if (!result) return -EINVAL;
    result->count = 0;
    if (!isfinite(confidence) || confidence < 0 || confidence > 1 ||
        !isfinite(nms_iou) || nms_iou < 0 || nms_iou > 1) return -EINVAL;
    int ret = infer(ctx, rgb, bytes, NULL, 0, confidence, timing, workspace, &count,
                    cancelled, opaque);
    if (ret) return ret;
    return yolo_finish_candidates(workspace, count, nms_iou, result, cancelled, opaque);
}

int yolo_rknn_channel_first(const struct yolo_rknn *ctx) { return ctx ? ctx->channel_first : -1; }
int yolo_rknn_detect_lights(struct yolo_rknn *ctx, uint8_t *rgb, size_t bytes,
    float confidence, float light_minimum, float nms_iou,
    struct yolo_decode_workspace *workspace, struct yolo_detections *result,
    struct yolo_timing *timing, yolo_cancel_fn cancelled, void *opaque)
{
    if (result) result->count = 0;
    if (!ctx || ctx->outputs != 6 || !isfinite(light_minimum) ||
        light_minimum < 0 || light_minimum > confidence) return -EINVAL;
    ctx->light_minimum = light_minimum;
    int rc = yolo_rknn_detect(ctx,rgb,bytes,confidence,nms_iou,workspace,result,timing,cancelled,opaque);
    ctx->light_minimum = 0;
    return rc;
}
void yolo_rknn_close(struct yolo_rknn *ctx)
{
    if (ctx) {
        rknn_destroy(ctx->handle);
        for (unsigned i = 0; i < 6; ++i) free(ctx->head_data[i]);
        free(ctx);
    }
}
