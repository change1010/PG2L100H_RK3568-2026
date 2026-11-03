#define _POSIX_C_SOURCE 200809L
#include "yolo_adas.h"
#include "traffic_light_color.h"
#include "light_temporal.h"
#include "yolo_rknn.h"
#include "yolo_preprocess.h"
#include "video_dma_frame.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct yolo_adas {
    struct adas_store *store;
    FILE *diagnostics;
    struct yolo_rknn *rknn;
    uint8_t *rgb;
    float confidence, nms_iou;
    struct yolo_decode_workspace workspace;
    struct yolo_detections boxes;
    struct yolo_detections strong_boxes;
    struct light_temporal temporal;
    struct light_color_workspace light_color;
    pthread_mutex_t mutex;
    struct yolo_adas_stats stats;
    unsigned consecutive_failures;
    int main_light_cpu;
};

static void store_retry_pause(void)
{
    /* Let the short-lived owner lock finish before retrying this same result. */
    const struct timespec delay = {0, 100000};
    nanosleep(&delay, NULL);
}

static uint64_t now_ns(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * UINT64_C(1000000000) + (uint64_t)ts.tv_nsec;
}
static int cancelled(void *opaque)
{
    const struct vdma_snapshot_input *input = opaque;
    return input->cancelled && input->cancelled(input->cancel_context);
}

struct yolo_adas *yolo_adas_create(struct adas_store *store, const char *model,
                                   float confidence, float nms_iou, FILE *diagnostics)
{
    if (!store || !model || !isfinite(confidence) || confidence < 0 || confidence > 1 ||
        !isfinite(nms_iou) || nms_iou < 0 || nms_iou > 1) { errno = EINVAL; return NULL; }
    struct yolo_adas *p = calloc(1, sizeof(*p));
    if (!p) return NULL;
    const char *light_cpu = getenv("VDMA_MAIN_LIGHT_CPU");
    if (!light_cpu) light_cpu = getenv("VDMA_TRIAL_MAIN_LIGHT_CPU");
    if (light_cpu && strcmp(light_cpu, "0") && strcmp(light_cpu, "1")) {
        free(p); errno = EINVAL; return NULL;
    }
    p->main_light_cpu = !light_cpu || strcmp(light_cpu, "0");
    int rc = pthread_mutex_init(&p->mutex, NULL);
    if (rc) { free(p); errno = rc; return NULL; }
    p->store = store; p->diagnostics = diagnostics;
    p->confidence = confidence; p->nms_iou = nms_iou;
    p->rgb = calloc(YOLO_INPUT_BYTES, 1);
    if (!p->rgb) { yolo_adas_destroy(p); errno = ENOMEM; return NULL; }
    rc = yolo_rknn_open(&p->rknn, model, diagnostics);
    if (rc) { yolo_adas_destroy(p); errno = -rc; return NULL; }
    if (diagnostics)
        fprintf(diagnostics, "YOLO_ADAS path=fused_int8 input=%ux%u candidates=%u algorithm_version=%u decode_ns=included_in_inference\n",
                YOLO_INPUT_WIDTH, YOLO_INPUT_HEIGHT, YOLO_CANDIDATE_COUNT,
                YOLO_ALGORITHM_VERSION);
    if (p->main_light_cpu) light_temporal_open_trace(&p->temporal);
    if (diagnostics)
        fprintf(diagnostics, "TRIAL_MAIN_LIGHT_CPU enabled=%d model_graph_unchanged=1\n",
                p->main_light_cpu);
    return p;
}

static enum vdma_snapshot_status process_frame(void *opaque, const struct vdma_snapshot_input *input,
    struct vdma_snapshot_output *output, struct adas_detection *frozen)
{
    struct yolo_adas *p = opaque;
    struct adas_token token; struct adas_detection candidate;
    struct yolo_adas_stats delta = {0};
    uint64_t received = now_ns(), started;
    int rc = 0, stage_error = 0;
    const char *store_operation = "token";
    enum vdma_snapshot_status status = VDMA_PROCESS_OK;
    memset(output, 0, sizeof(*output));
    if (frozen) memset(frozen, 0, sizeof(*frozen));
    pthread_mutex_lock(&p->mutex); int halted = p->stats.halted; pthread_mutex_unlock(&p->mutex);
    if (halted) return VDMA_PROCESS_OK;
    delta.attempts = 1;
    if (cancelled((void *)input)) goto cancel;
    if (!input->pixels || !vdma_frame_valid(input->identity.slot, &input->header) ||
        input->identity.source_id != input->identity.layout || input->identity.source_id >= 4 ||
        input->identity.canvas_width != 1920 || input->identity.canvas_height != 1080 ||
        input->header.sequence != input->identity.frame_sequence || input->header.layout != input->identity.layout) {
        delta.failed = 1; delta.last_error = EPROTO; output->error = EPROTO; status = VDMA_PROCESS_FAILED; goto done;
    }
    struct adas_identity identity = {input->identity.stream_epoch, input->identity.frame_sequence,
        input->identity.source_id, input->identity.layout, input->identity.config_generation, 1920, 1080};
    /* The owner can advance the store clock after this worker samples it.
     * Retry with a fresh timestamp; persistent invalid inputs still fail. */
    if (frozen) {
        memset(&candidate, 0, sizeof(candidate));
        const struct adas_producer registration = yolo_adas_registration(input->session, 1);
        candidate.header = (struct adas_branch_header){.origin = ADAS_YOLO,
            .identity_present = 1, .identity = identity, .input_profile_generation = 1,
            .application_session = input->session, .producer_session = input->session,
            .algorithm_id = registration.algorithm_id, .algorithm_version = registration.algorithm_version,
            .algorithm_config_generation = registration.algorithm_config_generation,
            .observed_ns = input->observed_ns, .ttl_ns = ADAS_DETECTION_TTL_NS};
        candidate.class_map_version = registration.class_map_version;
        candidate.semantic_supported_mask = registration.semantic_supported_mask;
        candidate.signal_capability_mask = registration.signal_capability_mask;
    } else {
        unsigned clock_retries = 0;
        for (unsigned retry = 0; retry < 32; ++retry) {
            rc = adas_get_token(p->store, ADAS_DETECTION, &identity,
                                input->observed_ns, now_ns(), &token);
            if (rc == ADAS_BUSY) { store_retry_pause(); continue; }
            if (rc != ADAS_INVALID || ++clock_retries >= 3) break;
            ++delta.token_invalid_retries;
        }
        if (rc != ADAS_OK_NEW) goto rejected;
        store_operation = "prepare";
        for (unsigned retry = 0; retry < 32; ++retry) {
            rc = adas_prepare_detection(p->store, &token, &candidate);
            if (rc != ADAS_BUSY) break;
            store_retry_pause();
        }
        if (rc != ADAS_OK_NEW) goto rejected;
    }
    started = now_ns();
    if (input->small_rgb) {
        delta.small_inputs = 1;
        const struct vdma_rgb_frame *f = &input->rgb_descriptor;
        if (f->version != 1 || f->size != sizeof(*f) ||
            f->epoch != input->identity.stream_epoch ||
            f->sequence != input->identity.frame_sequence ||
            f->config != input->identity.config_generation ||
            f->layout != input->identity.layout || f->format != 2 ||
            f->preprocessing != 1 || f->width != 640 || f->height != 360 ||
            f->host_slot >= VDMA_RGB_SLOTS || f->reserved[0] || f->reserved[1])
            rc = -EPROTO;
        else rc = yolo_preprocess_small_rgb(input->small_rgb, f->bytes, f->stride,
                                p->rgb, YOLO_INPUT_BYTES, cancelled, (void *)input);
    } else {
        delta.cpu_inputs = 1;
        rc = yolo_preprocess_rgb565(input->pixels, input->header.bytes, input->header.stride,
                                p->rgb, YOLO_INPUT_BYTES, cancelled, (void *)input);
    }
    delta.preprocess_ns = now_ns() - started;
    if (rc == -ECANCELED || cancelled((void *)input)) goto cancel;
    if (!rc) {
        struct yolo_timing timing = {0};
        started = now_ns();
        rc = yolo_rknn_detect_lights(p->rknn, p->rgb, YOLO_INPUT_BYTES, p->confidence,
                              fminf(p->confidence,LIGHT_ASSOCIATION_MINIMUM),
                              p->nms_iou, &p->workspace, &p->boxes, &timing,
                              cancelled, (void *)input);
        delta.inference_ns = now_ns() - started;
        if (!rc) {
            delta.rknn_completed = 1;
            delta.rknn_inputs_ns = (uint64_t)(timing.inputs_ms * 1e6);
            delta.rknn_run_ns = (uint64_t)(timing.run_ms * 1e6);
            delta.rknn_outputs_ns = (uint64_t)(timing.outputs_ms * 1e6);
        }
    }
    if (cancelled((void *)input)) goto cancel;
    if (rc == -ECANCELED || cancelled((void *)input)) goto cancel;
    stage_error = rc ? -rc : 0;
    p->strong_boxes.count = 0;
    if (!stage_error)
        for (size_t i = 0; i < p->boxes.count; ++i)
            if (p->boxes.boxes[i].score > p->confidence)
                p->strong_boxes.boxes[p->strong_boxes.count++] = p->boxes.boxes[i];
    if (yolo_adas_fill(&candidate, &p->strong_boxes, received, now_ns(), stage_error)) {
        stage_error = EPROTO;
        yolo_adas_fill(&candidate, NULL, received, now_ns(), stage_error);
    }
    if (!stage_error && p->main_light_cpu) {
        started = now_ns();
        for (uint32_t i = 0; i < candidate.signals_meta.count; ++i) {
            if (cancelled((void *)input)) goto cancel;
            struct adas_signal *signal = &candidate.signals[i];
            struct light_color_result color;
            rc = traffic_light_color_rgb565(input->pixels, input->header.bytes,
                input->header.stride, input->header.width, input->header.height,
                &signal->box, &p->light_color, &color);
            if (rc) {
                stage_error = -rc;
                yolo_adas_fill(&candidate, NULL, received, now_ns(), stage_error);
                break;
            }
            signal->color = color.color;
            signal->color_valid = color.color != ADAS_COLOR_UNKNOWN;
            if (signal->color_valid) {
                signal->color_evidence_present = 1;
                signal->color_evidence_permille = color.evidence_permille;
            }
        }
        delta.light_color_ns = now_ns() - started;
        if (!stage_error && !frozen) {
            started = now_ns();
            rc = light_temporal_update(&p->temporal,&candidate,&p->boxes,p->confidence,
                input->pixels,input->header.bytes,input->header.stride,&p->light_color);
            delta.light_color_ns += now_ns()-started;
            if (rc) {
                stage_error = -rc;
                yolo_adas_fill(&candidate,NULL,received,now_ns(),stage_error);
            }
        }
        candidate.header.finished_ns = candidate.header.state_changed_ns = now_ns();
    }
    if (stage_error) {
        delta.failed = 1; delta.last_error = (uint32_t)stage_error;
    } else p->consecutive_failures = 0;
    if (cancelled((void *)input)) goto cancel;
    if (frozen) {
        *frozen = candidate;
        output->diagnostic_value = !stage_error;
        if (stage_error) { output->error = stage_error; status = VDMA_PROCESS_FAILED; }
        goto done;
    }
    store_operation = "publish";
    for (unsigned retry = 0; retry < 32; ++retry) {
        rc = adas_publish_detection(p->store, &token, &candidate, now_ns());
        if (rc != ADAS_BUSY) break;
        store_retry_pause();
    }
    if (rc == ADAS_OK_NEW) { delta.published = 1; output->diagnostic_value = 1; }
    else goto rejected;
    if (stage_error) { output->error = stage_error; status = VDMA_PROCESS_FAILED; }
    goto done;
rejected:
    if (rc == ADAS_BUSY) delta.busy = 1;
    else if (rc == ADAS_REJECTED_STALE) delta.stale = 1;
    else {
        delta.invalid = 1;
        if (p->diagnostics)
            fprintf(p->diagnostics, "YOLO_STORE_REJECT operation=%s rc=%d sequence=%u\n",
                    store_operation, rc, input->identity.frame_sequence);
    }
    goto done;
cancel:
    delta.cancelled = 1; status = VDMA_PROCESS_CANCELLED;
done:
    if (stage_error || delta.cancelled || delta.stale || delta.invalid) {
        memset(p->temporal.tracks,0,sizeof(p->temporal.tracks));
        p->temporal.initialized = 0;
    }
    if (delta.failed && ++p->consecutive_failures >= 3) delta.halted = 1;
    pthread_mutex_lock(&p->mutex);
    p->stats.attempts += delta.attempts; p->stats.published += delta.published;
    p->stats.failed += delta.failed; p->stats.cancelled += delta.cancelled;
    p->stats.busy += delta.busy; p->stats.stale += delta.stale; p->stats.invalid += delta.invalid;
    p->stats.preprocess_ns += delta.preprocess_ns; p->stats.decode_ns += delta.decode_ns;
    p->stats.inference_ns += delta.inference_ns; p->stats.halted |= delta.halted;
    p->stats.light_color_ns += delta.light_color_ns;
    p->stats.rknn_completed += delta.rknn_completed;
    p->stats.rknn_inputs_ns += delta.rknn_inputs_ns;
    p->stats.rknn_run_ns += delta.rknn_run_ns;
    p->stats.rknn_outputs_ns += delta.rknn_outputs_ns;
    p->stats.token_invalid_retries += delta.token_invalid_retries;
    p->stats.small_inputs += delta.small_inputs;
    p->stats.cpu_inputs += delta.cpu_inputs;
    if (delta.last_error) p->stats.last_error = delta.last_error;
    pthread_mutex_unlock(&p->mutex);
    return status;
}

static enum vdma_snapshot_status process(void *opaque, const struct vdma_snapshot_input *input,
                                         struct vdma_snapshot_output *output)
{
    return process_frame(opaque, input, output, NULL);
}

enum vdma_snapshot_status yolo_adas_analyze_frozen(struct yolo_adas *p,
    const struct vdma_snapshot_input *input, struct adas_detection *detection,
    struct vdma_snapshot_output *output)
{
    if (!p || !input || !detection || !output) return VDMA_PROCESS_FAILED;
    memset(p->temporal.tracks, 0, sizeof(p->temporal.tracks));
    p->temporal.initialized = 0;
    return process_frame(p, input, output, detection);
}

void yolo_adas_processor(struct yolo_adas *p, struct vdma_snapshot_processor *out)
{
    *out = (struct vdma_snapshot_processor){YOLO_PROCESSOR_TYPE, YOLO_ALGORITHM_VERSION, process, p};
}
void yolo_adas_get_stats(struct yolo_adas *p, struct yolo_adas_stats *out)
{
    memset(out, 0, sizeof(*out));
    if (p) { pthread_mutex_lock(&p->mutex); *out = p->stats; pthread_mutex_unlock(&p->mutex); }
}
void yolo_adas_destroy(struct yolo_adas *p)
{
    if (p) { light_temporal_close(&p->temporal); yolo_rknn_close(p->rknn); free(p->rgb); pthread_mutex_destroy(&p->mutex); free(p); }
}
