#include "yolo_adas.h"
#include <errno.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

struct adas_producer yolo_adas_registration(uint64_t session, uint32_t generation)
{
    const char *light_cpu = getenv("VDMA_MAIN_LIGHT_CPU");
    if (!light_cpu) light_cpu = getenv("VDMA_TRIAL_MAIN_LIGHT_CPU");
    const uint32_t color_capability = light_cpu && !strcmp(light_cpu, "0") ?
        0 : ADAS_SIGNAL_COLOR;
    return (struct adas_producer){.enabled = 1, .origin = ADAS_YOLO,
        .producer_session = session, .algorithm_id = YOLO_ALGORITHM_ID,
        .algorithm_version = YOLO_ALGORITHM_VERSION, .algorithm_config_generation = generation,
        .class_map_version = 1, .semantic_supported_mask = ADAS_CLASS_VEHICLE |
            ADAS_CLASS_PEDESTRIAN | ADAS_CLASS_OTHER,
        .signal_capability_mask = ADAS_SIGNAL_LOCATION | color_capability};
}

static void collection(struct adas_collection *meta, uint32_t total, uint32_t capacity)
{
    *meta = (struct adas_collection){.state = total ? ADAS_VALID : ADAS_ABSENT,
        .count = total < capacity ? total : capacity, .observed_count = total,
        .total_count_known = 1, .flags = total > capacity ? ADAS_TRUNCATED : 0};
}

int yolo_adas_fill(struct adas_detection *out, const struct yolo_detections *boxes,
                   uint64_t received_ns, uint64_t finished_ns, int error)
{
    uint32_t objects = 0, signals = 0;
    if (!out || error < 0 || (!error && !boxes) || received_ns < out->header.observed_ns ||
        finished_ns < received_ns || (!error && boxes->count > YOLO_MAX_CANDIDATES)) return -EINVAL;
    out->header.received_ns = received_ns; out->header.finished_ns = finished_ns;
    out->header.state_changed_ns = finished_ns; out->header.flags = 0;
    out->header.error_errno = (uint32_t)error;
    memset(out->objects, 0, sizeof(out->objects)); memset(out->signals, 0, sizeof(out->signals));
    memset(&out->objects_meta, 0, sizeof(out->objects_meta));
    memset(&out->signals_meta, 0, sizeof(out->signals_meta));
    out->light_visual_valid = out->light_visual_count = 0;
    memset(out->light_visual,0,sizeof(out->light_visual));
    if (error) {
        out->header.state = out->objects_meta.state = out->signals_meta.state = ADAS_FAILED;
        out->header.reason = out->objects_meta.reason = out->signals_meta.reason = ADAS_PROCESSOR_ERROR;
        return 0;
    }
    for (size_t i = 0; i < boxes->count; ++i) {
        const struct yolo_box *b = &boxes->boxes[i];
        if (!isfinite(b->score) || b->score < 0 || b->score > 1 ||
            !isfinite(b->x0) || !isfinite(b->y0) || !isfinite(b->x1) || !isfinite(b->y1) ||
            b->x0 < 0 || b->y0 < 0 || b->x1 > 1920 || b->y1 > 1080 ||
            b->x1 <= b->x0 || b->y1 <= b->y0 || floorf(b->x0) != b->x0 ||
            floorf(b->y0) != b->y0 || floorf(b->x1) != b->x1 || floorf(b->y1) != b->y1 ||
            !(b->class_id == 0 || b->class_id == 1 || b->class_id == 2 || b->class_id == 3 ||
              b->class_id == 5 || b->class_id == 7 || b->class_id == 9 || b->class_id == 11)) return -EINVAL;
    }
    for (size_t i = 0; i < boxes->count; ++i) {
        const struct yolo_box *b = &boxes->boxes[i];
        struct adas_box box = {(int32_t)b->x0, (int32_t)b->y0, (int32_t)b->x1, (int32_t)b->y1};
        uint32_t score = (uint32_t)lroundf(b->score * 10000);
        if (b->class_id == 9) {
            if (signals < ADAS_MAX_SIGNALS) out->signals[signals] = (struct adas_signal){
                .signal_id = signals + 1, .box = box, .localization_confidence_present = 1,
                .localization_confidence_q10000 = score, .origin = ADAS_YOLO, .original_index = b->candidate_index};
            ++signals;
        } else {
            if (objects < ADAS_MAX_OBJECTS) out->objects[objects] = (struct adas_object){
                .object_id = objects + 1, .semantic_class = b->class_id == 0 ? ADAS_PEDESTRIAN :
                    b->class_id == 11 ? ADAS_OTHER : ADAS_VEHICLE, .model_class_id = b->class_id,
                .class_map_version = 1, .box = box, .confidence_present = 1, .confidence_q10000 = score,
                .origin = ADAS_YOLO, .original_index = b->candidate_index};
            ++objects;
        }
    }
    collection(&out->objects_meta, objects, ADAS_MAX_OBJECTS);
    collection(&out->signals_meta, signals, ADAS_MAX_SIGNALS);
    out->header.flags = out->objects_meta.flags | out->signals_meta.flags;
    out->header.state = ADAS_VALID; out->header.reason = ADAS_REASON_NONE;
    return 0;
}
