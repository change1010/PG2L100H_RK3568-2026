#ifndef YOLO_ADAS_H
#define YOLO_ADAS_H
#include "adas_result.h"
#include "yolo_decode.h"
#include "video_dma_snapshot.h"

#define YOLO_ALGORITHM_ID 3U
#define YOLO_PROCESSOR_TYPE 3U
#define YOLO_ALGORITHM_VERSION 4U

struct yolo_adas;
struct yolo_adas_stats {
    uint64_t attempts, published, failed, cancelled, busy, stale, invalid;
    uint64_t preprocess_ns, decode_ns, inference_ns, light_color_ns;
    uint64_t rknn_completed, rknn_inputs_ns, rknn_run_ns, rknn_outputs_ns;
    uint64_t token_invalid_retries;
    uint64_t small_inputs, cpu_inputs;
    uint32_t halted, last_error;
};
struct adas_producer yolo_adas_registration(uint64_t session, uint32_t generation);
/* Fill a prepared candidate. No token/store mutation, heap allocation or NPU. */
int yolo_adas_fill(struct adas_detection *out, const struct yolo_detections *boxes,
                   uint64_t received_ns, uint64_t finished_ns, int error);
struct yolo_adas *yolo_adas_create(struct adas_store *store, const char *model,
                                   float confidence, float nms_iou, FILE *diagnostics);
void yolo_adas_processor(struct yolo_adas *producer, struct vdma_snapshot_processor *out);
/* Same serialized worker as process(); analyzes owned frozen pixels without
 * publishing into the live store or reusing temporal display evidence. */
enum vdma_snapshot_status yolo_adas_analyze_frozen(struct yolo_adas *producer,
    const struct vdma_snapshot_input *input, struct adas_detection *detection,
    struct vdma_snapshot_output *output);
void yolo_adas_get_stats(struct yolo_adas *producer, struct yolo_adas_stats *out);
/* Worker must be joined before destroy. Failures halt after three runs until restart. */
void yolo_adas_destroy(struct yolo_adas *producer);
#endif
