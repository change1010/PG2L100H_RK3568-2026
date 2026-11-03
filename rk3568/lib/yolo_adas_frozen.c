#include "yolo_adas_frozen.h"
#include "yolo_adas.h"
#include "detection_overlay.h"
#include "adas_pedestrian_test.h"
#include <string.h>

int yolo_adas_frozen_pedestrian(struct yolo_adas *producer,
    const struct vdma_snapshot_input *input, struct vdma_snapshot_output *output,
    struct detection_overlay *overlay, struct adas_ped_observation *observation)
{
    struct adas_detection detection = {0};
    memset(output,0,sizeof(*output));
    enum vdma_snapshot_status status = yolo_adas_analyze_frozen(producer,input,&detection,output);
    adas_ped_from_detection(observation,&detection);
    if (status == VDMA_PROCESS_OK && output->diagnostic_value)
        detection_overlay_from_frozen(overlay,&detection,detection.header.finished_ns);
    return status;
}

int yolo_adas_frozen_overlay(struct yolo_adas *producer,
    const struct vdma_snapshot_input *input, struct vdma_snapshot_output *output,
    struct detection_overlay *overlay)
{
    struct adas_detection detection;
    enum vdma_snapshot_status status = yolo_adas_analyze_frozen(producer, input, &detection, output);
    if (status == VDMA_PROCESS_OK && output->diagnostic_value)
        detection_overlay_from_frozen(overlay, &detection, detection.header.finished_ns);
    return status;
}
