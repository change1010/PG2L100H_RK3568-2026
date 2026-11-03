#ifndef YOLO_ADAS_FROZEN_H
#define YOLO_ADAS_FROZEN_H
struct yolo_adas;
struct vdma_snapshot_input;
struct vdma_snapshot_output;
struct detection_overlay;
struct adas_ped_observation;
#ifdef __cplusplus
extern "C" {
#endif
int yolo_adas_frozen_overlay(struct yolo_adas *producer,
    const struct vdma_snapshot_input *input, struct vdma_snapshot_output *output,
    struct detection_overlay *overlay);
int yolo_adas_frozen_pedestrian(struct yolo_adas *producer,
    const struct vdma_snapshot_input *input, struct vdma_snapshot_output *output,
    struct detection_overlay *overlay, struct adas_ped_observation *observation);
#ifdef __cplusplus
}
#endif
#endif
