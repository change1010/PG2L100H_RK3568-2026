#ifndef YOLO_CONFIG_H
#define YOLO_CONFIG_H
#include <stdio.h>
#define YOLO_MODEL_PATH_BYTES 4096
#define YOLO_CONFIG_CONTRACT "decoded80_rgb565_1920x1080_letterbox640x384_fused_v1"
struct yolo_config {
    char model[YOLO_MODEL_PATH_BYTES];
    char sha256[65];
    unsigned int hz;
    float confidence, nms_iou;
};
/* Strict JSON and model digest validation; no partial configuration on error. */
int yolo_config_load(struct yolo_config *out, const char *path, FILE *diagnostics);
#endif
