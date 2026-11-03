#ifndef FPGA_ROAD_OSD_H
#define FPGA_ROAD_OSD_H
#include <stdint.h>
#include "detection_overlay.h"
#include "road_overlay_geometry.h"
#include "video_dma_feature_uapi.h"

#ifdef __cplusplus
extern "C" {
#endif
unsigned int fpga_road_osd_rgb565(uint16_t *pixels, uint32_t stride_pixels,
    uint32_t width, uint32_t height, const struct vdma_frame_identity *identity,
    const struct road_overlay_geometry *road, const struct detection_overlay *detection);
#ifdef __cplusplus
}
#endif
#endif
