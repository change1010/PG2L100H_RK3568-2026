#ifndef VIDEO_DMA_UI_LAYOUT_H
#define VIDEO_DMA_UI_LAYOUT_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Versioned, fixed 1080p static UI test layout; no inference or hardware config. */
struct vdma_ui_rect { uint32_t x, y, width, height; };
struct vdma_ui_layout {
    uint32_t version, output_width, output_height, source_width, source_height;
    struct vdma_ui_rect video;
};
int vdma_ui_layout_validate(const struct vdma_ui_layout *layout);
int vdma_ui_layout_load(struct vdma_ui_layout *layout, const char *path);
#ifdef __cplusplus
}
#endif
#endif
