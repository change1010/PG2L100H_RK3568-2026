#ifndef VIDEO_DMA_OSD_PROBE_H
#define VIDEO_DMA_OSD_PROBE_H
#include <stddef.h>
#include <stdint.h>
#include "video_dma_ui_layout.h"

/* Static probe by default; flip_buffer also supports owner-managed immutable
 * ARGB scanout buffers. Never draw into a FRONT/PENDING buffer or video slot. */
struct vdma_osd_probe {
    int fd;
    uint32_t plane, crtc, handle, framebuffer, pitch;
    uint64_t bytes;
    void *pixels;
    uint32_t property[3];
    uint64_t previous[3];
    unsigned int changed;
    int active;
    int attached;
    uint64_t toggles;
    uint32_t primary, primary_fb_property;
    uint32_t geometry[10];
    uint32_t primary_geometry[10];
    uint64_t primary_previous[10];
    struct vdma_ui_layout layout;
    int layout_enabled, layout_attached, layout_tested;
};
struct vdma_osd_writeback {
    uint32_t connector, crtc_property, fb_property, fence_property, framebuffer;
    int *fence;
};
int vdma_osd_probe_init(struct vdma_osd_probe *osd, int fd, uint32_t crtc,
                        const char *argb_path);
int vdma_osd_probe_init_blank(struct vdma_osd_probe *osd, int fd, uint32_t crtc);
int vdma_osd_probe_flip_buffer(struct vdma_osd_probe *osd, uint32_t video_fb,
                               uint32_t argb_fb, void *event_data, int test_only);
int vdma_osd_probe_flip_buffer_record(struct vdma_osd_probe *osd, uint32_t video_fb,
                                      uint32_t argb_fb, void *event_data,
                                      const struct vdma_osd_writeback *writeback);
int vdma_osd_probe_show(struct vdma_osd_probe *osd, int visible);
int vdma_osd_probe_flip(struct vdma_osd_probe *osd, uint32_t video_framebuffer,
                        int visible, void *event_data);
int vdma_osd_probe_set_layout(struct vdma_osd_probe *osd, const struct vdma_ui_layout *layout);
int vdma_osd_probe_test_layout(struct vdma_osd_probe *osd, uint32_t video_framebuffer);
int vdma_osd_probe_close(struct vdma_osd_probe *osd);
#endif
