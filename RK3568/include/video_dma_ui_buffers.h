#ifndef VIDEO_DMA_UI_BUFFERS_H
#define VIDEO_DMA_UI_BUFFERS_H
#include <stdint.h>

enum vdma_ui_buffer_state { UI_FREE, UI_DRAWING, UI_READY, UI_PENDING, UI_FRONT };
/* Serialize externally. The current/pending buffers are never drawable. */
struct vdma_ui_buffers {
    enum vdma_ui_buffer_state state[2];
    uint64_t revision[2];
    int front, pending;
};
void vdma_ui_buffers_init(struct vdma_ui_buffers *b);
int vdma_ui_buffer_acquire(struct vdma_ui_buffers *b);
int vdma_ui_buffer_ready(struct vdma_ui_buffers *b, int index, uint64_t revision);
int vdma_ui_buffer_newest(const struct vdma_ui_buffers *b);
int vdma_ui_buffer_submitted(struct vdma_ui_buffers *b, int index);
int vdma_ui_buffer_complete(struct vdma_ui_buffers *b);
int vdma_ui_buffer_cancel_draw(struct vdma_ui_buffers *b, int index);
#endif
