#include "video_dma_ui_buffers.h"

void vdma_ui_buffers_init(struct vdma_ui_buffers *b)
{
    *b = (struct vdma_ui_buffers){.front = -1, .pending = -1};
}
int vdma_ui_buffer_acquire(struct vdma_ui_buffers *b)
{
    for (int i = 0; i < 2; i++)
        if (b->state[i] == UI_FREE) { b->state[i] = UI_DRAWING; return i; }
    return -1;
}
int vdma_ui_buffer_ready(struct vdma_ui_buffers *b, int i, uint64_t revision)
{
    if (i < 0 || i > 1 || b->state[i] != UI_DRAWING) return -1;
    /* Rendering revisions are monotonic. Never resurrect an older UI. */
    for (int n = 0; n < 2; n++) {
        if (n != i && b->state[n] != UI_FREE && revision <= b->revision[n]) {
            b->state[i] = UI_FREE;
            return -1;
        }
    }
    for (int n = 0; n < 2; n++)
        if (n != i && b->state[n] == UI_READY) b->state[n] = UI_FREE;
    b->revision[i] = revision; b->state[i] = UI_READY; return 0;
}
int vdma_ui_buffer_newest(const struct vdma_ui_buffers *b)
{
    int result = -1;
    for (int i = 0; i < 2; i++)
        if (b->state[i] == UI_READY && (result < 0 || b->revision[i] > b->revision[result])) result = i;
    return result;
}
int vdma_ui_buffer_submitted(struct vdma_ui_buffers *b, int i)
{
    if (i < 0 || i > 1 || b->pending >= 0 || b->state[i] != UI_READY) return -1;
    b->state[i] = UI_PENDING; b->pending = i; return 0;
}
int vdma_ui_buffer_complete(struct vdma_ui_buffers *b)
{
    if (b->pending < 0) return -1;
    if (b->front >= 0) b->state[b->front] = UI_FREE;
    b->front = b->pending; b->pending = -1; b->state[b->front] = UI_FRONT; return 0;
}
int vdma_ui_buffer_cancel_draw(struct vdma_ui_buffers *b, int i)
{
    if (i < 0 || i > 1 || b->state[i] != UI_DRAWING) return -1;
    b->state[i] = UI_FREE; return 0;
}
