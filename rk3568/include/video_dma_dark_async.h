#ifndef VIDEO_DMA_DARK_ASYNC_H
#define VIDEO_DMA_DARK_ASYNC_H
#include <stdint.h>
#include "video_dma_dark.h"
#include "video_dma_feature_uapi.h"
enum vdma_dark_phase { DARK_OFFLINE, DARK_IDLE, DARK_QUEUED, DARK_WAIT, DARK_TIMEOUT, DARK_FAULT };
/* Externally serialized. tick performs at most one status and one command
 * ioctl, never sleeps; no automatic APPLY/retry/cancellation. */
struct vdma_dark_async {
    struct vdma_dark_status actual;
    struct vdma_dark_command target;
    enum vdma_dark_phase phase;
    int available, error;
    uint64_t next_poll_ms, submitted_ms;
    uint32_t before_generation;
};
void vdma_dark_async_init(struct vdma_dark_async *a);
int vdma_dark_async_queue(struct vdma_dark_async *a, unsigned int opcode,
                          unsigned int threshold, unsigned int strength);
void vdma_dark_async_tick(struct vdma_dark_async *a, int fd, uint64_t now_ms);
int vdma_dark_frame_matches(const struct vdma_dark_status *s,
                             const struct vdma_frame_identity *frame);
#endif
