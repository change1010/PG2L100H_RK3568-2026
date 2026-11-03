#ifndef VIDEO_DMA_SNAPSHOT_ASYNC_H
#define VIDEO_DMA_SNAPSHOT_ASYNC_H

#include <stdatomic.h>
#include "video_dma_snapshot.h"

/* One copy in flight. The owner retains identity.slot/token until reap.
 * The helper queries identity but never acquires/releases slots or stops DMA. */
struct vdma_snapshot_async {
    pthread_mutex_t mutex;
    pthread_cond_t ready;
    pthread_t thread;
    bool initialized, started, stopping, requested, active;
    atomic_bool done;
    struct vdma_snapshot_runtime *runtime;
    int fd, error;
    struct vdma_frame_identity identity;
    struct vdma_frame_header header;
    uint64_t observed_ns, submitted, busy, completed, max_ns;
};

int vdma_snapshot_async_init(struct vdma_snapshot_async *copy,
    struct vdma_snapshot_runtime *runtime, int fd);
int vdma_snapshot_async_submit(struct vdma_snapshot_async *copy,
    const struct vdma_frame_identity *identity,
    const struct vdma_frame_header *header, uint64_t observed_ns);
/* Nonblocking; only the owner calls submit/reap/holds. */
int vdma_snapshot_async_reap(struct vdma_snapshot_async *copy);
bool vdma_snapshot_async_holds(const struct vdma_snapshot_async *copy,
    const struct vdma_buffer *frame);
/* Join before releasing video or destroying runtime/DMA-BUF resources. */
int vdma_snapshot_async_stop(struct vdma_snapshot_async *copy);
void vdma_snapshot_async_destroy(struct vdma_snapshot_async *copy);

#endif
