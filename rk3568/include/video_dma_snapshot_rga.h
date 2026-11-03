#ifndef VIDEO_DMA_SNAPSHOT_RGA_H
#define VIDEO_DMA_SNAPSHOT_RGA_H

#include "video_dma_snapshot.h"

/* Initialization only: display exports exist, DMA/sampling has not started. */
int vdma_snapshot_enable_rga(struct vdma_snapshot_runtime *runtime, const int exported[4]);
/* RR exports exist, but observe/sampling and the transport worker have not started. */
int vdma_snapshot_enable_reference_rga(struct vdma_snapshot_runtime *runtime,
    const int exported[3]);
int vdma_snapshot_rga_copy(void *backend, unsigned int image, unsigned int slot);
int vdma_snapshot_rga_cpu_access(void *backend, unsigned int image, int start);
void vdma_snapshot_rga_destroy(void *backend);

#endif
