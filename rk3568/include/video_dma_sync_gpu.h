#ifndef VIDEO_DMA_SYNC_GPU_H
#define VIDEO_DMA_SYNC_GPU_H
#include <stdint.h>
#define VDMA_SYNC_SLOTS 16
#define VDMA_SYNC_BYTES (1920U * 1080U * 2U)
#ifdef __cplusplus
extern "C" {
#endif
void *vdma_sync_gpu_create(int drm, const int source[4]);
int vdma_sync_gpu_copy(void *gpu, unsigned int target, unsigned int source);
int vdma_sync_gpu_access(void *gpu, unsigned int target, int start);
void *vdma_sync_gpu_pixels(void *gpu, unsigned int target);
uint32_t vdma_sync_gpu_framebuffer(void *gpu, unsigned int target);
/* Caller has joined copy/draw and detached DRM. */
void vdma_sync_gpu_destroy(void *gpu);
#ifdef __cplusplus
}
#endif
#endif
