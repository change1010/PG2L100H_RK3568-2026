#define _POSIX_C_SOURCE 200809L
#include "video_dma_sync_gpu.h"
#include <errno.h>
#ifdef VDMA_HAVE_RGA
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <im2d.h>
struct sync_gpu {
    int drm, fd[VDMA_SYNC_SLOTS];
    void *pixels[VDMA_SYNC_SLOTS];
    uint32_t handle[VDMA_SYNC_SLOTS], fb[VDMA_SYNC_SLOTS];
    rga_buffer_handle_t source[4], target[VDMA_SYNC_SLOTS];
};
void vdma_sync_gpu_destroy(void *opaque)
{
    struct sync_gpu *g = opaque;
    if (!g) return;
    for (int i = 0; i < 4; ++i)
        if (g->source[i]) releasebuffer_handle(g->source[i]);
    for (int i = 0; i < VDMA_SYNC_SLOTS; ++i) {
        if (g->target[i]) releasebuffer_handle(g->target[i]);
        if (g->fb[i]) drmModeRmFB(g->drm, g->fb[i]);
        if (g->handle[i]) {
            struct drm_gem_close close_request = {.handle = g->handle[i]};
            drmIoctl(g->drm, DRM_IOCTL_GEM_CLOSE, &close_request);
        }
        if (g->pixels[i] != MAP_FAILED) munmap(g->pixels[i], VDMA_SYNC_BYTES);
        if (g->fd[i] >= 0) close(g->fd[i]);
    }
    free(g);
}
void *vdma_sync_gpu_create(int drm, const int source[4])
{
    struct sync_gpu *g = calloc(1, sizeof(*g));
    if (!g) return NULL;
    g->drm = drm;
    for (int i = 0; i < VDMA_SYNC_SLOTS; ++i) {
        g->fd[i] = -1; g->pixels[i] = MAP_FAILED;
    }
    int heap = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (heap < 0) { vdma_sync_gpu_destroy(g); return NULL; }
    im_handle_param_t param = {.width = 1920, .height = 1080, .format = RK_FORMAT_RGB_565};
    for (int i = 0; i < 4; ++i) {
        g->source[i] = importbuffer_fd(source[i], &param);
        if (!g->source[i]) { errno = EIO; goto fail; }
    }
    for (int i = 0; i < VDMA_SYNC_SLOTS; ++i) {
        struct dma_heap_allocation_data a = {.len = VDMA_SYNC_BYTES, .fd_flags = O_RDWR | O_CLOEXEC};
        if (ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &a)) goto fail;
        g->fd[i] = a.fd;
        g->pixels[i] = mmap(NULL, VDMA_SYNC_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, g->fd[i], 0);
        if (g->pixels[i] == MAP_FAILED) goto fail;
        g->target[i] = importbuffer_fd(g->fd[i], &param);
        if (!g->target[i]) { errno = EIO; goto fail; }
        if (drmPrimeFDToHandle(drm, g->fd[i], &g->handle[i])) goto fail;
        uint32_t handles[4] = {g->handle[i]}, pitches[4] = {3840}, offsets[4] = {0};
        if (drmModeAddFB2(drm, 1920, 1080, DRM_FORMAT_RGB565,
                         handles, pitches, offsets, &g->fb[i], 0)) goto fail;
    }
    close(heap);
    return g;
fail: {
    int error = errno; close(heap); vdma_sync_gpu_destroy(g); errno = error; return NULL;
}
}
int vdma_sync_gpu_copy(void *opaque, unsigned int target, unsigned int source)
{
    struct sync_gpu *g = opaque;
    if (!g || target >= VDMA_SYNC_SLOTS || source >= 4) { errno = EINVAL; return -1; }
    rga_buffer_t a = wrapbuffer_handle_t(g->source[source], 1920, 1080, 1920, 1080, RK_FORMAT_RGB_565);
    rga_buffer_t b = wrapbuffer_handle_t(g->target[target], 1920, 1080, 1920, 1080, RK_FORMAT_RGB_565);
    if (imcopy_t(a, b, 1) != IM_STATUS_SUCCESS) { errno = EIO; return -1; }
    return 0;
}
int vdma_sync_gpu_access(void *opaque, unsigned int target, int start)
{
    struct sync_gpu *g = opaque;
    struct dma_buf_sync sync = {.flags = DMA_BUF_SYNC_RW | (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END)};
    return ioctl(g->fd[target], DMA_BUF_IOCTL_SYNC, &sync);
}
void *vdma_sync_gpu_pixels(void *opaque, unsigned int target)
{ return ((struct sync_gpu *)opaque)->pixels[target]; }
uint32_t vdma_sync_gpu_framebuffer(void *opaque, unsigned int target)
{ return ((struct sync_gpu *)opaque)->fb[target]; }
#else
void *vdma_sync_gpu_create(int drm, const int source[4])
{ (void)drm; (void)source; errno = ENOTSUP; return 0; }
int vdma_sync_gpu_copy(void *g, unsigned int t, unsigned int s)
{ (void)g; (void)t; (void)s; errno = ENOTSUP; return -1; }
int vdma_sync_gpu_access(void *g, unsigned int t, int s)
{ (void)g; (void)t; (void)s; errno = ENOTSUP; return -1; }
void *vdma_sync_gpu_pixels(void *g, unsigned int t) { (void)g; (void)t; return 0; }
uint32_t vdma_sync_gpu_framebuffer(void *g, unsigned int t) { (void)g; (void)t; return 0; }
void vdma_sync_gpu_destroy(void *g) { (void)g; }
#endif
