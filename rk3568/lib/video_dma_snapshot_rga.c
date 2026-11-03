#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include "video_dma_snapshot_rga.h"

#ifdef VDMA_HAVE_RGA
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <im2d.h>

struct snapshot_rga {
    int fd[VDMA_SNAPSHOT_BUFFERS];
    void *pixels[VDMA_SNAPSHOT_BUFFERS];
    rga_buffer_handle_t source[4], destination[VDMA_SNAPSHOT_BUFFERS];
    unsigned int source_count, height;
};

void vdma_snapshot_rga_destroy(void *opaque)
{
    struct snapshot_rga *backend = opaque;
    if (!backend) return;
    for (int i = 0; i < 4; i++)
        if (backend->source[i]) releasebuffer_handle(backend->source[i]);
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) {
        if (backend->destination[i]) releasebuffer_handle(backend->destination[i]);
        if (backend->pixels[i] != MAP_FAILED)
            munmap(backend->pixels[i], VDMA_SNAPSHOT_FRAME_BYTES);
        if (backend->fd[i] >= 0) close(backend->fd[i]);
    }
    free(backend);
}

static int enable_rga(struct vdma_snapshot_runtime *runtime, const int *exported,
    unsigned int count, unsigned int height)
{
    if (!runtime || !exported || !runtime->initialized || runtime->rga || runtime->bound ||
        runtime->stopping || runtime->stats.attempts) { errno = EINVAL; return -1; }
    struct snapshot_rga *backend = calloc(1, sizeof(*backend));
    if (!backend) return -1;
    backend->source_count = count;
    backend->height = height;
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) {
        backend->fd[i] = -1; backend->pixels[i] = MAP_FAILED;
    }
    int heap = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (heap < 0) { free(backend); return -1; }
    for (unsigned int i = 0; i < count; i++) {
        im_handle_param_t param = {.width = 1920, .height = height, .format = RK_FORMAT_RGB_565};
        backend->source[i] = importbuffer_fd(exported[i], &param);
        if (!backend->source[i]) { errno = EIO; goto fail; }
    }
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) {
        struct dma_heap_allocation_data allocation = {
            .len = VDMA_SNAPSHOT_FRAME_BYTES, .fd_flags = O_RDWR | O_CLOEXEC};
        if (ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &allocation)) goto fail;
        backend->fd[i] = allocation.fd;
        backend->pixels[i] = mmap(NULL, VDMA_SNAPSHOT_FRAME_BYTES, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, backend->fd[i], 0);
        if (backend->pixels[i] == MAP_FAILED) goto fail;
        struct dma_buf_sync sync = {.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE};
        if (ioctl(backend->fd[i], DMA_BUF_IOCTL_SYNC, &sync)) goto fail;
        memset(backend->pixels[i], 0, VDMA_SNAPSHOT_FRAME_BYTES);
        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
        if (ioctl(backend->fd[i], DMA_BUF_IOCTL_SYNC, &sync)) goto fail;
        im_handle_param_t param = {.width = 1920, .height = height, .format = RK_FORMAT_RGB_565};
        backend->destination[i] = importbuffer_fd(backend->fd[i], &param);
        if (!backend->destination[i]) { errno = EIO; goto fail; }
    }
    close(heap);
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) {
        free(runtime->images[i].pixels);
        runtime->images[i].pixels = backend->pixels[i];
    }
    runtime->rga = backend;
    runtime->rga_reference = count == RR_HOST_SLOT_COUNT;
    return 0;
fail: {
    int saved = errno;
    close(heap); vdma_snapshot_rga_destroy(backend); errno = saved; return -1;
}
}

int vdma_snapshot_enable_rga(struct vdma_snapshot_runtime *runtime, const int exported[4])
{
    return enable_rga(runtime, exported, 4, 1080);
}

int vdma_snapshot_enable_reference_rga(struct vdma_snapshot_runtime *runtime,
    const int exported[3])
{
    /* RGB565 here is a byte-preserving same-format blit of the packed payload. */
    _Static_assert(RR_PAYLOAD_BYTES == 1920U * 1020U * 2U, "RR RGA byte geometry");
    return enable_rga(runtime, exported, RR_HOST_SLOT_COUNT, 1020);
}

int vdma_snapshot_rga_copy(void *opaque, unsigned int image, unsigned int slot)
{
    struct snapshot_rga *backend = opaque;
    if (!backend || image >= VDMA_SNAPSHOT_BUFFERS || slot >= backend->source_count) {
        errno = EINVAL; return -1;
    }
    rga_buffer_t source = wrapbuffer_handle_t(backend->source[slot], 1920, backend->height,
                                             1920, backend->height, RK_FORMAT_RGB_565);
    rga_buffer_t target = wrapbuffer_handle_t(backend->destination[image], 1920, backend->height,
                                             1920, backend->height, RK_FORMAT_RGB_565);
    IM_STATUS status = imcopy_t(source, target, 1); /* synchronous, no in-flight slot release */
    if (status != IM_STATUS_SUCCESS) {
        fprintf(stderr, "Snapshot RGA copy: %s\n", imStrError_t(status));
        errno = EIO; return -1;
    }
    return 0;
}

int vdma_snapshot_rga_cpu_access(void *opaque, unsigned int image, int start)
{
    struct snapshot_rga *backend = opaque;
    struct dma_buf_sync sync = {.flags = DMA_BUF_SYNC_READ |
        (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END)};
    return ioctl(backend->fd[image], DMA_BUF_IOCTL_SYNC, &sync);
}
#else
int vdma_snapshot_enable_rga(struct vdma_snapshot_runtime *runtime, const int exported[4])
{
    (void)runtime; (void)exported; errno = ENOTSUP; return -1;
}
int vdma_snapshot_enable_reference_rga(struct vdma_snapshot_runtime *runtime,
    const int exported[3])
{
    (void)runtime; (void)exported; errno = ENOTSUP; return -1;
}
int vdma_snapshot_rga_copy(void *backend, unsigned int image, unsigned int slot)
{
    (void)backend; (void)image; (void)slot; errno = ENOTSUP; return -1;
}
int vdma_snapshot_rga_cpu_access(void *backend, unsigned int image, int start)
{
    (void)backend; (void)image; (void)start; errno = ENOTSUP; return -1;
}
void vdma_snapshot_rga_destroy(void *backend) { (void)backend; }
#endif
