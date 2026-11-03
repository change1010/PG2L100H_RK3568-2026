#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <string.h>
#include <time.h>
#include "video_dma_snapshot_async.h"

static uint64_t copy_now_ns(void)
{
    struct timespec stamp;
    clock_gettime(CLOCK_MONOTONIC, &stamp);
    return (uint64_t)stamp.tv_sec * UINT64_C(1000000000) + stamp.tv_nsec;
}

static void *copy_worker(void *opaque)
{
    struct vdma_snapshot_async *copy = opaque;
    pthread_mutex_lock(&copy->mutex);
    for (;;) {
        while (!copy->requested && !copy->stopping)
            pthread_cond_wait(&copy->ready, &copy->mutex);
        if (!copy->requested && copy->stopping) break;
        copy->requested = false;
        pthread_mutex_unlock(&copy->mutex);
        uint64_t started = copy_now_ns();
        int rc = vdma_snapshot_sample(copy->runtime, copy->fd,
            &copy->identity, &copy->header, copy->observed_ns);
        /* A source change before the helper starts deliberately cancels it. */
        copy->error = rc && errno != ESTALE ? errno : 0;
        uint64_t elapsed = copy_now_ns() - started;
        if (elapsed > copy->max_ns) copy->max_ns = elapsed;
        atomic_store_explicit(&copy->done, true, memory_order_release);
        pthread_mutex_lock(&copy->mutex);
    }
    pthread_mutex_unlock(&copy->mutex);
    return NULL;
}

int vdma_snapshot_async_init(struct vdma_snapshot_async *copy,
    struct vdma_snapshot_runtime *runtime, int fd)
{
    if (!copy || !runtime || !runtime->initialized || !runtime->rga ||
        runtime->rga_reference || runtime->plane_enabled || fd < 0) {
        errno = EINVAL; return -1;
    }
    memset(copy, 0, sizeof(*copy));
    atomic_init(&copy->done, false);
    int error = pthread_mutex_init(&copy->mutex, NULL);
    if (error) { errno = error; return -1; }
    error = pthread_cond_init(&copy->ready, NULL);
    if (error) { pthread_mutex_destroy(&copy->mutex); errno = error; return -1; }
    copy->initialized = true;
    copy->runtime = runtime;
    copy->fd = fd;
    error = pthread_create(&copy->thread, NULL, copy_worker, copy);
    if (error) { vdma_snapshot_async_destroy(copy); errno = error; return -1; }
    copy->started = true;
    return 0;
}

int vdma_snapshot_async_reap(struct vdma_snapshot_async *copy)
{
    if (!copy || !copy->initialized || !copy->active ||
        !atomic_load_explicit(&copy->done, memory_order_acquire)) return 0;
    copy->active = false;
    copy->completed++;
    if (copy->error) { errno = copy->error; return -1; }
    return 1;
}

int vdma_snapshot_async_submit(struct vdma_snapshot_async *copy,
    const struct vdma_frame_identity *identity,
    const struct vdma_frame_header *header, uint64_t observed_ns)
{
    if (!copy || !copy->started || copy->stopping || !identity || !header) {
        errno = EINVAL; return -1;
    }
    if (copy->active) { copy->busy++; return 0; }
    pthread_mutex_lock(&copy->mutex);
    copy->identity = *identity;
    copy->header = *header;
    copy->observed_ns = observed_ns;
    copy->error = 0;
    atomic_store_explicit(&copy->done, false, memory_order_relaxed);
    copy->active = copy->requested = true;
    copy->submitted++;
    pthread_cond_signal(&copy->ready);
    pthread_mutex_unlock(&copy->mutex);
    return 1;
}

bool vdma_snapshot_async_holds(const struct vdma_snapshot_async *copy,
    const struct vdma_buffer *frame)
{
    return copy && copy->initialized && copy->active &&
        copy->identity.slot == frame->slot && copy->identity.token == frame->token;
}

int vdma_snapshot_async_stop(struct vdma_snapshot_async *copy)
{
    if (!copy || !copy->initialized) return 0;
    pthread_mutex_lock(&copy->mutex);
    copy->stopping = true;
    pthread_cond_signal(&copy->ready);
    pthread_mutex_unlock(&copy->mutex);
    if (copy->started) { pthread_join(copy->thread, NULL); copy->started = false; }
    return vdma_snapshot_async_reap(copy) < 0 ? -1 : 0;
}

void vdma_snapshot_async_destroy(struct vdma_snapshot_async *copy)
{
    if (!copy || !copy->initialized) return;
    vdma_snapshot_async_stop(copy);
    pthread_cond_destroy(&copy->ready);
    pthread_mutex_destroy(&copy->mutex);
    copy->initialized = false;
}
