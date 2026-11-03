#define _POSIX_C_SOURCE 200809L
#include "video_dma_snapshot.h"
#include "video_dma_frame.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static uint64_t now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * UINT64_C(1000000000) + t.tv_nsec;
}

static enum vdma_snapshot_status no_publish(void *context,
    const struct vdma_snapshot_input *input, struct vdma_snapshot_output *output)
{
    (void)context; (void)input; (void)output;
    return VDMA_PROCESS_OK;
}

/* Pixel transport diagnostic only: no source writes, NPU, DRM or public ADAS
 * results. Source-control health remains a separate production entry gate. */
int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s DEVICE (exclusive owner, at most 12 seconds)\n", argv[0]);
        return 2;
    }
    int fd = open(argv[1], O_RDWR | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) { perror("pair open"); return 1; }
    struct vdma_snapshot_runtime *r = calloc(1, sizeof(*r));
    struct vdma_snapshot_processor processor = {.type = 1, .version = 1, .process = no_publish};
    int result = 1, started = 0, exported[4] = {-1, -1, -1, -1};
    uint64_t frames = 0, start = now_ns();
    if (!r || vdma_snapshot_init_processor(r, 30, 0, &processor) ||
        vdma_snapshot_enable_rgb(r, fd, 3)) {
        perror("pair initialize"); goto done;
    }
    for (unsigned slot = 0; slot < 4; ++slot) {
        struct vdma_export request = {slot, 0};
        exported[slot] = ioctl(fd, VDMA_EXPORT, &request);
        if (exported[slot] < 0) { perror("pair export"); goto done; }
    }
    if (ioctl(fd, VDMA_START)) { perror("pair start"); goto done; }
    started = 1;
    while (now_ns() - start < UINT64_C(12000000000)) {
        if (vdma_snapshot_rgb_drain(r)) { perror("pair RGB"); break; }
        struct vdma_buffer frame;
        if (ioctl(fd, VDMA_ACQUIRE_NEXT, &frame)) {
            if (errno != EAGAIN && errno != EINTR) { perror("pair acquire"); break; }
            struct timespec pause = {.tv_nsec = 500000};
            nanosleep(&pause, NULL);
            continue;
        }
        struct vdma_frame_identity id = {.version = 1, .size = sizeof(id),
            .slot = frame.slot, .token = frame.token};
        int error = ioctl(fd, VDMA_FRAME_IDENTITY, &id) ? errno : 0;
        if (!error && (!vdma_frame_valid(frame.slot, &frame.header) ||
            frame.header.sequence != id.frame_sequence || frame.header.layout != id.layout ||
            id.source_id != 2 || id.layout != 2 || id.flags ||
            id.canvas_width != 1920 || id.canvas_height != 1080)) error = EPROTO;
        if (!error) {
            vdma_snapshot_observe(r, &id);
            if (vdma_snapshot_sample(r, fd, &id, &frame.header, now_ns())) error = errno;
        }
        struct vdma_release_buffer release = {frame.slot, frame.token};
        if (ioctl(fd, VDMA_RELEASE, &release)) error = errno;
        frames++;
        if (error) { errno = error; perror("pair held input"); break; }
        struct vdma_snapshot_rgb_stats stats;
        vdma_snapshot_rgb_get_stats(r, &stats);
        pthread_mutex_lock(&r->mutex);
        int worker_error = r->worker_error;
        pthread_mutex_unlock(&r->mutex);
        if (worker_error || stats.mismatched || stats.verified >= 30) {
            result = stats.verified >= 30 && !worker_error && !stats.mismatched ? 0 : 1;
            break;
        }
    }
done:
    if (r && r->initialized) {
        vdma_snapshot_stop(r);
        if (r->worker_error) result = 1;
        vdma_snapshot_report(r, stdout);
    }
    if (started && ioctl(fd, VDMA_STOP)) { perror("pair video stop"); result = 1; }
    for (unsigned i = 0; i < 4; ++i) if (exported[i] >= 0) close(exported[i]);
    if (r) vdma_snapshot_destroy(r);
    close(fd);
    printf("PAIR_PROBE frames=%" PRIu64 " elapsed_s=%.6f pass=%u public_results=0 "
        "source_status_gate_tested=0\n", frames, (now_ns() - start) / 1e9, !result);
    free(r);
    return result;
}
