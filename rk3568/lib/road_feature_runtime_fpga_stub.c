#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <time.h>
#include "road_feature_runtime.h"

/*
 * FPGA-road production has no legacy image-feature lease/packet consumer.
 * Keep the display owner's timing ABI available while making accidental
 * --road-features use fail closed instead of linking the retired runtime.
 */
uint64_t vdma_road_now_ns(void)
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t)) return 0;
    return (uint64_t)t.tv_sec * UINT64_C(1000000000) + (uint64_t)t.tv_nsec;
}

int vdma_road_runtime_init(struct vdma_road_runtime *runtime, int fd)
{
    (void)runtime; (void)fd;
    errno = EOPNOTSUPP;
    return -1;
}

int vdma_road_runtime_set_profile(struct vdma_road_runtime *runtime,
    const struct vdma_feature_config *profile)
{
    (void)runtime; (void)profile;
    errno = EOPNOTSUPP;
    return -1;
}

int vdma_road_runtime_video_at(struct vdma_road_runtime *runtime,
    const struct vdma_frame_identity *identity, uint64_t observed_ns)
{
    (void)runtime; (void)identity; (void)observed_ns;
    errno = EOPNOTSUPP;
    return -1;
}

int vdma_road_runtime_step(struct vdma_road_runtime *runtime)
{
    (void)runtime;
    errno = EOPNOTSUPP;
    return -1;
}

int vdma_road_runtime_take(struct vdma_road_runtime *runtime,
    struct vdma_road_runtime_event *event)
{
    (void)runtime; (void)event;
    errno = EOPNOTSUPP;
    return -1;
}

int vdma_road_runtime_close(struct vdma_road_runtime *runtime)
{
    (void)runtime;
    return 0;
}
