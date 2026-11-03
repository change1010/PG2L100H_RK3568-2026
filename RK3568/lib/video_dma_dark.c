#define _POSIX_C_SOURCE 200809L
#include "video_dma_dark.h"
#include "video_dma_dark_core.h"
#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>

static unsigned long long monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (unsigned long long)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

int vdma_dark_get(int fd, struct vdma_dark_status *status)
{
    struct vdma_dark_status query = {
        .version = VDMA_DARK_API_VERSION, .size = sizeof(query)
    };
    if (!status)
        return -EINVAL;
    if (ioctl(fd, VDMA_DARK_STATUS, &query))
        return -errno;
    if (query.version != VDMA_DARK_API_VERSION || query.size != sizeof(query))
        return -EPROTO;
    *status = query;
    return 0;
}

int vdma_dark_command(int fd, struct vdma_dark_command *command)
{
    if (!command)
        return -EINVAL;
    return ioctl(fd, VDMA_DARK_COMMAND, command) ? -errno : 0;
}

int vdma_dark_wait(int fd, const struct vdma_dark_command *command,
                   unsigned int timeout_ms, struct vdma_dark_status *status)
{
    struct timespec pause = {.tv_sec = 0, .tv_nsec = 5000000};
    unsigned long long deadline = monotonic_ms() + timeout_ms;
    int result;
    if (!command || !status || !command->request || !timeout_ms || timeout_ms > 10000 ||
        (command->opcode != VDMA_DARK_APPLY && command->opcode != VDMA_DARK_DISABLE) ||
        command->request != vdma_dark_next(command->expected_request))
        return -EINVAL;
    for (;;) {
        result = vdma_dark_get(fd, status);
        if (!result) {
            if (status->error)
                return -EIO;
            if (status->request != command->expected_request &&
                status->request != command->request)
                return -ESTALE;
            /* A reset while waiting must not turn into a successful confirmation. */
            if (!status->generation && command->expected_request)
                return -ESTALE;
            if (status->request == command->request && vdma_dark_image_confirmed(status)) {
                if (!!(status->status & VDMA_DARK_ENABLED) !=
                    (command->opcode == VDMA_DARK_APPLY) ||
                    (command->opcode == VDMA_DARK_APPLY &&
                     (status->active_threshold != command->threshold ||
                      status->active_strength != command->strength)))
                    return -EPROTO;
                return 0;
            }
        } else if (result != -EAGAIN) {
            return result;
        }
        if (monotonic_ms() >= deadline)
            return -ETIMEDOUT; /* Hardware still owns a pending command. */
        if (nanosleep(&pause, NULL) && errno == EINTR)
            return -EINTR;
    }
}
