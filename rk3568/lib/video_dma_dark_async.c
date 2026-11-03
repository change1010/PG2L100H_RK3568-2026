#include "video_dma_dark_async.h"
#include "video_dma_dark_core.h"
#include <errno.h>
#include <string.h>

void vdma_dark_async_init(struct vdma_dark_async *a)
{
    memset(a, 0, sizeof(*a)); a->phase = DARK_OFFLINE;
}

int vdma_dark_frame_matches(const struct vdma_dark_status *s,
                             const struct vdma_frame_identity *frame)
{
    if (s->error || (s->status & VDMA_DARK_PENDING) || frame->flags ||
        frame->source_id > 4 || frame->source_id != frame->layout) return 0;
    /* Reset defaults have no applied record. DARK generation is independent
     * from ROAD config_generation; image changes advance the stream epoch. */
    if (!s->generation) return !(s->status & VDMA_DARK_ENABLED);
    if (!vdma_dark_image_confirmed(s)) return 0;
    uint32_t distance = frame->stream_epoch - s->applied_epoch;
    if (distance >= 0x80000000U) return 0;
    return distance || frame->frame_sequence - s->applied_sequence < 0x80000000U;
}

int vdma_dark_async_queue(struct vdma_dark_async *a, unsigned int opcode,
                          unsigned int threshold, unsigned int strength)
{
    if (!a->available || a->phase != DARK_IDLE || (a->actual.status & VDMA_DARK_PENDING)) return -EBUSY;
    if ((opcode != VDMA_DARK_APPLY && opcode != VDMA_DARK_DISABLE && opcode != VDMA_DARK_STAGE) ||
        threshold > 255 || strength > 255 ||
        (opcode == VDMA_DARK_DISABLE && (threshold || strength))) return -EINVAL;
    a->target = (struct vdma_dark_command){.version = 1, .size = sizeof(a->target),
        .opcode = opcode, .threshold = threshold, .strength = strength,
        .expected_request = a->actual.request};
    a->before_generation = a->actual.generation;
    a->phase = DARK_QUEUED; a->error = 0; a->next_poll_ms = 0;
    return 0;
}

void vdma_dark_async_tick(struct vdma_dark_async *a, int fd, uint64_t now_ms)
{
    if (now_ms < a->next_poll_ms) return;
    int waiting = a->phase == DARK_WAIT || a->phase == DARK_TIMEOUT;
    int result = vdma_dark_get(fd, &a->actual);
    a->next_poll_ms = now_ms + (waiting || a->phase == DARK_QUEUED ? 20 : 250);
    if (result) {
        a->available = 0; a->error = result;
        if (waiting && now_ms - a->submitted_ms >= 2000) a->phase = DARK_TIMEOUT;
        else if (!waiting && a->phase != DARK_QUEUED) a->phase = DARK_OFFLINE;
        return;
    }
    a->available = 1;
    if (a->actual.error) { a->phase = DARK_FAULT; a->error = -EIO; return; }
    if (a->phase == DARK_QUEUED) {
        if (a->actual.request != a->target.expected_request ||
            a->actual.generation != a->before_generation || (a->actual.status & VDMA_DARK_PENDING)) {
            a->phase = DARK_FAULT; a->error = -ESTALE; return;
        }
        result = vdma_dark_command(fd, &a->target);
        a->submitted_ms = now_ms;
        /* Even an ambiguous ioctl failure is observed, never resubmitted. */
        a->target.request = a->target.opcode == VDMA_DARK_STAGE ? 0 : vdma_dark_next(a->target.expected_request);
        a->phase = DARK_WAIT; a->error = result;
        if (result == -EINVAL || result == -ESTALE || result == -EBUSY || result == -EACCES || result == -EOPNOTSUPP) {
            a->phase = DARK_FAULT;
        }
        a->next_poll_ms = now_ms + 20;
        return;
    }
    if (waiting) {
        if (a->target.opcode == VDMA_DARK_STAGE) {
            if (a->actual.request == a->target.expected_request &&
                a->actual.generation == a->before_generation &&
                a->actual.shadow_threshold == a->target.threshold &&
                a->actual.shadow_strength == a->target.strength) { a->phase = DARK_IDLE; a->error = 0; return; }
        } else if (a->actual.request == a->target.request && vdma_dark_image_confirmed(&a->actual)) {
            int enabled = a->target.opcode == VDMA_DARK_APPLY;
            if (a->actual.generation != vdma_dark_next(a->before_generation) ||
                !!(a->actual.status & VDMA_DARK_ENABLED) != enabled ||
                (enabled && (a->actual.active_threshold != a->target.threshold ||
                             a->actual.active_strength != a->target.strength))) {
                a->phase = DARK_FAULT; a->error = -EPROTO; return;
            }
            a->phase = DARK_IDLE; a->error = 0; return;
        }
        if ((a->actual.request != a->target.expected_request && a->actual.request != a->target.request) ||
            (!a->actual.generation && a->before_generation)) {
            a->phase = DARK_FAULT; a->error = -ESTALE; return;
        }
        if (now_ms - a->submitted_ms >= 2000) { a->phase = DARK_TIMEOUT; a->error = -ETIMEDOUT; }
        return;
    }
    if (a->phase == DARK_OFFLINE) a->phase = DARK_IDLE;
    if (a->phase == DARK_IDLE) a->error = 0;
}
