#ifndef VIDEO_DMA_DARK_CORE_H
#define VIDEO_DMA_DARK_CORE_H

#include "video_dma_dark_uapi.h"
#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/string.h>
#else
#include <errno.h>
#include <string.h>
#endif

/* BAR byte offsets; writes always cover the aligned low 32 bits. */
#define DARK_MAGIC 0x1600U
#define DARK_VERSION 0x1610U
#define DARK_STATUS 0x1620U
#define DARK_THRESHOLD 0x1630U
#define DARK_STRENGTH 0x1640U
#define DARK_CONTROL 0x1650U
#define DARK_ACTIVE_THRESHOLD 0x1660U
#define DARK_ACTIVE_STRENGTH 0x1670U
#define DARK_GENERATION 0x1680U
#define DARK_REQUEST 0x1690U
#define DARK_ERROR 0x16a0U
#define DARK_CAPS 0x16b0U
#define DARK_ACTIVE_REQUEST 0x16c0U
#define DARK_IMAGE_FLAGS 0x16d0U
#define DARK_REVISION 0x1760U
#define DARK_BAR_BYTES 0x1770U

/* Caller holds the existing owner's mutex for the WHOLE operation.
 * read must order preceding writes (readl/writel in the kernel backend).
 * No pixel-boundary polling or sleeping while holding this mutex.
 */
struct vdma_dark_io {
    void *context;
    __u32 (*read)(void *context, __u32 offset);
    void (*write)(void *context, __u32 offset, __u32 value);
};

static inline __u32 vdma_dark_next(__u32 value)
{
    return value == 0xffffffffU ? 1U : value + 1U;
}

static inline int vdma_dark_zero(const __u32 *values, unsigned int count)
{
    unsigned int index;
    for (index = 0; index < count; index++)
        if (values[index])
            return 0;
    return 1;
}

static inline int vdma_dark_probe(const struct vdma_dark_io *io)
{
    __u32 magic = io->read(io->context, DARK_MAGIC);
    __u32 version, caps;
    if (magic == 0xffffffffU)
        return -ENODEV;
    if (magic != 0x4441524bU)
        return -EOPNOTSUPP;
    version = io->read(io->context, DARK_VERSION);
    caps = io->read(io->context, DARK_CAPS);
    if (version == 0xffffffffU || caps == 0xffffffffU)
        return -ENODEV;
    if (version != 2 || (caps >> 16) != 2 || (caps & 0x7fU) != 0x7fU)
        return -EOPNOTSUPP;
    return 0;
}

static inline void vdma_dark_read_active(const struct vdma_dark_io *io, __u32 *a)
{
    a[0] = io->read(io->context, DARK_STATUS);
    a[1] = io->read(io->context, DARK_ACTIVE_THRESHOLD);
    a[2] = io->read(io->context, DARK_ACTIVE_STRENGTH);
    a[3] = io->read(io->context, DARK_GENERATION);
    a[4] = io->read(io->context, DARK_ACTIVE_REQUEST);
    a[5] = io->read(io->context, DARK_ERROR);
}

/* revision protects IMAGE only. Independently bracket it with active reads.
 * Return EAGAIN instead of publishing a torn snapshot. A reconnect/reset must
 * create a new software session; this counter is not a hardware reset nonce.
 */
static inline int vdma_dark_read_status(const struct vdma_dark_io *io,
                                       struct vdma_dark_status *out)
{
    struct vdma_dark_status s;
    __u32 before[6], after[6], image[9], revision;
    unsigned int index;
    int result = vdma_dark_probe(io);
    if (result)
        return result;
    memset(&s, 0, sizeof(s));
    vdma_dark_read_active(io, before);
    revision = io->read(io->context, DARK_REVISION);
    for (index = 0; index < 9; index++)
        image[index] = io->read(io->context, DARK_IMAGE_FLAGS + 16U * index);
    s.revision = io->read(io->context, DARK_REVISION);
    s.shadow_threshold = io->read(io->context, DARK_THRESHOLD);
    s.shadow_strength = io->read(io->context, DARK_STRENGTH);
    vdma_dark_read_active(io, after);
    s.capabilities = io->read(io->context, DARK_CAPS);
    result = vdma_dark_probe(io);
    if (result)
        return result;
    if (before[0] == 0xffffffffU || after[0] == 0xffffffffU)
        return -ENODEV;
    if (revision != s.revision || memcmp(before, after, sizeof(before)) ||
        !(before[0] & VDMA_DARK_ACTIVE_VALID))
        return -EAGAIN;
    if ((before[0] & ~31U) || before[1] > 255 || before[2] > 255 ||
        before[5] > 2 || s.shadow_threshold > 255 || s.shadow_strength > 255 ||
        (image[0] & ~3U) || ((image[0] & 1U) && image[5] > 4) ||
        ((image[0] & 2U) && (!(image[0] & 1U) || image[8] > 4)))
        return -EPROTO;
    s.version = VDMA_DARK_API_VERSION;
    s.size = sizeof(s);
    s.hardware_version = 2;
    s.status = before[0];
    s.active_threshold = before[1];
    s.active_strength = before[2];
    s.generation = before[3];
    s.request = before[4];
    s.error = before[5];
    s.image_flags = image[0];
    s.image_generation = image[1];
    s.image_request = image[2];
    s.applied_epoch = image[3];
    s.applied_sequence = image[4];
    s.applied_source = image[5];
    s.capture_epoch = image[6];
    s.capture_sequence = image[7];
    s.capture_source = image[8];
    *out = s;
    return 0;
}

static inline int vdma_dark_image_confirmed(const struct vdma_dark_status *s)
{
    return (s->status & VDMA_DARK_ACTIVE_VALID) &&
        !(s->status & VDMA_DARK_PENDING) && !s->error &&
        (s->image_flags & VDMA_DARK_IMAGE_APPLIED) &&
        s->image_generation == s->generation && s->image_request == s->request;
}

static inline int vdma_dark_submit(const struct vdma_dark_io *io,
                                  struct vdma_dark_command *command)
{
    struct vdma_dark_status s;
    __u32 next, status, error;
    int result;
    if (command->version != VDMA_DARK_API_VERSION || command->size != sizeof(*command) ||
        command->flags || command->request || !vdma_dark_zero(command->reserved, 4) ||
        (command->opcode != VDMA_DARK_APPLY && command->opcode != VDMA_DARK_DISABLE &&
         command->opcode != VDMA_DARK_RESET_DEFAULTS && command->opcode != VDMA_DARK_STAGE) ||
        command->threshold > 255 || command->strength > 255 ||
        (command->opcode != VDMA_DARK_APPLY && command->opcode != VDMA_DARK_STAGE &&
                                   (command->threshold || command->strength)))
        return -EINVAL;
    result = vdma_dark_read_status(io, &s);
    if (result)
        return result;
    if (s.status & VDMA_DARK_PENDING)
        return -EBUSY;
    if (s.request != command->expected_request)
        return -ESTALE;
    if ((command->opcode == VDMA_DARK_APPLY || command->opcode == VDMA_DARK_DISABLE) &&
        s.generation &&
        !vdma_dark_image_confirmed(&s))
        return -EAGAIN;
    if (command->opcode == VDMA_DARK_RESET_DEFAULTS) {
        io->write(io->context, DARK_CONTROL, VDMA_DARK_RESET_DEFAULTS);
        /* Flush posted writes and verify shadows; this command has no request. */
        status = io->read(io->context, DARK_STATUS);
        if (status == 0xffffffffU)
            return -ENODEV;
        if (io->read(io->context, DARK_ERROR) ||
            io->read(io->context, DARK_THRESHOLD) != 64 ||
            io->read(io->context, DARK_STRENGTH) != 128)
            return -EIO;
        return 0;
    }
    next = vdma_dark_next(s.request);
    if (command->opcode == VDMA_DARK_APPLY || command->opcode == VDMA_DARK_STAGE) {
        io->write(io->context, DARK_THRESHOLD, command->threshold);
        io->write(io->context, DARK_STRENGTH, command->strength);
        if (io->read(io->context, DARK_THRESHOLD) != command->threshold ||
            io->read(io->context, DARK_STRENGTH) != command->strength)
            return -EIO;
    }
    if (command->opcode == VDMA_DARK_STAGE) {
        status = io->read(io->context, DARK_STATUS);
        error = io->read(io->context, DARK_ERROR);
        if (status == 0xffffffffU || error == 0xffffffffU)
            return -ENODEV;
        return error ? -EIO : 0;
    }
    io->write(io->context, DARK_REQUEST, next);
    if (io->read(io->context, DARK_REQUEST) != next)
        return -EIO;
    io->write(io->context, DARK_CONTROL, command->opcode);
    status = io->read(io->context, DARK_STATUS); /* posted write flush */
    error = io->read(io->context, DARK_ERROR);
    if (status == 0xffffffffU || error == 0xffffffffU)
        return -ENODEV;
    if (error || (!(status & VDMA_DARK_PENDING) &&
                  io->read(io->context, DARK_ACTIVE_REQUEST) != next))
        return -EIO; /* Ambiguous after CONTROL: inspect state, never blind retry. */
    command->request = next;
    return 0;
}

#endif
