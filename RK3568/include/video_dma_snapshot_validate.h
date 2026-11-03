#ifndef VIDEO_DMA_SNAPSHOT_VALIDATE_H
#define VIDEO_DMA_SNAPSHOT_VALIDATE_H

#ifdef __KERNEL__
#include <linux/errno.h>
#else
#include <errno.h>
#endif
#include "video_dma_snapshot_uapi.h"

static inline int vdma_snapshot_request_valid(const struct vdma_snapshot_copy *r)
{
    const struct vdma_frame_identity *id = &r->expected;
    if (r->version != VDMA_SNAPSHOT_VERSION || r->size != sizeof(*r) ||
        r->flags || r->reserved[0] || r->reserved[1] || r->reserved[2] ||
        id->version != 1 || id->size != sizeof(*id) || id->slot >= 4 ||
        id->slot != ((id->token - 1U) & 3U) || id->source_id > VDMA_LAYOUT_MAX ||
        id->source_id != id->layout || id->canvas_width != 1920 ||
        id->canvas_height != 1080 || id->flags)
        return -EINVAL;
    if (r->capacity < VDMA_SNAPSHOT_FRAME_BYTES)
        return -EMSGSIZE;
    if (!r->destination)
        return -EFAULT;
    return 0;
}

static inline int vdma_snapshot_identity_equal(const struct vdma_frame_identity *a,
                                                const struct vdma_frame_identity *b)
{
    return a->version == b->version && a->size == b->size &&
        a->slot == b->slot && a->token == b->token &&
        a->stream_epoch == b->stream_epoch && a->frame_sequence == b->frame_sequence &&
        a->source_id == b->source_id && a->layout == b->layout &&
        a->config_generation == b->config_generation &&
        a->canvas_width == b->canvas_width && a->canvas_height == b->canvas_height &&
        a->flags == b->flags;
}

#endif
