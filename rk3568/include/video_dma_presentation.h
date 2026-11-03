#ifndef VIDEO_DMA_PRESENTATION_H
#define VIDEO_DMA_PRESENTATION_H
#include <stdint.h>
#include "video_dma_snapshot_uapi.h"

enum vdma_geometry_kind { VDMA_GEOMETRY_NONE, VDMA_GEOMETRY_EXACT, VDMA_GEOMETRY_TRACKED };
struct vdma_present_geometry {
    uint32_t kind;
    struct vdma_frame_identity anchor, target;
    uint64_t observed_ns, valid_until_ns;
};
struct vdma_presentation {
    uint32_t video_fb, overlay_fb;
    struct vdma_frame_identity video;
    uint64_t effective_ns, hold_ns;
    struct vdma_present_geometry road, detection;
};
static inline int vdma_same_context(const struct vdma_frame_identity *a,
                                    const struct vdma_frame_identity *b)
{
    return a->stream_epoch == b->stream_epoch && a->source_id == b->source_id &&
        a->layout == b->layout && a->config_generation == b->config_generation;
}
static inline int vdma_same_frame(const struct vdma_frame_identity *a,
                                  const struct vdma_frame_identity *b)
{
    return vdma_same_context(a, b) && a->frame_sequence == b->frame_sequence;
}
static inline int vdma_present_geometry_valid(const struct vdma_present_geometry *g,
    const struct vdma_frame_identity *video, uint64_t effective, uint64_t hold)
{
    if (g->kind == VDMA_GEOMETRY_NONE) return 1;
    if (g->kind > VDMA_GEOMETRY_TRACKED || !vdma_same_frame(&g->target, video) ||
        !vdma_same_context(&g->anchor, video) || g->observed_ns > effective ||
        hold > UINT64_MAX - effective || effective + hold >= g->valid_until_ns) return 0;
    if (g->kind == VDMA_GEOMETRY_EXACT) return vdma_same_frame(&g->anchor, video);
    /* A target stamp alone never proves tracking; the caller must own the
     * validated image chain. This contract additionally forbids future anchors. */
    const uint32_t delta = video->frame_sequence - g->anchor.frame_sequence;
    return delta > 0 && delta <= 3;
}
static inline int vdma_presentation_valid(const struct vdma_presentation *p)
{
    return p && p->video_fb && p->overlay_fb && p->video.stream_epoch &&
        p->video.source_id < 4 && p->video.layout == p->video.source_id &&
        vdma_present_geometry_valid(&p->road, &p->video, p->effective_ns, p->hold_ns) &&
        vdma_present_geometry_valid(&p->detection, &p->video, p->effective_ns, p->hold_ns);
}
#endif
