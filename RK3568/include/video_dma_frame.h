#ifndef VIDEO_DMA_FRAME_H
#define VIDEO_DMA_FRAME_H

#include "video_dma_uapi.h"

static inline int vdma_frame_valid(__u32 slot, const struct vdma_frame_header *header)
{
    return slot < 4 && header->magic == 0x5646524dU &&
           header->width == 1920 && header->height == 1080 &&
           header->stride == 3840 && header->bytes == 4147200 &&
           header->format == 0x36314752U && header->layout <= VDMA_LAYOUT_MAX;
}

static inline const char *vdma_layout_name(__u32 layout)
{
    switch (layout) {
    case VDMA_LAYOUT_CMOS1: return "cmos1";
    case VDMA_LAYOUT_CMOS2: return "cmos2";
    case VDMA_LAYOUT_HDMI: return "hdmi";
    case VDMA_LAYOUT_ETHERNET: return "ethernet";
    case VDMA_LAYOUT_QUAD: return "quad";
    default: return "invalid";
    }
}

#endif
