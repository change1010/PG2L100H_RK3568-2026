#ifndef VIDEO_DMA_DARK_H
#define VIDEO_DMA_DARK_H

#include "video_dma_dark_uapi.h"

/* Use the video owner's existing fd; do not open a second owner for Qt.
 * Returns 0 or negative errno. No automatic command retry or startup enable.
 */
int vdma_dark_get(int fd, struct vdma_dark_status *status);
int vdma_dark_command(int fd, struct vdma_dark_command *command);
int vdma_dark_wait(int fd, const struct vdma_dark_command *command,
                   unsigned int timeout_ms, struct vdma_dark_status *status);

#endif
