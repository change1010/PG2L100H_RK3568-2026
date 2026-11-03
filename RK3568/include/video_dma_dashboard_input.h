#ifndef VIDEO_DMA_DASHBOARD_INPUT_H
#define VIDEO_DMA_DASHBOARD_INPUT_H
#include <stddef.h>
/* Numeric editor: replace on first digit; keep invalid text visible rather
 * than clamp and silently apply another value. */
struct vdma_ui_number { char text[5]; int replace; };
void vdma_ui_number_set(struct vdma_ui_number *n, unsigned int value);
void vdma_ui_number_digit(struct vdma_ui_number *n, unsigned int digit);
void vdma_ui_number_backspace(struct vdma_ui_number *n);
int vdma_ui_number_value(const struct vdma_ui_number *n, unsigned int *value);
#endif
