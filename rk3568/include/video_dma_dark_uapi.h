#ifndef VIDEO_DMA_DARK_UAPI_H
#define VIDEO_DMA_DARK_UAPI_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define VDMA_DARK_API_VERSION 1U
#define VDMA_DARK_APPLY 1U
#define VDMA_DARK_DISABLE 2U
#define VDMA_DARK_RESET_DEFAULTS 4U
/* Software operation: edit shadows without writing hardware CONTROL. */
#define VDMA_DARK_STAGE 8U
#define VDMA_DARK_ENABLED 1U
#define VDMA_DARK_PENDING 2U
#define VDMA_DARK_ACTIVE_VALID 16U
#define VDMA_DARK_IMAGE_APPLIED 1U
#define VDMA_DARK_IMAGE_CAPTURE 2U

struct vdma_dark_status {
    __u32 version, size;
    __u32 hardware_version, capabilities;
    __u32 status, error;
    __u32 shadow_threshold, shadow_strength;
    __u32 active_threshold, active_strength;
    __u32 generation, request;
    __u32 image_flags, image_generation, image_request;
    __u32 applied_epoch, applied_sequence, applied_source;
    __u32 capture_epoch, capture_sequence, capture_source;
    __u32 revision, reserved[4];
};

/* expected_request prevents a stale UI from silently replacing a new state.
 * APPLY/DISABLE return the allocated request; DEFAULTS/STAGE return request=0.
 * Successful submit means accepted, not applied. On ANY ambiguous failure,
 * query status before deciding what to do; timeout never cancels a command.
 * threshold/strength must be zero except for APPLY/STAGE. All reserved/flags are zero.
 */
struct vdma_dark_command {
    __u32 version, size, opcode, threshold, strength;
    __u32 expected_request, request, flags, reserved[4];
};

#define VDMA_DARK_STATUS _IOWR('V', 0x51, struct vdma_dark_status)
#define VDMA_DARK_COMMAND _IOWR('V', 0x52, struct vdma_dark_command)

#endif
