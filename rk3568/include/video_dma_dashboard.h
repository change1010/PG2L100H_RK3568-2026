#ifndef VIDEO_DMA_DASHBOARD_H
#define VIDEO_DMA_DASHBOARD_H
#include <stdint.h>
#include "video_dma_feature_uapi.h"
#include "road_feature_runtime.h"
#include "road_overlay_geometry.h"
#include "detection_overlay.h"
#include "adas_decision.h"
#include "video_dma_uapi.h"
struct yolo_adas;
struct vdma_snapshot_input;
struct vdma_snapshot_output;
#ifdef __cplusplus
extern "C" {
#endif
/* Calls below belong to the one C video/DRM owner. Qt only renders and queues. */
int vdma_dashboard_init(int drm, uint32_t crtc, int video);
void vdma_dashboard_tick(void);
int vdma_dashboard_flip(uint32_t video_fb, const struct vdma_frame_identity *target,
                         void *event_data, int ui_only);
int vdma_dashboard_history_enabled(void);
/* Producer callbacks retain owned values only; never take the Qt UI mutex. */
void vdma_dashboard_history_road(const struct road_overlay_geometry *geometry);
void vdma_dashboard_history_detection(const struct detection_overlay *overlay);
void vdma_dashboard_alert_publish(const struct adas_result *result, uint32_t branch, uint64_t now_ns);
void vdma_dashboard_complete(void);
void vdma_dashboard_pause(void);
int vdma_dashboard_close(int detached);
void vdma_dashboard_video(const struct vdma_frame_identity *identity);
void vdma_dashboard_fps(double video_fps);
void vdma_dashboard_adas(const struct road_overlay_geometry *geometry);
void vdma_dashboard_detection(const struct detection_overlay *overlay);
/* Owner-thread callback; the result is borrowed only for this call. */
void vdma_dashboard_decision(const struct adas_result *result, uint64_t now_ns);
/* Called only while the YOLO worker owns this RGB565 snapshot; copies or drops. */
void vdma_dashboard_preview_rgb(const struct vdma_frame_identity *identity, const void *pixels, uint64_t observed_ns);
/* Serialized YOLO worker hook. Returns 1 if a test task consumed this callback,
 * 0 for the unchanged live path. No live-store publication of frozen evidence. */
int vdma_dashboard_test_yolo(struct yolo_adas *producer,
    const struct vdma_snapshot_input *input, struct vdma_snapshot_output *output);
void vdma_dashboard_held(int fd, const struct vdma_frame_identity *identity);
void vdma_dashboard_road(const struct vdma_road_runtime_event *event,
                          const struct vdma_feature_config *profile);
int vdma_dashboard_recognition(void);
int vdma_dashboard_road_recognition(void);
int vdma_dashboard_frame_usable(void);
int vdma_dashboard_take_invalidation(void);
int vdma_dashboard_sync_init(const int sources[4]);
int vdma_dashboard_sync_enabled(void);
int vdma_dashboard_sync_capture(const struct vdma_buffer *buffer,
    const struct vdma_frame_identity *identity, uint64_t observed_ns);
int vdma_dashboard_sync_holds(const struct vdma_buffer *buffer);
/* A positive peek pins its buffer until exactly one submit or cancel. */
int vdma_dashboard_sync_peek(uint64_t now_ns, uint32_t *fb,
    struct vdma_frame_identity *identity, struct vdma_frame_header *header);
void vdma_dashboard_sync_cancel(int index);
void vdma_dashboard_sync_submit(int index);
void vdma_dashboard_sync_complete(uint64_t now_ns);
void vdma_dashboard_sync_stop(void);
#ifdef __cplusplus
}
#endif
#endif
