#ifndef ROAD_FEATURE_RUNTIME_H
#define ROAD_FEATURE_RUNTIME_H
#include "road_feature_transport.h"
#define VDMA_ROAD_RUNTIME_PENDING 4U
#define VDMA_ROAD_RUNTIME_VIDEOS 64U
#define VDMA_ROAD_RUNTIME_TTL_NS UINT64_C(200000000)
/* Owned raw candidates, not semantic lanes. Shared five-field identity shape. */
struct vdma_road_runtime_event {
    struct vdma_color_identity identity;
    struct vdma_road_result road;
    uint64_t observed_ns, received_ns, ready_ns;
    /* Owned evidence copy; never expose a pending/lease buffer to a writer. */
    unsigned char raw[VDMA_FEATURE_ROAD_MAX_PACKET_BYTES];
    uint32_t bytes;
};
/* One owner. Each step reads <= one page. CONFIG can wait 250ms; capture
 * video observation BEFORE calling video_at. Untaken events block step/video.
 */
struct vdma_road_runtime {
    int fd, configured_source, leased, faulted, closed, event_valid;
    uint32_t request, generation, offset;
    struct vdma_feature_config profiles[4];
    struct vdma_feature_lease lease;
    unsigned char packet[VDMA_FEATURE_ROAD_MAX_PACKET_BYTES];
    uint64_t lease_started_ns, last_now_ns;
    uint64_t (*now_ns)(void);
    struct { struct vdma_color_identity identity; uint64_t observed_ns; }
        videos[VDMA_ROAD_RUNTIME_VIDEOS];
    unsigned int video_count, video_cursor;
    struct {
        struct vdma_color_identity identity;
        struct vdma_road_result road;
        unsigned char raw[VDMA_FEATURE_ROAD_MAX_PACKET_BYTES];
        uint32_t bytes;
        uint64_t received_ns;
        int used;
    } pending[VDMA_ROAD_RUNTIME_PENDING];
    struct vdma_color_identity tombstones[16], last_video, watermark;
    unsigned int tombstone_count, tombstone_cursor;
    int last_video_valid, watermark_valid;
    struct vdma_road_runtime_event event;
    uint64_t packets, matched, expired, duplicate, conflicts, malformed;
    uint64_t orphaned, stale, read_errors, read_pages, read_ns, read_max_ns;
    uint64_t source_packets[4];
};
uint64_t vdma_road_now_ns(void);
/* Probe only; first video chooses its source-specific CONFIG v3. */
int vdma_road_runtime_init(struct vdma_road_runtime *runtime, int fd);
int vdma_road_runtime_set_profile(struct vdma_road_runtime *runtime,
                                  const struct vdma_feature_config *profile);
int vdma_road_runtime_video_at(struct vdma_road_runtime *runtime,
                               const struct vdma_frame_identity *identity, uint64_t observed_ns);
/* 1 owned matched event ready; 0 pending; -1 error. take rechecks TTL. */
int vdma_road_runtime_step(struct vdma_road_runtime *runtime);
int vdma_road_runtime_take(struct vdma_road_runtime *runtime, struct vdma_road_runtime_event *event);
int vdma_road_runtime_close(struct vdma_road_runtime *runtime);
#endif
