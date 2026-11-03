#ifndef VIDEO_DMA_FEATURE_CONSUMER_H
#define VIDEO_DMA_FEATURE_CONSUMER_H

#include "video_dma_feature_uapi.h"
#include "video_dma_features.h"
#include "road_feature_consumer.h"

/* One owner, one outstanding lease. step() reads at most one 2KiB page. */
struct vdma_feature_event {
    struct vdma_color_identity identity;
    uint32_t packet_count, batch_id, received_mask;
    uint64_t frame_observed_ns, feature_received_ns, complete_ready_ns;
    const unsigned char *packets[4];
    const struct vdma_road_result *road; /* non-NULL only for a complete joint input */
    const unsigned char *road_packet;
    uint32_t road_packet_bytes;
};

struct vdma_feature_consumer {
    int fd, configured_source, automatic, leased;
    uint32_t generation, request, profile_mask, offset, analysis_period;
    uint32_t min_value, min_chroma;
    struct vdma_feature_lease lease;
    int joint_mode;
    uint32_t next_type;
    struct vdma_feature_config profiles[4];
    struct {
        struct vdma_color_identity identity;
        uint64_t first_ns,color_ns,road_ns;
        unsigned int mask,used;
        unsigned char color[VDMA_FEATURE_COLOR_PACKET_BYTES];
        unsigned char road_raw[VDMA_FEATURE_ROAD_MAX_PACKET_BYTES];
        uint32_t road_bytes;
        struct vdma_road_result road;
    } pairs[4];
    struct { struct vdma_color_identity identity; uint64_t until_ns; } closed[16];
    unsigned int closed_cursor;
    unsigned char event_color[VDMA_FEATURE_COLOR_PACKET_BYTES];
    struct vdma_road_result event_road;
    unsigned char event_road_raw[VDMA_FEATURE_ROAD_MAX_PACKET_BYTES];
    uint64_t read_pages,read_max_ns,read_latency_us[1001];
    uint64_t step_last_ns,step_max_ns;
    uint64_t joint_matched,road_packets,orphaned,expired,conflicts;
    struct vdma_color_identity last_pair;
    int last_pair_valid;
    struct vdma_color_identity videos[VDMA_COLOR_IDENTITY_CACHE];
    uint64_t video_observed_ns[VDMA_COLOR_IDENTITY_CACHE];
    struct vdma_feature_event event;
    int event_taken;
    uint64_t complete_single_inputs, events, duplicate_inputs, malformed;
    struct vdma_color_identity last_video;
    int last_video_valid, faulted;
    uint64_t last_now_ns;
    /* Test clock injection: replace after init, before the first video/step. */
    uint64_t (*now_ns)(void);
    /* Called once when a transport-consistent input is complete, BEFORE video matching.
     * No borrowed payload and no hardware operations are allowed in this callback. */
    void (*complete_input)(void *, const struct vdma_color_identity *, uint64_t);
    void *complete_context;
    void (*conflicting_input)(void *, const struct vdma_color_identity *, uint64_t);
    uint64_t region_packets[4];
    uint32_t first_drop;
    int drop_valid;
    uint64_t drop_delta;
    unsigned int video_count, video_cursor;
    unsigned char packet[VDMA_FEATURE_COLOR_PACKET_BYTES];
    uint32_t last_packet_sequence;
    uint64_t packets, single_matched, unmatched, stale, read_errors, read_ns;
    uint32_t drop_snapshot;
};
uint64_t vdma_feature_now_ns(void);
int vdma_feature_consumer_init(struct vdma_feature_consumer *consumer, int fd,
                              int source, unsigned int min_value, unsigned int min_chroma);
int vdma_feature_consumer_init_joint(struct vdma_feature_consumer *consumer, int fd);
/* Before first video, change a source profile; applied atomically on source selection. */
int vdma_feature_consumer_set_profile(struct vdma_feature_consumer *consumer,
                                    const struct vdma_feature_config *profile);
int vdma_feature_consumer_video(struct vdma_feature_consumer *consumer,
                               const struct vdma_frame_identity *identity);
/* observed_ns must be captured before automatic CONFIG can block. */
int vdma_feature_consumer_video_at(struct vdma_feature_consumer *consumer,
                                  const struct vdma_frame_identity *identity,
                                  uint64_t observed_ns);
/* Take once after a positive step. Packets remain borrowed until the next
 * video/step/close call. video/step reject an untaken event with EBUSY;
 * close explicitly cancels it. Read the borrowed bytes before any such call.
 */
int vdma_feature_consumer_take_event(struct vdma_feature_consumer *consumer,
                                    struct vdma_feature_event *event);
/* Returns 2 for a single or complete joint result; quad emits no result, 0 while pending. */
int vdma_feature_consumer_step(struct vdma_feature_consumer *consumer);
int vdma_feature_consumer_close(struct vdma_feature_consumer *consumer);
/* Cancel cached input after RK-only configuration commit, preserving ACK. */
int vdma_feature_consumer_reset_inputs(struct vdma_feature_consumer *consumer);
/* Page ioctl histogram: 10us upper edges, capped at 10000us; max stays exact. */
unsigned int vdma_feature_consumer_read_percentile_us(const struct vdma_feature_consumer *consumer,unsigned int percentile);

#endif
