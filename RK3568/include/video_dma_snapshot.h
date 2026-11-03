#ifndef VIDEO_DMA_SNAPSHOT_H
#define VIDEO_DMA_SNAPSHOT_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "video_dma_snapshot_uapi.h"
#include "video_dma_rgb_uapi.h"
#include "video_dma_road_ref_uapi.h"
#include "../reference/road_ref/road_ref.h"

#define VDMA_SNAPSHOT_BUFFERS 3
#define VDMA_SNAPSHOT_LATENCY_BUCKETS 1001
#define VDMA_SNAPSHOT_SOURCE_COUNT 4
#define VDMA_SNAPSHOT_RESULT_VERSION 1
#define VDMA_SNAPSHOT_PAYLOAD_BYTES 256
#define VDMA_SNAPSHOT_PROCESSOR_CRC32 1
#define VDMA_SNAPSHOT_GRAY_WIDTH 640U
#define VDMA_SNAPSHOT_GRAY_HEIGHT 360U
#define VDMA_SNAPSHOT_GRAY_BYTES (VDMA_SNAPSHOT_GRAY_WIDTH * VDMA_SNAPSHOT_GRAY_HEIGHT)
#define VDMA_SNAPSHOT_YC_BYTES (VDMA_SNAPSHOT_GRAY_BYTES * 2U)
#define VDMA_SNAPSHOT_GRAY_CACHE 8

/* Application-owned pixels, valid only during the processor callback. */
struct vdma_snapshot_gray {
    const unsigned char *pixels;
    struct vdma_plane_frame_v2 descriptor;
    /* V3 uses the same Y pointer but carries its full descriptor here. */
    uint32_t format;
    const unsigned char *chroma;
    struct vdma_plane_frame_v3 yc_descriptor;
};

struct vdma_snapshot_yc {
    const unsigned char *y;
    const unsigned char *c;
    struct vdma_plane_frame_v3 descriptor;
    bool valid;
};

static inline bool vdma_snapshot_gray_matches(const struct vdma_snapshot_gray *gray,
                                              const struct vdma_frame_identity *id)
{
    if (gray->format == VDMA_PLANE_FORMAT_YC8) {
        const struct vdma_plane_frame_v3 *d = &gray->yc_descriptor;
        return gray->pixels && gray->chroma && d->frame.version == 3 &&
            d->frame.size == sizeof(*d) &&
            d->frame.flags == (VDMA_PLANE_SOURCE_ID | VDMA_PLANE_YC) &&
            d->frame.format == VDMA_PLANE_FORMAT_YC8 &&
            d->frame.geometry == ((VDMA_SNAPSHOT_GRAY_HEIGHT << 16) |
                                  VDMA_SNAPSHOT_GRAY_WIDTH) &&
            d->frame.length == VDMA_SNAPSHOT_YC_BYTES &&
            d->y_offset == 0 && d->y_length == VDMA_SNAPSHOT_GRAY_BYTES &&
            d->c_offset == VDMA_SNAPSHOT_GRAY_BYTES &&
            d->c_length == VDMA_SNAPSHOT_GRAY_BYTES &&
            d->stride == VDMA_SNAPSHOT_GRAY_WIDTH &&
            d->metadata_version == 3 && d->kind == 3 &&
            !d->frame.reserved[0] && !d->frame.reserved[1] &&
            !d->frame.reserved[2] &&
            id->canvas_width == 1920 && id->canvas_height == 1080 &&
            id->source_id == id->layout && id->source_id < VDMA_SNAPSHOT_SOURCE_COUNT &&
            d->source_epoch == id->stream_epoch && d->source_sequence == id->frame_sequence &&
            d->source_layout == id->layout &&
            d->source_config_generation == id->config_generation;
    }
    const struct vdma_plane_frame_v2 *d = &gray->descriptor;
    return gray->format == VDMA_PLANE_FORMAT_GRAY8 &&
        gray->pixels && d->frame.version == 2 && d->frame.size == sizeof(*d) &&
        d->frame.flags == VDMA_PLANE_SOURCE_ID && d->frame.format == 0 &&
        d->frame.geometry == ((VDMA_SNAPSHOT_GRAY_HEIGHT << 16) | VDMA_SNAPSHOT_GRAY_WIDTH) &&
        d->frame.length == VDMA_SNAPSHOT_GRAY_BYTES &&
        !d->frame.reserved[0] && !d->frame.reserved[1] && !d->frame.reserved[2] &&
        id->canvas_width == 1920 && id->canvas_height == 1080 &&
        id->source_id == id->layout && id->source_id < VDMA_SNAPSHOT_SOURCE_COUNT &&
        d->source_epoch == id->stream_epoch && d->source_sequence == id->frame_sequence &&
        d->source_layout == id->layout && d->source_config_generation == id->config_generation;
}

static inline bool vdma_snapshot_yc_matches(const struct vdma_snapshot_yc *yc,
                                            const struct vdma_frame_identity *id)
{
#ifdef __cplusplus
    struct vdma_snapshot_gray gray{};
#else
    struct vdma_snapshot_gray gray = {0};
#endif
    gray.pixels = yc ? yc->y : NULL;
    gray.format = VDMA_PLANE_FORMAT_YC8;
    gray.chroma = yc ? yc->c : NULL;
    if (yc) gray.yc_descriptor = yc->descriptor;
    return yc && yc->valid && vdma_snapshot_gray_matches(&gray, id);
}

enum vdma_snapshot_status { VDMA_PROCESS_OK, VDMA_PROCESS_FAILED, VDMA_PROCESS_CANCELLED };
struct vdma_snapshot_input {
    const unsigned char *pixels;
    const unsigned char *small_rgb;
    struct vdma_rgb_frame rgb_descriptor;
    struct vdma_snapshot_gray gray;
    struct vdma_snapshot_yc yc;
    struct vdma_frame_header header;
    struct vdma_frame_identity identity;
    uint64_t session, observed_ns, copy_started_ns, copy_finished_ns, processing_started_ns;
    /* Only valid during the callback. Poll between bounded processing stages. */
    bool (*cancelled)(void *context);
    void *cancel_context;
    const RoadRefPlanesFrame *reference_planes;
    struct vdma_native_identity native;
};
struct vdma_snapshot_output {
    int error; /* Positive errno for FAILED; zero for OK/CANCELLED. */
    uint32_t diagnostic_value, payload_bytes;
    unsigned char payload[VDMA_SNAPSHOT_PAYLOAD_BYTES];
};
struct vdma_snapshot_processor {
    uint32_t type, version;
    enum vdma_snapshot_status (*process)(void *context,
        const struct vdma_snapshot_input *input, struct vdma_snapshot_output *output);
    void *context;
};
struct vdma_snapshot_result {
    uint32_t version, size, processor_type, processor_version;
    enum vdma_snapshot_status status;
    struct vdma_frame_identity identity;
    uint64_t session, observed_ns, copy_started_ns, copy_finished_ns;
    uint64_t processing_started_ns, processing_finished_ns;
    struct vdma_snapshot_output output;
};

enum vdma_snapshot_state { SNAP_FREE, SNAP_FILLING, SNAP_PENDING, SNAP_PROCESSING, SNAP_WAIT_GRAY, SNAP_WAIT_RGB };
struct vdma_snapshot_image {
    unsigned char *pixels;
    unsigned char *small_rgb;
    struct vdma_rgb_frame rgb_descriptor;
    unsigned char *gray_pixels;
    unsigned char *chroma_pixels;
    struct vdma_plane_frame_v2 gray_descriptor;
    struct vdma_plane_frame_v3 yc_descriptor;
    bool gray_valid;
    bool yc_valid;
    enum vdma_snapshot_state state;
    struct vdma_frame_identity identity;
    struct vdma_frame_header header;
    uint64_t session, observed_ns, copy_started_ns, copy_finished_ns;
    uint64_t wait_started_ns;
    bool reference_valid;
    RoadRefPlanesFrame reference;
    struct vdma_native_identity native;
};
struct vdma_snapshot_gray_entry {
    unsigned char *pixels;
    unsigned char *chroma_pixels;
    struct vdma_plane_frame_v2 descriptor;
    struct vdma_plane_frame_v3 yc_descriptor;
    uint32_t format;
    bool valid;
};
struct vdma_snapshot_stats {
    uint64_t attempts, copied, stale, errors, skipped_rate, skipped_source, skipped_busy;
    uint64_t replaced, processed, discarded, cancelled, copy_ns, copy_max_ns;
    uint64_t processing_max_ns, latency_us[VDMA_SNAPSHOT_LATENCY_BUCKETS];
    uint64_t processing_latency[VDMA_SNAPSHOT_LATENCY_BUCKETS];
    uint64_t end_to_end_latency[VDMA_SNAPSHOT_LATENCY_BUCKETS], end_to_end_max_ns;
    uint64_t processor_failed, processor_cancelled;
    uint64_t gray_acquired, gray_paired, gray_missing, gray_evicted, gray_errors;
    uint64_t gray_copy_ns, gray_copy_max_ns;
    uint64_t reference_checked, reference_errors, reference_validation_ns, reference_validation_max_ns;
    uint32_t last_crc;
    struct vdma_frame_identity last_identity;
    uint64_t source_copied[VDMA_SNAPSHOT_SOURCE_COUNT];
    uint64_t source_processed[VDMA_SNAPSHOT_SOURCE_COUNT];
    uint64_t source_discarded[VDMA_SNAPSHOT_SOURCE_COUNT];
    uint64_t source_errors[VDMA_SNAPSHOT_SOURCE_COUNT];
    uint64_t source_max_span_ns[VDMA_SNAPSHOT_SOURCE_COUNT];
};
struct vdma_snapshot_rgb_stats {
    uint64_t received, paired, missing, evicted, errors, copy_ns, wait_ns;
    uint64_t verified, mismatched;
};
struct vdma_snapshot_runtime {
    pthread_mutex_t mutex;
    pthread_cond_t ready;
    pthread_t thread;
    bool initialized, started, stopping, bound;
    int pending;
    int waiting_gray;
    unsigned int delay_ms;
    uint64_t session, period_ns, next_sample_ns, context_started_ns;
    struct vdma_frame_identity context;
    struct vdma_snapshot_image images[VDMA_SNAPSHOT_BUFFERS];
    struct vdma_snapshot_stats stats;
    void *rga;
    bool rga_reference;
    int worker_error;
    struct vdma_snapshot_processor processor;
    struct vdma_snapshot_result latest_result;
    bool result_valid;
    bool plane_enabled;
    bool plane_direct;
    void *plane_mapping;
    size_t plane_mapping_bytes;
    uint32_t plane_mapping_offset;
    uint32_t plane_format;
    uint32_t plane_payload_bytes;
    uint32_t plane_y_offset;
    uint32_t plane_y_length;
    uint32_t plane_c_offset;
    uint32_t plane_c_length;
    uint32_t plane_stride;
    unsigned int gray_next;
    struct vdma_snapshot_gray_entry gray_cache[VDMA_SNAPSHOT_GRAY_CACHE];
    struct {
        unsigned char *pixels;
        struct vdma_rgb_frame descriptor;
        bool valid;
    } rgb_cache[8];
    unsigned int rgb_mode, rgb_next;
    int rgb_fd;
    void *rgb_mapping;
    bool rgb_stop_sent, rgb_phase_valid;
    struct vdma_rgb_frame rgb_phase;
    struct vdma_snapshot_rgb_stats rgb_stats;
    unsigned char *rgb_reference;
    FILE *rgb_audit;
};

int vdma_snapshot_probe(int fd);
/* Modes: 1=shadow, 2=small, 3=bounded pixel verification. Owner drains leases;
 * the copy helper and processor access only owned buffers under the mutex. */
int vdma_snapshot_enable_rgb(struct vdma_snapshot_runtime *runtime, int fd, unsigned int mode);
int vdma_snapshot_rgb_drain(struct vdma_snapshot_runtime *runtime);
void vdma_snapshot_rgb_get_stats(struct vdma_snapshot_runtime *runtime,
    struct vdma_snapshot_rgb_stats *stats);
/* Enable before the first observe/sample. The current YC path uses the V3
 * descriptor with source identity; old GRAY8 images continue through V2.
 * Both calls run on the display/owner thread; drain copies at most one DMA
 * window per call. The window is released before any algorithm runs. */
int vdma_snapshot_enable_plane(struct vdma_snapshot_runtime *runtime, int fd);
/* Opt in to a V3 worker driven by plane arrivals. RGB stays on the display
 * path; observe() still supplies the active source/configuration context. */
int vdma_snapshot_enable_plane_direct(struct vdma_snapshot_runtime *runtime);
int vdma_snapshot_plane_drain(struct vdma_snapshot_runtime *runtime, int fd);
int vdma_snapshot_init(struct vdma_snapshot_runtime *runtime, unsigned int hz,
                        unsigned int delay_ms);
/* The processor descriptor is copied before worker startup and fixed until destroy.
 * Its context must remain alive until stop joins the worker. The callback runs
 * without the runtime mutex; pixels/input/cancel_context cannot be retained.
 * Output is copied by value; payload must never contain borrowed pointers.
 * Stop is cooperative: custom processors must poll input->cancelled and bound
 * individual blocking operations. Do not call stop/destroy from the callback.
 */
int vdma_snapshot_init_processor(struct vdma_snapshot_runtime *runtime, unsigned int hz,
    unsigned int delay_ms, const struct vdma_snapshot_processor *processor);
/* Copies the latest envelope; ENODATA after initialization/invalidation/stop. */
int vdma_snapshot_get_result(struct vdma_snapshot_runtime *runtime,
                             struct vdma_snapshot_result *result);
/* Default CRC processing is a transport diagnostic, not a YOLO result.
 * last_crc/last_identity are cleared on invalidation and stop.
 * Acquire/release/STOP stay on the display owner. A copy helper may query a
 * retained slot; its owner must join/reap before releasing that slot.
 * The processor worker reads only owned copies.
 * Observe every acquired layout, even if it will not be sampled.
 */
void vdma_snapshot_observe(struct vdma_snapshot_runtime *runtime,
                           const struct vdma_frame_identity *identity);
void vdma_snapshot_invalidate(struct vdma_snapshot_runtime *runtime);
int vdma_snapshot_sample(struct vdma_snapshot_runtime *runtime, int fd,
                         const struct vdma_frame_identity *identity,
                         const struct vdma_frame_header *header, uint64_t observed_ns);
/* Owner copies a validated native frame into an existing free worker buffer.
 * Native and display identities have already been matched by the transport.
 * Returns 1 queued, 0 deliberately skipped, -1 invalid or transport failure. */
int vdma_snapshot_submit_reference(struct vdma_snapshot_runtime *runtime,
    const struct vdma_frame_identity *display, const struct vdma_native_identity *native,
    const RoadRefPlanesFrame *planes, uint64_t observed_ns);
/* Copy first, then synchronously validate the owned contiguous planes before
 * publishing them to the worker. The validator returns 0 or -1 with errno.
 * It runs without the runtime mutex and must not retain the borrowed views. */
int vdma_snapshot_submit_reference_checked(struct vdma_snapshot_runtime *runtime,
    const struct vdma_frame_identity *display, const struct vdma_native_identity *native,
    const RoadRefPlanesFrame *planes, uint64_t observed_ns,
    int (*validate)(void *context, const RoadRefPlanesFrame *owned), void *context);
/* Hardware copy uses the leased RR slot, never the matched display slot.
 * Requires the reference RGA backend and a mandatory owned-plane validator. */
int vdma_snapshot_submit_reference_rga_checked(struct vdma_snapshot_runtime *runtime,
    const struct vdma_frame_identity *display, const struct vdma_native_identity *native,
    const RoadRefPlanesFrame *planes, uint64_t observed_ns, unsigned int rr_slot,
    int (*validate)(void *context, const RoadRefPlanesFrame *owned), void *context);
void vdma_snapshot_stop(struct vdma_snapshot_runtime *runtime);
void vdma_snapshot_report(struct vdma_snapshot_runtime *runtime, FILE *output);
void vdma_snapshot_destroy(struct vdma_snapshot_runtime *runtime);
uint32_t vdma_snapshot_crc32(const unsigned char *pixels, uint32_t bytes);

#endif
