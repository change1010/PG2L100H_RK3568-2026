#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include "video_dma_snapshot.h"
#include "video_dma_snapshot_rga.h"
#include "video_dma_snapshot_validate.h"
#include "video_dma_frame.h"

static uint64_t snapshot_now_ns(void)
{
    struct timespec stamp;
    clock_gettime(CLOCK_MONOTONIC, &stamp);
    return (uint64_t)stamp.tv_sec * UINT64_C(1000000000) + stamp.tv_nsec;
}

static int snapshot_source_eligible(uint32_t source_id, uint32_t layout)
{
    return source_id < VDMA_SNAPSHOT_SOURCE_COUNT && source_id == layout;
}

uint32_t vdma_snapshot_crc32(const unsigned char *pixels, uint32_t bytes)
{
    uint32_t crc = UINT32_MAX, table[256];
    for (unsigned int i = 0; i < 256; i++) {
        uint32_t value = i;
        for (unsigned int bit = 0; bit < 8; bit++)
            value = (value >> 1) ^ (UINT32_C(0xedb88320) & (0U - (value & 1U)));
        table[i] = value;
    }
    for (uint32_t i = 0; i < bytes; i++)
        crc = (crc >> 8) ^ table[(crc ^ pixels[i]) & 255U];
    return ~crc;
}

int vdma_snapshot_probe(int fd)
{
    struct vdma_snapshot_caps caps = {.version = 1, .size = sizeof(caps)};
    if (ioctl(fd, VDMA_SNAPSHOT_CAPS, &caps)) return -1;
    if (caps.version != 1 || caps.size != sizeof(caps) ||
        caps.capabilities != VDMA_SNAPSHOT_COPY_HELD ||
        caps.frame_bytes != VDMA_SNAPSHOT_FRAME_BYTES || caps.format != 0x36314752U ||
        caps.width != 1920 || caps.height != 1080 || caps.stride != 3840 ||
        caps.reserved[0] || caps.reserved[1] || caps.reserved[2] || caps.reserved[3]) {
        errno = EPROTO; return -1;
    }
    return 0;
}

static void invalidate_locked(struct vdma_snapshot_runtime *r)
{
    r->session++;
    r->next_sample_ns = 0;
    r->context_started_ns = 0;
    r->stats.last_crc = 0;
    r->result_valid = false;
    memset(&r->latest_result, 0, sizeof(r->latest_result));
    memset(&r->stats.last_identity, 0, sizeof(r->stats.last_identity));
    for (int i = 0; i < VDMA_SNAPSHOT_GRAY_CACHE; i++) r->gray_cache[i].valid = false;
    r->rgb_phase_valid = false;
    for (int i = 0; i < 8; ++i) r->rgb_cache[i].valid = false;
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; ++i)
        if (r->images[i].state == SNAP_WAIT_RGB) {
            r->images[i].state = SNAP_FREE; r->rgb_stats.missing++;
        }
    if (r->waiting_gray >= 0) {
        struct vdma_snapshot_image *image = &r->images[r->waiting_gray];
        r->stats.source_discarded[image->identity.source_id]++;
        image->state = SNAP_FREE;
        r->waiting_gray = -1;
        r->stats.cancelled++;
    }
    if (r->pending >= 0) {
        unsigned int source = r->images[r->pending].identity.source_id;
        if (source < VDMA_SNAPSHOT_SOURCE_COUNT) r->stats.source_discarded[source]++;
        r->images[r->pending].state = SNAP_FREE;
        r->pending = -1;
        r->stats.cancelled++;
    }
}

static void queue_locked(struct vdma_snapshot_runtime *r, int index)
{
    if (r->pending >= 0) {
        unsigned int source = r->images[r->pending].identity.source_id;
        r->images[r->pending].state = SNAP_FREE;
        r->stats.replaced++;
        r->stats.source_discarded[source]++;
    }
    r->images[index].state = SNAP_PENDING;
    r->pending = index;
    pthread_cond_signal(&r->ready);
}

#include "video_dma_snapshot_rgb_impl.h"

static bool same_gray_context(const struct vdma_plane_frame_v2 *d,
                              const struct vdma_frame_identity *id)
{
    return d->source_epoch == id->stream_epoch && d->source_layout == id->layout &&
        id->source_id == id->layout && d->source_config_generation == id->config_generation;
}

static bool same_yc_context(const struct vdma_plane_frame_v3 *d,
                            const struct vdma_frame_identity *id)
{
    return d->source_epoch == id->stream_epoch && d->source_layout == id->layout &&
        id->source_id == id->layout &&
        d->source_config_generation == id->config_generation;
}

static bool gray_entry_matches(const struct vdma_snapshot_gray_entry *entry,
                               const struct vdma_frame_identity *id)
{
    if (!entry->valid) return false;
    if (entry->format == VDMA_PLANE_FORMAT_YC8) {
        struct vdma_snapshot_gray gray = {
            .pixels = entry->pixels, .format = VDMA_PLANE_FORMAT_YC8,
            .chroma = entry->chroma_pixels, .yc_descriptor = entry->yc_descriptor};
        return vdma_snapshot_gray_matches(&gray, id);
    }
    struct vdma_snapshot_gray gray = {
        .pixels = entry->pixels, .descriptor = entry->descriptor};
    return vdma_snapshot_gray_matches(&gray, id);
}

static bool gray_entry_same_context(const struct vdma_snapshot_gray_entry *entry,
                                    const struct vdma_frame_identity *id)
{
    return entry->format == VDMA_PLANE_FORMAT_YC8 ?
        same_yc_context(&entry->yc_descriptor, id) :
        same_gray_context(&entry->descriptor, id);
}

static uint32_t gray_entry_sequence(const struct vdma_snapshot_gray_entry *entry)
{
    return entry->format == VDMA_PLANE_FORMAT_YC8 ?
        entry->yc_descriptor.source_sequence : entry->descriptor.source_sequence;
}

static void missing_gray_locked(struct vdma_snapshot_runtime *r)
{
    int index = r->waiting_gray;
    r->waiting_gray = -1;
    r->images[index].gray_valid = false;
    r->images[index].yc_valid = false;
    r->stats.gray_missing++;
    /* Preserve RGB zebra processing. The producer publishes UNKNOWN lanes for
     * this exact video identity, rather than repeating an old valid lane. */
    queue_locked(r, index);
}

/* Only the owner thread calls this, under the runtime mutex. A waiting image
 * already owns RGB, so DRM may release/reuse its original display slot. */
static void pair_waiting_locked(struct vdma_snapshot_runtime *r, uint64_t now)
{
    if (r->waiting_gray < 0) return;
    int index = r->waiting_gray;
    struct vdma_snapshot_image *image = &r->images[index];
    bool passed = false;
    for (int i = 0; i < VDMA_SNAPSHOT_GRAY_CACHE; i++) {
        struct vdma_snapshot_gray_entry *entry = &r->gray_cache[i];
        if (!entry->valid) continue;
        if (gray_entry_matches(entry, &image->identity)) {
            memcpy(image->gray_pixels, entry->pixels, VDMA_SNAPSHOT_GRAY_BYTES);
            image->gray_descriptor = entry->descriptor;
            image->yc_valid = entry->format == VDMA_PLANE_FORMAT_YC8;
            if (image->yc_valid) {
                memcpy(image->chroma_pixels, entry->chroma_pixels,
                       VDMA_SNAPSHOT_GRAY_BYTES);
                image->yc_descriptor = entry->yc_descriptor;
            }
            image->gray_valid = true;
            entry->valid = false;
            r->waiting_gray = -1;
            r->stats.gray_paired++;
            queue_locked(r, index);
            return;
        }
        uint32_t distance = gray_entry_sequence(entry) - image->identity.frame_sequence;
        if (gray_entry_same_context(entry, &image->identity) &&
            distance && distance < 0x80000000U) passed = true;
    }
    if (passed || now - image->wait_started_ns >= UINT64_C(50000000)) {
        missing_gray_locked(r);
    }
}

int vdma_snapshot_enable_plane(struct vdma_snapshot_runtime *r, int fd)
{
    struct vdma_plane_info info = {0};
    struct vdma_mmap_info video = {0};
    struct vdma_plane_caps caps = {
        .version = VDMA_PLANE_CAPS_VERSION, .size = sizeof(caps)};
    if (!r || !r->initialized || r->bound || r->plane_mapping || r->plane_enabled ||
        r->stopping || r->stats.attempts) { errno = EINVAL; return -1; }
    if (ioctl(fd, VDMA_PLANE_INFO, &info) || ioctl(fd, VDMA_MMAP_INFO, &video)) return -1;
    if (info.magic != 0x504c414eU || info.size != sizeof(info) ||
        info.geometry != ((VDMA_SNAPSHOT_GRAY_HEIGHT << 16) | VDMA_SNAPSHOT_GRAY_WIDTH) ||
        info.slot_count != 1 || !(info.control & 1U) ||
        video.version != 1 || video.slots != 4 || video.frame_bytes != VDMA_SNAPSHOT_FRAME_BYTES ||
        !video.slot_bytes || video.slot_bytes > UINT32_MAX / video.slots) {
        errno = EPROTO; return -1;
    }
    if (info.version == 3 && info.format == VDMA_PLANE_FORMAT_YC8) {
        if (ioctl(fd, VDMA_PLANE_CAPS, &caps) ||
            caps.version != VDMA_PLANE_CAPS_VERSION || caps.size != sizeof(caps) ||
            caps.hardware_version != 3 || caps.format != VDMA_PLANE_FORMAT_YC8 ||
            caps.flags != VDMA_PLANE_YC || caps.kind != 3 ||
            caps.metadata_version != 3 ||
            caps.geometry != ((VDMA_SNAPSHOT_GRAY_HEIGHT << 16) |
                              VDMA_SNAPSHOT_GRAY_WIDTH) ||
            caps.stride != VDMA_SNAPSHOT_GRAY_WIDTH ||
            caps.payload_bytes != VDMA_SNAPSHOT_YC_BYTES ||
            caps.y_offset != 0 || caps.y_length != VDMA_SNAPSHOT_GRAY_BYTES ||
            caps.c_offset != VDMA_SNAPSHOT_GRAY_BYTES ||
            caps.c_length != VDMA_SNAPSHOT_GRAY_BYTES ||
            !caps.mmap_bytes ||
            caps.mmap_bytes < caps.payload_bytes ||
            caps.reserved[0] || caps.reserved[1] || caps.reserved[2] || caps.reserved[3]) {
            errno = EPROTO; return -1;
        }
        r->plane_format = caps.format;
        r->plane_payload_bytes = caps.payload_bytes;
        r->plane_y_offset = caps.y_offset;
        r->plane_y_length = caps.y_length;
        r->plane_c_offset = caps.c_offset;
        r->plane_c_length = caps.c_length;
        r->plane_stride = caps.stride;
        r->plane_mapping_bytes = caps.mmap_bytes;
    } else {
        if (info.version != 2 || info.format != VDMA_PLANE_FORMAT_GRAY8) {
            errno = EPROTO; return -1;
        }
        r->plane_format = VDMA_PLANE_FORMAT_GRAY8;
        r->plane_payload_bytes = VDMA_SNAPSHOT_GRAY_BYTES;
        r->plane_y_offset = 0;
        r->plane_y_length = VDMA_SNAPSHOT_GRAY_BYTES;
        r->plane_c_offset = 0;
        r->plane_c_length = 0;
        r->plane_stride = VDMA_SNAPSHOT_GRAY_WIDTH;
        long page = sysconf(_SC_PAGESIZE);
        if (page <= 0) { errno = EPROTO; return -1; }
        r->plane_mapping_bytes =
            ((VDMA_SNAPSHOT_GRAY_BYTES + (size_t)page - 1) / page) * page;
    }
    long page = sysconf(_SC_PAGESIZE);
    if (page <= 0 || video.slot_bytes % (unsigned long)page ||
        r->plane_mapping_bytes % (size_t)page) { errno = EPROTO; return -1; }
    r->plane_mapping_offset = video.slots * video.slot_bytes;
    void *mapping = mmap(NULL, r->plane_mapping_bytes, PROT_READ, MAP_SHARED, fd,
                         (off_t)r->plane_mapping_offset);
    if (mapping == MAP_FAILED) return -1;
    r->plane_mapping = mapping;
    for (int i = 0; i < VDMA_SNAPSHOT_GRAY_CACHE; i++) {
        r->gray_cache[i].pixels = calloc(1, VDMA_SNAPSHOT_GRAY_BYTES);
        if (!r->gray_cache[i].pixels) return -1;
        if (r->plane_format == VDMA_PLANE_FORMAT_YC8) {
            r->gray_cache[i].chroma_pixels = calloc(1, VDMA_SNAPSHOT_GRAY_BYTES);
            if (!r->gray_cache[i].chroma_pixels) return -1;
        }
    }
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) {
        r->images[i].gray_pixels = calloc(1, VDMA_SNAPSHOT_GRAY_BYTES);
        if (!r->images[i].gray_pixels) return -1;
        if (r->plane_format == VDMA_PLANE_FORMAT_YC8) {
            r->images[i].chroma_pixels = calloc(1, VDMA_SNAPSHOT_GRAY_BYTES);
            if (!r->images[i].chroma_pixels) return -1;
        }
    }
    r->plane_enabled = true;
    return 0;
}

int vdma_snapshot_enable_plane_direct(struct vdma_snapshot_runtime *r)
{
    if (!r || !r->initialized || !r->plane_enabled || r->bound ||
        r->plane_format != VDMA_PLANE_FORMAT_YC8 || r->stopping ||
        r->stats.attempts) { errno = EINVAL; return -1; }
    r->plane_direct = true;
    return 0;
}

/* Called after RELEASE under the mutex. RGB identities select the active
 * context; only matching plane identities may enter the finite worker queue. */
static void queue_plane_locked(struct vdma_snapshot_runtime *r,
                                struct vdma_snapshot_gray_entry *entry,
                                uint64_t observed)
{
    if (r->stopping || r->worker_error) {
        entry->valid = false;
        return;
    }
    if (!r->bound || !same_yc_context(&entry->yc_descriptor, &r->context) ||
        !snapshot_source_eligible(r->context.source_id, r->context.layout)) {
        entry->valid = false;
        r->stats.skipped_source++;
        return;
    }
    if (observed < r->next_sample_ns) {
        entry->valid = false;
        r->stats.skipped_rate++;
        return;
    }
    if (!r->next_sample_ns) r->next_sample_ns = observed;
    r->next_sample_ns += ((observed - r->next_sample_ns) / r->period_ns + 1) * r->period_ns;
    r->stats.attempts++;
    int index = -1;
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++)
        if (r->images[i].state == SNAP_FREE) { index = i; break; }
    if (index < 0) {
        r->stats.skipped_busy++;
        entry->valid = false;
        return;
    }
    struct vdma_snapshot_image *image = &r->images[index];
    image->identity = r->context;
    image->reference_valid = false;
    memset(&image->native, 0, sizeof(image->native));
    image->identity.frame_sequence = entry->yc_descriptor.source_sequence;
    image->header = (struct vdma_frame_header){
        .sequence = image->identity.frame_sequence, .width = 1920, .height = 1080,
        .layout = image->identity.layout}; /* bytes=0: callback has no RGB */
    image->session = r->session;
    image->observed_ns = observed;
    image->copy_started_ns = observed;
    memcpy(image->gray_pixels, entry->pixels, VDMA_SNAPSHOT_GRAY_BYTES);
    memcpy(image->chroma_pixels, entry->chroma_pixels, VDMA_SNAPSHOT_GRAY_BYTES);
    image->yc_descriptor = entry->yc_descriptor;
    image->yc_valid = image->gray_valid = true;
    image->copy_finished_ns = snapshot_now_ns();
    entry->valid = false;
    r->stats.copied++;
    r->stats.source_copied[image->identity.source_id]++;
    r->stats.gray_paired++;
    queue_locked(r, index);
}

int vdma_snapshot_plane_drain(struct vdma_snapshot_runtime *r, int fd)
{
    if (!r || !r->initialized || !r->plane_enabled) { errno = EINVAL; return -1; }
    struct vdma_plane_frame_v2 d2 = {0};
    struct vdma_plane_frame_v3 d3 = {0};
    struct vdma_plane_frame *frame = r->plane_format == VDMA_PLANE_FORMAT_YC8 ?
        &d3.frame : &d2.frame;
    unsigned long acquire = r->plane_format == VDMA_PLANE_FORMAT_YC8 ?
        VDMA_PLANE_ACQUIRE_V3 : VDMA_PLANE_ACQUIRE_V2;
    if (ioctl(fd, acquire, r->plane_format == VDMA_PLANE_FORMAT_YC8 ?
              (void *)&d3 : (void *)&d2)) {
        if (errno != EAGAIN && errno != EINTR) {
            pthread_mutex_lock(&r->mutex); r->stats.gray_errors++; pthread_mutex_unlock(&r->mutex);
            return -1;
        }
        pthread_mutex_lock(&r->mutex);
        pair_waiting_locked(r, snapshot_now_ns());
        pthread_mutex_unlock(&r->mutex);
        return 0;
    }
    int error = 0;
    uint64_t observed = snapshot_now_ns();
    if (r->plane_format == VDMA_PLANE_FORMAT_YC8) {
        if (d3.frame.version != 3 || d3.frame.size != sizeof(d3) ||
            d3.frame.flags != (VDMA_PLANE_SOURCE_ID | VDMA_PLANE_YC) ||
            d3.frame.format != VDMA_PLANE_FORMAT_YC8 ||
            d3.frame.geometry != ((VDMA_SNAPSHOT_GRAY_HEIGHT << 16) |
                                  VDMA_SNAPSHOT_GRAY_WIDTH) ||
            d3.frame.length != VDMA_SNAPSHOT_YC_BYTES ||
            d3.frame.offset != r->plane_mapping_offset ||
            d3.y_offset != r->plane_y_offset ||
            d3.y_length != r->plane_y_length ||
            d3.c_offset != r->plane_c_offset ||
            d3.c_length != r->plane_c_length ||
            d3.stride != r->plane_stride ||
            d3.metadata_version != 3 || d3.kind != 3 ||
            d3.source_layout > VDMA_LAYOUT_MAX ||
            d3.frame.reserved[0] || d3.frame.reserved[1] ||
            d3.frame.reserved[2]) error = EPROTO;
    } else if (d2.frame.version != 2 || d2.frame.size != sizeof(d2) ||
        d2.frame.flags != VDMA_PLANE_SOURCE_ID ||
        d2.frame.format != VDMA_PLANE_FORMAT_GRAY8 ||
        d2.frame.geometry != ((VDMA_SNAPSHOT_GRAY_HEIGHT << 16) |
                              VDMA_SNAPSHOT_GRAY_WIDTH) ||
        d2.frame.length != VDMA_SNAPSHOT_GRAY_BYTES ||
        d2.frame.offset != r->plane_mapping_offset ||
        d2.source_layout > VDMA_LAYOUT_MAX || d2.frame.reserved[0] ||
        d2.frame.reserved[1] || d2.frame.reserved[2]) error = EPROTO;
    pthread_mutex_lock(&r->mutex);
    struct vdma_snapshot_gray_entry *entry = &r->gray_cache[r->gray_next];
    if (!error) {
        uint64_t start = snapshot_now_ns();
        /* One sequential read from uncached coherent DMA, then cached CPU work. */
        memcpy(entry->pixels, (const unsigned char *)r->plane_mapping + r->plane_y_offset,
               r->plane_y_length);
        if (r->plane_format == VDMA_PLANE_FORMAT_YC8)
            memcpy(entry->chroma_pixels,
                   (const unsigned char *)r->plane_mapping + r->plane_c_offset,
                   r->plane_c_length);
        uint64_t elapsed = snapshot_now_ns() - start;
        r->stats.gray_copy_ns += elapsed;
        if (elapsed > r->stats.gray_copy_max_ns) r->stats.gray_copy_max_ns = elapsed;
    }
    /* Always give back even a rejected descriptor. Never queue before RELEASE. */
    if (ioctl(fd, VDMA_PLANE_RELEASE, frame)) error = errno;
    if (error) {
        entry->valid = false;
        r->stats.gray_errors++;
    } else {
        r->stats.gray_acquired++;
        if (entry->valid) r->stats.gray_evicted++;
        entry->format = r->plane_format;
        if (r->plane_format == VDMA_PLANE_FORMAT_YC8)
            entry->yc_descriptor = d3;
        else
            entry->descriptor = d2;
        entry->valid = true;
        r->gray_next = (r->gray_next + 1) % VDMA_SNAPSHOT_GRAY_CACHE;
        if (r->plane_direct)
            queue_plane_locked(r, entry, observed);
        else
            pair_waiting_locked(r, snapshot_now_ns());
    }
    pthread_mutex_unlock(&r->mutex);
    if (error) { errno = error; return -1; }
    return 0;
}

struct snapshot_cancel_context {
    struct vdma_snapshot_runtime *runtime;
    uint64_t session;
};

static bool snapshot_cancelled(void *opaque)
{
    struct snapshot_cancel_context *c = opaque;
    pthread_mutex_lock(&c->runtime->mutex);
    bool cancelled = c->runtime->stopping || c->session != c->runtime->session;
    pthread_mutex_unlock(&c->runtime->mutex);
    return cancelled;
}

static enum vdma_snapshot_status snapshot_crc_process(void *opaque,
    const struct vdma_snapshot_input *input, struct vdma_snapshot_output *output)
{
    (void)opaque;
    if (input->cancelled(input->cancel_context)) return VDMA_PROCESS_CANCELLED;
    output->diagnostic_value = vdma_snapshot_crc32(input->pixels, input->header.bytes);
    return VDMA_PROCESS_OK;
}

static void *snapshot_worker(void *opaque)
{
    struct vdma_snapshot_runtime *r = opaque;
    pthread_mutex_lock(&r->mutex);
    while (!r->stopping) {
        while (!r->stopping && r->pending < 0)
            pthread_cond_wait(&r->ready, &r->mutex);
        if (r->stopping) break;
        int index = r->pending;
        struct vdma_snapshot_image *image = &r->images[index];
        r->pending = -1;
        image->state = SNAP_PROCESSING;
        pthread_mutex_unlock(&r->mutex);
        uint64_t started = snapshot_now_ns();
        struct snapshot_cancel_context cancel = {r, image->session};
        struct vdma_snapshot_input input = {
            .pixels = image->header.bytes ? image->pixels : NULL,
            .small_rgb = r->rgb_mode >= 2 ? image->small_rgb : NULL,
            .rgb_descriptor = image->rgb_descriptor,
            .gray = {
                .pixels = image->gray_valid ? image->gray_pixels : NULL,
                .descriptor = image->gray_descriptor,
                .format = image->yc_valid ? VDMA_PLANE_FORMAT_YC8 : 0,
                .chroma = image->yc_valid ? image->chroma_pixels : NULL,
                .yc_descriptor = image->yc_descriptor},
            .yc = {
                .y = image->yc_valid ? image->gray_pixels : NULL,
                .c = image->yc_valid ? image->chroma_pixels : NULL,
                .descriptor = image->yc_descriptor, .valid = image->yc_valid},
            .header = image->header, .identity = image->identity, .session = image->session,
            .observed_ns = image->observed_ns, .copy_started_ns = image->copy_started_ns,
            .copy_finished_ns = image->copy_finished_ns, .processing_started_ns = started,
            .cancelled = snapshot_cancelled, .cancel_context = &cancel};
        input.reference_planes = image->reference_valid ? &image->reference : NULL;
        input.native = image->native;
        struct vdma_snapshot_output output = {0};
        enum vdma_snapshot_status status = VDMA_PROCESS_CANCELLED;
        int cpu_access = r->rga && (input.pixels || input.reference_planes);
        int error = cpu_access &&
            vdma_snapshot_rga_cpu_access(r->rga, index, 1) ? errno : 0;
        int reading = cpu_access && !error;
        if (!error && r->rgb_mode == 3) error = snapshot_rgb_verify(r, &input);
        if (!error) status = r->processor.process(r->processor.context, &input, &output);
        if (reading && vdma_snapshot_rga_cpu_access(r->rga, index, 0) && !error)
            error = errno;
        if (status < VDMA_PROCESS_OK || status > VDMA_PROCESS_CANCELLED ||
            output.payload_bytes > VDMA_SNAPSHOT_PAYLOAD_BYTES ||
            (status == VDMA_PROCESS_FAILED ? output.error <= 0 : output.error != 0)) {
            memset(&output, 0, sizeof(output));
            output.error = EPROTO; status = VDMA_PROCESS_FAILED;
        }
        /* Do not publish callback scratch bytes outside its declared payload. */
        memset(output.payload + output.payload_bytes, 0,
            VDMA_SNAPSHOT_PAYLOAD_BYTES - output.payload_bytes);
        pthread_mutex_lock(&r->mutex);
        if (error) {
            r->worker_error = error; r->stats.errors++;
            invalidate_locked(r); /* A transport failure also revokes the latest envelope. */
            if (image->identity.source_id < VDMA_SNAPSHOT_SOURCE_COUNT)
                r->stats.source_errors[image->identity.source_id]++;
        }
        if (r->delay_ms && !r->stopping) {
            struct timespec until;
            uint64_t deadline = snapshot_now_ns() + (uint64_t)r->delay_ms * 1000000;
            until.tv_sec = deadline / UINT64_C(1000000000);
            until.tv_nsec = deadline % UINT64_C(1000000000);
            while (!r->stopping) {
                int error = pthread_cond_timedwait(&r->ready, &r->mutex, &until);
                if (error == ETIMEDOUT) break;
            }
        }
        uint64_t finished = snapshot_now_ns();
        uint64_t elapsed = finished - started;
        if (elapsed > r->stats.processing_max_ns) r->stats.processing_max_ns = elapsed;
        uint64_t bucket = (elapsed + 99999) / 100000;
        if (bucket >= VDMA_SNAPSHOT_LATENCY_BUCKETS) bucket = VDMA_SNAPSHOT_LATENCY_BUCKETS - 1;
        r->stats.processing_latency[bucket]++;
        if (error || r->stopping || image->session != r->session) {
            r->stats.discarded++;
            if (image->identity.source_id < VDMA_SNAPSHOT_SOURCE_COUNT)
                r->stats.source_discarded[image->identity.source_id]++;
        }
        else {
            unsigned int source = image->identity.source_id;
            r->latest_result = (struct vdma_snapshot_result){
                .version = VDMA_SNAPSHOT_RESULT_VERSION, .size = sizeof(r->latest_result),
                .processor_type = r->processor.type, .processor_version = r->processor.version,
                .status = status, .identity = image->identity, .session = image->session,
                .observed_ns = image->observed_ns, .copy_started_ns = image->copy_started_ns,
                .copy_finished_ns = image->copy_finished_ns, .processing_started_ns = started,
                .processing_finished_ns = finished, .output = output};
            r->result_valid = true;
            if (status == VDMA_PROCESS_OK) {
                uint64_t age = finished >= image->observed_ns ? finished - image->observed_ns : 0;
                if (age > r->stats.end_to_end_max_ns) r->stats.end_to_end_max_ns = age;
                bucket = (age + 99999) / 100000;
                if (bucket >= VDMA_SNAPSHOT_LATENCY_BUCKETS) bucket = VDMA_SNAPSHOT_LATENCY_BUCKETS - 1;
                r->stats.end_to_end_latency[bucket]++;
                r->stats.processed++; r->stats.source_processed[source]++;
                r->stats.last_crc = r->processor.type == VDMA_SNAPSHOT_PROCESSOR_CRC32 ?
                    output.diagnostic_value : 0;
                r->stats.last_identity = image->identity;
            } else {
                r->stats.discarded++; r->stats.source_discarded[source]++;
                r->stats.last_crc = 0;
                memset(&r->stats.last_identity, 0, sizeof(r->stats.last_identity));
                if (status == VDMA_PROCESS_FAILED) {
                    r->stats.processor_failed++; r->stats.source_errors[source]++;
                } else r->stats.processor_cancelled++;
            }
        }
        image->state = SNAP_FREE;
    }
    pthread_mutex_unlock(&r->mutex);
    return NULL;
}

int vdma_snapshot_init(struct vdma_snapshot_runtime *r, unsigned int hz, unsigned int delay_ms)
{
    const struct vdma_snapshot_processor crc = {.type = VDMA_SNAPSHOT_PROCESSOR_CRC32,
        .version = 1, .process = snapshot_crc_process};
    return vdma_snapshot_init_processor(r, hz, delay_ms, &crc);
}

int vdma_snapshot_init_processor(struct vdma_snapshot_runtime *r, unsigned int hz,
    unsigned int delay_ms, const struct vdma_snapshot_processor *processor)
{
    pthread_condattr_t attr;
    int error;
    if (!r || hz < 1 || hz > 30 || delay_ms > 5000 || !processor ||
        !processor->type || !processor->version || !processor->process) {
        errno = EINVAL; return -1;
    }
    struct vdma_snapshot_processor descriptor = *processor;
    memset(r, 0, sizeof(*r));
    r->processor = descriptor;
    r->pending = -1; r->waiting_gray = -1; r->session = 1; r->delay_ms = delay_ms;
    r->period_ns = UINT64_C(1000000000) / hz;
    error = pthread_mutex_init(&r->mutex, NULL);
    if (error) { errno = error; return -1; }
    error = pthread_condattr_init(&attr);
    if (error) { pthread_mutex_destroy(&r->mutex); errno = error; return -1; }
    error = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    if (!error) error = pthread_cond_init(&r->ready, &attr);
    pthread_condattr_destroy(&attr);
    if (error) { pthread_mutex_destroy(&r->mutex); errno = error; return -1; }
    r->initialized = true;
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) {
        r->images[i].pixels = malloc(VDMA_SNAPSHOT_FRAME_BYTES);
        if (!r->images[i].pixels) { vdma_snapshot_destroy(r); return -1; }
        /* Touch each page before DMA starts; this is not a hard real-time guarantee. */
        memset(r->images[i].pixels, 0, VDMA_SNAPSHOT_FRAME_BYTES);
    }
    error = pthread_create(&r->thread, NULL, snapshot_worker, r);
    if (error) { vdma_snapshot_destroy(r); errno = error; return -1; }
    r->started = true;
    return 0;
}

int vdma_snapshot_get_result(struct vdma_snapshot_runtime *r, struct vdma_snapshot_result *result)
{
    if (!r || !r->initialized || !result) { errno = EINVAL; return -1; }
    pthread_mutex_lock(&r->mutex);
    int error = r->result_valid ? 0 : ENODATA;
    if (!error) *result = r->latest_result;
    else memset(result, 0, sizeof(*result));
    pthread_mutex_unlock(&r->mutex);
    if (error) { errno = error; return -1; }
    return 0;
}

void vdma_snapshot_observe(struct vdma_snapshot_runtime *r,
                           const struct vdma_frame_identity *id)
{
    pthread_mutex_lock(&r->mutex);
    if (!r->bound || id->stream_epoch != r->context.stream_epoch ||
        id->source_id != r->context.source_id || id->layout != r->context.layout ||
        id->config_generation != r->context.config_generation) {
        invalidate_locked(r);
        r->context = *id; r->bound = true;
        r->context_started_ns = snapshot_now_ns();
    }
    if (snapshot_source_eligible(id->source_id, id->layout)) {
        uint64_t span = snapshot_now_ns() - r->context_started_ns;
        if (span > r->stats.source_max_span_ns[id->source_id])
            r->stats.source_max_span_ns[id->source_id] = span;
    }
    pthread_mutex_unlock(&r->mutex);
}

void vdma_snapshot_invalidate(struct vdma_snapshot_runtime *r)
{
    if (!r || !r->initialized) return;
    pthread_mutex_lock(&r->mutex);
    invalidate_locked(r); r->bound = false;
    pthread_mutex_unlock(&r->mutex);
}

int vdma_snapshot_sample(struct vdma_snapshot_runtime *r, int fd,
                         const struct vdma_frame_identity *id,
                         const struct vdma_frame_header *header, uint64_t observed)
{
    int index = -1;
    if (r->rga_reference) { errno = EINVAL; return -1; }
    struct vdma_snapshot_copy validation = {.version = 1, .size = sizeof(validation),
        .destination = 1, .capacity = VDMA_SNAPSHOT_FRAME_BYTES, .expected = *id};
    if (vdma_snapshot_request_valid(&validation) || !vdma_frame_valid(id->slot, header) ||
        header->sequence != id->frame_sequence || header->layout != id->layout || !observed) {
        errno = EINVAL; return -1;
    }
    if (r->plane_direct) return 0;
    pthread_mutex_lock(&r->mutex);
    if (r->worker_error) {
        int error = r->worker_error;
        pthread_mutex_unlock(&r->mutex); errno = error; return -1;
    }
    if (r->stopping) { pthread_mutex_unlock(&r->mutex); errno = EPIPE; return -1; }
    if (!r->bound || id->stream_epoch != r->context.stream_epoch ||
        id->source_id != r->context.source_id || id->layout != r->context.layout ||
        id->config_generation != r->context.config_generation) {
        pthread_mutex_unlock(&r->mutex); errno = ESTALE; return -1;
    }
    if (!snapshot_source_eligible(id->source_id, id->layout)) {
        r->stats.skipped_source++; pthread_mutex_unlock(&r->mutex); return 0;
    }
    if (r->rgb_mode && (!r->rgb_phase_valid ||
        r->rgb_phase.epoch != id->stream_epoch ||
        r->rgb_phase.config != id->config_generation ||
        r->rgb_phase.layout != id->layout ||
        ((id->frame_sequence - r->rgb_phase.sequence) & 1U))) {
        r->stats.skipped_rate++; pthread_mutex_unlock(&r->mutex); return 0;
    }
    if (!r->rgb_mode && observed < r->next_sample_ns) {
        r->stats.skipped_rate++; pthread_mutex_unlock(&r->mutex); return 0;
    }
    if (!r->rgb_mode) {
        if (!r->next_sample_ns) r->next_sample_ns = observed;
        r->next_sample_ns += ((observed - r->next_sample_ns) / r->period_ns + 1) * r->period_ns;
    }
    r->stats.attempts++;
    /* A newer sample replaces a still unmatched RGB copy. No unbounded queue. */
    if (r->waiting_gray >= 0) {
        missing_gray_locked(r);
    }
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++)
        if (r->images[i].state == SNAP_FREE) { index = i; break; }
    if (index < 0) {
        r->stats.skipped_busy++; pthread_mutex_unlock(&r->mutex); return 0;
    }
    struct vdma_snapshot_image *image = &r->images[index];
    image->state = SNAP_FILLING; image->session = r->session;
    image->reference_valid = false;
    memset(&image->native, 0, sizeof(image->native));
    image->gray_valid = false;
    image->yc_valid = false;
    image->identity = *id; image->header = *header; image->observed_ns = observed;
    pthread_mutex_unlock(&r->mutex);
    struct vdma_snapshot_copy copy = {.version = 1, .size = sizeof(copy),
        .destination = (uintptr_t)image->pixels, .capacity = VDMA_SNAPSHOT_FRAME_BYTES,
        .expected = *id};
    image->copy_started_ns = snapshot_now_ns();
    int error;
    if (r->rga) {
        /* The display owner retains the slot until this copy call completes. */
        struct vdma_frame_identity query = {.version = 1, .size = sizeof(query),
            .slot = id->slot, .token = id->token};
        struct vdma_frame_identity actual = query;
        error = ioctl(fd, VDMA_FRAME_IDENTITY, &actual) ? errno : 0;
        if (!error && !vdma_snapshot_identity_equal(id, &actual)) error = ESTALE;
        if (!error && vdma_snapshot_rga_copy(r->rga, index, id->slot)) error = errno;
        if (!error) {
            actual = query;
            if (ioctl(fd, VDMA_FRAME_IDENTITY, &actual)) error = errno;
            else if (!vdma_snapshot_identity_equal(id, &actual)) error = ESTALE;
        }
        copy.actual = actual; copy.header = *header; copy.copied_bytes = VDMA_SNAPSHOT_FRAME_BYTES;
    } else error = ioctl(fd, VDMA_COPY_HELD_FRAME, &copy) ? errno : 0;
    image->copy_finished_ns = snapshot_now_ns();
    if (!error && (copy.copied_bytes != VDMA_SNAPSHOT_FRAME_BYTES ||
        !vdma_snapshot_identity_equal(id, &copy.actual) ||
        !vdma_frame_valid(id->slot, &copy.header) ||
        memcmp(header, &copy.header, sizeof(*header)))) error = EPROTO;
    pthread_mutex_lock(&r->mutex);
    uint64_t elapsed = image->copy_finished_ns - image->copy_started_ns;
    r->stats.copy_ns += elapsed;
    if (elapsed > r->stats.copy_max_ns) r->stats.copy_max_ns = elapsed;
    uint64_t bucket = (elapsed + 99999) / 100000;
    if (bucket >= VDMA_SNAPSHOT_LATENCY_BUCKETS) bucket = VDMA_SNAPSHOT_LATENCY_BUCKETS - 1;
    r->stats.latency_us[bucket]++;
    if (error) {
        image->state = SNAP_FREE;
        if (error == ESTALE) r->stats.stale++;
        else { r->stats.errors++; r->stats.source_errors[id->source_id]++; }
        pthread_mutex_unlock(&r->mutex);
        errno = error; return error == ESTALE ? 0 : -1;
    }
    r->stats.copied++;
    r->stats.source_copied[id->source_id]++;
    if (r->stopping || image->session != r->session) {
        image->state = SNAP_FREE; r->stats.cancelled++;
        r->stats.source_discarded[id->source_id]++;
    } else {
        if (r->rgb_mode) {
            image->state = SNAP_WAIT_RGB;
            image->wait_started_ns = snapshot_now_ns();
            rgb_pair_locked(r, image->wait_started_ns);
        } else if (r->plane_enabled) {
            image->state = SNAP_WAIT_GRAY;
            image->wait_started_ns = snapshot_now_ns();
            r->waiting_gray = index;
            pair_waiting_locked(r, image->wait_started_ns);
        } else {
            queue_locked(r, index);
        }
    }
    pthread_mutex_unlock(&r->mutex);
    return 0;
}

int vdma_snapshot_submit_reference(struct vdma_snapshot_runtime *r,
    const struct vdma_frame_identity *id, const struct vdma_native_identity *native,
    const RoadRefPlanesFrame *planes, uint64_t observed)
{
    return vdma_snapshot_submit_reference_checked(r, id, native, planes, observed, NULL, NULL);
}

static int submit_reference_checked(struct vdma_snapshot_runtime *r,
    const struct vdma_frame_identity *id, const struct vdma_native_identity *native,
    const RoadRefPlanesFrame *planes, uint64_t observed, int rr_slot,
    int (*validate)(void *context, const RoadRefPlanesFrame *owned), void *context)
{
    if (!r || !r->initialized || !id || !native || !planes || !observed ||
        id->source_id != VDMA_LAYOUT_HDMI || id->layout != VDMA_LAYOUT_HDMI ||
        id->version != 1 || id->size != sizeof(*id) || id->slot >= 4 ||
        id->canvas_width != 1920 || id->canvas_height != 1080 ||
        native->version != 1 || native->size != sizeof(*native) ||
        native->slot != id->slot || native->token != id->token ||
        planes->size != sizeof(*planes) || planes->abi != ROAD_REF_ABI ||
        planes->plane_count != 3 || planes->flags || planes->source_width != 1920 ||
        planes->source_height != 1080 || planes->source_id != VDMA_LAYOUT_HDMI ||
        planes->source_epoch != native->native_epoch || planes->sequence != native->native_sequence) {
        errno = EINVAL; return -1;
    }
    size_t bytes = 0;
    for (unsigned int i = 0; i < 3; ++i) {
        const RoadRefPlane *p = &planes->planes[i];
        int width = i ? 1280 : 1920, height = i ? 720 : 1080;
        if (!p->data || p->kind != i + 1 || p->width != width || p->height != height ||
            p->stride != (size_t)width || p->data_bytes != (size_t)width * height) {
            errno = EINVAL; return -1;
        }
        bytes += p->data_bytes;
    }
    if (bytes > VDMA_SNAPSHOT_FRAME_BYTES) { errno = EINVAL; return -1; }
    if ((rr_slot >= 0 && (!r->rga || !r->rga_reference || rr_slot >= 3 || !validate)) ||
        (rr_slot < 0 && r->rga)) { errno = EINVAL; return -1; }
    pthread_mutex_lock(&r->mutex);
    if (r->stopping || r->worker_error) {
        pthread_mutex_unlock(&r->mutex); errno = EPIPE; return -1;
    }
    if (!r->bound || id->stream_epoch != r->context.stream_epoch ||
        id->source_id != r->context.source_id || id->layout != r->context.layout ||
        id->config_generation != r->context.config_generation) {
        r->stats.stale++; pthread_mutex_unlock(&r->mutex); return 0;
    }
    int index = -1;
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; ++i)
        if (r->images[i].state == SNAP_FREE) { index = i; break; }
    r->stats.attempts++;
    if (index < 0) {
        r->stats.skipped_busy++; pthread_mutex_unlock(&r->mutex); return 0;
    }
    struct vdma_snapshot_image *image = &r->images[index];
    image->state = SNAP_FILLING; image->session = r->session;
    image->identity = *id; image->native = *native; image->reference = *planes;
    memset(&image->header, 0, sizeof(image->header));
    image->gray_valid = image->yc_valid = false;
    image->reference_valid = true; image->observed_ns = observed;
    pthread_mutex_unlock(&r->mutex);
    image->copy_started_ns = snapshot_now_ns();
    int error = rr_slot >= 0 &&
        vdma_snapshot_rga_copy(r->rga, index, (unsigned int)rr_slot) ? errno : 0;
    int reading = 0;
    if (!error && rr_slot >= 0) {
        if (vdma_snapshot_rga_cpu_access(r->rga, index, 1)) error = errno;
        else reading = 1;
    }
    size_t offset = 0;
    for (unsigned int i = 0; i < 3; ++i) {
        if (rr_slot < 0)
            memcpy(image->pixels + offset, planes->planes[i].data, planes->planes[i].data_bytes);
        image->reference.planes[i].data = image->pixels + offset;
        offset += planes->planes[i].data_bytes;
    }
    image->copy_finished_ns = snapshot_now_ns();
    if (!error && validate && validate(context, &image->reference)) error = errno;
    if (reading && vdma_snapshot_rga_cpu_access(r->rga, index, 0) && !error) error = errno;
    uint64_t validation_ns = validate ? snapshot_now_ns() - image->copy_finished_ns : 0;
    pthread_mutex_lock(&r->mutex);
    uint64_t elapsed = image->copy_finished_ns - image->copy_started_ns;
    r->stats.copy_ns += elapsed;
    if (elapsed > r->stats.copy_max_ns) r->stats.copy_max_ns = elapsed;
    if (validate) {
        r->stats.reference_checked++;
        r->stats.reference_errors += error != 0;
        r->stats.reference_validation_ns += validation_ns;
        if (validation_ns > r->stats.reference_validation_max_ns)
            r->stats.reference_validation_max_ns = validation_ns;
    }
    int queued = !error && image->session == r->session && !r->stopping && offset == bytes;
    if (queued) {
        r->stats.copied++; r->stats.source_copied[id->source_id]++;
        queue_locked(r, index);
    } else {
        image->state = SNAP_FREE; r->stats.discarded++;
        if (error) {
            r->stats.errors++;
            r->stats.source_errors[id->source_id]++;
        }
    }
    pthread_mutex_unlock(&r->mutex);
    if (error) { errno = error; return -1; }
    return queued;
}

int vdma_snapshot_submit_reference_checked(struct vdma_snapshot_runtime *r,
    const struct vdma_frame_identity *id, const struct vdma_native_identity *native,
    const RoadRefPlanesFrame *planes, uint64_t observed,
    int (*validate)(void *context, const RoadRefPlanesFrame *owned), void *context)
{
    return submit_reference_checked(r, id, native, planes, observed, -1, validate, context);
}

int vdma_snapshot_submit_reference_rga_checked(struct vdma_snapshot_runtime *r,
    const struct vdma_frame_identity *id, const struct vdma_native_identity *native,
    const RoadRefPlanesFrame *planes, uint64_t observed, unsigned int rr_slot,
    int (*validate)(void *context, const RoadRefPlanesFrame *owned), void *context)
{
    if (rr_slot >= RR_HOST_SLOT_COUNT) { errno = EINVAL; return -1; }
    return submit_reference_checked(r, id, native, planes, observed, (int)rr_slot, validate, context);
}

void vdma_snapshot_stop(struct vdma_snapshot_runtime *r)
{
    if (!r || !r->initialized) return;
    pthread_mutex_lock(&r->mutex);
    if (!r->stopping) { invalidate_locked(r); r->bound = false; r->stopping = true; }
    pthread_cond_broadcast(&r->ready);
    pthread_mutex_unlock(&r->mutex);
    if (r->started) { pthread_join(r->thread, NULL); r->started = false; }
    if (r->rgb_mode && !r->rgb_stop_sent) {
        struct vdma_rgb_control control = {1, sizeof(control), 0, 0};
        r->rgb_stop_sent = true;
        if (ioctl(r->rgb_fd, VDMA_RGB_CONTROL, &control)) r->worker_error = errno;
    }
}

static unsigned int percentile(const uint64_t *histogram, unsigned int p)
{
    uint64_t total = 0, count = 0;
    for (unsigned int i = 0; i < VDMA_SNAPSHOT_LATENCY_BUCKETS; i++) total += histogram[i];
    if (!total) return 0;
    uint64_t target = (total * p + 99) / 100;
    for (unsigned int i = 0; i < VDMA_SNAPSHOT_LATENCY_BUCKETS; i++) {
        count += histogram[i];
        if (count >= target) return i * 100;
    }
    return 100000;
}

void vdma_snapshot_report(struct vdma_snapshot_runtime *r, FILE *out)
{
    pthread_mutex_lock(&r->mutex);
    struct vdma_snapshot_stats *s = &r->stats;
    const struct vdma_snapshot_rgb_stats *rgb = &r->rgb_stats;
    fprintf(out, "SNAPSHOT_RGB mode=%u received=%" PRIu64 " paired=%" PRIu64
        " missing=%" PRIu64 " evicted=%" PRIu64 " errors=%" PRIu64
        " copy_mean_ms=%.6f wait_mean_ms=%.6f verified=%" PRIu64
        " mismatched=%" PRIu64 "\n", r->rgb_mode, rgb->received, rgb->paired,
        rgb->missing, rgb->evicted, rgb->errors,
        rgb->received ? rgb->copy_ns / 1e6 / rgb->received : 0,
        rgb->paired ? rgb->wait_ns / 1e6 / rgb->paired : 0,
        rgb->verified, rgb->mismatched);
    fprintf(out, "SNAPSHOT attempts=%" PRIu64 " copied=%" PRIu64 " stale=%" PRIu64
        " errors=%" PRIu64 " rate_skip=%" PRIu64 " source_skip=%" PRIu64
        " busy_skip=%" PRIu64 " replaced=%" PRIu64 " processed=%" PRIu64
        " discarded=%" PRIu64 " cancelled=%" PRIu64
        " mean_us=%.3f p50_upper_us=%u p95_upper_us=%u p99_upper_us=%u max_us=%.3f"
        " worker_max_ms=%.3f last_crc=%08x epoch=%u seq=%u source=%u config=%u"
        " owned_bytes=%u session=%" PRIu64 " histogram_cap_us=100000 backend=%s\n",
        s->attempts, s->copied, s->stale, s->errors, s->skipped_rate, s->skipped_source,
        s->skipped_busy, s->replaced, s->processed, s->discarded, s->cancelled,
        s->attempts - s->skipped_busy ? s->copy_ns / 1000.0 / (s->attempts - s->skipped_busy) : 0,
        percentile(s->latency_us, 50), percentile(s->latency_us, 95),
        percentile(s->latency_us, 99), s->copy_max_ns / 1000.0,
        s->processing_max_ns / 1e6, s->last_crc, s->last_identity.stream_epoch,
        s->last_identity.frame_sequence, s->last_identity.source_id,
        s->last_identity.config_generation, VDMA_SNAPSHOT_BUFFERS * VDMA_SNAPSHOT_FRAME_BYTES,
        r->session, r->rga ? "rga" : "cpu");
    fprintf(out, "SNAPSHOT_PROCESSOR type=%u version=%u failed=%" PRIu64
        " cancelled=%" PRIu64 " result_valid=%u\n", r->processor.type,
        r->processor.version, s->processor_failed, s->processor_cancelled, r->result_valid);
    if (s->reference_checked)
        fprintf(out, "SNAPSHOT_REFERENCE checked=%" PRIu64 " errors=%" PRIu64
            " validation_mean_ms=%.3f validation_max_ms=%.3f\n",
            s->reference_checked, s->reference_errors,
            s->reference_validation_ns / 1e6 / s->reference_checked,
            s->reference_validation_max_ns / 1e6);
    fprintf(out, "SNAPSHOT_GRAY enabled=%u acquired=%" PRIu64 " paired=%" PRIu64
        " missing=%" PRIu64 " evicted=%" PRIu64 " errors=%" PRIu64
        " copy_mean_ms=%.3f copy_max_ms=%.3f waiting=%u\n", r->plane_enabled,
        s->gray_acquired, s->gray_paired, s->gray_missing, s->gray_evicted, s->gray_errors,
        s->gray_acquired ? s->gray_copy_ns / 1e6 / s->gray_acquired : 0,
        s->gray_copy_max_ns / 1e6, r->waiting_gray >= 0);
    fprintf(out, "SNAPSHOT_PLANE format=%u payload=%u y_offset=%u y_length=%u "
        "c_offset=%u c_length=%u stride=%u direct=%u\n", r->plane_format,
        r->plane_payload_bytes, r->plane_y_offset, r->plane_y_length,
        r->plane_c_offset, r->plane_c_length, r->plane_stride, r->plane_direct);
    fprintf(out, "SNAPSHOT_LATENCY worker_p95_upper_us=%u end_to_end_p95_upper_us=%u "
        "end_to_end_max_ms=%.3f histogram_cap_us=100000 gray_owned_bytes=%u gray_mapped_bytes=%zu\n",
        percentile(s->processing_latency, 95), percentile(s->end_to_end_latency, 95),
        s->end_to_end_max_ns / 1e6, r->plane_enabled ?
        (VDMA_SNAPSHOT_GRAY_CACHE + VDMA_SNAPSHOT_BUFFERS) *
            (r->plane_format == VDMA_PLANE_FORMAT_YC8 ?
             VDMA_SNAPSHOT_YC_BYTES : VDMA_SNAPSHOT_GRAY_BYTES) : 0,
        r->plane_mapping ? r->plane_mapping_bytes : 0);
    for (unsigned int source = 0; source < VDMA_SNAPSHOT_SOURCE_COUNT; source++)
        fprintf(out, "SNAPSHOT_SOURCE source=%u copied=%" PRIu64 " processed=%" PRIu64
            " discarded=%" PRIu64 " errors=%" PRIu64 " max_span_ms=%.3f\n", source,
            s->source_copied[source], s->source_processed[source],
            s->source_discarded[source], s->source_errors[source],
            s->source_max_span_ns[source] / 1e6);
    pthread_mutex_unlock(&r->mutex);
}

void vdma_snapshot_destroy(struct vdma_snapshot_runtime *r)
{
    if (!r || !r->initialized) return;
    vdma_snapshot_stop(r);
    if (r->rga) { vdma_snapshot_rga_destroy(r->rga); r->rga = NULL; }
    else for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) free(r->images[i].pixels);
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) r->images[i].pixels = NULL;
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; i++) {
        free(r->images[i].gray_pixels); r->images[i].gray_pixels = NULL;
        free(r->images[i].chroma_pixels); r->images[i].chroma_pixels = NULL;
    }
    for (int i = 0; i < VDMA_SNAPSHOT_GRAY_CACHE; i++) {
        free(r->gray_cache[i].pixels); r->gray_cache[i].pixels = NULL;
        free(r->gray_cache[i].chroma_pixels); r->gray_cache[i].chroma_pixels = NULL;
    }
    if (r->plane_mapping) munmap(r->plane_mapping, r->plane_mapping_bytes);
    for (int i = 0; i < 8; ++i) free(r->rgb_cache[i].pixels);
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; ++i) free(r->images[i].small_rgb);
    free(r->rgb_reference);
    if (r->rgb_audit) fclose(r->rgb_audit);
    if (r->rgb_mapping) munmap(r->rgb_mapping, VDMA_RGB_MAP_BYTES);
    r->plane_mapping = NULL;
    pthread_cond_destroy(&r->ready); pthread_mutex_destroy(&r->mutex);
    r->initialized = false;
}
