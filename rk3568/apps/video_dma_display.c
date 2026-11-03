#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/eventfd.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include "video_dma_uapi.h"
#include "video_dma_frame.h"
#include "road_feature_runtime.h"
#include "adas_result.h"
#include "video_dma_snapshot.h"
#include "video_dma_snapshot_rga.h"
#include "video_dma_snapshot_async.h"
#include "road_zebra_adas.h"
#include "road_zebra_osd.h"
#ifdef VDMA_HAVE_FPGA_ROAD
#include "fpga_road.h"
#include "video_dma_road_ref_uapi.h"
#endif
#ifdef VDMA_HAVE_ROAD_REF
#include "road_ref_adas.h"
#include "road_ref_transport.h"
#endif
#include "video_dma_osd_probe.h"
#ifdef VDMA_DASHBOARD
#include "video_dma_dashboard.h"
#endif
#ifdef VDMA_HAVE_YOLO
#include "yolo_adas.h"
#include "yolo_config.h"
#endif

enum owner_stage {
    OWNER_CONTROL, OWNER_POLL, OWNER_EVENT, OWNER_ACQUIRE, OWNER_FLIP,
    OWNER_YOLO_COPY, OWNER_RESULTS, OWNER_STAGE_COUNT
};

struct owner_frame_trace {
    uint64_t stamp_ns;
    uint32_t sequence, token;
    struct vdma_stats stats;
    unsigned int retired;
    int error;
};

struct display {
    int drm;
    int video;
    int exported[4];
    uint32_t handles[4];
    uint32_t framebuffers[4];
    uint32_t connector;
    uint32_t crtc;
    uint32_t blank_handle;
    uint32_t blank_framebuffer;
    drmModeModeInfo mode;
    struct vdma_buffer current;
    struct vdma_buffer next;
    bool current_valid;
    bool next_valid;
    bool pending;
    bool active;
    bool failed;
    bool sequence_valid;
    bool vblank_valid;
    uint32_t last_sequence;
    uint32_t last_vblank;
    uint64_t received;
    uint64_t flips;
    uint64_t gaps;
    uint64_t repeated_refreshes;
    uint32_t displayed_layout_mask;
    bool features_enabled;
    struct vdma_road_runtime *features;
    bool quiet;
    struct adas_store *adas;
    int result_event;
    bool result_event_live;
    atomic_uint_fast64_t result_event_errors;
    uint64_t result_wakes, result_notifications;
    struct adas_context adas_context;
    uint32_t adas_policy_generation;
    bool adas_bound;
    struct vdma_road_runtime_event *road_event;
    uint32_t last_epoch;
    bool epoch_valid;
    double steady_seconds, steady_elapsed, steady_cpu;
    uint64_t steady_frames, steady_flips, steady_gaps, steady_repeats;
    uint64_t loop_max_ns, last_loop_ns, control_max_ns;
    uint64_t drm_event_max_ns, last_drm_event_ns;
    uint64_t owner_stage_ns[OWNER_STAGE_COUNT], owner_previous_ns[OWNER_STAGE_COUNT];
    struct { uint64_t count, total_ns, max_ns; } owner_stages[OWNER_STAGE_COUNT];
    struct {
        uint64_t stamp_ns;
        uint32_t kind, sequence, distance;
        uint64_t previous_ns[OWNER_STAGE_COUNT], current_ns[OWNER_STAGE_COUNT];
    } owner_anomalies[32];
    unsigned int owner_anomaly_count;
    uint64_t owner_anomaly_overflow;
    uint64_t owner_tick_retries;
    bool trace_frames;
    struct owner_frame_trace frame_trace[16], anomaly_trace[32][16];
    unsigned int frame_trace_cursor, frame_trace_count, anomaly_trace_count[32];
    struct { uint64_t elapsed_ms, frames, flips, gaps, repeats, inputs, results; } summaries[120];
    unsigned int summary_cursor, summary_count;
    uint64_t summary_overwrites;
    unsigned int snapshot_hz, snapshot_delay_ms;
    struct vdma_snapshot_runtime *snapshot;
    struct vdma_frame_identity snapshot_identity;
    struct vdma_frame_identity next_identity;
    uint64_t snapshot_observed_ns;
    bool snapshot_rga;
    struct vdma_snapshot_async yolo_copy;
    struct vdma_buffer retired[4];
    unsigned int retired_count;
    uint64_t retired_max, release_deferred;
    const char *osd_probe_path;
    struct vdma_osd_probe *osd_probe;
    bool road_zebra;
    bool road_lane_only;
    enum road_zebra_detector zebra_detector;
    struct road_zebra_adas *zebra;
#ifdef VDMA_HAVE_ROAD_REF
    bool road_reference_rgb565;
    bool road_reference;
    unsigned int road_reference_period;
    struct road_ref_adas *reference;
    struct road_ref_transport *reference_transport;
    struct road_ref_adas_stats reference_start, reference_steady;
#endif
    const char *zebra_dump;
    bool zebra_osd, zebra_osd_disabled;
    struct vdma_osd_probe *zebra_overlay;
    struct adas_cursor zebra_cursor;
    uint64_t zebra_sequence;
    unsigned int zebra_drawn;
#ifdef VDMA_HAVE_FPGA_ROAD
    bool fpga_road;
    struct fpga_road *fpga_road_state;
    struct fpga_road_stats fpga_road_start, fpga_road_steady;
#endif
    const char *osd_dump;
    bool osd_dumped;
    const char *ui_probe_path, *ui_layout_path;
    struct vdma_ui_layout ui_layout;
#ifdef VDMA_HAVE_YOLO
    const char *yolo_path;
    struct yolo_config yolo_config;
    struct yolo_adas *yolo;
    struct vdma_snapshot_processor yolo_processor;
    struct vdma_snapshot_runtime *yolo_snapshot;
    struct adas_cursor yolo_cursor;
    uint64_t yolo_report_ns, yolo_previous;
    uint64_t yolo_steady_start, yolo_steady_published;
    bool yolo_audit, yolo_audit_present;
    uint32_t yolo_audit_epoch, yolo_audit_sequence;
    double yolo_fps;
#endif
#ifdef VDMA_DASHBOARD
    bool dashboard_initialized;
    bool recognition_requested;
    bool synchronized_video;
    struct vdma_frame_header synchronized_next;
#endif
};

static volatile sig_atomic_t stopping;

#ifdef VDMA_HAVE_YOLO
static enum vdma_snapshot_status yolo_preview_process(void *opaque, const struct vdma_snapshot_input *input,
                                                       struct vdma_snapshot_output *output)
{
    struct display *display = opaque;
    if (vdma_dashboard_test_yolo(display->yolo, input, output))
        return output->error ? VDMA_PROCESS_FAILED : VDMA_PROCESS_OK;
    enum vdma_snapshot_status rc = display->yolo_processor.process(display->yolo_processor.context, input, output);
    if (rc == VDMA_PROCESS_OK && output->diagnostic_value &&
        !(input->cancelled && input->cancelled(input->cancel_context)))
        vdma_dashboard_preview_rgb(&input->identity, input->pixels, input->observed_ns);
    return rc;
}
#endif

static void stop_signal(int number)
{
    (void)number;
    stopping = 1;
}

static int install_signals(void)
{
    struct sigaction action = {0};
    sigemptyset(&action.sa_mask);
    action.sa_handler = stop_signal;
    if (sigaction(SIGINT, &action, NULL) || sigaction(SIGTERM, &action, NULL) ||
        sigaction(SIGHUP, &action, NULL)) return -1;
    action.sa_handler = SIG_IGN; /* Retired COLOR config reload. */
    return sigaction(SIGUSR1, &action, NULL);
}

static double now_seconds(void)
{
    struct timespec stamp;

    clock_gettime(CLOCK_MONOTONIC, &stamp);
    return stamp.tv_sec + stamp.tv_nsec / 1e9;
}

static void owner_stage_end(struct display *display, enum owner_stage stage, uint64_t started)
{
    uint64_t elapsed = vdma_road_now_ns() - started;
    display->owner_stage_ns[stage] += elapsed;
    display->owner_stages[stage].count++;
    display->owner_stages[stage].total_ns += elapsed;
    if (elapsed > display->owner_stages[stage].max_ns)
        display->owner_stages[stage].max_ns = elapsed;
}

static void owner_anomaly(struct display *display, uint32_t kind, uint32_t sequence, uint32_t distance)
{
    if (display->owner_anomaly_count == 32) { display->owner_anomaly_overflow++; return; }
    unsigned int index = display->owner_anomaly_count++;
    display->owner_anomalies[index].stamp_ns = vdma_road_now_ns();
    display->owner_anomalies[index].kind = kind;
    display->owner_anomalies[index].sequence = sequence;
    display->owner_anomalies[index].distance = distance;
    memcpy(display->owner_anomalies[index].previous_ns, display->owner_previous_ns,
           sizeof(display->owner_previous_ns));
    memcpy(display->owner_anomalies[index].current_ns, display->owner_stage_ns,
           sizeof(display->owner_stage_ns));
    display->anomaly_trace_count[index] = display->frame_trace_count;
    for (unsigned int i = 0; i < display->frame_trace_count; ++i)
        display->anomaly_trace[index][i] = display->frame_trace[
            (display->frame_trace_cursor + 16 - display->frame_trace_count + i) % 16];
}

static void owner_trace_frame(struct display *display, const struct vdma_buffer *frame)
{
    if (!display->trace_frames) return;
    struct owner_frame_trace *trace = &display->frame_trace[display->frame_trace_cursor];
    memset(trace, 0, sizeof(*trace));
    trace->stamp_ns = vdma_road_now_ns();
    trace->sequence = frame->header.sequence; trace->token = frame->token;
    trace->retired = display->retired_count;
    int saved = errno;
    if (ioctl(display->video, VDMA_STATS, &trace->stats)) trace->error = errno;
    errno = saved;
    display->frame_trace_cursor = (display->frame_trace_cursor + 1) % 16;
    if (display->frame_trace_count < 16) display->frame_trace_count++;
}

static int owner_priority(void)
{
    const char *setting = getenv("VDMA_DISPLAY_NICE");
    if (setting) {
        char *end = NULL;
        errno = 0;
        long value = strtol(setting, &end, 10);
        if (errno || end == setting || *end || value < -20 || value > 19) {
            errno = EINVAL; return -1;
        }
        /* Linux PRIO_PROCESS with who=0 changes only this calling thread.
         * Existing Qt/copy/algorithm workers keep their original priority. */
        if (setpriority(PRIO_PROCESS, 0, (int)value)) return -1;
    }
    errno = 0;
    int value = getpriority(PRIO_PROCESS, 0);
    if (errno) return -1;
    printf("DISPLAY_SCHED nice=%d explicit=%u\n", value, setting != NULL);
    return 0;
}

static int owner_tick(struct display *display, uint64_t stamp)
{
    int rc = adas_tick(display->adas, stamp);
    /* Another producer may commit a later timestamp between clock sampling
     * and the store's nonblocking lock. Retry with a fresh time, as YOLO does. */
    for (unsigned int retry = 0; rc == ADAS_INVALID && retry < 2; ++retry) {
        display->owner_tick_retries++;
        rc = adas_tick(display->adas, vdma_road_now_ns());
    }
    if (rc < 0) fprintf(stderr, "OWNER_TICK_FAILED rc=%d retries=%" PRIu64 "\n",
                       rc, display->owner_tick_retries);
    return rc < 0 ? -1 : 0;
}

static int owner_poll_timeout(const struct display *display)
{
#ifdef VDMA_DASHBOARD
    if (display->synchronized_video) return 1;
#endif
    if (!display->pending) return 0;
    if (display->features) return 1;
#ifdef VDMA_HAVE_ROAD_REF
    /* Reference copies run independently. DRM events observe video identities
     * at 60Hz; the legacy ROAD packet path still needs its 1ms polling. */
    if (display->reference_transport) return 20;
#endif
    return display->road_zebra ? 1 : 20;
}

#ifdef VDMA_HAVE_FPGA_ROAD
static int drain_fpga_road(struct display *display, int enabled, uint64_t now)
{
    if (!enabled) fpga_road_clear(display->fpga_road_state);
    for (unsigned int i = 0; i < 4; ++i) {
        struct vdma_fpga_road packet = {.version = 1, .size = sizeof(packet)};
        if (ioctl(display->video, VDMA_FPGA_ROAD_READ, &packet)) {
            if (errno == EAGAIN || errno == EINTR) break;
            return -1;
        }
        if (enabled && fpga_road_accept(display->fpga_road_state, &packet, now)) return -1;
    }
    return enabled ? fpga_road_flush(display->fpga_road_state, now) : 0;
}
#endif

static void result_notify(void *context)
{
    struct display *display = context;
    const uint64_t value = 1;
    ssize_t bytes;
    do { bytes = write(display->result_event, &value, sizeof(value)); }
    while (bytes < 0 && errno == EINTR);
    /* A saturated eventfd is already readable; latest snapshots coalesce it. */
    if (bytes != sizeof(value) && !(bytes < 0 && errno == EAGAIN))
        atomic_fetch_add(&display->result_event_errors, 1);
}

#ifdef VDMA_DASHBOARD
static void presentation_observe(void *context, const struct adas_result *snapshot,
                                  uint32_t branch, uint64_t now)
{
    (void)context;
    vdma_dashboard_alert_publish(snapshot, branch, now);
    if (branch == ADAS_ROAD) {
        struct road_overlay_geometry geometry;
        road_zebra_osd_geometry(&geometry, &snapshot->road);
        geometry.publication_sequence = snapshot->publication_sequence;
        vdma_dashboard_history_road(&geometry);
    } else {
        struct detection_overlay overlay;
        detection_overlay_from_result(&overlay, snapshot, now);
        vdma_dashboard_history_detection(&overlay);
    }
}
#endif

static int drain_retired(struct display *display)
{
    if (vdma_snapshot_async_reap(&display->yolo_copy) < 0) {
        perror("YOLO async copy"); return -1;
    }
    while (display->retired_count) {
        const struct vdma_buffer *frame = &display->retired[0];
        if (vdma_snapshot_async_holds(&display->yolo_copy, frame)) break;
#ifdef VDMA_DASHBOARD
        if (display->synchronized_video && vdma_dashboard_sync_holds(frame)) break;
#endif
        struct vdma_release_buffer released = {frame->slot, frame->token};
        if (ioctl(display->video, VDMA_RELEASE, &released) < 0) {
            perror("VDMA_RELEASE"); return -1;
        }
        display->retired_count--;
        memmove(display->retired, display->retired + 1,
                display->retired_count * sizeof(display->retired[0]));
    }
    return 0;
}

static int release_frame(struct display *display, const struct vdma_buffer *frame)
{
    if (display->retired_count == 4) { errno = EOVERFLOW; return -1; }
    display->retired[display->retired_count++] = *frame;
    if (display->retired_count > display->retired_max)
        display->retired_max = display->retired_count;
    if (vdma_snapshot_async_holds(&display->yolo_copy, &display->retired[0]))
        display->release_deferred++;
    return drain_retired(display);
}

static void flip_complete(int descriptor, unsigned int sequence, unsigned int seconds,
                          unsigned int microseconds, void *opaque)
{
    struct display *display = opaque;

    (void)descriptor;
    (void)seconds;
    (void)microseconds;
#ifdef VDMA_DASHBOARD
    vdma_dashboard_complete();
    if (display->synchronized_video) {
        vdma_dashboard_sync_complete(vdma_road_now_ns());
        if (display->vblank_valid) {
            uint32_t distance = sequence - display->last_vblank;
            if (distance > 1 && distance < 0x80000000U)
                display->repeated_refreshes += distance - 1;
        }
        display->last_vblank = sequence; display->vblank_valid = true;
        display->current.header = display->synchronized_next;
        display->current_valid = true; display->pending = false; display->flips++;
        return;
    }
    if (!display->next_valid) { display->pending = false; return; }
#endif
    if (display->current_valid && release_frame(display, &display->current)) {
        display->failed = true;
        display->pending = false;
        return;
    }
    if (display->vblank_valid) {
        uint32_t distance = sequence - display->last_vblank;

        if (distance > 1 && distance < 0x80000000U) {
            display->repeated_refreshes += distance - 1;
            owner_anomaly(display, 2, sequence, distance);
        }
    }
    display->last_vblank = sequence;
    display->vblank_valid = true;
    if (!display->current_valid ||
        display->current.header.layout != display->next.header.layout) {
        printf("Displayed layout=%u source=%s seq=%u\n", display->next.header.layout,
               vdma_layout_name(display->next.header.layout), display->next.header.sequence);
        fflush(stdout);
    }
    display->displayed_layout_mask |= 1U << display->next.header.layout;
    display->current = display->next;
    display->current_valid = true;
    display->next_valid = false;
    display->pending = false;
    display->flips++;
}

static int select_output(struct display *display)
{
    drmModeRes *resources = drmModeGetResources(display->drm);
    int result = -1;

    if (!resources)
        return -1;
    for (int index = 0; index < resources->count_connectors; index++) {
        drmModeConnector *connector = drmModeGetConnector(display->drm, resources->connectors[index]);
        uint32_t possible = 0;
        uint32_t preferred_crtc = 0;
        bool found_mode = false;

        if (!connector)
            continue;
        if (connector->connection != DRM_MODE_CONNECTED ||
            connector->connector_type != DRM_MODE_CONNECTOR_HDMIA ||
            (display->connector && display->connector != connector->connector_id)) {
            drmModeFreeConnector(connector);
            continue;
        }
        for (int mode_index = 0; mode_index < connector->count_modes; mode_index++) {
            drmModeModeInfo *mode = &connector->modes[mode_index];
            double refresh = mode->htotal && mode->vtotal ?
                mode->clock * 1000.0 / mode->htotal / mode->vtotal : 0;

            if (mode->hdisplay == 1920 && mode->vdisplay == 1080 &&
                !(mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLSCAN)) &&
                refresh > 59.99 && refresh < 60.01) {
                display->mode = *mode;
                found_mode = true;
                break;
            }
        }
        if (!found_mode) {
            drmModeFreeConnector(connector);
            continue;
        }
        for (int encoder_index = 0; encoder_index < connector->count_encoders; encoder_index++) {
            drmModeEncoder *encoder = drmModeGetEncoder(display->drm, connector->encoders[encoder_index]);

            if (encoder) {
                possible |= encoder->possible_crtcs;
                if (encoder->encoder_id == connector->encoder_id)
                    preferred_crtc = encoder->crtc_id;
                drmModeFreeEncoder(encoder);
            }
        }
        for (int crtc_index = 0; crtc_index < resources->count_crtcs; crtc_index++) {
            if (possible & (1U << crtc_index)) {
                display->crtc = resources->crtcs[crtc_index];
                if (display->crtc == preferred_crtc)
                    break;
            }
        }
        display->connector = connector->connector_id;
        drmModeFreeConnector(connector);
        if (display->crtc) {
            result = 0;
            break;
        }
    }
    drmModeFreeResources(resources);
    if (result)
        fprintf(stderr, "No connected HDMI with progressive 1920x1080 at 60.00Hz\n");
    return result;
}

static int import_frames(struct display *display)
{
    for (unsigned int slot = 0; slot < 4; slot++) {
        struct vdma_export request = { slot, 0 };
        uint32_t handles[4] = { 0 };
        uint32_t pitches[4] = { 3840, 0, 0, 0 };
        uint32_t offsets[4] = { 0 };

        display->exported[slot] = ioctl(display->video, VDMA_EXPORT, &request);
        if (display->exported[slot] < 0) {
            perror("VDMA_EXPORT");
            return -1;
        }
        if (drmPrimeFDToHandle(display->drm, display->exported[slot], &display->handles[slot])) {
            perror("DRM PRIME import");
            return -1;
        }
        handles[0] = display->handles[slot];
        if (drmModeAddFB2(display->drm, 1920, 1080, DRM_FORMAT_RGB565,
                         handles, pitches, offsets, &display->framebuffers[slot], 0)) {
            perror("DRM RGB565 framebuffer");
            return -1;
        }
    }
    return 0;
}

static int prepare_output(struct display *display)
{
    struct drm_mode_create_dumb request = { .height = 1080, .width = 1920, .bpp = 16 };
    uint32_t handles[4] = { 0 };
    uint32_t pitches[4] = { 0 };
    uint32_t offsets[4] = { 0 };

    if (drmIoctl(display->drm, DRM_IOCTL_MODE_CREATE_DUMB, &request))
        return -1;
    display->blank_handle = handles[0] = request.handle;
    pitches[0] = request.pitch;
    if (drmModeAddFB2(display->drm, 1920, 1080, DRM_FORMAT_RGB565,
                      handles, pitches, offsets, &display->blank_framebuffer, 0))
        return -1;
    if (drmModeSetCrtc(display->drm, display->crtc, display->blank_framebuffer,
                       0, 0, &display->connector, 1, &display->mode))
        return -1;
    display->active = true;
    return 0;
}

static int acquire_frame(struct display *display, struct vdma_buffer *frame)
{
    if (ioctl(display->video, VDMA_ACQUIRE_NEXT, frame) < 0)
        return errno == EAGAIN || errno == EINTR ? 0 : -1;
    display->next_valid = true;
    if (!vdma_frame_valid(frame->slot, &frame->header)) {
        errno = EPROTO;
        return -1;
    }
    if (display->features || display->snapshot || display->adas
#ifdef VDMA_DASHBOARD
        || display->dashboard_initialized
#endif
       ) {
        uint64_t observed = vdma_road_now_ns();
        struct vdma_frame_identity identity = {.version = 1, .size = sizeof(identity),
            .slot = frame->slot, .token = frame->token};
        if (ioctl(display->video, VDMA_FRAME_IDENTITY, &identity)) {
            perror("VDMA_FRAME_IDENTITY");
            return -1;
        }
        if (identity.frame_sequence != frame->header.sequence || identity.layout != frame->header.layout ||
            identity.version != 1 || identity.size != sizeof(identity) || identity.source_id > 4 ||
            identity.source_id != identity.layout || identity.flags ||
            identity.canvas_width != 1920 || identity.canvas_height != 1080) {
            fprintf(stderr, "FRAME_IDENTITY_INVALID epoch=%u seq=%u/%u source=%u layout=%u/%u\n",
                identity.stream_epoch, identity.frame_sequence, frame->header.sequence,
                identity.source_id, identity.layout, frame->header.layout);
            errno = EPROTO;
            return -1;
        }
#ifdef VDMA_HAVE_FPGA_ROAD
        const char *presentation = getenv("VDMA_PRESENT_MODE");
        /* The fullscreen diagnostic is HDMI-only. The Qt dashboard must
         * keep presenting legal FPGA sources even without recognition. */
        if (presentation && !strcmp(presentation, "exact") && identity.source_id != 2) {
            fprintf(stderr, "Exact presentation requires HDMI source, got %u\n",
                identity.source_id);
            errno = EOPNOTSUPP;
            return -1;
        }
#endif
#ifdef VDMA_DASHBOARD
        display->next_identity = identity;
        vdma_dashboard_video(&identity);
        vdma_dashboard_held(display->video, &identity);
#endif
#ifdef VDMA_HAVE_FPGA_ROAD
        if (display->fpga_road && identity.source_id == 2) {
            struct vdma_native_identity native = {.version = 1, .size = sizeof(native),
                .slot = identity.slot, .token = identity.token};
            if (ioctl(display->video, VDMA_NATIVE_IDENTITY, &native) ||
                fpga_road_bind(display->fpga_road_state, &identity, &native, observed)) {
                perror("FPGA road native identity");
                return -1;
            }
        } else if (display->fpga_road) {
            fpga_road_clear(display->fpga_road_state);
        }
        display->snapshot_identity = identity;
        display->snapshot_observed_ns = observed;
#endif
        if (display->snapshot) {
            display->snapshot_identity = identity;
            display->snapshot_observed_ns = observed;
#ifdef VDMA_DASHBOARD
            /* YC arrivals also queue work directly; keep that path unbound
             * while recognition is off or the hardware context is unsettled. */
            if (!display->road_zebra ||
                (vdma_dashboard_road_recognition() && vdma_dashboard_frame_usable()))
#endif
                vdma_snapshot_observe(display->snapshot, &identity);
        }
#ifdef VDMA_HAVE_YOLO
        if (display->yolo_snapshot) {
            display->snapshot_identity = identity;
            display->snapshot_observed_ns = observed;
            vdma_snapshot_observe(display->yolo_snapshot, &identity);
        }
#endif
        /* The source-native identity is copied only while this display slot
         * is leased, and is distinct from the display canvas sequence. */
#ifdef VDMA_HAVE_ROAD_REF
        if (display->reference_transport &&
            road_ref_transport_observe(display->reference_transport, &identity, observed)) return -1;
#endif
        uint64_t control_started = observed;
        if (display->features && vdma_road_runtime_video_at(display->features, &identity, observed)) return -1;
        uint64_t control_ns = vdma_road_now_ns() - control_started;
        if (control_ns > display->control_max_ns) display->control_max_ns = control_ns;
        if (display->adas
#ifdef VDMA_DASHBOARD
            && vdma_dashboard_frame_usable()
#endif
           ) {
            struct adas_context context = {identity.stream_epoch, identity.source_id,
                identity.layout, identity.config_generation, 1};
            if (!display->adas_bound || memcmp(&context, &display->adas_context, sizeof(context))) {
                struct adas_setup setup = {.policy_generation = display->adas_policy_generation + 1U, .context_present = 1,
                    .context = context, .detection_ttl_ns = ADAS_DETECTION_TTL_NS,
                    .road_ttl_ns = ADAS_ROAD_TTL_NS, .fusion_window_ns = ADAS_FUSION_WINDOW_NS};
                /* Enable changes require a new policy; commit only after a successful bind. */
                if (!setup.policy_generation) setup.policy_generation = 1;
#ifdef VDMA_HAVE_YOLO
                if (display->yolo) setup.detection = yolo_adas_registration(1, 1);
#endif
                /* Register the available producer independently of consumers.
                 * UI masks gate scheduling/drain, never the session ABI. */
                if (display->road_zebra) {
                    /* producer_valid() rejects an enabled producer whose session or
                     * algorithm identity is zero, and road_cv_publish_adas() checks
                     * origin and algorithm against this registration. Leaving all
                     * zero would make every publication fail silently. */
                    setup.road = (struct adas_producer){.enabled = 1, .origin = ADAS_ROAD_CV,
                        .producer_session = 1, .algorithm_id = ROAD_CV_ALGORITHM_ID,
                        .algorithm_version = ROAD_CV_ALGORITHM_VERSION,
                        .algorithm_config_generation = 1,
                        .road_supported_tasks = ADAS_ROAD_LEFT | ADAS_ROAD_RIGHT |
                            (display->road_lane_only ? 0U : ADAS_ROAD_ZEBRA)};
#ifdef VDMA_HAVE_ROAD_REF
                    if (display->reference) {
                        setup.road.algorithm_id = ROAD_REF_ALGORITHM_ID;
                        setup.road.algorithm_version = ROAD_REF_ALGORITHM_VERSION;
                    }
#endif
#ifdef VDMA_HAVE_FPGA_ROAD
                    if (display->fpga_road)
                        setup.road = fpga_road_registration();
#endif
                }
                uint32_t reason = ADAS_WAIT_INPUT;
                if (display->adas_bound) {
                    reason = context.stream_epoch != display->adas_context.stream_epoch ? ADAS_EPOCH_CHANGED :
                        context.source_id != display->adas_context.source_id ? ADAS_SOURCE_CHANGED : ADAS_FPGA_CONFIG_CHANGED;
                }
                int session_result = adas_begin_session(display->adas, &setup, vdma_road_now_ns(), reason);
                if (session_result == ADAS_BUSY) {
                    display->adas_bound = false;
                } else if (session_result != ADAS_OK_NEW) {
                    fprintf(stderr, "ADAS_BEGIN_SESSION rc=%d reason=%u epoch=%u source=%u layout=%u\n",
                        session_result, reason, context.stream_epoch, context.source_id, context.layout);
                    errno = EPROTO;
                    return -1;
                } else {
                    display->adas_context = context; display->adas_bound = true;
                    display->adas_policy_generation = setup.policy_generation;
                    /* A session starts after acquire; sampling must not predate it. */
                    display->snapshot_observed_ns = vdma_road_now_ns();
                }
            }
        }
        if (display->epoch_valid && display->last_epoch != identity.stream_epoch) display->sequence_valid = false;
        display->last_epoch = identity.stream_epoch; display->epoch_valid = true;
    }
#ifdef VDMA_HAVE_FPGA_ROAD
    if (display->fpga_road && fpga_road_flush(display->fpga_road_state, vdma_road_now_ns())) {
        perror("FPGA road pending result");
        return -1;
    }
#endif
    owner_trace_frame(display, frame);
    if (display->sequence_valid) {
        uint32_t distance = frame->header.sequence - display->last_sequence;

        if (!distance || distance >= 0x80000000U) {
            errno = EPROTO;
            return -1;
        }
        display->gaps += distance - 1;
        if (distance > 1) owner_anomaly(display, 1, frame->header.sequence, distance);
    }
    display->sequence_valid = true;
    display->last_sequence = frame->header.sequence;
    display->received++;
    return 1;
}

/* Road snapshots go to the existing Qt owner in dashboard mode. Standalone
 * display has its own overlay and redraws only when the publication changes. */
static int zebra_overlay_update(struct display *display)
{
    if (!display->adas || !display->zebra_osd) return 0;
#ifndef VDMA_DASHBOARD
    if (!display->zebra_overlay) return 0;
    struct vdma_osd_probe *overlay = display->zebra_overlay;
    uint32_t *pixels = overlay->pixels;
    uint32_t pitch = overlay->pitch / 4U;
    if (!pixels || !pitch) return 0;
#endif
    struct adas_cursor cursor = display->zebra_cursor;
    struct adas_result latest;
    int status = adas_get_latest(display->adas, vdma_road_now_ns(), &cursor, &latest);
    display->zebra_cursor = cursor;
    if (status != ADAS_OK_NEW && status != ADAS_OK_SAME) {
#ifdef VDMA_DASHBOARD
        vdma_dashboard_adas(NULL);
#else
        if (display->zebra_drawn) {
            road_zebra_osd_clear(pixels, pitch, 1920, 1080);
            display->zebra_drawn = 0;
            printf("ZEBRA_OSD boxes=0 status=%d\n", status);
            fflush(stdout);
        }
#endif
        return 0;
    }
    if (status == ADAS_OK_SAME && latest.publication_sequence == display->zebra_sequence) return 0;
    display->zebra_sequence = latest.publication_sequence;
#ifdef VDMA_DASHBOARD
    struct road_overlay_geometry geometry;
    road_zebra_osd_geometry(&geometry, &latest.road);
    geometry.publication_sequence = latest.publication_sequence;
    vdma_dashboard_adas(&geometry);
#else
    int drawn = road_zebra_osd_draw(pixels, pitch, 1920, 1080, &latest.road, 3);
    if (drawn < 0) return -1;
    display->zebra_drawn = (unsigned int)drawn;
    printf("ROAD_OSD primitives=%d state=%u seq=%llu\n", drawn, latest.road.header.state,
           (unsigned long long)latest.publication_sequence);
    fflush(stdout);
    if (drawn && display->osd_dump && !display->osd_dumped) {
        if (road_zebra_osd_dump(pixels, pitch, 1080, display->osd_dump)) perror("OSD dump");
        else display->osd_dumped = true;
    }
#endif
    return 0;
}

#ifdef VDMA_DASHBOARD
static int invalidate_recognition(struct display *display)
{
    if (display->adas && display->adas_bound) {
        /* Source/config changes invalidate context but retain producer registration. */
        int result = adas_invalidate(display->adas, vdma_road_now_ns(), ADAS_WAIT_INPUT);
        if (result != ADAS_OK_NEW) return result;
    }
    display->adas_bound = false;
    return ADAS_OK_NEW;
}
#endif

static int run_display(struct display *display, double duration)
{
    display->trace_frames = getenv("VDMA_OWNER_TRACE") != NULL;
    drmEventContext events = { .version = 2, .page_flip_handler = flip_complete };
    double started = now_seconds();
    clock_t cpu_started = clock();
    double report_time = started;
    double progress_time = started;
    uint64_t report_received = 0;
    uint64_t report_flips = 0;
#if defined(VDMA_HAVE_ROAD_REF) || !defined(VDMA_ROAD_REF_ONLY)
    uint64_t report_published = 0;
#endif
#ifndef VDMA_ROAD_REF_ONLY
    uint64_t report_lane_gray = 0;
#endif
#ifdef VDMA_DASHBOARD
    bool invalidation_pending = false;
#endif
    double steady_start = started + 2, steady_end = steady_start + display->steady_seconds;
    clock_t steady_cpu_start = 0;
    uint64_t frames_start = 0, flips_start = 0, gaps_start = 0, repeats_start = 0;
    int measuring = 0, measured = 0;

    if (owner_priority()) { perror("Display thread priority"); return -1; }
    if (ioctl(display->video, VDMA_START) < 0) {
        perror("VDMA_START");
        return -1;
    }
    while (!stopping && !display->failed &&
           (!duration || now_seconds() - started < duration)) {
        uint64_t loop_now = vdma_road_now_ns();
        memcpy(display->owner_previous_ns, display->owner_stage_ns, sizeof(display->owner_stage_ns));
        memset(display->owner_stage_ns, 0, sizeof(display->owner_stage_ns));
        if (display->last_loop_ns && loop_now - display->last_loop_ns > display->loop_max_ns)
            display->loop_max_ns = loop_now - display->last_loop_ns;
        display->last_loop_ns = loop_now;
#ifdef VDMA_DASHBOARD
        vdma_dashboard_tick();
        bool recognition = vdma_dashboard_recognition();
        if (vdma_dashboard_take_invalidation() || recognition != display->recognition_requested)
            invalidation_pending = true;
        if (invalidation_pending) {
            int invalidated = invalidate_recognition(display);
            if (invalidated == ADAS_BUSY) {
                struct timespec delay = {0, 500000};
                nanosleep(&delay, NULL);
                continue;
            }
            if (invalidated != ADAS_OK_NEW) return -1;
            invalidation_pending = false;
            display->recognition_requested = recognition;
            if (display->snapshot) vdma_snapshot_invalidate(display->snapshot);
#ifdef VDMA_HAVE_YOLO
            if (display->yolo_snapshot) vdma_snapshot_invalidate(display->yolo_snapshot);
            vdma_dashboard_detection(NULL);
            display->yolo_previous = 0; display->yolo_report_ns = 0; display->yolo_fps = 0;
#endif
        }
#endif
#ifdef VDMA_HAVE_ROAD_REF
        if (display->reference_transport &&
            road_ref_transport_drain(display->reference_transport, display->snapshot,
                display->adas_bound
#ifdef VDMA_DASHBOARD
                && vdma_dashboard_road_recognition() && vdma_dashboard_frame_usable()
#endif
            )) { perror("Reference three-plane input"); return -1; }
#endif
#ifdef VDMA_HAVE_YOLO
        if (display->yolo_snapshot && vdma_snapshot_rgb_drain(display->yolo_snapshot)) {
            perror("Small RGB drain"); return -1;
        }
#endif
        double wall = now_seconds();
        if (display->steady_seconds && !measuring && !measured && wall >= steady_start) {
            steady_start = wall; steady_end = wall + display->steady_seconds;
            steady_cpu_start = clock(); measuring = 1;
            frames_start = display->received; flips_start = display->flips;
            gaps_start = display->gaps; repeats_start = display->repeated_refreshes;
#ifdef VDMA_HAVE_FPGA_ROAD
            if (display->fpga_road_state)
                fpga_road_get_stats(display->fpga_road_state, &display->fpga_road_start);
#endif
#ifdef VDMA_HAVE_ROAD_REF
            if (display->reference)
                road_ref_adas_stats(display->reference, &display->reference_start);
#endif
#ifdef VDMA_HAVE_YOLO
            if (display->yolo) {
                struct yolo_adas_stats stats; yolo_adas_get_stats(display->yolo, &stats);
                display->yolo_steady_start = stats.published;
                if (display->yolo_audit)
                    printf("YOLO_WINDOW_BEGIN monotonic_s=%.9f published=%llu\n",
                        wall, (unsigned long long)stats.published);
            }
#endif
        }
        if (measuring && wall >= steady_end) {
            display->steady_elapsed = wall - steady_start;
            display->steady_cpu = 100.0 * (clock() - steady_cpu_start) / CLOCKS_PER_SEC / display->steady_elapsed;
            display->steady_frames = display->received - frames_start; display->steady_flips = display->flips - flips_start;
            display->steady_gaps = display->gaps - gaps_start; display->steady_repeats = display->repeated_refreshes - repeats_start;
            measuring = 0; measured = 1;
#ifdef VDMA_HAVE_FPGA_ROAD
            if (display->fpga_road_state) {
                struct fpga_road_stats s;
                fpga_road_get_stats(display->fpga_road_state, &s);
                display->fpga_road_steady = (struct fpga_road_stats){
                    s.received - display->fpga_road_start.received,
                    s.published - display->fpga_road_start.published,
                    s.unmatched - display->fpga_road_start.unmatched,
                    s.rejected - display->fpga_road_start.rejected,
                    s.dropped - display->fpga_road_start.dropped,
                    s.last_epoch, s.last_sequence};
            }
#endif
#ifdef VDMA_HAVE_ROAD_REF
            if (display->reference) {
                struct road_ref_adas_stats stats;
                road_ref_adas_stats(display->reference, &stats);
                display->reference_steady.processed = stats.processed - display->reference_start.processed;
                display->reference_steady.published = stats.published - display->reference_start.published;
                display->reference_steady.expired = stats.expired - display->reference_start.expired;
            }
#endif
#ifdef VDMA_HAVE_YOLO
            if (display->yolo) {
                struct yolo_adas_stats stats; yolo_adas_get_stats(display->yolo, &stats);
                display->yolo_steady_published = stats.published - display->yolo_steady_start;
                if (display->yolo_audit)
                    printf("YOLO_WINDOW_END monotonic_s=%.9f published=%llu\n",
                        wall, (unsigned long long)stats.published);
            }
#endif
        }
        if (measured && wall >= steady_end + 0.3) break;
        if (display->snapshot && display->snapshot->plane_enabled &&
            vdma_snapshot_plane_drain(display->snapshot, display->video)) {
            perror("GRAY8 plane"); return -1;
        }
        struct pollfd event_fd[2] = {
            {.fd = display->drm, .events = POLLIN},
            {.fd = display->result_event_live ? display->result_event : -1, .events = POLLIN}};
        owner_stage_end(display, OWNER_CONTROL, loop_now);
        uint64_t stage_started = vdma_road_now_ns();
        int ready = poll(event_fd, display->result_event_live ? 2 : 1, owner_poll_timeout(display));
        owner_stage_end(display, OWNER_POLL, stage_started);

        if (ready < 0 && errno != EINTR) {
            perror("DRM poll");
            return -1;
        }
        if (ready > 0) {
            stage_started = vdma_road_now_ns();
            if (event_fd[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                fprintf(stderr, "DRM_POLL_FAILED revents=%#x\n", event_fd[0].revents);
                return -1;
            }
            if ((event_fd[0].revents & POLLIN) && drmHandleEvent(display->drm, &events)) {
                perror("DRM handle event");
                return -1;
            }
            if (event_fd[0].revents & POLLIN) {
                uint64_t stamp = vdma_road_now_ns();
                if (display->last_drm_event_ns && stamp - display->last_drm_event_ns > display->drm_event_max_ns)
                    display->drm_event_max_ns = stamp - display->last_drm_event_ns;
                display->last_drm_event_ns = stamp;
            }
            if (event_fd[1].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                fputs("ADAS result event poll failed\n", stderr); return -1;
            }
            if (event_fd[1].revents & POLLIN) {
                uint64_t count;
                ssize_t bytes;
                do { bytes = read(display->result_event, &count, sizeof(count)); }
                while (bytes < 0 && errno == EINTR);
                if (bytes == sizeof(count)) {
                    display->result_wakes++; display->result_notifications += count;
                } else if (!(bytes < 0 && errno == EAGAIN)) {
                    perror("ADAS result event read"); return -1;
                }
            }
            owner_stage_end(display, OWNER_EVENT, stage_started);
        }
        if (display->failed)
            break;
        if (drain_retired(display)) return -1;
#ifdef VDMA_DASHBOARD
        if (display->synchronized_video) {
            /* Capture remains live even while a historical cache slot scans.
             * Only these real DMA leases enter ordered retirement. */
            if (!display->next_valid && display->retired_count < 3) {
                int acquired = acquire_frame(display,&display->next);
                if (acquired < 0) { perror("Synchronized acquire"); return -1; }
                if (acquired) {
                    if (vdma_dashboard_sync_capture(&display->next,&display->snapshot_identity,
                            display->snapshot_observed_ns)) {
                        perror("Synchronized capture"); return -1;
                    }
#ifdef VDMA_HAVE_YOLO
                    if (display->yolo_snapshot && vdma_dashboard_frame_usable() && display->adas_bound) {
                        struct yolo_adas_stats stats; yolo_adas_get_stats(display->yolo,&stats);
                        int copied = stats.halted ? 0 :
                            display->yolo_copy.started ?
                            vdma_snapshot_async_submit(&display->yolo_copy,&display->snapshot_identity,
                                &display->next.header,display->snapshot_observed_ns) :
                            vdma_snapshot_sample(display->yolo_snapshot,display->video,
                                &display->snapshot_identity,&display->next.header,display->snapshot_observed_ns);
                        if (copied < 0) { perror("Synchronized YOLO copy"); return -1; }
                    }
#endif
                    if (release_frame(display,&display->next)) return -1;
                    display->next_valid = false;
                }
            }
            if (!display->pending) {
                uint32_t framebuffer = 0; struct vdma_frame_identity presented;
                struct vdma_frame_header header;
                int selected = vdma_dashboard_sync_peek(vdma_road_now_ns(),&framebuffer,&presented,&header);
                if (selected < 0) { perror("Synchronized presentation"); return -1; }
                if (selected) {
                    int submitted = vdma_dashboard_flip(framebuffer,&presented,display,0);
                    if (submitted <= 0) {
                        int submit_error = errno;
                        vdma_dashboard_sync_cancel(selected-1);
                        errno = submit_error;
                    }
                    if (submitted < 0) { perror("Synchronized flip"); return -1; }
                    if (submitted) {
                        vdma_dashboard_sync_submit(selected-1);
                        display->synchronized_next = header;
                        display->pending = true; progress_time = now_seconds();
                    }
                }
            }
        } else
#endif
        if (!display->pending) {
            /* Never release a source still read by RGA to make queue space. */
            if (display->retired_count + display->current_valid +
                display->next_valid >= 4) continue;
            stage_started = vdma_road_now_ns();
            int acquired = display->next_valid ? 1 : acquire_frame(display, &display->next);
            owner_stage_end(display, OWNER_ACQUIRE, stage_started);

            if (acquired < 0) {
                perror("VDMA_ACQUIRE_NEXT");
                return -1;
            }
            if (acquired) {
                stage_started = vdma_road_now_ns();
                display->next_valid = true;
                /* Submit video and immutable test overlay together, avoiding a
                 * separate blocking plane update in the video flip loop. */
                if (display->ui_probe_path && !display->osd_probe->layout_tested) {
                    if (vdma_osd_probe_test_layout(display->osd_probe,
                        display->framebuffers[display->next.slot])) {
                        perror("UI layout TEST_ONLY on actual DMA-BUF"); return -1;
                    }
                    printf("UI_LAYOUT TEST_ONLY passed on held DMA-BUF slot=%u\n", display->next.slot);
                }
                int submitted;
#ifdef VDMA_DASHBOARD
                submitted = vdma_dashboard_flip(display->framebuffers[display->next.slot],
                    &display->next_identity, display, 0);
                if (!submitted) {
                    if (now_seconds() - progress_time > 3) { errno = ETIMEDOUT; return -1; }
                    struct timespec delay = {0, 500000};
                    nanosleep(&delay, NULL);
                    continue; /* Hold slot until first UI is READY. */
                }
                submitted = submitted < 0 ? -1 : 0;
#else
                submitted = display->osd_probe ? vdma_osd_probe_flip(display->osd_probe,
                    display->framebuffers[display->next.slot],
                    display->ui_probe_path || ((unsigned int)(wall - started) / 5U) % 2U == 0, display) :
                    drmModePageFlip(display->drm, display->crtc,
                                    display->framebuffers[display->next.slot],
                                    DRM_MODE_PAGE_FLIP_EVENT, display);
#endif
                if (submitted) {
                    perror("DRM page flip");
                    return -1;
                }
                display->pending = true;
                owner_stage_end(display, OWNER_FLIP, stage_started);
                progress_time = now_seconds();
                if (display->snapshot
#ifdef VDMA_HAVE_ROAD_REF
                    && !display->reference_transport
#endif
#ifdef VDMA_DASHBOARD
                    && vdma_dashboard_frame_usable() && (!display->adas || display->adas_bound)
                    && (!display->road_zebra || vdma_dashboard_road_recognition())
#endif
                    && vdma_snapshot_sample(display->snapshot, display->video,
                    &display->snapshot_identity, &display->next.header,
                    display->snapshot_observed_ns)) {
                    perror("Held frame snapshot"); return -1;
                }
                if (display->snapshot && display->snapshot->plane_enabled &&
                    vdma_snapshot_plane_drain(display->snapshot, display->video)) {
                    perror("GRAY8 plane after snapshot"); return -1;
                }
#ifdef VDMA_HAVE_YOLO
                if (display->yolo_snapshot && vdma_dashboard_frame_usable() && display->adas_bound) {
                    stage_started = vdma_road_now_ns();
                    struct yolo_adas_stats stats; yolo_adas_get_stats(display->yolo, &stats);
                    int copied = stats.halted ? 0 :
                        display->yolo_copy.started ?
                        vdma_snapshot_async_submit(&display->yolo_copy, &display->snapshot_identity,
                            &display->next.header, display->snapshot_observed_ns) :
                        vdma_snapshot_sample(display->yolo_snapshot, display->video,
                            &display->snapshot_identity, &display->next.header, display->snapshot_observed_ns);
                    if (copied < 0) {
                        perror("YOLO held frame snapshot"); return -1;
                    }
                    owner_stage_end(display, OWNER_YOLO_COPY, stage_started);
                }
#endif
            } else {
#ifdef VDMA_DASHBOARD
                if (display->current_valid) {
                    int submitted = vdma_dashboard_flip(display->framebuffers[display->current.slot],
                        &display->next_identity, display, 1);
                    if (submitted < 0) { perror("Dashboard UI-only flip"); return -1; }
                    if (submitted) display->pending = true;
                }
#endif
                struct timespec delay = { 0, 500000 };

                nanosleep(&delay, NULL);
            }
        }
        stage_started = vdma_road_now_ns();
        if (display->features) {
            int ready = vdma_road_runtime_step(display->features);
            if (ready < 0) { perror("ROAD runtime"); return -1; }
            if (ready) {
                int taken = vdma_road_runtime_take(display->features, display->road_event);
                if (taken < 0) return -1;
#ifdef VDMA_DASHBOARD
                if (taken) vdma_dashboard_road(display->road_event,
                    &display->features->profiles[display->road_event->identity.source_id]);
#endif
            }
            /* raw ROAD event is owned diagnostics; no semantic algorithm yet. */
        }
        if (display->adas && owner_tick(display, vdma_road_now_ns())) return -1;
#ifdef VDMA_HAVE_FPGA_ROAD
#ifdef VDMA_DASHBOARD
        if (display->fpga_road && drain_fpga_road(display,
                (!display->epoch_valid || display->next_identity.source_id == 2) &&
                vdma_dashboard_road_recognition(),
                vdma_road_now_ns())) {
            perror("FPGA road result");
            return -1;
        }
#endif
#endif
#ifdef VDMA_DASHBOARD
        {
            struct adas_result decision_result;
            struct adas_cursor decision_cursor = {0};
            const uint64_t stamp = vdma_road_now_ns();
            const int rc = display->adas && display->adas_bound ?
                adas_get_latest(display->adas, stamp, &decision_cursor, &decision_result) : ADAS_NO_DATA;
            vdma_dashboard_decision(rc == ADAS_OK_NEW || rc == ADAS_OK_SAME ? &decision_result : NULL, stamp);
        }
#endif
#ifdef VDMA_HAVE_YOLO
        if (display->yolo) {
            uint64_t stamp = vdma_road_now_ns();
            struct yolo_adas_stats stats; yolo_adas_get_stats(display->yolo, &stats);
            if (!display->yolo_report_ns) { display->yolo_report_ns = stamp; display->yolo_previous = stats.published; }
            if (stamp - display->yolo_report_ns >= UINT64_C(1000000000)) {
                display->yolo_fps = (stats.published - display->yolo_previous) * 1e9 / (stamp - display->yolo_report_ns);
                printf("DASHBOARD_YOLO fps=%.3f published=%llu failed=%llu stale=%llu invalid=%llu halted=%u elapsed_s=%.3f busy=%llu rknn_completed=%llu\n",
                    display->yolo_fps, (unsigned long long)stats.published, (unsigned long long)stats.failed,
                    (unsigned long long)stats.stale, (unsigned long long)stats.invalid, stats.halted,
                    now_seconds() - started,(unsigned long long)stats.busy,
                    (unsigned long long)stats.rknn_completed);
                if (display->yolo_snapshot->rgb_mode) {
                    struct vdma_snapshot_rgb_stats r;
                    struct vdma_rgb_stats hw = {0};
                    vdma_snapshot_rgb_get_stats(display->yolo_snapshot, &r);
                    if (ioctl(display->video, VDMA_RGB_STATS, &hw)) {
                        perror("RGB statistics"); return -1;
                    }
                    printf("RGB_RUNTIME received=%llu paired=%llu missing=%llu evicted=%llu "
                        "errors=%llu verified=%llu mismatched=%llu producer=%u consumer=%u "
                        "dropped=%u committed=%u writer_rejected=%u mailbox_skipped=%u "
                        "capture_rejected=%u memory_status=%#x control=%#x small_inputs=%llu cpu_inputs=%llu\n",
                        (unsigned long long)r.received, (unsigned long long)r.paired,
                        (unsigned long long)r.missing, (unsigned long long)r.evicted,
                        (unsigned long long)r.errors, (unsigned long long)r.verified,
                        (unsigned long long)r.mismatched, hw.producer, hw.consumer,
                        hw.dropped, hw.committed, hw.writer_rejected, hw.mailbox_skipped,
                        hw.capture_rejected, hw.memory_status, hw.control,
                        (unsigned long long)stats.small_inputs, (unsigned long long)stats.cpu_inputs);
                }
                display->yolo_report_ns = stamp; display->yolo_previous = stats.published;
            }
            struct adas_result latest;
            int rc = adas_get_latest(display->adas, stamp, &display->yolo_cursor, &latest);
            if (display->yolo_audit && (rc == ADAS_OK_NEW || rc == ADAS_OK_SAME) &&
                adas_branch_ready(&latest, ADAS_DETECTION, stamp, 0)) {
                const struct adas_branch_header *h = &latest.detection.header;
                if (!display->yolo_audit_present || h->identity.stream_epoch != display->yolo_audit_epoch ||
                    h->identity.frame_sequence != display->yolo_audit_sequence) {
                    display->yolo_audit_present = true;
                    display->yolo_audit_epoch = h->identity.stream_epoch;
                    display->yolo_audit_sequence = h->identity.frame_sequence;
                    printf("DASHBOARD_YOLO_RESULT epoch=%u sequence=%u source=%u layout=%u config=%u observed_ns=%llu finished_ns=%llu\n",
                        h->identity.stream_epoch, h->identity.frame_sequence, h->identity.source_id,
                        h->identity.layout, h->identity.config_generation,
                        (unsigned long long)h->observed_ns, (unsigned long long)h->finished_ns);
                }
            }
            struct detection_overlay overlay;
            /* Lock contention is not a new empty/error result. Keep the
             * publication observer's immutable detection until the next read. */
            if (rc != ADAS_BUSY || stats.halted) {
                detection_overlay_from_result(&overlay, rc == ADAS_OK_NEW || rc == ADAS_OK_SAME ? &latest : NULL, stamp);
                overlay.fps = display->yolo_fps;
                if (stats.halted) { overlay.valid = 0; overlay.state = DETECTION_ERROR; }
                vdma_dashboard_detection(&overlay);
            }
        }
#endif
        if (zebra_overlay_update(display)) return -1;
        double now = now_seconds();

        if (now - progress_time > 3) {
            fprintf(stderr, "No frame/flip progress for 3 seconds\n");
            return -1;
        }
        if (now - report_time >= 1) {
#ifdef VDMA_DASHBOARD
            vdma_dashboard_fps((display->flips - report_flips) / (now - report_time));
#endif
            struct vdma_stats stats;

            if (ioctl(display->video, VDMA_STATS, &stats) < 0)
                return -1;
            if (!display->quiet) printf("received=%" PRIu64 " flips=%" PRIu64 " recv_fps=%.2f flip_fps=%.2f "
                   "seq=%u layout=%u source=%s gaps=%" PRIu64 " repeat_refreshes=%" PRIu64
                   " occupancy=%u errors=%u\n", display->received, display->flips,
                   (display->received - report_received) / (now - report_time),
                   (display->flips - report_flips) / (now - report_time),
                   display->current.header.sequence, display->current.header.layout,
                   vdma_layout_name(display->current.header.layout),
                   display->gaps, display->repeated_refreshes,
                   stats.producer - stats.consumer, stats.errors);
            if (!display->quiet) fflush(stdout);
            if (display->features && !display->quiet) {
                struct vdma_road_runtime *c = display->features;
                printf("ROAD packets=%llu matched=%llu expired=%llu malformed=%llu stale=%llu read_errors=%llu\n",
                    (unsigned long long)c->packets, (unsigned long long)c->matched,
                    (unsigned long long)c->expired, (unsigned long long)c->malformed,
                    (unsigned long long)c->stale, (unsigned long long)c->read_errors);
            }
#ifndef VDMA_ROAD_REF_ONLY
            if (display->zebra && !display->quiet) {
                struct road_zebra_adas_stats z;

                road_zebra_adas_stats(display->zebra, &z);
                printf("ROAD_PERF publish_fps=%.3f lane_gray_fps=%.3f lane_gray=%llu "
                       "lane_missing=%llu lane_mean_ms=%.3f lane_max_ms=%.3f\n",
                    (z.published - report_published) / (now - report_time),
                    (z.lane_gray - report_lane_gray) / (now - report_time),
                    (unsigned long long)z.lane_gray, (unsigned long long)z.lane_missing,
                    z.lane_gray ? z.lane_ns / 1e6 / z.lane_gray : 0, z.lane_max_ns / 1e6);
                report_published = z.published;
                report_lane_gray = z.lane_gray;
                printf("%s attempts=%llu published=%llu busy=%llu stale=%llu invalid=%llu "
                       "extract_failed=%llu max_ms=%.2f token_rejects=%llu publish_rejects=%llu "
                        "token_rc=%d publish_rc=%d publish_busy=%llu publish_stale=%llu "
                        "publish_invalid=%llu analyze_failed=%llu\n",
                       display->road_lane_only ? "LANE" : "ZEBRA",
                       (unsigned long long)z.attempts, (unsigned long long)z.published,
                       (unsigned long long)z.busy, (unsigned long long)z.stale,
                       (unsigned long long)z.invalid, (unsigned long long)z.extract_failed,
                       (double)z.max_ns / 1e6,
                       (unsigned long long)z.token_rejected,
                       (unsigned long long)z.publication_rejected,
                        z.last_token_status, z.last_publication_status,
                        (unsigned long long)z.publish_busy,
                        (unsigned long long)z.publish_stale,
                        (unsigned long long)z.publish_invalid,
                        (unsigned long long)z.analyze_failed);
                printf("%s valid=%llu absent=%llu unknown=%llu failed=%llu observed=%llu "
                       "zebras=%llu state=%u reason=%u count=%u box=%u,%u,%u,%u stripes=%u evidence=%u\n",
                       display->road_lane_only ? "LANE_STATE" : "ZEBRA_STATE",
                       (unsigned long long)z.state_valid, (unsigned long long)z.state_absent,
                       (unsigned long long)z.state_unknown, (unsigned long long)z.state_failed,
                       (unsigned long long)z.observed, (unsigned long long)z.zebra_total,
                       z.last_state, z.last_reason, z.last_zebras, z.last_box[0], z.last_box[1],
                       z.last_box[2], z.last_box[3], z.last_stripes, z.last_evidence);
                if (!display->road_lane_only)
                    printf("ZEBRA_DIAG candidate_rows=%llu complete_rows=%llu "
                           "truncated_rows=%llu pattern_rows=%llu reject_truncated=%llu "
                           "reject_pattern=%llu reject_geometry=%llu reject_evidence=%llu "
                           "raw_state=%u raw_count=%u raw_evidence=%u held_frames=%llu\n",
                           (unsigned long long)z.zebra_candidate_rows,
                           (unsigned long long)z.zebra_complete_rows,
                           (unsigned long long)z.zebra_truncated_rows,
                           (unsigned long long)z.zebra_pattern_rows,
                           (unsigned long long)z.zebra_rejected_truncated_rows,
                           (unsigned long long)z.zebra_rejected_pattern_rows,
                           (unsigned long long)z.zebra_rejected_geometry_rows,
                           (unsigned long long)z.zebra_rejected_evidence,
                           z.raw_zebra_state, z.raw_zebras, z.raw_zebra_evidence,
                           (unsigned long long)z.zebra_held_frames);
                if (!display->road_lane_only && z.detector == ROAD_ZEBRA_PIXEL)
                    printf("ZEBRA_PIXEL components=%u retained=%u capacity=%u "
                           "contrast_reject=%u edge_reject=%u geometry_reject=%u "
                           "hypotheses=%u stripes=%u scale=%u\n",
                           z.pixel_diagnostics.components, z.pixel_diagnostics.retained_components,
                           z.pixel_diagnostics.capacity_dropped,
                           z.pixel_diagnostics.rejected_contrast,
                           z.pixel_diagnostics.rejected_edge_pair,
                           z.pixel_diagnostics.rejected_geometry,
                           z.pixel_diagnostics.hypotheses, z.pixel_diagnostics.best_stripes,
                           z.pixel_diagnostics.best_scale);
                fflush(stdout);
            }
#endif
#ifdef VDMA_HAVE_ROAD_REF
            if (display->reference && !display->quiet) {
                struct road_ref_adas_stats r;
                road_ref_adas_stats(display->reference, &r);
                printf("ROAD_REF publish_fps=%.3f attempts=%llu processed=%llu published=%llu "
                    "failed=%llu rejected=%llu cancelled=%llu mean_ms=%.3f max_ms=%.3f "
                    "bgr=%llu planes=%llu sequence=%llu raw=%u confirmed=%u cross=%u stable=%u truncated=%u\n",
                    (r.published - report_published) / (now - report_time),
                    (unsigned long long)r.attempts, (unsigned long long)r.processed,
                    (unsigned long long)r.published, (unsigned long long)r.failed,
                    (unsigned long long)r.rejected, (unsigned long long)r.cancelled,
                    r.attempts ? r.total_ns / 1e6 / r.attempts : 0., r.max_ns / 1e6,
                    (unsigned long long)r.bgr_frames, (unsigned long long)r.plane_frames,
                    (unsigned long long)r.last_sequence, r.raw_count, r.confirmed_count,
                    r.cross_detected, r.cross_stable, r.truncated);
                printf("ROAD_REF_VALIDITY committed=%llu fresh=%llu expired=%llu "
                    "input_age_mean_ms=%.3f end_to_end_mean_ms=%.3f end_to_end_max_ms=%.3f "
                    "lane_mean_ms=%.3f cross_mean_ms=%.3f temporal_mean_ms=%.3f\n",
                    (unsigned long long)r.committed, (unsigned long long)r.published,
                    (unsigned long long)r.expired,
                    r.attempts ? r.input_age_ns / 1e6 / r.attempts : 0.,
                    r.committed ? r.end_to_end_ns / 1e6 / r.committed : 0.,
                    r.end_to_end_max_ns / 1e6,
                    r.processed ? r.lane_ns / 1e6 / r.processed : 0.,
                    r.processed ? r.crosswalk_ns / 1e6 / r.processed : 0.,
                    r.processed ? r.temporal_ns / 1e6 / r.processed : 0.);
                report_published = r.published;
                road_ref_transport_report(display->reference_transport, stdout);
                fflush(stdout);
            }
#endif
#ifdef VDMA_HAVE_FPGA_ROAD
            if (display->fpga_road_state) {
                struct fpga_road_stats s;
                struct vdma_fpga_road_status hw = {.version = 1, .size = sizeof(hw)};
                fpga_road_get_stats(display->fpga_road_state, &s);
                if (ioctl(display->video, VDMA_FPGA_ROAD_STATUS, &hw)) {
                    perror("FPGA road status"); return -1;
                }
                printf("FPGA_ROAD_RUNTIME received=%llu published=%llu unmatched=%llu rejected=%llu "
                    "epoch=%u sequence=%u head_valid=%u dropped=%u consumed=%u\n",
                    (unsigned long long)s.received, (unsigned long long)s.published,
                    (unsigned long long)s.unmatched, (unsigned long long)s.rejected,
                    s.last_epoch, s.last_sequence, hw.head_valid, hw.dropped, hw.consumed);
            }
#endif
            if (stats.errors)
                return -1;
            unsigned int s = display->summary_cursor;
            display->summaries[s].elapsed_ms = (uint64_t)((now - started) * 1000);
            display->summaries[s].frames = display->received;
            display->summaries[s].flips = display->flips;
            display->summaries[s].gaps = display->gaps;
            display->summaries[s].repeats = display->repeated_refreshes;
            display->summaries[s].inputs = display->features ? display->features->packets : 0;
            display->summaries[s].results = 0;
#ifndef VDMA_ROAD_REF_ONLY
            if (display->zebra) {
                struct road_zebra_adas_stats z;

                road_zebra_adas_stats(display->zebra, &z);
                display->summaries[s].results = z.published;
            }
#endif
#ifdef VDMA_HAVE_ROAD_REF
            if (display->reference) {
                struct road_ref_adas_stats r;
                road_ref_adas_stats(display->reference, &r);
                display->summaries[s].results = r.published;
            }
#endif
            display->summary_cursor = (s + 1) % 120;
            if (display->summary_count < 120) display->summary_count++; else display->summary_overwrites++;
            report_time = now;
            report_received = display->received;
            report_flips = display->flips;
        }
        owner_stage_end(display, OWNER_RESULTS, stage_started);
    }
    if (display->features && !display->quiet) {
        struct vdma_road_runtime *c = display->features;
        double elapsed = now_seconds() - started;
        printf("ROAD PERF elapsed_s=%.3f matched_per_s=%.3f cpu_pct_one_core=%.2f\n",
            elapsed, c->matched/elapsed, 100.0*(clock()-cpu_started)/CLOCKS_PER_SEC/elapsed);
    }
    if (display->steady_seconds && !measured) return -1;
    return display->failed ? -1 : 0;
}

static int cleanup(struct display *display)
{
    int result = 0;
    bool detached = !display->active;
#ifdef VDMA_DASHBOARD
    if (display->dashboard_initialized) vdma_dashboard_pause();
    if (display->synchronized_video) vdma_dashboard_sync_stop();
#endif
    if (display->adas && display->adas_bound && adas_stop(display->adas, vdma_road_now_ns()) < 0) result = -1;
    if (display->snapshot) vdma_snapshot_invalidate(display->snapshot);
    if (display->yolo_copy.initialized) {
        vdma_snapshot_invalidate(display->yolo_copy.runtime);
        if (vdma_snapshot_async_stop(&display->yolo_copy)) result = -1;
    }
    /* Join workers before closing any DMA-BUF, video fd or NPU context. */
#ifdef VDMA_HAVE_ROAD_REF
    if (road_ref_transport_stop(display->reference_transport)) result = -1;
#endif
    if (display->snapshot) vdma_snapshot_stop(display->snapshot);
#ifdef VDMA_HAVE_YOLO
    if (display->yolo_snapshot) {
        vdma_snapshot_invalidate(display->yolo_snapshot);
        vdma_snapshot_stop(display->yolo_snapshot);
    }
#endif
    if (display->result_event_live) {
        adas_store_set_publish_notify(display->adas, NULL, NULL);
        close(display->result_event); display->result_event_live = false;
        printf("ADAS_NOTIFY wakes=%" PRIu64 " publications=%" PRIu64 " errors=%" PRIuFAST64 "\n",
            display->result_wakes, display->result_notifications,
            atomic_load(&display->result_event_errors));
    }
#ifdef VDMA_DASHBOARD
    if (display->adas) adas_store_set_publish_observer(display->adas, NULL, NULL);
#endif
    if (display->features && vdma_road_runtime_close(display->features)) result = -1;
#ifdef VDMA_HAVE_ROAD_REF
    if (display->reference) {
        struct road_ref_adas_stats r;
        road_ref_adas_stats(display->reference, &r);
        printf("ROAD_REF_FINAL processed=%llu committed=%llu fresh=%llu expired=%llu "
            "failed=%llu rejected=%llu cancelled=%llu bgr=%llu planes=%llu\n",
            (unsigned long long)r.processed, (unsigned long long)r.committed,
            (unsigned long long)r.published, (unsigned long long)r.expired,
            (unsigned long long)r.failed, (unsigned long long)r.rejected,
            (unsigned long long)r.cancelled, (unsigned long long)r.bgr_frames,
            (unsigned long long)r.plane_frames);
        printf("ROAD_REF_STEADY elapsed_s=%.6f processed=%llu fresh=%llu expired=%llu "
            "fresh_fps=%.6f\n", display->steady_elapsed,
            (unsigned long long)display->reference_steady.processed,
            (unsigned long long)display->reference_steady.published,
            (unsigned long long)display->reference_steady.expired,
            display->steady_elapsed ?
                display->reference_steady.published / display->steady_elapsed : 0.);
    }
    road_ref_transport_report(display->reference_transport, stdout);
    if (road_ref_transport_destroy(display->reference_transport)) result = -1;
    display->reference_transport = NULL;
#endif

    if (display->video >= 0 && ioctl(display->video, VDMA_STOP) < 0) {
        perror("DMA STOP: buffers retained by driver");
        result = -1;
    }
    if (display->active) {
        if (drmModeSetCrtc(display->drm, display->crtc, 0, 0, 0, NULL, 0, NULL)) {
            perror("DRM disable: held slots will not be returned");
            result = -1;
        } else {
            detached = true;
        }
    }
    /* Blocking CRTC disable serializes a last pending atomic flip before
     * detaching/freeing the immutable overlay. close retains it on failure. */
    if (display->zebra_overlay && vdma_osd_probe_close(display->zebra_overlay)) result = -1;
    if (display->osd_probe && (!detached || vdma_osd_probe_close(display->osd_probe))) {
        perror("OSD detach/cleanup"); result = -1;
    }
#ifdef VDMA_DASHBOARD
    if (display->dashboard_initialized && vdma_dashboard_close(detached)) result = -1;
#endif
    if (detached && !display->failed) {
        if (drain_retired(display)) result = -1;
        if (display->current_valid
#ifdef VDMA_DASHBOARD
            && !display->synchronized_video
#endif
            && release_frame(display, &display->current))
            result = -1;
        if (display->next_valid && release_frame(display, &display->next))
            result = -1;
    }
    if (display->drm >= 0) {
        for (unsigned int slot = 0; slot < 4; slot++) {
            if (display->framebuffers[slot])
                drmModeRmFB(display->drm, display->framebuffers[slot]);
            if (display->handles[slot]) {
                struct drm_gem_close close_request = { .handle = display->handles[slot] };

                drmIoctl(display->drm, DRM_IOCTL_GEM_CLOSE, &close_request);
            }
        }
        if (display->blank_framebuffer)
            drmModeRmFB(display->drm, display->blank_framebuffer);
        if (display->blank_handle) {
            struct drm_mode_destroy_dumb request = { .handle = display->blank_handle };

            drmIoctl(display->drm, DRM_IOCTL_MODE_DESTROY_DUMB, &request);
        }
        drmDropMaster(display->drm);
        close(display->drm);
    }
    for (unsigned int slot = 0; slot < 4; slot++)
        if (display->exported[slot] >= 0)
            close(display->exported[slot]);
    if (display->video >= 0)
        close(display->video);
    if (display->snapshot) vdma_snapshot_stop(display->snapshot);
    /* The worker has joined, so the producer context is no longer referenced. */
#ifndef VDMA_ROAD_REF_ONLY
    road_zebra_adas_destroy(display->zebra);
#endif
    display->zebra = NULL;
#ifdef VDMA_HAVE_ROAD_REF
    road_ref_adas_destroy(display->reference);
    display->reference = NULL;
#endif
    return result;
}

int main(int argc, char **argv)
{
    if (getenv("VDMA_LINE_LOG")) setvbuf(stdout, NULL, _IOLBF, 0);
    struct display display = { .drm = -1, .video = -1, .exported = { -1, -1, -1, -1 } };
#ifdef VDMA_HAVE_FPGA_ROAD
    display.fpga_road = true;
#endif
    const char *video_path = NULL;
    const char *drm_path = "/dev/dri/card0";
    double duration = 0;
    glob_t matches = { 0 };
    int result = EXIT_FAILURE;

    for (int index = 1; index < argc; index++) {
        if (!strcmp(argv[index], "--help")) {
            printf("Usage: %s [--device PATH] [--drm PATH] [--connector ID] [--seconds N] "
                   "[--quiet] [--steady-seconds N] "
                   "[--osd-probe-argb PATH] "
                   "[--ui-layout-probe-argb PATH --ui-layout-probe-config PATH] "
                   "[--snapshot-hz 1..30] [--snapshot-rga] [--snapshot-delay-ms 0..5000] "
#ifndef VDMA_ROAD_REF_ONLY
                   "[--road-features] "
                   "[--road-lane | --road-zebra] [--zebra-detector legacy|pixel] "
#endif
                   "[--no-zebra-osd] [--osd-dump PATH]\n"
                   "INT/TERM/HUP stop. "
#ifndef VDMA_ROAD_REF_ONLY
                   "--road-lane uses FPGA Y for lanes; "
                   "--road-zebra also enables Y/C crosswalks with legacy RGB fallback (default 30 Hz). "
                   "--road-features reads raw ROAD candidates. "
#endif
                   "OSD probe is an explicit test image, not recognition.\n", argv[0]);
#ifdef VDMA_HAVE_ROAD_REF
            puts("--road-ref uses FPGA three-plane reference lane+crosswalk; "
                 "--road-ref-period 1..15 (default 2 input frames).");
#ifndef VDMA_ROAD_REF_ONLY
            puts("--road-ref-rgb565 is development input only.");
#else
            puts("Reference-only production build: --road-ref is required.");
#endif
#endif
            return EXIT_SUCCESS;
        }
#ifdef VDMA_ROAD_REF_ONLY
        if (!strcmp(argv[index], "--road-features") ||
            !strcmp(argv[index], "--road-lane") || !strcmp(argv[index], "--road-zebra") ||
            !strcmp(argv[index], "--zebra-detector") || !strcmp(argv[index], "--zebra-dump") ||
            !strcmp(argv[index], "--road-ref-rgb565")) {
            fprintf(stderr, "%s unavailable in reference-only production build; use --road-ref\n",
                    argv[index]);
            return EXIT_FAILURE;
        }
#endif
        if (!strcmp(argv[index], "--snapshot-rga")) { display.snapshot_rga = true; continue; }
#ifdef VDMA_HAVE_ROAD_REF
        if (!strcmp(argv[index], "--road-ref-period")) {
            if (++index >= argc) { fputs("--road-ref-period needs 1..15\n", stderr); return EXIT_FAILURE; }
            char *end; const char *value = argv[index];
            unsigned long parsed = strtoul(value, &end, 10);
            if (!*value || *end || parsed < 1 || parsed > 15) return EXIT_FAILURE;
            display.road_reference_period = (unsigned int)parsed;
            continue;
        }
        if (!strcmp(argv[index], "--road-ref-rgb565")) {
            display.road_reference = display.road_reference_rgb565 = true;
            display.road_zebra = true; continue;
        }
        if (!strcmp(argv[index], "--road-ref")) {
            display.road_reference = true; display.road_zebra = true; continue;
        }
#endif
#ifdef VDMA_HAVE_YOLO
        if (!strcmp(argv[index], "--yolo-audit")) { display.yolo_audit = true; continue; }
        if (!strcmp(argv[index], "--yolo-config") && index + 1 < argc) {
            display.yolo_path = argv[++index]; continue;
        }
#endif
#ifdef VDMA_HAVE_FPGA_ROAD
        if (!strcmp(argv[index], "--road-features")) {
            fprintf(stderr, "--road-features is retired from the FPGA-road production build.\n");
            return EXIT_FAILURE;
        }
#else
        if (!strcmp(argv[index], "--road-features")) { display.features_enabled = true; continue; }
#endif
        if (!strcmp(argv[index], "--features") || !strcmp(argv[index], "--color-regions") ||
            !strcmp(argv[index], "--joint-features") || !strcmp(argv[index], "--color-config")) {
            fprintf(stderr, "COLOR/joint live options retired.\n");
            return EXIT_FAILURE;
        }
        if (!strcmp(argv[index], "--road-zebra")) {
            if (display.road_lane_only) {
                fprintf(stderr, "Choose either --road-lane or --road-zebra.\n");
                return EXIT_FAILURE;
            }
            display.road_zebra = true; continue;
        }
        if (!strcmp(argv[index], "--road-lane")) {
            if (display.road_zebra && !display.road_lane_only) {
                fprintf(stderr, "Choose either --road-lane or --road-zebra.\n");
                return EXIT_FAILURE;
            }
            display.road_zebra = true; display.road_lane_only = true; continue;
        }
        if (!strcmp(argv[index], "--no-zebra-osd")) { display.zebra_osd_disabled = true; continue; }
        if (!strcmp(argv[index], "--zebra-detector") && index + 1 < argc) {
            const char *mode = argv[++index];
            if (!strcmp(mode, "legacy")) display.zebra_detector = ROAD_ZEBRA_LEGACY;
            else if (!strcmp(mode, "pixel")) display.zebra_detector = ROAD_ZEBRA_PIXEL;
            else { fprintf(stderr, "Expected zebra detector legacy|pixel\n"); return EXIT_FAILURE; }
            continue;
        }
        if (!strcmp(argv[index], "--osd-dump") && index + 1 < argc) {
            display.osd_dump = argv[++index];
            continue;
        }
        if (!strcmp(argv[index], "--zebra-osd")) { display.zebra_osd = true; continue; }
        if (!strcmp(argv[index], "--zebra-dump")) {
            if (++index >= argc) { fprintf(stderr, "--zebra-dump needs a path\n"); return EXIT_FAILURE; }
            display.zebra_dump = argv[index];
            continue;
        }
        if (!strcmp(argv[index], "--quiet")) { display.quiet = true; continue; }
        if (!strcmp(argv[index], "--osd")) {
            fprintf(stderr, "--osd was removed: the diagnostic status bar is retired; "
                            "detection overlays are not implemented yet.\n");
            return EXIT_FAILURE;
        }
        if (index + 1 >= argc) {
            fprintf(stderr, "Missing option value\n");
            return EXIT_FAILURE;
        }
        if (!strcmp(argv[index], "--snapshot-hz") || !strcmp(argv[index], "--snapshot-delay-ms")) {
            bool rate = !strcmp(argv[index], "--snapshot-hz");
            char *end; const char *value = argv[++index];
            unsigned long parsed = strtoul(value, &end, 10);
            if (!*value || *end || parsed > (rate ? 30UL : 5000UL) || (rate && !parsed))
                return EXIT_FAILURE;
            if (rate) display.snapshot_hz = parsed;
            else display.snapshot_delay_ms = parsed;
        }
        else if (!strcmp(argv[index], "--steady-seconds")) {
            char *end; display.steady_seconds = strtod(argv[++index], &end);
            if (*end || !(display.steady_seconds > 0 && display.steady_seconds <= 3600)) return EXIT_FAILURE;
        }
        else if (!strcmp(argv[index], "--device"))
            video_path = argv[++index];
        else if (!strcmp(argv[index], "--drm"))
            drm_path = argv[++index];
        else if (!strcmp(argv[index], "--osd-probe-argb"))
            display.osd_probe_path = argv[++index];
        else if (!strcmp(argv[index], "--ui-layout-probe-argb"))
            display.ui_probe_path = argv[++index];
        else if (!strcmp(argv[index], "--ui-layout-probe-config"))
            display.ui_layout_path = argv[++index];
        else if (!strcmp(argv[index], "--connector"))
            display.connector = strtoul(argv[++index], NULL, 10);
        else if (!strcmp(argv[index], "--seconds")) {
            char *end;

            duration = strtod(argv[++index], &end);
            if (*end || !(duration > 0 && duration < 1e9))
                return EXIT_FAILURE;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[index]);
            return EXIT_FAILURE;
        }
    }
#ifdef VDMA_ROAD_REF_ONLY
#ifdef VDMA_HAVE_FPGA_ROAD
    if (!display.fpga_road) {
        fputs("FPGA-road production build requires HDMI FPGA road path\n", stderr);
        return EXIT_FAILURE;
    }
#else
    if (!display.road_reference) {
        fputs("Reference-only production build requires --road-ref\n", stderr);
        return EXIT_FAILURE;
    }
#endif
#endif
    if (install_signals()) { perror("sigaction"); goto out; }
#ifdef VDMA_HAVE_YOLO
    if (display.yolo_path && yolo_config_load(&display.yolo_config, display.yolo_path, stderr)) {
        fprintf(stderr, "YOLO configuration/model verification failed\n"); goto out;
    }
#endif
    if (!!display.ui_probe_path != !!display.ui_layout_path ||
        (display.ui_probe_path && display.osd_probe_path)) {
        fprintf(stderr,"UI probe requires ARGB + config and is mutually exclusive with OSD probe\n"); goto out;
    }
    if (display.ui_layout_path && vdma_ui_layout_load(&display.ui_layout, display.ui_layout_path)) {
        perror("UI layout config"); goto out;
    }
#ifdef VDMA_DASHBOARD
    display.road_zebra = true;
#ifdef VDMA_HAVE_FPGA_ROAD
    display.fpga_road = true;
    display.snapshot_hz = 0;
    printf("FPGA_ROAD input=HDMI native algorithm=FRD1 cadence=FPGA_CAP osd=previous-style\n");
#else
    if (!display.snapshot_hz) display.snapshot_hz = 10;
#ifdef VDMA_HAVE_ROAD_REF
    if (display.road_reference) {
        if (!display.road_reference_period) display.road_reference_period = 2;
        if (display.road_lane_only) { fprintf(stderr, "Reference requires lane and crosswalk together\n"); goto out; }
        display.snapshot_hz = 30;
        printf("ROAD_REF input=%s algorithm=reference lane=1280x720 cross=1920x1080 period=%u\n",
            display.road_reference_rgb565 ? "RGB565_DEVELOPMENT" : "FPGA_THREE_PLANES",
            display.road_reference_period);
    }
#endif
#endif
    printf("DASHBOARD_ROAD configured=1 lanes=1 zebra=%d snapshot_hz=%u\n",
        !display.road_lane_only, display.snapshot_hz);
#endif
#ifndef VDMA_HAVE_FPGA_ROAD
    if (display.road_zebra && !display.snapshot_hz) display.snapshot_hz = 30;
#endif
#ifdef VDMA_HAVE_RGA
#ifndef VDMA_HAVE_FPGA_ROAD
    if (display.road_zebra) display.snapshot_rga = true;
#ifdef VDMA_HAVE_ROAD_REF
    if (display.road_reference && !display.road_reference_rgb565) display.snapshot_rga = false;
#endif
#endif
#endif
    if ((display.snapshot_delay_ms || display.snapshot_rga) && !display.snapshot_hz) {
        fprintf(stderr, "--snapshot-delay-ms/--snapshot-rga requires --snapshot-hz\n"); goto out;
    }
    if (!video_path) {
        if (glob("/dev/video_dma_*", 0, NULL, &matches) || matches.gl_pathc != 1) {
            fprintf(stderr, "Specify --device; expected one video_dma node\n");
            goto out;
        }
        video_path = matches.gl_pathv[0];
    }
    display.drm = open(drm_path, O_RDWR | O_CLOEXEC);
    if (display.drm < 0 || drmSetMaster(display.drm)) {
        perror("DRM master: stop Weston first");
        goto out;
    }
    if (select_output(&display))
        goto out;
    display.video = open(video_path,
#ifdef VDMA_DASHBOARD
        O_RDWR |
#else
        (display.features_enabled ? O_RDWR : O_RDONLY) |
#endif
        O_NONBLOCK | O_CLOEXEC);
    if (display.video < 0) {
        perror("video device");
        goto out;
    }
    /* Detected boxes go on screen by default; --no-zebra-osd keeps the plane off. */
    if (display.road_zebra && !display.zebra_osd_disabled) display.zebra_osd = true;
    /* The ADAS store must exist before the snapshot worker can publish into it. */
    if (display.features_enabled || display.snapshot_hz || display.road_zebra
#ifdef VDMA_HAVE_YOLO
        || display.yolo_path
#endif
       ) {
        display.adas = calloc(1, sizeof(*display.adas));
        if (!display.adas || adas_store_init(display.adas) < 0) goto out;
#ifdef VDMA_DASHBOARD
        if (adas_store_set_publish_observer(display.adas, presentation_observe, NULL) != ADAS_OK_NEW)
            goto out;
#endif
        const char *notify = getenv("VDMA_ADAS_NOTIFY");
        if (notify && strcmp(notify, "0") && strcmp(notify, "1")) {
            fputs("VDMA_ADAS_NOTIFY expects 0 or 1\n", stderr); goto out;
        }
        atomic_init(&display.result_event_errors, 0);
        if (!notify || strcmp(notify, "0")) {
            display.result_event = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
            if (display.result_event < 0) { perror("ADAS result eventfd"); goto out; }
            display.result_event_live = true;
            if (adas_store_set_publish_notify(display.adas, result_notify, &display) != ADAS_OK_NEW)
                goto out;
        }
        printf("ADAS_NOTIFY_INIT enabled=%u\n", display.result_event_live);
    }
#ifdef VDMA_HAVE_FPGA_ROAD
    if (display.fpga_road) {
        display.fpga_road_state = fpga_road_create(display.adas);
        if (!display.fpga_road_state) { perror("FPGA road adapter"); goto out; }
    }
#endif
    if (display.snapshot_hz) {
        if (vdma_snapshot_probe(display.video)) { perror("Snapshot capability"); goto out; }
        display.snapshot = calloc(1, sizeof(*display.snapshot));
        if (!display.snapshot) { perror("Snapshot buffers"); goto out; }
        if (display.road_zebra) {
#if defined(VDMA_HAVE_ROAD_REF) || !defined(VDMA_ROAD_REF_ONLY)
            struct vdma_snapshot_processor processor;
#endif

#ifdef VDMA_HAVE_ROAD_REF
            if (display.road_reference) {
                display.reference = road_ref_adas_create(display.adas);
                if (!display.reference ||
                    (display.road_reference_rgb565 ?
                        road_ref_adas_rgb565_processor(display.reference, &processor) :
                        road_ref_adas_processor(display.reference, &processor)) ||
                    vdma_snapshot_init_processor(display.snapshot, display.snapshot_hz,
                        display.snapshot_delay_ms, &processor)) {
                    perror("Reference producer"); goto out;
                }
                if (!display.road_reference_rgb565) {
                    display.reference_transport = road_ref_transport_create(display.video,
                        display.road_reference_period);
                    if (!display.reference_transport) {
                        perror("FPGA three-plane ABI/driver required"); goto out;
                    }
                    const char *copy_backend = getenv("VDMA_ROAD_REF_COPY_BACKEND");
                    if (copy_backend && strcmp(copy_backend, "cpu")) {
                        if (strcmp(copy_backend, "rga")) {
                            fprintf(stderr, "VDMA_ROAD_REF_COPY_BACKEND must be cpu or rga\n");
                            goto out;
                        }
                        int exported[RR_HOST_SLOT_COUNT] = {-1, -1, -1};
                        int error = 0;
                        for (unsigned int i = 0; i < RR_HOST_SLOT_COUNT; ++i) {
                            struct vdma_rr_export request = {
                                .version = 1, .size = sizeof(request), .slot = i};
                            exported[i] = ioctl(display.video, VDMA_RR_EXPORT, &request);
                            if (exported[i] < 0) { error = errno; break; }
                        }
                        if (!error && vdma_snapshot_enable_reference_rga(display.snapshot, exported))
                            error = errno;
                        for (unsigned int i = 0; i < RR_HOST_SLOT_COUNT; ++i)
                            if (exported[i] >= 0) close(exported[i]);
                        if (error) { errno = error; perror("Reference RGA required"); goto out; }
                        fprintf(stderr, "ROAD_REF_COPY backend=rga bytes=%u synchronous=1 crc=all\n",
                            RR_PAYLOAD_BYTES);
                    }
                    if (road_ref_transport_start_async(display.reference_transport)) {
                        perror("Reference copy worker"); goto out;
                    }
                }
            }
#ifndef VDMA_ROAD_REF_ONLY
            else
#endif
#endif
#ifndef VDMA_ROAD_REF_ONLY
            {
            display.zebra = road_zebra_adas_create_tasks(display.adas,
                ADAS_ROAD_LEFT | ADAS_ROAD_RIGHT | (display.road_lane_only ? 0U : ADAS_ROAD_ZEBRA));
            if (!display.zebra ||
                road_zebra_adas_set_detector(display.zebra, display.zebra_detector) ||
                road_zebra_adas_processor(display.zebra, &processor) ||
                vdma_snapshot_init_processor(display.snapshot, display.snapshot_hz,
                    display.snapshot_delay_ms, &processor)) {
                perror("Crosswalk producer"); goto out;
            }
            if (display.zebra_dump) road_zebra_adas_dump(display.zebra, display.zebra_dump);
            if (vdma_snapshot_enable_plane(display.snapshot, display.video)) {
                perror("Plane requires a matching V2 GRAY8 or V3 YC driver"); goto out;
            }
            if (display.snapshot->plane_format == VDMA_PLANE_FORMAT_YC8 &&
                !display.zebra_dump &&
                vdma_snapshot_enable_plane_direct(display.snapshot)) {
                perror("YC plane worker"); goto out;
            }
            }
#endif
        } else if (vdma_snapshot_init(display.snapshot,
            display.snapshot_hz, display.snapshot_delay_ms)) {
            perror("Snapshot buffers"); goto out;
        }
    }
#ifdef VDMA_HAVE_YOLO
    if (display.yolo_path) {
        struct vdma_snapshot_processor processor;
        display.yolo = yolo_adas_create(display.adas, display.yolo_config.model,
            display.yolo_config.confidence, display.yolo_config.nms_iou, stderr);
        display.yolo_snapshot = calloc(1, sizeof(*display.yolo_snapshot));
        if (!display.yolo || !display.yolo_snapshot) goto out;
        yolo_adas_processor(display.yolo, &processor);
        display.yolo_processor = processor;
        processor.process = yolo_preview_process; processor.context = &display;
        if (vdma_snapshot_probe(display.video) || vdma_snapshot_init_processor(display.yolo_snapshot,
            display.yolo_config.hz, 0, &processor)) goto out;
        const char *mode = getenv("VDMA_SMALL_RGB_MODE");
#ifdef VDMA_HAVE_FPGA_ROAD
        if (!mode) mode = "small";
#else
        if (!mode) mode = "cpu";
#endif
        unsigned int rgb_mode = 0;
        if (!strcmp(mode, "shadow")) rgb_mode = 1;
        else if (!strcmp(mode, "small")) rgb_mode = 2;
        else if (!strcmp(mode, "verify")) rgb_mode = 3;
        else if (strcmp(mode, "cpu")) {
            fprintf(stderr, "VDMA_SMALL_RGB_MODE expects cpu, shadow, small, or verify\n"); goto out;
        }
        if (rgb_mode && display.yolo_config.hz != 30) {
            fprintf(stderr, "Small RGB requires a 30Hz YOLO configuration\n");
            goto out;
        }
        if (rgb_mode && vdma_snapshot_enable_rgb(display.yolo_snapshot, display.video, rgb_mode)) {
            fprintf(stderr, "Small RGB ABI4 initialization: %s\n", strerror(errno));
            goto out;
        }
        printf("YOLO_INPUT mode=%s exact_pair=1 automatic_fallback=0 full_snapshot=lamp_color_and_tracking\n", mode);
    }
#endif
    if (display.features_enabled) {
        display.features = calloc(1, sizeof(*display.features));
        display.road_event = calloc(1, sizeof(*display.road_event));
        if (!display.features || !display.road_event || vdma_road_runtime_init(display.features, display.video)) {
            perror("ROAD-only capability"); goto out;
        }
    }
    struct vdma_stats initial_stats;

    if (ioctl(display.video, VDMA_STATS, &initial_stats) < 0) {
        perror("DMA preflight");
        goto out;
    }
    if (initial_stats.errors || initial_stats.status == UINT32_MAX ||
        (initial_stats.status & 9) != 9) {
        fprintf(stderr, "DMA not ready: status=%#x errors=%#x\n",
                initial_stats.status, initial_stats.errors);
        goto out;
    }
    if (import_frames(&display))
        goto out;
    if (display.snapshot_rga && vdma_snapshot_enable_rga(display.snapshot, display.exported)) {
        perror("Snapshot RGA initialization"); goto out;
    }
#ifdef VDMA_HAVE_YOLO
    if (display.yolo_snapshot && vdma_snapshot_enable_rga(display.yolo_snapshot, display.exported)) {
        perror("YOLO RGA initialization"); goto out;
    }
    if (display.yolo_snapshot) {
        const char *copy_mode = getenv("VDMA_YOLO_COPY_MODE");
        if (copy_mode && strcmp(copy_mode, "sync") && strcmp(copy_mode, "async")) {
            fprintf(stderr, "VDMA_YOLO_COPY_MODE must be sync or async\n"); goto out;
        }
        if ((!copy_mode || strcmp(copy_mode, "sync")) &&
            vdma_snapshot_async_init(&display.yolo_copy, display.yolo_snapshot, display.video)) {
            perror("YOLO async copy initialization"); goto out;
        }
        printf("YOLO_COPY mode=%s bounded=1 ordered_release=1\n",
               display.yolo_copy.started ? "async" : "sync");
    }
#endif
    if (prepare_output(&display)) {
        perror("DRM prepare RGB565 output");
        goto out;
    }
#ifdef VDMA_DASHBOARD
    if (display.osd_probe_path || display.ui_probe_path) {
        fprintf(stderr,"Dashboard cannot be combined with static OSD probe\n"); goto out;
    }
    display.dashboard_initialized = true; /* Also clean partial init. */
    if (vdma_dashboard_init(display.drm, display.crtc, display.video)) {
        perror("Dashboard initialization"); goto out;
    }
    if (vdma_dashboard_sync_init(display.exported)) {
        perror("Synchronized video initialization"); goto out;
    }
    display.synchronized_video = vdma_dashboard_sync_enabled();
#endif
    if (display.osd_probe_path || display.ui_probe_path) {
        display.osd_probe = calloc(1, sizeof(*display.osd_probe));
        if (!display.osd_probe || vdma_osd_probe_init(display.osd_probe, display.drm,
                display.crtc, display.ui_probe_path ? display.ui_probe_path : display.osd_probe_path)) {
            perror("OSD test initialization"); goto out;
        }
        if (display.ui_probe_path && vdma_osd_probe_set_layout(display.osd_probe, &display.ui_layout)) {
            perror("UI layout geometry"); goto out;
        }
        if (display.ui_probe_path) printf("UI_LAYOUT video=%ux%u+%u+%u source=1920x1080 static_test_only=1\n",
            display.ui_layout.video.width, display.ui_layout.video.height,
            display.ui_layout.video.x, display.ui_layout.video.y);
    }
#ifndef VDMA_DASHBOARD
    if (display.zebra_osd) {
        display.zebra_overlay = calloc(1, sizeof(*display.zebra_overlay));
        if (!display.zebra_overlay || vdma_osd_probe_init_blank(display.zebra_overlay,
                display.drm, display.crtc) || vdma_osd_probe_show(display.zebra_overlay, 1)) {
            perror("Crosswalk OSD overlay");
            free(display.zebra_overlay);
            display.zebra_overlay = NULL;
            display.zebra_osd = false;
        } else {
            printf("ROAD_OSD plane=%u pitch=%u lanes_and_zebras=1 blend=premultiplied\n",
                   display.zebra_overlay->plane, display.zebra_overlay->pitch);
        }
    }
#else
    if (display.zebra_osd) printf("ROAD_OSD owner=dashboard lanes_and_zebras=1\n");
#endif
    printf("HDMI connector=%u crtc=%u 1920x1080@60 RGB565; ordered DMA-BUF display\n",
           display.connector, display.crtc);
    if (!run_display(&display, duration))
        result = EXIT_SUCCESS;
out:
    if (cleanup(&display)) result = EXIT_FAILURE;
    for (unsigned int i = 0; i < display.owner_anomaly_count; ++i)
        for (unsigned int j = 0; j < display.anomaly_trace_count[i]; ++j) {
            const struct owner_frame_trace *t = &display.anomaly_trace[i][j];
            printf("OWNER_FRAME anomaly=%u stamp_ns=%" PRIu64 " sequence=%u token=%u "
                   "producer=%u consumer=%u captured=%u dropped=%u errors=%u retired=%u query_error=%d\n",
                   i, t->stamp_ns, t->sequence, t->token, t->stats.producer,
                   t->stats.consumer, t->stats.captured, t->stats.dropped,
                   t->stats.errors, t->retired, t->error);
        }
    printf("YOLO_COPY_FINAL submitted=%" PRIu64 " completed=%" PRIu64
           " busy=%" PRIu64 " max_ms=%.3f release_deferred=%" PRIu64
           " retired_max=%" PRIu64 " remaining=%u\n",
           display.yolo_copy.submitted, display.yolo_copy.completed,
           display.yolo_copy.busy, display.yolo_copy.max_ns / 1e6,
           display.release_deferred, display.retired_max, display.retired_count);
    vdma_snapshot_async_destroy(&display.yolo_copy);
    if (display.osd_probe) {
        printf("OSD_PROBE Summary: toggles=%llu active=%d test_only=1\n",
            (unsigned long long)display.osd_probe->toggles, display.osd_probe->active);
        free(display.osd_probe);
    }
    if (display.zebra_overlay) free(display.zebra_overlay);
    if (display.snapshot && display.snapshot->initialized) {
        vdma_snapshot_report(display.snapshot, stdout);
        if (display.snapshot->worker_error) result = EXIT_FAILURE;
        /* Brief visits may be invalidated before a background result can finish.
         * Fixed-source board validation separately requires copied/processed > 0. */
        uint64_t minimum_span = display.snapshot->period_ns + UINT64_C(1000000000) +
            (uint64_t)display.snapshot_delay_ms * UINT64_C(1000000);
        for (unsigned int source = 0; source < VDMA_SNAPSHOT_SOURCE_COUNT; source++) {
            const struct vdma_snapshot_stats *s = &display.snapshot->stats;
            if ((display.displayed_layout_mask & (1U << source)) &&
                s->source_max_span_ns[source] >= minimum_span &&
                (!s->source_copied[source] || !s->source_processed[source])) {
                fprintf(stderr, "SNAPSHOT: source %u displayed long enough but no owned image processed\n",
                    source);
                result = EXIT_FAILURE;
            }
        }
        vdma_snapshot_destroy(display.snapshot);
    }
    free(display.snapshot);
#ifdef VDMA_HAVE_YOLO
    if (display.yolo_snapshot) {
        vdma_snapshot_report(display.yolo_snapshot, stdout);
        if (display.yolo_snapshot->worker_error) result = EXIT_FAILURE;
        vdma_snapshot_destroy(display.yolo_snapshot); free(display.yolo_snapshot);
    }
    if (display.yolo) {
        struct yolo_adas_stats stats; yolo_adas_get_stats(display.yolo, &stats);
        printf("YOLO Summary: published=%llu failed=%llu stale=%llu invalid=%llu halted=%u\n",
            (unsigned long long)stats.published, (unsigned long long)stats.failed,
            (unsigned long long)stats.stale, (unsigned long long)stats.invalid, stats.halted);
        printf("YOLO_INPUT_FINAL small=%llu cpu=%llu preprocess_mean_ms=%.3f inference_mean_ms=%.3f light_mean_ms=%.3f\n",
            (unsigned long long)stats.small_inputs, (unsigned long long)stats.cpu_inputs,
            stats.attempts ? stats.preprocess_ns / 1e6 / stats.attempts : 0.,
            stats.rknn_completed ? stats.inference_ns / 1e6 / stats.rknn_completed : 0.,
            stats.rknn_completed ? stats.light_color_ns / 1e6 / stats.rknn_completed : 0.);
        printf("YOLO_WORKER attempts=%llu busy=%llu cancelled=%llu rknn_completed=%llu "
            "inputs_ms=%.3f run_ms=%.3f outputs_ms=%.3f\n",
            (unsigned long long)stats.attempts, (unsigned long long)stats.busy,
            (unsigned long long)stats.cancelled, (unsigned long long)stats.rknn_completed,
            stats.rknn_completed ? stats.rknn_inputs_ns / 1e6 / stats.rknn_completed : 0.,
            stats.rknn_completed ? stats.rknn_run_ns / 1e6 / stats.rknn_completed : 0.,
            stats.rknn_completed ? stats.rknn_outputs_ns / 1e6 / stats.rknn_completed : 0.);
        printf("YOLO_STEADY elapsed_s=%.6f published=%llu fps=%.6f token_invalid_retries=%llu\n",
            display.steady_elapsed, (unsigned long long)display.yolo_steady_published,
            display.steady_elapsed ? display.yolo_steady_published / display.steady_elapsed : 0,
            (unsigned long long)stats.token_invalid_retries);
        if (stats.failed || stats.invalid || stats.halted) result = EXIT_FAILURE;
        yolo_adas_destroy(display.yolo);
    }
#endif
    printf("Summary: received=%" PRIu64 " completed_flips=%" PRIu64
           " gaps=%" PRIu64 " repeat_refreshes=%" PRIu64 " layout_mask=%#x\n",
           display.received, display.flips, display.gaps, display.repeated_refreshes,
           display.displayed_layout_mask);
    printf("STEADY elapsed_s=%.6f cpu_pct_one_core=%.3f frames=%" PRIu64 " flips=%" PRIu64
        " gaps=%" PRIu64 " repeats=%" PRIu64 " loop_max_ns=%" PRIu64 " control_max_ns=%" PRIu64
        " drm_event_max_ns=%" PRIu64 " summaries=%u overwritten=%" PRIu64 " display_bytes=%zu\n", display.steady_elapsed,
        display.steady_cpu, display.steady_frames, display.steady_flips, display.steady_gaps,
        display.steady_repeats, display.loop_max_ns, display.control_max_ns,
        display.drm_event_max_ns, display.summary_count, display.summary_overwrites, sizeof(display));
#ifdef VDMA_HAVE_FPGA_ROAD
    printf("FPGA_ROAD_STEADY elapsed_s=%.6f received=%llu published=%llu receive_fps=%.6f publish_fps=%.6f "
        "unmatched=%llu rejected=%llu dropped=%u\n", display.steady_elapsed,
        (unsigned long long)display.fpga_road_steady.received,
        (unsigned long long)display.fpga_road_steady.published,
        display.steady_elapsed ? display.fpga_road_steady.received / display.steady_elapsed : 0.,
        display.steady_elapsed ? display.fpga_road_steady.published / display.steady_elapsed : 0.,
        (unsigned long long)display.fpga_road_steady.unmatched,
        (unsigned long long)display.fpga_road_steady.rejected, display.fpga_road_steady.dropped);
#endif
    const char *stage_names[] = {"control", "poll", "event", "acquire", "flip", "yolo_copy", "results"};
    for (unsigned int i = 0; i < OWNER_STAGE_COUNT; i++)
        printf("OWNER_STAGE name=%s count=%" PRIu64 " mean_us=%.3f max_us=%.3f\n",
            stage_names[i], display.owner_stages[i].count,
            display.owner_stages[i].count ? display.owner_stages[i].total_ns / 1000.0 / display.owner_stages[i].count : 0.,
            display.owner_stages[i].max_ns / 1000.0);
    printf("OWNER_ANOMALIES count=%u overflow=%" PRIu64 " stage_order=control,poll,event,acquire,flip,yolo_copy,results\n",
        display.owner_anomaly_count, display.owner_anomaly_overflow);
    printf("OWNER_TICK retries=%" PRIu64 "\n", display.owner_tick_retries);
    for (unsigned int i = 0; i < display.owner_anomaly_count; i++) {
        printf("OWNER_ANOMALY kind=%s stamp_ns=%" PRIu64 " sequence=%u distance=%u previous_us=",
            display.owner_anomalies[i].kind == 1 ? "source_gap" : "drm_repeat",
            display.owner_anomalies[i].stamp_ns, display.owner_anomalies[i].sequence,
            display.owner_anomalies[i].distance);
        for (unsigned int j = 0; j < OWNER_STAGE_COUNT; j++)
            printf("%s%.3f", j ? "," : "", display.owner_anomalies[i].previous_ns[j] / 1000.0);
        printf(" current_us=");
        for (unsigned int j = 0; j < OWNER_STAGE_COUNT; j++)
            printf("%s%.3f", j ? "," : "", display.owner_anomalies[i].current_ns[j] / 1000.0);
        putchar('\n');
    }
    unsigned int summary_start = (display.summary_cursor + 120 - display.summary_count) % 120;
    for (unsigned int i = 0; i < display.summary_count; i++) {
        unsigned int s = (summary_start + i) % 120;
        printf("SECOND elapsed_ms=%" PRIu64 " frames=%" PRIu64 " flips=%" PRIu64
            " gaps=%" PRIu64 " repeats=%" PRIu64 " inputs=%" PRIu64 " results=%" PRIu64 "\n",
            display.summaries[s].elapsed_ms, display.summaries[s].frames, display.summaries[s].flips,
            display.summaries[s].gaps, display.summaries[s].repeats, display.summaries[s].inputs,
            display.summaries[s].results);
    }
    if (display.features) {
        struct vdma_road_runtime *c = display.features;
        printf("ROAD Summary: packets=%llu matched=%llu expired=%llu orphaned=%llu conflicts=%llu malformed=%llu stale=%llu read_errors=%llu\n",
            (unsigned long long)c->packets, (unsigned long long)c->matched,
            (unsigned long long)c->expired, (unsigned long long)c->orphaned,
            (unsigned long long)c->conflicts, (unsigned long long)c->malformed,
            (unsigned long long)c->stale, (unsigned long long)c->read_errors);
        if ((display.displayed_layout_mask & 15U) && !c->matched) {
            fprintf(stderr, "ROAD: no candidate packet matched a video frame\n"); result = EXIT_FAILURE;
        }
        printf("PIO pages=%llu mean_us=%.3f max_us=%.3f runtime_bytes=%zu source_packets=%llu,%llu,%llu,%llu\n",
            (unsigned long long)c->read_pages, c->read_pages ? c->read_ns / 1000.0 / c->read_pages : 0,
            c->read_max_ns / 1000.0, sizeof(*c), (unsigned long long)c->source_packets[0],
            (unsigned long long)c->source_packets[1], (unsigned long long)c->source_packets[2],
            (unsigned long long)c->source_packets[3]);
    }
    if (display.adas) {
        printf("ADAS: ROAD_semantics=%s raw_ROAD_is_not_a_lane_result\n",
               display.road_zebra ? (display.road_lane_only ? "LANE_ONLY" : "LANE_AND_ZEBRA") : "DISABLED");
        adas_store_destroy(display.adas); free(display.adas);
    }
#ifdef VDMA_HAVE_FPGA_ROAD
    fpga_road_destroy(display.fpga_road_state);
    display.fpga_road_state = NULL;
#endif
    free(display.road_event);
    free(display.features);
    globfree(&matches);
    return result;
}
