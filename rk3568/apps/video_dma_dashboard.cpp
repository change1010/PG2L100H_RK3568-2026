#include <QGuiApplication>
#include <QImage>
#include <QTimer>
#include <QSocketNotifier>
#include <QFile>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSaveFile>
#include <QFontDatabase>
#include <QRawFont>
#include <QTextStream>
#include <atomic>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <deque>
#include <memory>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/input.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
extern "C" {
#include "video_dma_dashboard.h"
#include "video_dma_osd_probe.h"
#include "video_dma_ui_buffers.h"
#include "video_dma_dark_async.h"
#include "video_dma_dashboard_input.h"
#include "detection_light_tracker.h"
#include "video_dma_uapi.h"
#include "video_dma_snapshot_uapi.h"
#include "video_dma_snapshot.h"
#include "video_dma_frame.h"
#include "adas_signal_test.h"
#include "adas_pedestrian_test.h"
#include "light_specialist.h"
#ifdef VDMA_HAVE_YOLO
#include "yolo_adas_frozen.h"
#endif
#include "../reference/road_ref/road_ref.h"
int vdma_display_main(int argc, char **argv);
}
#include "pedestrian_alert.h"
#include "../lib/video_dma_dashboard_paint.h"
#include "../lib/video_dma_specialist_batch.h"
#include "../lib/video_dma_dashboard_lane_display.h"
#include "../lib/video_dma_dashboard_crosswalk_display.h"
#include "../lib/video_dma_dashboard_vehicle_display.h"
#include "../lib/video_dma_dashboard_vehicle_trace.h"
#include "../lib/video_dma_dashboard_crosswalk_trace.h"
#include "../lib/video_dma_dashboard_preview.h"
#include "../lib/video_dma_dashboard_record.h"
#include "../lib/video_dma_dashboard_wake.h"
#include "../lib/video_dma_dashboard_sync.h"
#include "../lib/video_dma_dashboard_history.h"
#include "../lib/video_dma_dashboard_layers.h"
#include "../lib/video_dma_dashboard_osd.h"
#include "fpga_road_osd.h"
static DashboardPreview preview;
static DashboardRecord recorder;
static DashboardWake renderWake;
static DashboardSync syncVideo;
static std::unique_ptr<DashboardHistory> resultHistory;
static std::atomic<unsigned> specialistProfile{1};
static unsigned specialistTaskPeriodMs=334;
static uint64_t residentLightLifetimeNs()
{
    /* Include image sampling and task execution without refreshing source age. */
    return uint64_t(std::min(1000U,std::max(500U,specialistTaskPeriodMs+450U)))*1000000;
}
static bool residentLightEnabled()
{
    const char *value = std::getenv("VDMA_LIGHT_RESIDENT");
    if (!value) value = std::getenv("VDMA_TRIAL_RESIDENT_LIGHT");
    return value && !std::strcmp(value,"1");
}
static bool specialistEnabled()
{
#ifdef VDMA_HAVE_YOLO
    const char *path = std::getenv("VDMA_LIGHT_MODEL");
    return path && *path;
#else
    return false;
#endif
}
static bool recordingFailed = false;

static const char *lightCaption(const detection_overlay_box &box)
{
    return box.display_held ? "暂未检出" :
        box.light_color == DETECTION_LIGHT_RED ? "红灯" :
        box.light_color == DETECTION_LIGHT_YELLOW ? "黄灯" :
        box.light_color == DETECTION_LIGHT_GREEN ? "绿灯" : "灯色未知";
}

static uint64_t milliseconds()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
static uint64_t driveNow()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Dashboard {
    std::mutex mutex;
    std::condition_variable drawn;
    vdma_osd_probe osd{};
    vdma_ui_buffers buffers{};
    DashboardOsd presentation;
    bool clean_presentation = false;
    bool exact_presentation = false;
    bool qt_exact = false;
    struct Argb {
        uint32_t handle = 0, framebuffer = 0, pitch = 0;
        uint64_t bytes = 0;
        void *pixels = nullptr;
    } extra[2];
    bool overlay_tested[4] = {};
    uint32_t handle = 0, fb = 0, pitch = 0;
    uint64_t bytes = 0;
    void *pixels = nullptr;
    int video = -1, drm = -1;
    uint32_t crtc = 0;
    bool live = false, drawing = false, recognition = false;
    bool masks[3] = {true, true, true};
    bool pedestrian_alert_enabled = false;
    uint64_t alert_control_revision = 0;
    adas_identity alert_context{};
    uint32_t alert_status = UINT32_MAX, alert_enabled_value = UINT32_MAX;
    vdma_dark_async dark{};
    vdma_frame_identity frame{};
    road_overlay_geometry road{};
    detection_overlay detection{};
    uint64_t road_revision = 0;
    uint64_t crosswalk_control_revision = 0;
    uint64_t vehicle_control_revision = 0;
    bool frame_bound = false;
    bool invalidate = false;
    double fps = 0;
    adas_decision drive{};
    adas_decision_config drive_config{};
    adas_signal_test signal_test{};
    adas_ped_test pedestrian_test{};
    bool pedestrian_mode = false, pedestrian_task_started = false;
    std::deque<adas_ped_observation> test_people;
    adas_lane_test_config lane_config = adas_lane_test_default_config();
    std::mutex test_road_mutex;
    std::deque<adas_lane_test_observation> test_roads;
    struct TestImage {
        vdma_frame_identity identity{};
        vdma_frame_header header{};
        uint64_t observed = 0;
        std::vector<unsigned char> pixels;
    };
    std::shared_ptr<const TestImage> latest_test_image, frozen_test_image;
    detection_overlay resident_light{};
    adas_signal_test_observation resident_observation{};
    unsigned resident_profile = UINT32_MAX;
    uint64_t resident_generation = 1, resident_tasks = 0, resident_rejected = 0;
    uint32_t resident_last_frame = 0;
    QImage test_picture;
    detection_overlay test_detection{};
    adas_signal_test_observation test_observation{};
    detection_overlay frozen_light_display{};
    uint64_t frozen_light_context = 0, frozen_light_signature = 0;
    unsigned frozen_light_profile = UINT32_MAX;
    bool frozen_light_locked = false, frozen_light_uncertain = false;
    DashboardSpecialistBatch test_batch;
    uint64_t test_task = 0;
    uint64_t test_capture_ns = 0;
    uint32_t intent = ADAS_DRIVE_UNKNOWN;
    bool intent_valid = false;
    uint64_t intent_generation = 0;
    uint64_t revision = 0, ui_commits = 0;
    uint64_t dark_changes = 0;
    uint64_t render_total_ns = 0, render_max_ns = 0, full_renders = 0;
    uint64_t chrome_renders = 0;
    uint64_t argb_copy_bytes = 0, paint_total_ns = 0, copy_total_ns = 0;
    uint64_t road_renders = 0, road_geometry_renders = 0;
    uint64_t clear_ns = 0, chrome_ns = 0, overlay_ns = 0, damage_ns = 0;
    struct RoadSlot {
        bool valid = false;
        vdma_frame_identity context{};
        uint32_t sequence = 0;
        uint64_t observed = 0, finished = 0, received = 0, render = 0, ready = 0, until = 0;
    } road_slots[4], submitted_road{}, last_presented_road{};
    uint64_t road_received_ns = 0;
    uint64_t road_presentations = 0, road_age_ns = 0, road_age_max_ns = 0;
    uint64_t road_wait_ns = 0, road_draw_ns = 0, road_flip_ns = 0, road_owner_ns = 0;
    uint64_t road_frame_gap = 0, road_frame_gap_max = 0, road_expired_flips = 0;
    vdma_frame_identity submitted_video{};
    bool flip_outstanding = false;
    int prior_phase = -1;
    Dashboard()
    {
        adas_decision_init(&drive);
        adas_signal_test_init(&signal_test);
        adas_ped_test_init(&pedestrian_test);
        drive_config.hazard_required = 1;
    }
} ui;

static std::mutex pedestrianAlertMutex;
static pedestrian_alert pedestrianAlert = [] {
    pedestrian_alert state{};
    ped_alert_init(&state);
    return state;
}();
static uint64_t pedestrianControlApplied = 0;
static std::atomic<uint64_t> pedestrianAlertDrops{0};
static std::atomic<bool> pedestrianAlertAccepting{false};

/* Never nest the UI and alert locks. A control serial rejects reordered copies. */
static void refreshPedestrianAlert()
{
    adas_identity id{};
    uint64_t serial;
    uint32_t enabled, status;
    {
        std::lock_guard<std::mutex> lock(ui.mutex);
        enabled = ui.pedestrian_alert_enabled;
        status = !ui.recognition ? PED_ALERT_RECOGNITION_OFF :
            !ui.frame_bound || ui.invalidate || ui.frame.flags ? PED_ALERT_UNSUPPORTED :
            PED_ALERT_WAITING;
        id = {ui.frame.stream_epoch,ui.frame.frame_sequence,ui.frame.source_id,ui.frame.layout,
            ui.frame.config_generation,ui.frame.canvas_width,ui.frame.canvas_height};
        id.frame_sequence = 0;
        if (enabled != ui.alert_enabled_value || status != ui.alert_status ||
            std::memcmp(&id,&ui.alert_context,sizeof(id))) {
            ++ui.alert_control_revision;
            ui.alert_enabled_value = enabled; ui.alert_status = status; ui.alert_context = id;
        }
        serial = ui.alert_control_revision;
    }
    std::lock_guard<std::mutex> lock(pedestrianAlertMutex);
    if (serial <= pedestrianControlApplied) return;
    pedestrianControlApplied = serial;
    const auto generation = pedestrianAlert.output.generation;
    ped_alert_control(&pedestrianAlert,enabled,status,&id,driveNow());
    pedestrianAlertAccepting.store(pedestrianAlert.running != 0);
    if (pedestrianAlert.output.generation != generation) {
        std::printf("PEDESTRIAN_ALERT_CONTROL enabled=%u status=%u generation=%llu source=%u epoch=%u config=%u roi_version=%u\n",
            enabled,pedestrianAlert.output.status,
            static_cast<unsigned long long>(pedestrianAlert.output.generation),
            id.source_id,id.stream_epoch,id.config_generation,pedestrianAlert.config.version);
        std::fflush(stdout); renderWake.request();
    }
}

extern "C" void vdma_dashboard_alert_publish(const adas_result *result, uint32_t branch, uint64_t now)
{
    if (!pedestrianAlertAccepting.load()) return;
    std::unique_lock<std::mutex> lock(pedestrianAlertMutex,std::try_to_lock);
    if (!lock.owns_lock()) { pedestrianAlertDrops.fetch_add(1); return; }
    ped_alert_publish(&pedestrianAlert,result,branch,now);
}

static ped_alert_output pedestrianAlertSnapshot()
{
    std::lock_guard<std::mutex> lock(pedestrianAlertMutex);
    auto output = pedestrianAlert.output;
    const auto now = driveNow();
    if (pedestrianAlert.running &&
        ((output.valid_until_ns && now >= output.valid_until_ns) ||
         (!output.observed_ns && now >= pedestrianAlert.accept_after_ns &&
          now-pedestrianAlert.accept_after_ns >= ADAS_DETECTION_TTL_NS))) {
        output.level = PED_ALERT_NONE; output.status = PED_ALERT_UNAVAILABLE;
        output.reason = PED_ALERT_STALE;
    }
    return output;
}

static void tickPedestrianAlert(uint64_t now)
{
    refreshPedestrianAlert();
    now = std::max(now,driveNow());
    std::lock_guard<std::mutex> lock(pedestrianAlertMutex);
    const uint64_t lost = pedestrianAlertDrops.exchange(0);
    if (lost && pedestrianAlert.running) {
        pedestrianAlert.output.dropped += lost;
        ped_alert_gap(&pedestrianAlert);
    }
    const auto before = pedestrianAlert.output.revision;
    ped_alert_tick(&pedestrianAlert,now,4);
    const auto &o = pedestrianAlert.output;
    if (before != o.revision) {
        std::printf("PEDESTRIAN_ALERT status=%u level=%u reason=%u frame=%u generation=%llu complete=%u road=%u age_ms=%.3f evaluations=%llu paired=%llu dropped=%llu rejected=%llu\n",
            o.status,o.level,o.reason,o.identity.frame_sequence,
            static_cast<unsigned long long>(o.generation),o.complete,o.road_available,
            o.observed_ns && now >= o.observed_ns ? double(now-o.observed_ns)/1e6 : 0.,
            static_cast<unsigned long long>(o.evaluations),static_cast<unsigned long long>(o.paired),
            static_cast<unsigned long long>(o.dropped),static_cast<unsigned long long>(o.rejected));
        std::fflush(stdout); renderWake.request();
    }
}

static void *overlayPixels(int index)
{ return index == 0 ? ui.osd.pixels : index == 1 ? ui.pixels : ui.extra[index-2].pixels; }
static uint32_t overlayPitch(int index)
{ return index == 0 ? ui.osd.pitch : index == 1 ? ui.pitch : ui.extra[index-2].pitch; }
static uint32_t overlayFramebuffer(int index)
{ return index == 0 ? ui.osd.framebuffer : index == 1 ? ui.fb : ui.extra[index-2].framebuffer; }

static int createArgb(Dashboard::Argb &out, int drm)
{
    drm_mode_create_dumb create{}; create.width = 1920; create.height = 1080; create.bpp = 32;
    if (drmIoctl(drm,DRM_IOCTL_MODE_CREATE_DUMB,&create)) return -1;
    out.handle = create.handle; out.pitch = create.pitch; out.bytes = create.size;
    if (out.pitch < 7680 || out.bytes < uint64_t(out.pitch)*1080) { errno = EPROTO; return -1; }
    drm_mode_map_dumb map{}; map.handle = out.handle;
    if (drmIoctl(drm,DRM_IOCTL_MODE_MAP_DUMB,&map)) return -1;
    out.pixels = mmap(nullptr,out.bytes,PROT_READ|PROT_WRITE,MAP_SHARED,drm,map.offset);
    if (out.pixels == MAP_FAILED) { out.pixels = nullptr; return -1; }
    std::memset(out.pixels,0,out.bytes);
    uint32_t handles[4] = {out.handle}, pitches[4] = {out.pitch}, offsets[4] = {};
    return drmModeAddFB2(drm,1920,1080,DRM_FORMAT_ARGB8888,handles,pitches,offsets,&out.framebuffer,0);
}
static int destroyArgb(Dashboard::Argb &out, int drm)
{
    int result = 0;
    if (out.framebuffer && drmModeRmFB(drm,out.framebuffer)) result = -1;
    if (out.pixels && munmap(out.pixels,out.bytes)) result = -1;
    if (out.handle) {
        drm_mode_destroy_dumb destroy{}; destroy.handle = out.handle;
        if (drmIoctl(drm,DRM_IOCTL_MODE_DESTROY_DUMB,&destroy)) result = -1;
    }
    out = Dashboard::Argb{}; return result;
}

/* Called under ui.mutex. The legacy paint fields mirror the selected input. */
static void syncDriveLocked()
{
    ui.drive.selected = ui.drive.applied = ui.pedestrian_mode ?
        ui.pedestrian_test.control : ui.signal_test.control;
    ui.drive.output = ui.pedestrian_mode ? ui.pedestrian_test.output : ui.signal_test.output;
    ui.drive.head = ui.drive.count = ui.drive.queue_lost = 0;
    ui.intent = ui.drive.selected.command;
    ui.intent_valid = ui.intent != ADAS_DRIVE_UNKNOWN;
    ui.intent_generation = ui.drive.selected.generation;
}
static void resetDriveLocked()
{
    adas_signal_test_clear(&ui.signal_test,driveNow());
    adas_ped_test_clear(&ui.pedestrian_test,driveNow());
    syncDriveLocked();
}
static void exitTestLocked()
{
    if (!ui.recognition || !ui.frame_bound || ui.invalidate || !ui.live) {
        ++ui.resident_generation;
        ui.resident_light = {};
        ui.resident_observation = {};
        ui.resident_last_frame = 0;
    }
    ui.frozen_light_display = {};
    ui.frozen_light_locked = ui.frozen_light_uncertain = false;
    ui.frozen_light_context = ui.frozen_light_signature = 0;
    ui.frozen_light_profile = UINT32_MAX;
    adas_signal_test_exit(&ui.signal_test,driveNow());
    adas_ped_test_exit(&ui.pedestrian_test,driveNow());
    ui.pedestrian_task_started = false;
    ui.frozen_test_image.reset();
    ui.latest_test_image.reset();
    ui.test_picture = QImage{};
    ui.test_detection = detection_overlay{};
    ui.test_observation = {};
    ui.test_batch = {};
    ui.test_capture_ns = 0;
    syncDriveLocked();
    renderWake.request();
}

static bool sameTestContext(const vdma_frame_identity &a, const vdma_frame_identity &b)
{
    return a.stream_epoch == b.stream_epoch && a.source_id == b.source_id &&
        a.layout == b.layout && a.config_generation == b.config_generation;
}

/* Raw publications only: display hold/prediction never supplies test evidence. */
static void rememberTestRoad(const adas_drive_frame &frame,
    const road_ref_geometry &geometry, uint64_t observed, uint64_t until, uint64_t now)
{
    if (!frame.valid || !frame.stream_epoch || frame.source_id != 2 || frame.layout != 2 ||
        !observed || observed > now || until <= now) return;
    std::unique_lock<std::mutex> lock(ui.test_road_mutex,std::try_to_lock);
    if (!lock.owns_lock()) return;
    for (const auto &old : ui.test_roads)
        if (adas_lane_test_same_frame(&old.frame,&frame)) return;
    adas_lane_test_observation observation{};
    observation.frame = frame;
    observation.observed_ns = observed; observation.valid_until_ns = until;
    adas_lane_test_select(&geometry,&ui.lane_config,observation.sides);
    ui.test_roads.push_back(observation);
    if (ui.test_roads.size() > 32) ui.test_roads.pop_front();
}

static void bindTestRoadLocked(uint64_t now)
{
    if (!ui.recognition || !ui.frame_bound || ui.invalidate) return;
    std::lock_guard<std::mutex> lock(ui.test_road_mutex);
    for (const auto &road : ui.test_roads)
        adas_signal_test_observe_road(&ui.signal_test,&road,now);
}

static bool specialistTaskNeededLocked(uint64_t now = driveNow())
{
    return ui.live && !ui.pedestrian_mode && ui.test_batch.needed(ui.signal_test,now);
}

static vdma_snapshot_input frozenTestInput(const vdma_snapshot_input &live,
    const Dashboard::TestImage &image, uint64_t context);

static detection_overlay pedestrianOverlay(const adas_ped_observation &o)
{
    detection_overlay overlay{};
    overlay.state = o.valid ? DETECTION_RUNNING : DETECTION_ERROR;
    overlay.valid = o.valid;
    overlay.stream_epoch=o.frame.stream_epoch; overlay.source_id=o.frame.source_id;
    overlay.layout=o.frame.layout; overlay.config_generation=o.frame.config_generation;
    overlay.frame_sequence=o.frame.frame_sequence; overlay.observed_ns=o.observed_ns;
    overlay.valid_until_ns=UINT64_MAX;
    for (unsigned i=0;i<o.count && i<ADAS_PED_MAX_OBJECTS;++i) {
        const auto &b=o.objects[i];
        overlay.boxes[overlay.count++]={b.x0,b.y0,b.x1,b.y1,0,b.confidence,0,0,0};
    }
    return overlay;
}

static void logPedestrianRaw(const adas_ped_test &s, bool cached)
{
    const auto &o=s.observation;
    std::printf("PEDESTRIAN_EVIDENCE context=%llu epoch=%u frame=%u source=%u layout=%u config=%u image_ns=%llu finished_ns=%llu model=%u/%u/%u input_profile=%u cached=%u valid=%u complete=%u count=%u risk=%u reason=%u\n",
        static_cast<unsigned long long>(s.context),o.frame.stream_epoch,o.frame.frame_sequence,
        o.frame.source_id,o.frame.layout,o.frame.config_generation,
        static_cast<unsigned long long>(o.observed_ns),static_cast<unsigned long long>(o.finished_ns),
        o.algorithm_id,o.algorithm_version,o.model_generation,o.input_profile_generation,
        cached,o.valid,o.complete,o.count,s.risk,s.reason);
    for (unsigned i=0;i<o.count && i<ADAS_PED_MAX_OBJECTS;++i) {
        const auto &b=o.objects[i];
        std::printf("PEDESTRIAN_RAW context=%llu index=%u box=%d,%d,%d,%d score=%u position_unknown=%u\n",
            static_cast<unsigned long long>(s.context),b.original_index,b.x0,b.y0,b.x1,b.y1,
            b.confidence,b.position_unknown);
    }
}

static int processPedestrianTask(yolo_adas *producer, const vdma_snapshot_input *input,
                                 vdma_snapshot_output *output)
{
    std::shared_ptr<const Dashboard::TestImage> image;
    uint64_t context;
    {
        std::lock_guard<std::mutex> lock(ui.mutex);
        const auto &s = ui.pedestrian_test;
        if (!ui.pedestrian_mode || !s.active || s.received || ui.pedestrian_task_started ||
            !ui.recognition || !ui.frame_bound || ui.invalidate ||
            driveNow()-s.started_ns >= UINT64_C(3000000000)) return 0;
        image = ui.frozen_test_image; context = s.context;
        if (!image) return 0;
        ui.pedestrian_task_started = true;
    }
    const auto fixed = frozenTestInput(*input,*image,context);
    adas_ped_observation observation{};
    detection_overlay detected{};
    std::memset(output,0,sizeof(*output));
#ifdef VDMA_HAVE_YOLO
    const int status = yolo_adas_frozen_pedestrian(producer,&fixed,output,&detected,&observation);
#else
    (void)producer;
    const int status = VDMA_PROCESS_FAILED;
    output->error = ENOSYS;
#endif
    std::lock_guard<std::mutex> lock(ui.mutex);
    if (!ui.pedestrian_mode || ui.frozen_test_image != image ||
        ui.pedestrian_test.context != context || !ui.pedestrian_test.active) return 1;
    const uint64_t now = driveNow();
    int accepted = adas_ped_test_observe(&ui.pedestrian_test,context,&observation,image->observed,1,now);
    if (status != VDMA_PROCESS_OK || !accepted) {
        ui.pedestrian_test.received = 1;
        ui.pedestrian_test.risk = ADAS_PED_UNKNOWN;
        ui.pedestrian_test.reason = ADAS_PED_FAILED;
    }
    if (accepted) ui.test_detection = pedestrianOverlay(observation);
    std::printf("PEDESTRIAN_READY context=%llu frame=%u path=%u model=%u/%u/%u input_profile=%u frozen_task=1 accepted=%d status=%d count=%u complete=%u risk=%u reason=%u wait_ms=%.3f\n",
        static_cast<unsigned long long>(context),image->identity.frame_sequence,
        ui.pedestrian_test.path.version,observation.algorithm_id,observation.algorithm_version,
        observation.model_generation,observation.input_profile_generation,accepted,status,
        observation.count,observation.complete,ui.pedestrian_test.risk,ui.pedestrian_test.reason,
        double(now-ui.pedestrian_test.started_ns)/1e6);
    if (accepted) logPedestrianRaw(ui.pedestrian_test,false);
    std::fflush(stdout);
    syncDriveLocked(); renderWake.request();
    return 1;
}

static vdma_snapshot_input frozenTestInput(const vdma_snapshot_input &live,
    const Dashboard::TestImage &image, uint64_t context)
{
    auto fixed = live;
    fixed.pixels = image.pixels.data(); fixed.identity = image.identity;
    fixed.header = image.header; fixed.observed_ns = image.observed;
    fixed.session = context;
    /* Auxiliary pointers belong to the current live callback, not this image. */
    fixed.small_rgb = nullptr; fixed.rgb_descriptor = {}; fixed.native = {};
    fixed.gray = {}; fixed.yc = {}; fixed.reference_planes = nullptr;
    return fixed;
}

static detection_overlay residentLightSnapshotLocked(const vdma_frame_identity &frame, uint64_t now)
{
    const auto &d = ui.resident_light;
    if (!residentLightEnabled() || !ui.recognition || !ui.frame_bound || ui.invalidate ||
        frame.source_id != 2 || frame.layout != 2 || !d.valid ||
        ui.resident_profile != specialistProfile.load() ||
        d.stream_epoch != frame.stream_epoch || d.source_id != frame.source_id ||
        d.layout != frame.layout || d.config_generation != frame.config_generation ||
        now < d.observed_ns || now >= d.valid_until_ns ||
        uint32_t(frame.frame_sequence-d.frame_sequence) >= UINT32_C(0x80000000)) return {};
    return d;
}

static void updateResidentLightLocked(const detection_overlay &detected, bool frozen, uint64_t now)
{
    if (detected.valid && detected.count) {
        ui.resident_light = detected;
        ui.resident_profile = specialistProfile.load();
        ui.resident_light.valid_until_ns = frozen ? UINT64_MAX :
            detected.observed_ns+residentLightLifetimeNs();
    } else if (!residentLightSnapshotLocked(ui.frame,now).valid) {
        ui.resident_light = {};
    }
}

static bool sameLightImage(const detection_overlay &d, const Dashboard::TestImage &image)
{
    return d.valid && d.stream_epoch == image.identity.stream_epoch &&
        d.source_id == image.identity.source_id && d.layout == image.identity.layout &&
        d.config_generation == image.identity.config_generation &&
        d.frame_sequence == image.identity.frame_sequence && d.observed_ns == image.observed;
}

/* Frozen geometry is display data; only new inference tasks supply votes. */
static void freezeLightResultLocked(const detection_overlay &d,
    const adas_signal_test_observation &o, unsigned profile, bool success)
{
    if (!residentLightEnabled() || ui.pedestrian_mode || !ui.signal_test.active ||
        !ui.frozen_test_image || !ui.recognition || !ui.frame_bound || ui.invalidate ||
        profile != specialistProfile.load()) return;
    if (success && !sameLightImage(d,*ui.frozen_test_image)) return;
    if (!ui.frozen_light_locked) {
        if (!success || !d.count) return;
        ui.frozen_light_display = d;
        ui.frozen_light_display.valid_until_ns = UINT64_MAX;
        ui.frozen_light_display.light_temporal_valid = 0;
        ui.frozen_light_context = ui.signal_test.context;
        ui.frozen_light_signature = o.group_signature;
        ui.frozen_light_profile = profile;
        ui.frozen_light_locked = true;
        std::printf("TRIAL_LIGHT_FROZEN context=%llu frame=%u boxes=%u profile=%u locked=1\n",
            (unsigned long long)ui.frozen_light_context,d.frame_sequence,d.count,profile);
        return;
    }
    if (ui.frozen_light_context != ui.signal_test.context ||
        ui.frozen_light_profile != profile || ui.frozen_light_uncertain) return;
    bool conflict = !success || d.count != ui.frozen_light_display.count ||
        !o.groups_valid || o.reason ||
        (ui.frozen_light_signature && o.group_signature != ui.frozen_light_signature);
    for (unsigned i=0; !conflict && i<d.count; ++i)
        conflict = d.boxes[i].light_color != ui.frozen_light_display.boxes[i].light_color;
    if (conflict) {
        for (unsigned i=0; i<ui.frozen_light_display.count; ++i)
            ui.frozen_light_display.boxes[i].light_color = DETECTION_LIGHT_UNKNOWN;
        ui.frozen_light_uncertain = true;
        std::printf("TRIAL_LIGHT_FROZEN context=%llu frame=%u conflict=1 geometry_retained=1\n",
            (unsigned long long)ui.frozen_light_context,ui.frozen_light_display.frame_sequence);
    }
}

static detection_overlay specialistDisplayLocked(const vdma_frame_identity &frame, uint64_t now)
{
    if (!ui.signal_test.active || ui.pedestrian_mode)
        return residentLightSnapshotLocked(frame,now);
    if (!ui.recognition || !ui.frame_bound || ui.invalidate || !ui.frozen_test_image ||
        !ui.frozen_light_locked || ui.frozen_light_context != ui.signal_test.context ||
        ui.frozen_light_profile != specialistProfile.load() ||
        !sameTestContext(frame,ui.frozen_test_image->identity) ||
        !sameLightImage(ui.frozen_light_display,*ui.frozen_test_image)) return {};
    return ui.frozen_light_display;
}

static bool residentTaskNeededLocked()
{
    return residentLightEnabled() && specialistEnabled() && ui.live && ui.recognition &&
        ui.frame_bound && !ui.invalidate && !ui.signal_test.active;
}

static int processResidentLightTask()
{
#ifdef VDMA_HAVE_YOLO
    std::shared_ptr<const Dashboard::TestImage> image;
    uint64_t generation, sequence;
    unsigned profileIndex;
    {
        std::lock_guard<std::mutex> lock(ui.mutex);
        if (!residentTaskNeededLocked() ||
            ui.frame.source_id != 2 || ui.frame.layout != 2) return 0;
        image = ui.latest_test_image;
        if (!image || !sameTestContext(image->identity,ui.frame)) return 0;
        const auto now = driveNow();
        if (now < image->observed || now-image->observed >= UINT64_C(500000000) ||
            image->identity.frame_sequence == ui.resident_last_frame) return 0;
        generation = ui.resident_generation;
        profileIndex = specialistProfile.load();
        sequence = ++ui.resident_tasks;
    }
    vdma_snapshot_input empty{};
    auto input = frozenTestInput(empty,*image,generation);
    detection_overlay detected{};
    adas_signal_test_observation observation{};
    observation.context = generation; observation.selection = 1;
    observation.perception = generation; observation.sequence = sequence;
    vdma_snapshot_output output{};
    const auto *profile = light_specialist_profile_at(profileIndex);
    const auto started = driveNow();
    const int status = light_specialist_analyze(&input,profile->scope,profile->layout,
        &output,&detected,&observation,nullptr);
    const auto finished = driveNow();
    std::lock_guard<std::mutex> lock(ui.mutex);
    const bool current = generation == ui.resident_generation && ui.recognition &&
        ui.frame_bound && !ui.invalidate && sameTestContext(ui.frame,image->identity) &&
        profileIndex == specialistProfile.load() && finished >= image->observed &&
        finished-image->observed < UINT64_C(500000000);
    const bool accepted = current && status == VDMA_PROCESS_OK && output.diagnostic_value;
    if (accepted) {
        updateResidentLightLocked(detected,false,finished);
        ui.resident_observation = observation;
        ui.resident_last_frame = image->identity.frame_sequence;
        freezeLightResultLocked(detected,observation,profileIndex,true);
    } else ++ui.resident_rejected;
    std::printf("TRIAL_RESIDENT_LIGHT task=%llu frame=%u generation=%llu accepted=%u status=%d "
        "groups=%u boxes=%u elapsed_ms=%.3f frozen=0 continuous=1 input_age_ms=%.3f "
        "display_boxes=%u display_until_ns=%llu\n",
        (unsigned long long)sequence,image->identity.frame_sequence,
        (unsigned long long)generation,accepted,status,observation.groups_valid,detected.count,
        double(finished-started)/1e6,double(finished-image->observed)/1e6,
        residentLightSnapshotLocked(ui.frame,finished).count,
        (unsigned long long)ui.resident_light.valid_until_ns);
    std::fflush(stdout);
    renderWake.request();
    return accepted ? 1 : -1;
#else
    return 0;
#endif
}

static detection_overlay residentDisplayLocked(const detection_overlay &main,
    const vdma_frame_identity &frame, uint64_t now)
{
    if (!residentLightEnabled()) return main;
    auto display = main;
    display.count = 0; display.light_temporal_valid = 0;
    for (uint32_t i = 0; i < main.count && i < DETECTION_OVERLAY_CAPACITY; ++i)
        if (!main.boxes[i].signal) display.boxes[display.count++] = main.boxes[i];
    const auto light = specialistDisplayLocked(frame,now);
    for (uint32_t i = 0; i < light.count && display.count < DETECTION_OVERLAY_CAPACITY; ++i)
        display.boxes[display.count++] = light.boxes[i];
    return display;
}

static int processTestTask(yolo_adas *producer,
    const vdma_snapshot_input *input, vdma_snapshot_output *output)
{
    std::shared_ptr<const Dashboard::TestImage> frozen;
    uint64_t context = 0, selection = 0, perception = 0, task = 0;
    unsigned profileIndex = specialistProfile.load();
    {
        std::lock_guard<std::mutex> lock(ui.mutex);
        if (!specialistTaskNeededLocked()) return 0;
        frozen = ui.frozen_test_image;
        context = ui.signal_test.context; selection = ui.signal_test.selection;
        perception = ui.signal_test.perception;
        task = ++ui.test_task;
    }
    std::memset(output,0,sizeof(*output));
    if (!frozen) { output->error = EINVAL; return 1; }
    const auto fixed = frozenTestInput(*input,*frozen,context);
    const uint64_t started = driveNow();
    detection_overlay detected{};
#ifdef VDMA_HAVE_YOLO
    adas_signal_test_observation specialistObservation{};
    specialistObservation.context = context;
    specialistObservation.selection = selection;
    specialistObservation.perception = perception;
    specialistObservation.sequence = task;
    int status;
    if (specialistEnabled()) {
        const auto *profile = light_specialist_profile_at(profileIndex);
        status = light_specialist_analyze(&fixed,profile->scope,profile->layout,
            output,&detected,&specialistObservation,nullptr);
    } else status = yolo_adas_frozen_overlay(producer,&fixed,output,&detected);
#else
    (void)producer;
    const auto status = VDMA_PROCESS_FAILED;
    output->error = ENOSYS;
#endif
    const uint64_t finished = driveNow();
    std::lock_guard<std::mutex> lock(ui.mutex);
    if (!ui.signal_test.active || context != ui.signal_test.context ||
        perception != ui.signal_test.perception || ui.frozen_test_image != frozen ||
        profileIndex != specialistProfile.load()) return 1;
    adas_signal_test_observation observation{};
#ifdef VDMA_HAVE_YOLO
    if (specialistEnabled()) observation = specialistObservation;
#endif
    observation.context = context; observation.selection = selection; observation.sequence = task;
    observation.perception = perception;
    const auto &id = frozen->identity;
    observation.frame = {1,id.stream_epoch,id.source_id,id.layout,id.config_generation,id.frame_sequence};
    observation.started_ns = started; observation.finished_ns = finished;
    /* TL-08 must certify physical groups and their scope. Until then the
     * raw detector must never manufacture direction or actionable evidence. */
    if (!specialistEnabled())
        observation.reason = status == VDMA_PROCESS_OK && output->diagnostic_value ?
            ADAS_TEST_GROUP_UNKNOWN : ADAS_TEST_FAILED;
    else if (status != VDMA_PROCESS_OK || !output->diagnostic_value)
        observation.reason = status == VDMA_PROCESS_CANCELLED ? ADAS_TEST_NOT_STABLE : ADAS_TEST_FAILED;
    freezeLightResultLocked(detected,observation,profileIndex,
        status == VDMA_PROCESS_OK && output->diagnostic_value);
    if (residentLightEnabled() && ui.frozen_light_uncertain && !observation.reason)
        observation.reason = ADAS_TEST_COLOR_CONFLICT;
    adas_signal_test_observe(&ui.signal_test,&observation,driveNow());
    ui.test_batch.observe(ui.signal_test,observation);
    std::printf("LIGHT_SPECIALIST_READY context=%llu selection=%llu task=%llu lane=%u count=%u readiness=%u reason=%u color=%u elapsed_ms=%.3f batch=%u left_count=%u left_ready=%u straight_count=%u straight_ready=%u\n",
        static_cast<unsigned long long>(context),static_cast<unsigned long long>(selection),
        static_cast<unsigned long long>(task),ui.signal_test.control.ego_lane,
        ui.signal_test.stable_count,ui.signal_test.readiness,
        ui.signal_test.reason,ui.signal_test.color,double(finished-started)/1e6,
        ui.signal_test.completed_tasks,ui.signal_test.directions[0].stable_count,
        ui.signal_test.directions[0].readiness,ui.signal_test.directions[1].stable_count,
        ui.signal_test.directions[1].readiness);
    std::fflush(stdout);
    ui.test_detection = detected;
    ui.test_observation = observation;
    syncDriveLocked();
    renderWake.request();
    return 1;
}

static bool testPictureChanged(const unsigned char *pixels, const Dashboard::TestImage &saved,
                               const detection_overlay &lights,
                               const adas_lane_test_observation *road = nullptr,
                               bool people = false)
{
    auto different = [&](int x,int y) {
        const size_t offset=size_t(y*1920+x)*2;
        const unsigned a=pixels[offset] | unsigned(pixels[offset+1])<<8;
        const unsigned b=saved.pixels[offset] | unsigned(saved.pixels[offset+1])<<8;
        return std::abs(int((a>>11)&31)-int((b>>11)&31))>3 ||
            std::abs(int((a>>5)&63)-int((b>>5)&63))>6 ||
            std::abs(int(a&31)-int(b&31))>3;
    };
    unsigned changed=0,total=0;
    for (int y=8;y<1080;y+=24) for (int x=8;x<1920;x+=24) {
        changed+=different(x,y);++total;
    }
    if (changed*100>total*6) return true;
    for (uint32_t i=0;i<lights.count && i<DETECTION_OVERLAY_CAPACITY;++i) {
        const auto &b=lights.boxes[i];
        if (!b.signal && !(people && b.model_class == 0)) continue;
        changed=total=0;
        for (int y=qMax(0,b.y0);y<qMin(1080,b.y1);y+=2)
            for (int x=qMax(0,b.x0);x<qMin(1920,b.x1);x+=2) {
                changed+=different(x,y);++total;
            }
        if (total && changed*100>total*15) return true;
    }
    if (people) {
        changed=total=0;
        for (int y=480;y<1080;y+=12) for (int x=8;x<1920;x+=12) {
            changed+=different(x,y); ++total;
        }
        if (changed*100>total) return true;
    }
    if (road) for (const auto &side : road->sides) {
        if (side.x0 >= side.x1 || side.y0 >= side.y1) continue;
        changed=total=0;
        for (int y=qMax(0,side.y0);y<=qMin(1079,side.y1);y+=2)
            for (int x=qMax(0,side.x0-8);x<=qMin(1919,side.x1+8);x+=2) {
                changed+=different(x,y);++total;
            }
        if (total && changed*100>total*3) return true;
    }
    return false;
}

extern "C" int vdma_dashboard_test_yolo(yolo_adas *producer,
    const vdma_snapshot_input *input, vdma_snapshot_output *output)
{
    if (!input->pixels || !vdma_frame_valid(input->identity.slot,&input->header) ||
        input->identity.canvas_width != 1920 || input->identity.canvas_height != 1080 ||
        input->identity.source_id != input->identity.layout || input->identity.source_id >= 4 ||
        input->header.sequence != input->identity.frame_sequence || input->header.layout != input->identity.layout ||
        input->header.width != 1920 || input->header.height != 1080 ||
        input->header.stride != 3840 || input->header.bytes != VDMA_SNAPSHOT_FRAME_BYTES) return 0;
    std::shared_ptr<const Dashboard::TestImage> frozen;
    detection_overlay lights{};
    adas_lane_test_observation road{};
    bool people = false;
    {
        std::lock_guard<std::mutex> lock(ui.mutex);
        if (!ui.recognition || !ui.frame_bound || ui.invalidate || !sameTestContext(ui.frame,input->identity))
            return 0;
        if (specialistEnabled() || ui.pedestrian_mode) {
            const uint64_t now=driveNow();
            if (ui.test_capture_ns && now-ui.test_capture_ns<UINT64_C(200000000)) return 0;
            ui.test_capture_ns=now;
            if (ui.signal_test.active) {
                frozen=ui.frozen_test_image;lights=ui.test_detection;road=ui.signal_test.road;
                people = ui.pedestrian_mode;
            }
        }
    }
    if (frozen) {
        if (testPictureChanged(input->pixels,*frozen,lights,&road,people)) {
            std::lock_guard<std::mutex> lock(ui.mutex);
            if (ui.signal_test.active && ui.frozen_test_image==frozen) {
                exitTestLocked();
                std::puts(people ? "PEDESTRIAN_PICTURE_CHANGED test_exited=1 live_pipeline_unchanged=1" :
                    "LIGHT_SPECIALIST_PICTURE_CHANGED test_exited=1 live_pipeline_unchanged=1");
            }
        }
        return people ? processPedestrianTask(producer,input,output) : 0;
    }
    auto owned = std::make_shared<Dashboard::TestImage>();
    owned->identity = input->identity; owned->header = input->header;
    owned->observed = input->observed_ns;
    owned->pixels.assign(input->pixels,input->pixels+VDMA_SNAPSHOT_FRAME_BYTES);
    {
        std::lock_guard<std::mutex> lock(ui.mutex);
        if (ui.recognition && ui.frame_bound && !ui.invalidate && sameTestContext(ui.frame,owned->identity)) {
            ui.latest_test_image = owned;
        }
    }
    // The dedicated worker owns specialist inference. Returning zero preserves
    // the normal YOLO processor, publication, vehicle overlays and live cadence.
    return specialistEnabled() ? 0 : processTestTask(producer,input,output);
}

struct EvidenceFrame {
    vdma_frame_identity identity{};
    vdma_dark_status dark{};
    std::vector<unsigned char> pixels;
    QString prefix;
};
struct EvidenceJob {
    std::shared_ptr<const EvidenceFrame> frame;
    QByteArray road;
    QJsonObject config;
};
/* Diagnostic only: bounded owned copies; all disk I/O belongs to this writer.
 * No leased ROAD pointer or borrowed DMA slot crosses the thread boundary. */
class Evidence {
    QString directory;
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<EvidenceJob> jobs;
    std::deque<std::shared_ptr<const EvidenceFrame>> cache; // owner only
    std::thread worker;
    bool stopping = false;
    uint32_t epoch = 0, config = 0, request = 0;
    uint32_t road_epoch = 0, road_config = 0, road_period = 0, road_phase = 0;
    uint64_t group_started = 0;
    unsigned int group = 0, total = 0;
    std::atomic<unsigned int> errors{0}, written{0}, matched{0}, skipped{0};
    static bool save(const QString &path, const QByteArray &data)
    {
        QSaveFile file(path);
        return file.open(QIODevice::WriteOnly) && file.write(data) == data.size() && file.commit();
    }
    void enqueue(EvidenceJob job)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (jobs.size() >= 8) { skipped++; return; }
        jobs.push_back(std::move(job)); wake.notify_one();
    }
    void run()
    {
        for (;;) {
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait(lock,[&]{ return stopping || !jobs.empty(); });
            if (jobs.empty()) return;
            EvidenceJob job = std::move(jobs.front()); jobs.pop_front(); lock.unlock();
            const auto &f = *job.frame; const auto &id = f.identity; const auto &s = f.dark;
            bool ok;
            if (job.road.isEmpty()) {
                QJsonObject metadata{{"epoch",int(id.stream_epoch)},{"sequence",int(id.frame_sequence)},
                    {"source",int(id.source_id)},{"layout",int(id.layout)},{"config_generation",int(id.config_generation)},
                    {"slot",int(id.slot)},{"token",int(id.token)}};
                QJsonObject dark{{"enabled",bool(s.status & VDMA_DARK_ENABLED)},
                    {"threshold",int(s.active_threshold)},{"strength",int(s.active_strength)},
                    {"generation",int(s.generation)},{"request",int(s.request)},
                    {"applied_epoch",int(s.applied_epoch)},{"applied_sequence",int(s.applied_sequence)}};
                ok = save(f.prefix+".rgb565",QByteArray(reinterpret_cast<const char *>(f.pixels.data()),int(f.pixels.size()))) &&
                    save(f.prefix+".identity.json",QJsonDocument(metadata).toJson()) &&
                    save(f.prefix+".dark.json",QJsonDocument(dark).toJson());
                if (ok) written++;
            } else {
                ok = save(f.prefix+".vft",job.road) && save(f.prefix+".config.json",QJsonDocument(job.config).toJson());
                if (ok) matched++;
            }
            if (!ok) errors++;
        }
    }
public:
    bool start(const QString &path)
    {
        if (path.isEmpty()) return true;
        directory = QDir(path).absolutePath();
        if (!QDir().mkpath(directory)) return false;
        worker = std::thread([this]{ run(); }); return true;
    }
    bool stop()
    {
        if (!worker.joinable()) return true;
        { std::lock_guard<std::mutex> lock(mutex); stopping = true; wake.notify_all(); }
        worker.join();
        std::printf("DASHBOARD_EVIDENCE pixels=%u road_pairs=%u skipped=%u errors=%u limit=32 validation_only=1\n",
            written.load(),matched.load(),skipped.load(),errors.load());
        cache.clear(); return errors == 0;
    }
    void held(int fd, const vdma_frame_identity &id, const vdma_dark_async &dark)
    {
        if (directory.isEmpty() || total >= 32 || !dark.available || dark.phase != DARK_IDLE ||
            !vdma_dark_frame_matches(&dark.actual,&id) || id.source_id > 3) return;
        if (epoch != id.stream_epoch || config != id.config_generation || request != dark.actual.request) {
            epoch = id.stream_epoch; config = id.config_generation; request = dark.actual.request; group = 0;
            group_started = milliseconds();
            cache.clear();
        }
        /* Wait out source CONFIG/font startup. Follow the actual ROAD phase
         * rather than burst-copy unrelated frames before ROAD is ready. */
        if (group >= 8 || milliseconds() - group_started < 1000 || !road_period ||
            road_epoch != id.stream_epoch || road_config != id.config_generation ||
            id.frame_sequence % road_period != road_phase) return;
        auto f = std::make_shared<EvidenceFrame>(); f->pixels.resize(VDMA_SNAPSHOT_FRAME_BYTES);
        vdma_snapshot_copy copy{}; copy.version = 1; copy.size = sizeof(copy);
        copy.destination = reinterpret_cast<uintptr_t>(f->pixels.data()); copy.capacity = VDMA_SNAPSHOT_FRAME_BYTES; copy.expected = id;
        if (ioctl(fd,VDMA_COPY_HELD_FRAME,&copy) || copy.copied_bytes != VDMA_SNAPSHOT_FRAME_BYTES ||
            std::memcmp(&copy.actual,&id,sizeof(id)) || copy.header.sequence != id.frame_sequence ||
            copy.header.layout != id.layout || copy.header.format != 0x36314752U) { errors++; return; }
        f->identity = id; f->dark = dark.actual;
        f->prefix = directory+QString("/e%1_s%2_g%3_r%4").arg(id.stream_epoch).arg(id.frame_sequence).arg(id.config_generation).arg(dark.actual.request);
        cache.push_back(f); if (cache.size() > 16) cache.pop_front();
        enqueue(EvidenceJob{f,QByteArray(),QJsonObject()}); group++; total++;
    }
    void road(const vdma_road_runtime_event &event,const vdma_feature_config &p)
    {
        if (directory.isEmpty() || !event.bytes || event.bytes > VDMA_FEATURE_ROAD_MAX_PACKET_BYTES || event.identity.source_id > 3) return;
        road_epoch = event.identity.stream_epoch; road_config = event.identity.config_generation;
        road_period = p.analysis_period;
        if (!road_period) return;
        road_phase = event.identity.frame_sequence % road_period;
        for (const auto &f : cache) {
            const auto &id = f->identity; const auto &e = event.identity;
            if (id.stream_epoch != e.stream_epoch || id.frame_sequence != e.frame_sequence || id.source_id != e.source_id ||
                id.layout != e.layout || id.config_generation != e.config_generation) continue;
            QJsonArray roi; for (auto x : p.road_roi) roi.append(int(x));
            QJsonObject config{{"epoch",int(id.stream_epoch)},{"sequence",int(id.frame_sequence)},
                {"source",int(id.source_id)},{"layout",int(id.layout)},{"config_generation",int(id.config_generation)},
                {"roi",roi},{"sample_rows",int(p.lane_rows)},{"candidates_per_row",int(p.lane_per_row)},
                {"gaussian_enable",bool(p.denoise_enable)},{"gx_threshold",int(p.gx_threshold)},
                {"gy_threshold",int(p.gy_threshold)},{"gradient_threshold",24},{"white_min_y",int(p.white_min_y)},
                {"white_max_chroma",int(p.white_max_chroma)},{"yellow_min_chroma",int(p.extract_min_chroma)},
                {"min_width",int(p.reserved[0])},{"max_width",int(p.reserved[1])},{"analysis_period",int(p.analysis_period)}};
            enqueue(EvidenceJob{f,QByteArray(reinterpret_cast<const char *>(event.raw),int(event.bytes)),config}); return;
        }
    }
} evidence;

extern "C" void vdma_dashboard_held(int fd,const vdma_frame_identity *identity)
{
    if (ui.exact_presentation) return;
    vdma_dark_async dark;
    { std::lock_guard<std::mutex> lock(ui.mutex); dark = ui.dark; }
    evidence.held(fd,*identity,dark);
}
extern "C" void vdma_dashboard_preview_rgb(const vdma_frame_identity *identity,const void *pixels,uint64_t observedNs)
{ if (!ui.exact_presentation) preview.owned(*identity,pixels,observedNs); }
extern "C" void vdma_dashboard_road(const vdma_road_runtime_event *event,const vdma_feature_config *profile)
{ if (!ui.exact_presentation) evidence.road(*event,*profile); }

extern "C" int vdma_dashboard_init(int drm, uint32_t crtc, int video)
{
    std::lock_guard<std::mutex> lock(ui.mutex);
    ui.drm = drm; ui.video = video; ui.crtc = crtc;
    if (ui.exact_presentation) {
        ui.live = true;
        std::puts("DASHBOARD_INIT mode=fullscreen_exact ui_osd=disabled");
        return 0;
    }
    vdma_ui_buffers_init(&ui.buffers); vdma_dark_async_init(&ui.dark);
    if (vdma_osd_probe_init_blank(&ui.osd, drm, crtc)) return -1;
    const vdma_ui_layout layout = {1,1920,1080,1920,1080,{0,146,1456,819}};
    if (vdma_osd_probe_set_layout(&ui.osd, &layout)) return -1;
    drm_mode_create_dumb create{}; create.width = 1920; create.height = 1080; create.bpp = 32;
    if (drmIoctl(drm, DRM_IOCTL_MODE_CREATE_DUMB, &create)) return -1;
    ui.handle = create.handle; ui.pitch = create.pitch; ui.bytes = create.size;
    if (create.pitch < 7680 || create.size < uint64_t(create.pitch) * 1080) { errno = EPROTO; return -1; }
    drm_mode_map_dumb map{}; map.handle = create.handle;
    if (drmIoctl(drm, DRM_IOCTL_MODE_MAP_DUMB, &map)) return -1;
    ui.pixels = mmap(nullptr, create.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm, map.offset);
    if (ui.pixels == MAP_FAILED) { ui.pixels = nullptr; return -1; }
    std::memset(ui.pixels, 0, create.size);
    uint32_t handles[4] = {create.handle}, pitches[4] = {create.pitch}, offsets[4] = {};
    if (drmModeAddFB2(drm,1920,1080,DRM_FORMAT_ARGB8888,handles,pitches,offsets,&ui.fb,0)) return -1;
    if (ui.clean_presentation) {
        for (auto &buffer : ui.extra) if (createArgb(buffer,drm)) return -1;
        std::printf("PRESENT_OSD_INIT mode=clean slots=4 bytes=%llu independent_geometry=1 target_contract=1\n",
            (unsigned long long)(ui.osd.bytes+ui.bytes+ui.extra[0].bytes+ui.extra[1].bytes));
    }
    ui.live = true;
    std::printf("DASHBOARD_INIT double_argb_bytes=%llu owner_fd=%d layout=1456x819+0+146 startup_command=NONE exact_video=%d\n",
        static_cast<unsigned long long>(ui.bytes + ui.osd.bytes), video,ui.qt_exact);
    if (!recorder.start(drm, crtc)) {
        recordingFailed = true; recorder.stop(); return -1;
    }
    return 0;
}
extern "C" void vdma_dashboard_tick()
{
    if (ui.exact_presentation) return;
    std::lock_guard<std::mutex> lock(ui.mutex);
    const auto before = ui.dark.phase;
    const auto generation = ui.dark.actual.generation;
    const int available = ui.dark.available;
    vdma_dark_async_tick(&ui.dark, ui.video, milliseconds());
    if (generation != ui.dark.actual.generation || (available && !ui.dark.available) ||
        (ui.dark.actual.status & VDMA_DARK_PENDING)) { ui.frame_bound = false; ui.invalidate = true; }
    if (ui.signal_test.active && ui.invalidate) exitTestLocked();
    if (ui.pedestrian_mode) adas_ped_test_tick(&ui.pedestrian_test,driveNow());
    else adas_signal_test_tick(&ui.signal_test,driveNow());
    syncDriveLocked();
    if (before == DARK_QUEUED || (ui.dark.phase != ui.prior_phase)) {
        std::printf("DASHBOARD_DARK phase=%d available=%d error=%d request=%u generation=%u enabled=%u active=%u/%u shadow=%u/%u target_request=%u\n",
            ui.dark.phase, ui.dark.available, ui.dark.error, ui.dark.actual.request, ui.dark.actual.generation,
            ui.dark.actual.status & VDMA_DARK_ENABLED, ui.dark.actual.active_threshold, ui.dark.actual.active_strength,
            ui.dark.actual.shadow_threshold, ui.dark.actual.shadow_strength, ui.dark.target.request);
        std::fflush(stdout); ui.prior_phase = ui.dark.phase;
    }
}
extern "C" int vdma_dashboard_flip(uint32_t video_fb, const vdma_frame_identity *target,
    void *event_data, int ui_only)
{
    if (!target) { errno = EINVAL; return -1; }
    if (ui.exact_presentation)
        return drmModePageFlip(ui.drm,ui.crtc,video_fb,DRM_MODE_PAGE_FLIP_EVENT,event_data) ? -1 : 1;
    std::lock_guard<std::mutex> lock(ui.mutex);
    const int ready = vdma_ui_buffer_newest(&ui.buffers);
    const int selected = ui.clean_presentation ?
        ui.presentation.select(video_fb,*target,driveNow()+UINT64_C(20000000),UINT64_C(20000000)) :
        ready >= 0 ? ready : ui.buffers.front;
    if (selected < 0 || (ui_only && (ui.clean_presentation ?
            selected == ui.presentation.front : ready < 0))) return 0;
    const uint32_t overlay = overlayFramebuffer(selected);
    if (ui.clean_presentation) {
        for (int i = 0; i < 4; ++i) if (!ui.overlay_tested[i]) {
            if (vdma_osd_probe_flip_buffer(&ui.osd,video_fb,overlayFramebuffer(i),nullptr,1))
                return recorder.retryFlip(errno) ? 0 : -1;
            ui.overlay_tested[i] = true;
            std::printf("PRESENT_OSD_TEST slot=%d passed=1 video_fb=%u overlay_fb=%u\n",
                i,video_fb,overlayFramebuffer(i));
        }
    } else if (!ui.osd.layout_tested &&
        vdma_osd_probe_flip_buffer(&ui.osd,video_fb,overlay,nullptr,1))
        return recorder.retryFlip(errno) ? 0 : -1;
    const vdma_osd_writeback *writeback = recorder.prepare();
    const int submitted = vdma_osd_probe_flip_buffer_record(&ui.osd,video_fb,overlay,event_data,writeback);
    const int submitError = errno;
    recorder.submitted(submitted, submitError);
    if (submitted) return recorder.retryFlip(submitError) ? 0 : -1;
    if (ui.clean_presentation ? !ui.presentation.submit(selected) :
        ready >= 0 && vdma_ui_buffer_submitted(&ui.buffers,ready)) { errno = EPROTO; return -1; }
    ui.submitted_road = ui.road_slots[selected];
    ui.submitted_video = *target;
    ui.flip_outstanding = true;
    if (ui_only) ui.ui_commits++;
    return 1;
}
extern "C" void vdma_dashboard_complete()
{
    if (ui.exact_presentation) return;
    std::lock_guard<std::mutex> lock(ui.mutex);
    if (ui.clean_presentation && ui.presentation.pending >= 0) {
        ui.presentation.complete(); renderWake.request();
    } else if (ui.buffers.pending >= 0) {
        vdma_ui_buffer_complete(&ui.buffers);
        renderWake.request();
    }
    if (!ui.flip_outstanding) return;
    ui.flip_outstanding = false;
    const auto &r = ui.submitted_road;
    const auto &v = ui.submitted_video;
    const uint64_t now = driveNow();
    const bool context = r.valid && r.context.stream_epoch == v.stream_epoch &&
        r.context.source_id == v.source_id && r.context.layout == v.layout &&
        r.context.config_generation == v.config_generation;
    if (context && now >= r.until) ui.road_expired_flips++;
    const auto &last = ui.last_presented_road;
    if (!context || !r.observed || r.finished < r.observed ||
        r.received < r.finished || r.render < r.received ||
        r.ready < r.render || now < r.ready || now >= r.until ||
        (last.valid && last.observed == r.observed && last.finished == r.finished)) return;
    ui.last_presented_road = r;
    const uint64_t age = now-r.observed;
    ui.road_presentations++; ui.road_age_ns += age;
    ui.road_age_max_ns = qMax(ui.road_age_max_ns,age);
    ui.road_owner_ns += r.received-r.finished;
    ui.road_wait_ns += r.render-r.received;
    ui.road_draw_ns += r.ready-r.render;
    ui.road_flip_ns += now-r.ready;
    const uint32_t gap = v.frame_sequence-r.sequence;
    if (gap < UINT32_C(0x80000000)) {
        ui.road_frame_gap += gap;
        ui.road_frame_gap_max = qMax(ui.road_frame_gap_max,uint64_t(gap));
    }
}
extern "C" void vdma_dashboard_pause()
{
    if (ui.exact_presentation) return;
    if (!recorder.stop()) recordingFailed = true;
    std::unique_lock<std::mutex> lock(ui.mutex); ui.live = false;
    exitTestLocked();
    ui.drawn.wait(lock, []{ return !ui.drawing; });
}
extern "C" int vdma_dashboard_close(int detached)
{
    syncVideo.close(detached);
    if (ui.exact_presentation) return detached ? 0 : -1;
    if (resultHistory)
        std::printf("PRESENT_HISTORY accepted=%llu rejected=%llu duplicate=%llu busy=%llu "
            "overwritten=%llu resets=%llu allocation_failures=%llu capacity_per_task=32\n",
            (unsigned long long)resultHistory->accepted.load(),
            (unsigned long long)resultHistory->rejected.load(),
            (unsigned long long)resultHistory->duplicates.load(),
            (unsigned long long)resultHistory->busy.load(),
            (unsigned long long)resultHistory->overwritten.load(),
            (unsigned long long)resultHistory->resets.load(),
            (unsigned long long)resultHistory->allocationFailures.load());
    std::lock_guard<std::mutex> lock(ui.mutex);
    if (!detached) return -1; /* Keep scanout alive until DRM fd closes. */
    for (auto &buffer : ui.extra) if (destroyArgb(buffer,ui.drm)) return -1;
    int result = vdma_osd_probe_close(&ui.osd);
    if (result) return result;
    if (ui.fb && drmModeRmFB(ui.drm, ui.fb)) result = -1;
    if (ui.pixels) munmap(ui.pixels, ui.bytes);
    if (ui.handle) {
        drm_mode_destroy_dumb destroy{}; destroy.handle = ui.handle;
        if (drmIoctl(ui.drm, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy)) result = -1;
    }
    ui.fb = ui.handle = 0; ui.pixels = nullptr;
    if (ui.clean_presentation)
        std::printf("PRESENT_OSD_FINAL clean_submits=%llu target_submits=%llu acquire_busy=%llu "
            "cancelled=%llu tested=%u/%u/%u/%u geometry_mode=none\n",
            (unsigned long long)ui.presentation.empty,(unsigned long long)ui.presentation.targets,
            (unsigned long long)ui.presentation.missed,(unsigned long long)ui.presentation.cancelled,
            ui.overlay_tested[0],ui.overlay_tested[1],ui.overlay_tested[2],ui.overlay_tested[3]);
    std::printf("DASHBOARD_SUMMARY renders=%llu ui_only_commits=%llu explicit_dark_requests=%llu\n",
        static_cast<unsigned long long>(ui.revision), static_cast<unsigned long long>(ui.ui_commits),
        static_cast<unsigned long long>(ui.dark_changes));
    std::printf("DASHBOARD_PAINT full=%llu mean_us=%.3f max_us=%.3f\n",
        static_cast<unsigned long long>(ui.full_renders),
        ui.revision ? ui.render_total_ns / 1000.0 / ui.revision : 0,ui.render_max_ns / 1000.0);
    std::printf("DASHBOARD_DAMAGE bytes=%llu paint_mean_us=%.3f copy_mean_us=%.3f\n",
        static_cast<unsigned long long>(ui.argb_copy_bytes),
        ui.revision ? ui.paint_total_ns / 1000.0 / ui.revision : 0,
        ui.revision ? ui.copy_total_ns / 1000.0 / ui.revision : 0);
    std::printf("DASHBOARD_CHROME paints=%llu\n",
        static_cast<unsigned long long>(ui.chrome_renders));
    std::printf("DASHBOARD_WAKE sent=%llu consumed=%llu errors=%llu fallback_ms=16\n",
        static_cast<unsigned long long>(renderWake.sent.load()),
        static_cast<unsigned long long>(renderWake.consumed.load()),
        static_cast<unsigned long long>(renderWake.errors.load()));
    const double renders = ui.revision ? double(ui.revision)*1000. : 1.;
    std::printf("DASHBOARD_STAGES clear_us=%.3f chrome_us=%.3f overlay_us=%.3f damage_us=%.3f\n",
        ui.clear_ns/renders,ui.chrome_ns/renders,ui.overlay_ns/renders,ui.damage_ns/renders);
    const double presentations = ui.road_presentations ? double(ui.road_presentations) : 1.;
    std::printf("DASHBOARD_ROAD_AGE first_presentations=%llu mean_ms=%.3f max_ms=%.3f "
        "owner_ms=%.3f qt_wait_ms=%.3f draw_ms=%.3f ready_flip_ms=%.3f "
        "sequence_gap_mean=%.3f sequence_gap_max=%llu expired_flips=%llu callback_not_physical_screen=1\n",
        static_cast<unsigned long long>(ui.road_presentations),ui.road_age_ns/presentations/1e6,
        ui.road_age_max_ns/1e6,ui.road_owner_ns/presentations/1e6,
        ui.road_wait_ns/presentations/1e6,ui.road_draw_ns/presentations/1e6,
        ui.road_flip_ns/presentations/1e6,ui.road_frame_gap/presentations,
        static_cast<unsigned long long>(ui.road_frame_gap_max),
        static_cast<unsigned long long>(ui.road_expired_flips));
    std::printf("DASHBOARD_ROAD_RENDER visible=%llu geometry=%llu\n",
        static_cast<unsigned long long>(ui.road_renders),
        static_cast<unsigned long long>(ui.road_geometry_renders));
    return result;
}
extern "C" void vdma_dashboard_video(const vdma_frame_identity *identity)
{
    if (resultHistory) resultHistory->setContext(*identity);
    if (ui.exact_presentation) {
        std::lock_guard<std::mutex> lock(ui.mutex);
        ui.frame = *identity;
        ui.frame_bound = identity->source_id == 2 && identity->layout == 2;
        return;
    }
    std::unique_lock<std::mutex> lock(ui.mutex);
    const bool contextChanged = ui.frame.stream_epoch != identity->stream_epoch || ui.frame.source_id != identity->source_id ||
        ui.frame.layout != identity->layout || ui.frame.config_generation != identity->config_generation;
    const bool bound = ui.frame_bound;
    if (contextChanged) {
        ++ui.resident_generation;
        ui.resident_light = {};
        ui.resident_last_frame = 0;
        exitTestLocked();
        ui.invalidate = true;
        ui.detection = {};
        ui.road = {};
        ui.road_revision++;
        if (ui.clean_presentation) ui.presentation.reset();
        if (ui.frame.source_id != identity->source_id || !ui.frame.stream_epoch) {
            std::printf("DASHBOARD_SOURCE_CHANGE from=%u to=%u epoch=%u gui=retained recognition=%s\n",
                ui.frame.source_id,identity->source_id,identity->stream_epoch,
                !ui.qt_exact || identity->source_id == 2 ? "eligible" : "suspended");
            std::fflush(stdout);
        }
    }
    ui.frame = *identity;
    /* Actual metadata, not historical first_capture. No producer can accept
     * this frame until its generation agrees with the confirmed hardware. */
    ui.frame_bound = ui.dark.available && ui.dark.phase == DARK_IDLE &&
        vdma_dark_frame_matches(&ui.dark.actual,identity) &&
        !identity->flags && identity->stream_epoch &&
        identity->source_id <= 4 && identity->layout == identity->source_id;
    if (contextChanged || bound != ui.frame_bound) {
        lock.unlock(); refreshPedestrianAlert(); renderWake.request();
    }
}
extern "C" int vdma_dashboard_history_enabled() { return bool(resultHistory); }
extern "C" void vdma_dashboard_history_road(const road_overlay_geometry *geometry)
{
    if (geometry && geometry->valid) {
        const adas_drive_frame frame = {1,geometry->stream_epoch,geometry->source_id,
            geometry->layout,geometry->config_generation,geometry->frame_sequence};
        rememberTestRoad(frame,geometry->reference,geometry->observed_ns,
            geometry->valid_until_ns,driveNow());
    }
    if (geometry && syncVideo.enabled) syncVideo.road(*geometry);
    if (resultHistory && geometry) resultHistory->road(*geometry);
}
extern "C" void vdma_dashboard_history_detection(const detection_overlay *overlay)
{
    if (overlay && syncVideo.enabled) syncVideo.detection(*overlay);
    if (resultHistory && overlay) resultHistory->detection(*overlay);
}
extern "C" void vdma_dashboard_fps(double fps)
{ if (!ui.exact_presentation) { std::lock_guard<std::mutex> lock(ui.mutex); ui.fps = fps; } }
extern "C" void vdma_dashboard_adas(const road_overlay_geometry *geometry)
{
    if (syncVideo.enabled && geometry) syncVideo.road(*geometry);
    if (ui.exact_presentation) return;
    std::lock_guard<std::mutex> lock(ui.mutex);
    /* A completed road envelope is immutable. The store's global publication
     * sequence also changes for YOLO; it is not a road geometry revision. */
    if (geometry && geometry->valid == ui.road.valid &&
        geometry->stream_epoch == ui.road.stream_epoch && geometry->source_id == ui.road.source_id &&
        geometry->layout == ui.road.layout && geometry->config_generation == ui.road.config_generation &&
        geometry->frame_sequence == ui.road.frame_sequence && geometry->observed_ns == ui.road.observed_ns &&
        geometry->finished_ns == ui.road.finished_ns && geometry->valid_until_ns == ui.road.valid_until_ns) return;
    if (!geometry && !ui.road.valid) return;
    ui.road_received_ns = driveNow();
    ui.road = geometry ? *geometry : road_overlay_geometry{};
    ui.road_revision++;
    renderWake.request();
}
extern "C" void vdma_dashboard_detection(const detection_overlay *overlay)
{
    /* Holding consumes immutable producer publications through history_detection.
     * A polling read can temporarily fail while the producer advances the store. */
    if (syncVideo.enabled && overlay &&
        (!syncVideo.holdYolo || overlay->state == DETECTION_ERROR))
        syncVideo.detection(*overlay);
    if (ui.exact_presentation) return;
    std::lock_guard<std::mutex> lock(ui.mutex);
    const bool changed = !overlay ? ui.detection.valid || ui.detection.state != DETECTION_NOT_LOADED :
        overlay->state != ui.detection.state || overlay->valid != ui.detection.valid ||
        overlay->observed_ns != ui.detection.observed_ns || overlay->frame_sequence != ui.detection.frame_sequence ||
        overlay->stream_epoch != ui.detection.stream_epoch || overlay->source_id != ui.detection.source_id ||
        overlay->layout != ui.detection.layout || overlay->config_generation != ui.detection.config_generation;
    ui.detection = overlay ? *overlay : detection_overlay{};
    if (changed) renderWake.request();
}
extern "C" void vdma_dashboard_decision(const adas_result *result,uint64_t nowNs)
{
    tickPedestrianAlert(std::max(nowNs,driveNow()));
    std::lock_guard<std::mutex> lock(ui.mutex);
    if (ui.pedestrian_mode && result) {
        adas_ped_observation people{};
        adas_ped_from_result(&people,result);
        if (people.frame.valid && people.observed_ns &&
            (ui.test_people.empty() || !adas_lane_test_same_frame(&ui.test_people.back().frame,&people.frame))) {
            ui.test_people.push_back(people);
            if (ui.test_people.size()>32) ui.test_people.pop_front();
        }
    }
    /* The raw publication hook retains exact road frames, including those
     * superseded by a newer store snapshot before this owner tick. */
    bindTestRoadLocked(std::max(nowNs,driveNow()));
    if (ui.pedestrian_mode) adas_ped_test_tick(&ui.pedestrian_test,std::max(nowNs,driveNow()));
    else adas_signal_test_tick(&ui.signal_test,std::max(nowNs,driveNow()));
    syncDriveLocked();
}
extern "C" int vdma_dashboard_recognition()
{ if (ui.exact_presentation) return 1; std::lock_guard<std::mutex> lock(ui.mutex); return ui.recognition; }
extern "C" int vdma_dashboard_road_recognition()
{
    if (ui.exact_presentation) return 1;
    std::lock_guard<std::mutex> lock(ui.mutex);
    return ui.recognition && (ui.masks[2] || ui.pedestrian_alert_enabled || ui.signal_test.active);
}
extern "C" int vdma_dashboard_frame_usable()
{ if (ui.exact_presentation) { std::lock_guard<std::mutex> lock(ui.mutex); return ui.frame_bound; }
  std::lock_guard<std::mutex> lock(ui.mutex);
  return ui.recognition && ui.frame_bound && (!ui.qt_exact || ui.frame.source_id == 2); }
extern "C" int vdma_dashboard_take_invalidation()
{
    if (ui.exact_presentation) return 0;
    std::lock_guard<std::mutex> lock(ui.mutex);
    bool value = ui.invalidate; ui.invalidate = false;
    if (value && syncVideo.enabled) syncVideo.reset();
    if (value && ui.clean_presentation) ui.presentation.reset();
    return value;
}

class View {
    detection_light_tracker lightTracker{};
    QImage background{1920,1080,QImage::Format_ARGB32_Premultiplied};
    QImage chromeLayer{1920,1080,QImage::Format_ARGB32_Premultiplied};
    QImage geometryLayer{1920,1080,QImage::Format_ARGB32_Premultiplied};
    QByteArray backgroundKey, backgroundChromeKey, backgroundControlsKey, lastKey;
    QByteArray geometryKey;
    QByteArray backgroundVideoFpsKey, backgroundYoloFpsKey, backgroundStatusKey, backgroundLampKey;
    QRegion overlayInk;
    QRegion slotDamage[4] = {QRegion(QRect(0,0,1920,1080)),QRegion(QRect(0,0,1920,1080)),
        QRegion(QRect(0,0,1920,1080)),QRegion(QRect(0,0,1920,1080))};
    QRect slotCursor[4];
    QPoint lastCursor{-100,-100};
public:
    enum PaintPass { PaintAll, PaintChrome, PaintOverlay };
    View() { geometryLayer.fill(Qt::transparent); }
    const QImage &cleanLayer() const { return chromeLayer; }
    const QImage &geometry() const { return geometryLayer; }
    bool clearGeometry = [] {
        const char *style = std::getenv("VDMA_OVERLAY_STYLE");
        return style && !std::strcmp(style,"clear");
    }();
    static void noteInk(QPainter &p, const QRectF &bounds, QRegion *ink)
    {
        if (ink) *ink |= p.transform().mapRect(bounds).toAlignedRect().adjusted(-4,-4,4,4)
            .intersected(QRect(0,146,1456,819));
    }
    static void noteBoxInk(QPainter &p, const QRectF &box, QRegion *ink)
    {
        noteInk(p,QRectF(box.topLeft(),box.topRight()),ink);
        noteInk(p,QRectF(box.bottomLeft(),box.bottomRight()),ink);
        noteInk(p,QRectF(box.topLeft(),box.bottomLeft()),ink);
        noteInk(p,QRectF(box.topRight(),box.bottomRight()),ink);
    }
    vdma_ui_number numbers[2]{};
    bool initialized = false, dirty = false;
    int focus = -1;
    QPoint cursor{960,540};
    QString message;
    struct VehicleCaption { uint32_t id; QPoint offset; };
    std::vector<VehicleCaption> vehicleCaptions;
    uint64_t vehicleCaptionRevision = 0;
    const QRect fields[2] = {QRect(1720,418,166,42),QRect(1720,468,166,42)};
    void selectIntentLocked(uint32_t intent)
    {
        if (intent > 3) return;
        if (ui.pedestrian_mode && (intent == ADAS_DRIVE_CHANGE_LEFT || intent == ADAS_DRIVE_CHANGE_RIGHT))
            return;
        const uint64_t clicked = driveNow();
        const int accepted = ui.pedestrian_mode ? adas_ped_test_click(&ui.pedestrian_test,intent,clicked) :
            adas_signal_test_click(&ui.signal_test,intent,clicked);
        if (accepted < 0)
            message = text("尚未进入测试模式");
        else message.clear();
        syncDriveLocked();
        renderWake.request();
        if (ui.pedestrian_mode) {
            std::printf("PEDESTRIAN_CLICK context=%llu frame=%u requested=%u risk=%u recommended=%u simulated=%u permission=%u reason=%u alarm=%u evaluated=%u elapsed_ms=%.3f\n",
                static_cast<unsigned long long>(ui.pedestrian_test.context),
                ui.pedestrian_test.frame.frame_sequence,intent,ui.pedestrian_test.risk,
                ui.pedestrian_test.recommended_action,ui.drive.output.action,ui.drive.output.permission,
                ui.pedestrian_test.reason,ui.drive.output.alarm_active,ui.drive.output.evaluated,
                double(driveNow()-clicked)/1e6);
            std::fflush(stdout); return;
        }
        if (intent == ADAS_DRIVE_CHANGE_LEFT || intent == ADAS_DRIVE_CHANGE_RIGHT) {
            const auto &road = ui.signal_test.road;
            const auto &side = road.sides[intent == ADAS_DRIVE_CHANGE_LEFT ? 0 : 1];
            std::printf("LANE_TEST_CLICK context=%llu frame=%u command=%u line=%u reason=%u permission=%u alarm=%u segments=%u support=%u span=%u x=%d bounds=%d,%d,%d,%d\n",
                static_cast<unsigned long long>(ui.signal_test.context),road.frame.frame_sequence,
                intent,side.line_type,side.reason,ui.drive.output.permission,ui.drive.output.alarm_active,
                side.segment_count,side.support_px,side.span_px,side.x,side.x0,side.y0,side.x1,side.y1);
        }
        const char *names[] = {"stop","change_left","advance","change_right","unknown"};
        std::printf("DASHBOARD_INTENT source=rk state=%s generation=%llu\n", names[intent],
            static_cast<unsigned long long>(ui.intent_generation));
        std::fflush(stdout);
        if (specialistEnabled()) {
            std::printf("LIGHT_SPECIALIST_CLICK context=%llu selection=%llu task=%llu command=%u lane=%u permission=%u reason=%u alarm=%u\n",
                static_cast<unsigned long long>(ui.signal_test.context),
                static_cast<unsigned long long>(ui.signal_test.selection),
                static_cast<unsigned long long>(ui.signal_test.last_task),intent,ui.signal_test.control.ego_lane,
                ui.signal_test.output.permission,ui.signal_test.result_reason,ui.signal_test.output.alarm_active);
            std::fflush(stdout);
        }
    }
    void toggleTestLocked()
    {
        if (ui.signal_test.active) { exitTestLocked(); message.clear(); return; }
        const uint64_t now = driveNow();
        const auto image = ui.latest_test_image;
        if (!image || !ui.recognition || !ui.frame_bound || ui.invalidate ||
            !sameTestContext(image->identity,ui.frame) || now < image->observed ||
            now-image->observed > UINT64_C(500000000)) {
            message = text("无有效画面，请检查输入"); return;
        }
        const auto &id = image->identity;
        const adas_drive_frame frame = {1,id.stream_epoch,id.source_id,id.layout,
            id.config_generation,id.frame_sequence};
        if (adas_signal_test_enter(&ui.signal_test,&frame,now)) return;
        if (ui.pedestrian_mode) {
            const auto path = adas_ped_default_path(frame.config_generation);
            if (adas_ped_test_enter(&ui.pedestrian_test,&frame,&path,now)) {
                exitTestLocked(); message = text("行人测试仅支持前视HDMI"); return;
            }
            for (const auto &people : ui.test_people)
                adas_ped_test_observe(&ui.pedestrian_test,ui.pedestrian_test.context,
                    &people,image->observed,0,now);
            ui.pedestrian_task_started = false;
        }
        bindTestRoadLocked(now);
        ui.frozen_test_image = image;
        ui.test_picture = QImage(image->pixels.data(),1920,1080,3840,QImage::Format_RGB16).copy();
        ui.test_detection = ui.pedestrian_mode && ui.pedestrian_test.received ?
            pedestrianOverlay(ui.pedestrian_test.observation) : detection_overlay{};
        ui.test_observation = {};
        ui.test_batch = {};
        ui.frozen_light_display = {};
        ui.frozen_light_locked = ui.frozen_light_uncertain = false;
        ui.frozen_light_context = ui.frozen_light_signature = 0;
        ui.frozen_light_profile = UINT32_MAX;
        const auto warm = residentLightSnapshotLocked(id,now);
        if (sameLightImage(warm,*image))
            freezeLightResultLocked(warm,ui.resident_observation,ui.resident_profile,true);
        syncDriveLocked(); message.clear(); renderWake.request();
        if (ui.pedestrian_mode) {
            const auto &p=ui.pedestrian_test.path;
            std::printf("PEDESTRIAN_MODE active=1 context=%llu epoch=%u frame=%u source=%u layout=%u config=%u image_ns=%llu path=%u polygon=%d,%d;%d,%d;%d,%d;%d,%d boundary=%d nearby=%d cached=%u\n",
                static_cast<unsigned long long>(ui.pedestrian_test.context),
                id.stream_epoch,id.frame_sequence,id.source_id,id.layout,id.config_generation,
                static_cast<unsigned long long>(image->observed),p.version,
                p.top_left,p.top_y,p.top_right,p.top_y,p.bottom_right,p.bottom_y,
                p.bottom_left,p.bottom_y,p.boundary,p.nearby,ui.pedestrian_test.received);
            if (ui.pedestrian_test.received) logPedestrianRaw(ui.pedestrian_test,true);
        } else std::printf("LIGHT_SPECIALIST_MODE active=1 frame=%u\n",id.frame_sequence);
        std::fflush(stdout);
    }
    void click()
    {
        for (int i = 0; i < 2; i++) if (fields[i].contains(cursor)) { focus = i; numbers[i].replace = 1; return; }
        focus = -1;
        std::unique_lock<std::mutex> lock(ui.mutex);
        if (specialistEnabled()) {
            std::printf("LIGHT_SPECIALIST_INPUT x=%d y=%d active=%u lane=%u\n",
                cursor.x(),cursor.y(),ui.signal_test.active,ui.signal_test.control.ego_lane);
            std::fflush(stdout);
        }
        const uint32_t codes[] = {1,2,0,3};
        for (int i = 0; i < 4; i++) if (QRect(1490+i*101,954,93,60).contains(cursor)) {
            selectIntentLocked(codes[i]); return;
        }
        const uint32_t lanes[] = {1,2,4};
        for (int i = 0; i < 3; i++) if (QRect(128+i*156,974,146,40).contains(cursor)) {
            if (ui.pedestrian_mode) return;
            adas_signal_test_configure(&ui.signal_test,ADAS_SCENE_INTERSECTION,lanes[i],driveNow());
            std::printf("LIGHT_SPECIALIST_LANE context=%llu lane=%u task=%llu batch=%u count=%u readiness=%u cached=1\n",
                static_cast<unsigned long long>(ui.signal_test.context),lanes[i],
                static_cast<unsigned long long>(ui.signal_test.last_task),ui.signal_test.completed_tasks,
                ui.signal_test.stable_count,ui.signal_test.readiness);
            std::fflush(stdout);
            syncDriveLocked(); return;
        }
        if (QRect(990,974,180,40).contains(cursor)) { resetDriveLocked(); return; }
        if (QRect(610,974,176,40).contains(cursor)) {
            ui.pedestrian_alert_enabled = !ui.pedestrian_alert_enabled;
            lock.unlock(); refreshPedestrianAlert(); return;
        }
        if (QRect(1184,974,238,40).contains(cursor)) {
            toggleTestLocked(); return;
        }
        if (QRect(1760,22,126,44).contains(cursor)) {
            ui.recognition = !ui.recognition; ui.invalidate = true;
            ++ui.resident_generation;
            ui.resident_light = {};
            ui.resident_last_frame = 0;
            ++ui.crosswalk_control_revision;
            ++ui.vehicle_control_revision;
            exitTestLocked();
            ui.road = road_overlay_geometry{}; ui.road_revision++;
            std::printf("DASHBOARD_RECOGNITION requested=%d road_requested=%d\n",
                ui.recognition,ui.recognition && ui.masks[2]);
            std::fflush(stdout); lock.unlock(); refreshPedestrianAlert(); return;
        }
        for (int i = 0; i < 3; i++) if (QRect(1760,700+i*56,126,44).contains(cursor)) {
            ui.masks[i] = !ui.masks[i];
            if (i == 1) ++ui.vehicle_control_revision;
            if (i == 2) {
                ++ui.crosswalk_control_revision;
                ui.road_revision++;
                std::printf("DASHBOARD_ROAD visible=%d backend_needed=%d lanes=1 zebra=1\n",
                    ui.masks[2],ui.recognition && (ui.masks[2] || ui.pedestrian_alert_enabled || ui.signal_test.active));
            } else {
                std::printf("DASHBOARD_OSD mask=%d visible=%d backend_unchanged=1\n",i,ui.masks[i]);
            }
            std::fflush(stdout); return;
        }
        if (QRect(1696,520,190,46).contains(cursor)) {
            vdma_ui_number_set(&numbers[0],64); vdma_ui_number_set(&numbers[1],128); dirty = true;
            message = text("默认值已填入，尚未提交"); return;
        }
        unsigned int threshold = 0, strength = 0, opcode = 0;
        if (QRect(1490,520,190,46).contains(cursor)) {
            if (vdma_ui_number_value(&numbers[0],&threshold) || vdma_ui_number_value(&numbers[1],&strength)) {
                message = text("请输入0～255整数"); return;
            }
            opcode = ui.dark.actual.status & VDMA_DARK_ENABLED ? VDMA_DARK_APPLY : VDMA_DARK_STAGE;
        } else if (QRect(1760,368,126,42).contains(cursor)) {
            if (ui.dark.actual.status & VDMA_DARK_ENABLED) opcode = VDMA_DARK_DISABLE;
            else {
                opcode = VDMA_DARK_APPLY;
                threshold = ui.dark.actual.shadow_threshold; strength = ui.dark.actual.shadow_strength;
            }
        } else return;
        const int result = vdma_dark_async_queue(&ui.dark,opcode,threshold,strength);
        if (result) message = text("尚未就绪或命令处理中");
        else {
            if (opcode != VDMA_DARK_STAGE) {
                ui.frame_bound = false; ui.invalidate = true; exitTestLocked();
            }
            if (opcode == VDMA_DARK_STAGE || (opcode == VDMA_DARK_APPLY &&
                threshold == unsignedValue(0) && strength == unsignedValue(1))) dirty = false;
            ui.dark_changes++; message.clear();
            std::printf("DASHBOARD_USER_COMMAND opcode=%u threshold=%u strength=%u\n",opcode,threshold,strength);
        }
        std::fflush(stdout);
    }
    unsigned int unsignedValue(int i) const
    { unsigned int v = 999; vdma_ui_number_value(&numbers[i],&v); return v; }
    void key(unsigned int code)
    {
        if (code == KEY_TAB) { focus = (focus + 1) % 2; numbers[focus].replace = 1; return; }
        if (code == KEY_ESC) { focus = -1; return; }
        if (focus < 0) {
            std::lock_guard<std::mutex> lock(ui.mutex);
            if (code == KEY_DOWN) selectIntentLocked(0);
            else if (code == KEY_LEFT) selectIntentLocked(1);
            else if (code == KEY_UP) selectIntentLocked(2);
            else if (code == KEY_RIGHT) selectIntentLocked(3);
            return;
        }
        if (code >= KEY_1 && code <= KEY_9) vdma_ui_number_digit(&numbers[focus],code - KEY_1 + 1);
        else if (code == KEY_0) vdma_ui_number_digit(&numbers[focus],0);
        else if (code == KEY_BACKSPACE || code == KEY_DELETE) vdma_ui_number_backspace(&numbers[focus]);
        else if (code == KEY_ENTER || code == KEY_KPENTER) { focus = -1; return; }
        else return;
        dirty = true; message.clear();
    }
    static uint32_t directionColor(const adas_signal_test *testing,
                                  const adas_signal_test_observation *observation, unsigned direction)
    {
        if (!testing || !testing->active || !observation || observation->groups_valid != 1 ||
            observation->reason || !observation->selection || !observation->sequence ||
            observation->context != testing->context || observation->selection > testing->selection ||
            observation->perception != testing->perception ||
            !adas_lane_test_same_frame(&observation->frame,&testing->frame) ||
            !testing->layout_signature || observation->group_signature != testing->layout_signature ||
            observation->group_count != testing->group_count ||
            (observation->group_count != 2 && observation->group_count != 3)) return 0;
        uint32_t color = 0;
        const unsigned end = direction ? observation->group_count : 1;
        for (unsigned i = direction; i < end; ++i) {
            const auto &g = observation->groups[i];
            if (!g.identity_valid || g.id != testing->group_ids[i]) return 0;
            if (!g.color_valid || g.color < DETECTION_LIGHT_RED || g.color > DETECTION_LIGHT_GREEN) continue;
            if (color && color != g.color) return 0;
            color = g.color;
        }
        return color;
    }
    static uint32_t previewDirectionColor(const detection_overlay *detection,unsigned direction)
    {
#ifdef VDMA_HAVE_YOLO
        if (!detection || !detection->valid) return 0;
        const auto *profile=light_specialist_profile_at(specialistProfile);
        std::vector<const detection_overlay_box *> lamps;
        for (uint32_t i=0;i<detection->count && i<DETECTION_OVERLAY_CAPACITY;++i) {
            const auto &b=detection->boxes[i];
            const int w=b.x1-b.x0,h=b.y1-b.y0;
            if (b.signal && !b.display_held && h>=7 && w>=18 && w>=2*h && w<=6*h &&
                b.x0>=profile->scope[0] && b.y0>=profile->scope[1] &&
                b.x1<=profile->scope[2] && b.y1<=profile->scope[3]) lamps.push_back(&b);
        }
        if (lamps.size()!=profile->layout && !(profile->layout==3 && lamps.size()==2)) return 0;
        std::sort(lamps.begin(),lamps.end(),[](const detection_overlay_box *a,const detection_overlay_box *b){
            return a->x0<b->x0;
        });
        int minY=1080,maxY=0,minH=1080,maxH=0,minW=1920,maxW=0;
        for (unsigned i=0;i<lamps.size();++i) {
            const auto &b=*lamps[i];
            if (i && lamps[i-1]->x1>=b.x0) return 0;
            minY=qMin(minY,(b.y0+b.y1)/2);maxY=qMax(maxY,(b.y0+b.y1)/2);
            minH=qMin(minH,b.y1-b.y0);maxH=qMax(maxH,b.y1-b.y0);
            minW=qMin(minW,b.x1-b.x0);maxW=qMax(maxW,b.x1-b.x0);
        }
        if (maxY-minY>maxH || maxW>minW*2 || maxH>minH*2) return 0;
        uint32_t color=0;
        for (unsigned i=direction;i<(direction ? lamps.size() : 1U);++i) {
            const uint32_t next=lamps[i]->light_color;
            if (next<DETECTION_LIGHT_RED || next>DETECTION_LIGHT_GREEN) continue;
            if (color && next!=color) return 0;
            color=next;
        }
        return color;
#else
        (void)detection;
        (void)direction;
        return 0;
#endif
    }
    void paint(QImage &image, const vdma_dark_async &dark, double fps,
               bool recognition, const bool *masks, uint32_t intent, bool intentValid, bool bound, int source = -1,
               bool pointer = true, const road_overlay_geometry *road = nullptr,
               const detection_overlay *detection = nullptr,
               const adas_decision *driving = nullptr, PaintPass pass = PaintAll,
               const QRegion *damage = nullptr, QRegion *drawnInk = nullptr,
               bool videoGeometry = true, bool nativeOverlay = false,
               const adas_signal_test *testing = nullptr, const QImage *testPicture = nullptr,
               const adas_signal_test_observation *observation = nullptr,
               const detection_overlay *liveMetrics = nullptr, bool pedestrian = false,
               const ped_alert_output *alert = nullptr,
               const detection_overlay *liveLights = nullptr,
               const DisplayedCrosswalk *crosswalk = nullptr,
               const DisplayedVehicles *vehicles = nullptr)
    {
        if (!initialized && dark.available && !dirty) {
            vdma_ui_number_set(&numbers[0],dark.actual.shadow_threshold);
            vdma_ui_number_set(&numbers[1],dark.actual.shadow_strength); initialized = true;
        }
        if (!initialized && numbers[0].text[0] == 0) {
            vdma_ui_number_set(&numbers[0],64); vdma_ui_number_set(&numbers[1],128);
        }
        QPainter p(&image); p.setRenderHint(QPainter::Antialiasing);
        if (nativeOverlay) {
            p.scale(1920.0/1456.0,1080.0/819.0);
            p.translate(0,-146);
        }
        if (damage) p.setClipRegion(*damage);
        if (pass != PaintOverlay) {
            p.setCompositionMode(QPainter::CompositionMode_Source);
            p.fillRect(image.rect(),QColor("#172a35"));
        }
        p.setCompositionMode(QPainter::CompositionMode_Source);
        if (pass != PaintOverlay) p.fillRect(QRect(0,146,1456,819),Qt::transparent);
        p.setCompositionMode(QPainter::CompositionMode_SourceOver);
        if (pass != PaintChrome && testing && testing->active) {
            const auto &behavior = driving ? driving->output : testing->output;
            if (!specialistEnabled() && testPicture && !testPicture->isNull()) {
                noteInk(p,QRectF(0,146,1456,819),drawnInk);
                p.drawImage(QRect(0,146,1456,819),*testPicture);
            }
            if (behavior.alarm_active) {
                p.setPen(QPen(QColor("#ff243c"),8)); p.setBrush(Qt::NoBrush);
                p.drawRect(QRect(4,150,1448,811));
                noteBoxInk(p,QRectF(4,150,1448,811),drawnInk);
            } else if (behavior.permission == ADAS_DRIVE_ALLOW) {
                p.save(); p.translate(728,820);
                if (behavior.path_direction == 1) p.rotate(-90);
                else if (behavior.path_direction == 4) p.rotate(90);
                QPolygon arrow;
                arrow << QPoint(0,-75) << QPoint(55,-15) << QPoint(20,-15) << QPoint(20,65)
                    << QPoint(-20,65) << QPoint(-20,-15) << QPoint(-55,-15);
                p.setPen(QPen(Qt::black,3)); p.setBrush(QColor("#35e5a3")); p.drawPolygon(arrow);
                noteInk(p,arrow.boundingRect(),drawnInk); p.restore();
            }
        }
        if (videoGeometry && pass != PaintChrome && road && road->valid && recognition && bound && masks[2]) {
            p.save(); p.setClipRect(QRect(0,146,1456,819));
            p.translate(0,146); p.scale(1456.0/1920.0,819.0/1080.0);
            const QRgb colors[] = {ROAD_OVERLAY_LEFT_COLOR, ROAD_OVERLAY_RIGHT_COLOR};
            for (unsigned int s = 0; s < 2; s++) {
                p.setPen(QPen(QColor::fromRgba(colors[s]),3,Qt::SolidLine,Qt::RoundCap));
                const auto &lane = road->lanes[s];
                QRect laneBounds;
                auto valid = [](const road_overlay_point &pt) { return pt.x >= 0 && pt.x < 1920 && pt.y >= 0 && pt.y < 1080; };
                for (uint32_t i = 0; i < qMin(lane.count,uint32_t(ROAD_OVERLAY_MAX_POINTS)); i++) {
                    const auto &b = lane.points[i]; if (!valid(b)) continue;
                    laneBounds |= QRect(QPoint(b.x,b.y),QSize(1,1));
                    p.drawPoint(QPoint(b.x,b.y));
                    if (!i) continue;
                    const auto &a = lane.points[i-1];
                    if (!valid(a)) continue;
                    const bool measured = (lane.measured_connection[i/32U] & (uint32_t(1) << (i%32U))) != 0;
                    const int dy = b.y-a.y;
                    if (valid(a) && dy > 0 && dy <= (measured ? 18 : 3) &&
                        std::abs(b.x-a.x) <= (measured ? 3*dy+3 : 12))
                        p.drawLine(a.x,a.y,b.x,b.y);
                }
                if (!laneBounds.isEmpty()) noteInk(p,laneBounds,drawnInk);
            }
            p.setBrush(Qt::NoBrush); p.setPen(QPen(QColor::fromRgba(0xFF18E018U),3));
            const uint32_t zebraCount = crosswalk ? unsigned(crosswalk->visible) :
                qMin(road->zebra_count,uint32_t(ROAD_OVERLAY_MAX_ZEBRAS));
            for (uint32_t i = 0; i < zebraCount; i++) {
                const auto &b = crosswalk ? crosswalk->box : road->zebras[i];
                if (b.x0 >= b.x1 || b.y0 >= b.y1) continue;
                const int x0 = qBound(0,b.x0,1920), y0 = qBound(0,b.y0,1080);
                const int x1 = qBound(0,b.x1,1920), y1 = qBound(0,b.y1,1080);
                if (x1 > x0 && y1 > y0) {
                    const QRect box(x0,y0,x1-x0,y1-y0);
                    p.drawRect(box); noteBoxInk(p,box,drawnInk);
                }
            }
            const auto &ref = road->reference;
            if (ref.present) {
                for (uint32_t i = 0; i < qMin(ref.confirmed_count, uint32_t(ROAD_REF_MAX_SEGMENTS)); ++i) {
                    const auto &s = ref.confirmed[i];
                    if (s.point_count < 3 || s.first_point > ref.point_count ||
                        s.point_count > ref.point_count - s.first_point) continue;
                    QPolygon polygon;
                    for (uint32_t j = 0; j < s.point_count; ++j) {
                        const auto &pt = ref.points[s.first_point + j];
                        polygon << QPoint(pt.x, pt.y);
                    }
                    const QColor color(s.line_type == ROAD_REF_SOLID ? "#ffbd45" :
                        s.line_type == ROAD_REF_DASHED ? "#35e5a3" : "#acb8bf");
                    const QPolygonF displayed = clearGeometry ? displayLaneQuad(QPolygonF(polygon)) : QPolygonF(polygon);
                    const double scale = nativeOverlay ? 1920.0/1456.0 : 1.0;
                    if (clearGeometry) {
                        QPen shadow(QColor("#10202a"),5*scale); shadow.setCosmetic(true);
                        shadow.setJoinStyle(Qt::RoundJoin);
                        p.setPen(shadow); p.setBrush(Qt::NoBrush); p.drawPolygon(displayed);
                    }
                    QPen pen(color,(clearGeometry ? 3.0 : 2.0)*scale); pen.setCosmetic(true);
                    if (clearGeometry) pen.setJoinStyle(Qt::RoundJoin);
                    p.setPen(pen); p.setBrush(Qt::NoBrush);
                    if (clearGeometry) p.drawPolygon(displayed); else p.drawPolygon(polygon);
                    for (int j = 0; j < displayed.size(); ++j)
                        noteInk(p,QRectF(displayed[j],displayed[(j+1)%displayed.size()]).normalized(),drawnInk);
                }
                if (ref.cross_detected) {
                    const QColor color(ref.cross_stable ? "#18e018" : "#e5c84a");
                    p.setBrush(Qt::NoBrush);
                    if (clearGeometry) {
                        QPen shadow(QColor("#10202a"),nativeOverlay ? 5.0*1920.0/1456.0 : 5.0,Qt::DashLine);
                        shadow.setCosmetic(true); p.setPen(shadow);
                        p.drawLine(0, ref.cross_y_begin, 1919, ref.cross_y_begin);
                        p.drawLine(0, ref.cross_y_end, 1919, ref.cross_y_end);
                    }
                    QPen crossPen(color,clearGeometry && nativeOverlay ? 3.0*1920.0/1456.0 : 3.0,Qt::DashLine);
                    if (clearGeometry) crossPen.setCosmetic(true);
                    p.setPen(crossPen);
                    p.drawLine(0, ref.cross_y_begin, 1919, ref.cross_y_begin);
                    p.drawLine(0, ref.cross_y_end, 1919, ref.cross_y_end);
                    noteInk(p,QRectF(0,ref.cross_y_begin,1919,0),drawnInk);
                    noteInk(p,QRectF(0,ref.cross_y_end,1919,0),drawnInk);
                }
            }
            p.restore();
        }
        if (videoGeometry && pass != PaintChrome && detection && detection->valid && recognition && bound) {
            p.save(); p.setClipRect(QRect(0,146,1456,819));
            std::vector<QRect> captions;
            if (!vehicles || vehicleCaptionRevision != vehicles->revision) vehicleCaptions.clear();
            if (vehicles) {
                vehicleCaptionRevision = vehicles->revision;
                vehicleCaptions.erase(std::remove_if(vehicleCaptions.begin(),vehicleCaptions.end(),
                    [&](const VehicleCaption &entry) {
                        return std::find(vehicles->ids.begin(),vehicles->ids.end(),entry.id) == vehicles->ids.end();
                    }),vehicleCaptions.end());
            }
            for (uint32_t i = 0; i < qMin(detection->count,uint32_t(DETECTION_OVERLAY_CAPACITY)); ++i) {
                const auto &b = detection->boxes[i];
                if (!(b.signal ? masks[0] : masks[1]) || b.x0 >= b.x1 || b.y0 >= b.y1) continue;
                const QRectF box(qBound(0,b.x0,1920)*1456.0/1920.0,146+qBound(0,b.y0,1080)*819.0/1080.0,
                    (qBound(0,b.x1,1920)-qBound(0,b.x0,1920))*1456.0/1920.0,
                    (qBound(0,b.y1,1080)-qBound(0,b.y0,1080))*819.0/1080.0);
                if (box.width() <= 0 || box.height() <= 0) continue;
                const QColor color(b.signal ?
                    b.light_color == DETECTION_LIGHT_RED ? "#ec354b" :
                    b.light_color == DETECTION_LIGHT_YELLOW ? "#f3c63e" :
                    b.light_color == DETECTION_LIGHT_GREEN ? "#18b878" : "#acb8bf" :
                    b.model_class == 0 ? "#35e5a3" : "#45c5ff");
                const Qt::PenStyle style = b.display_held ? Qt::DashLine : Qt::SolidLine;
                if (clearGeometry) {
                    QPen shadow(QColor("#10202a"),nativeOverlay ? 5.0*1920.0/1456.0 : 5.0,style);
                    shadow.setCosmetic(true); p.setPen(shadow); p.setBrush(Qt::NoBrush); p.drawRect(box);
                }
                QPen boxPen(color,clearGeometry ? (nativeOverlay ? 3.0*1920.0/1456.0 : 3.0) : 2.0,style);
                if (clearGeometry) boxPen.setCosmetic(true);
                p.setPen(boxPen);
                p.setBrush(Qt::NoBrush); p.drawRect(box);
                noteBoxInk(p,box,drawnInk);
                const char *name = "目标";
                switch (b.model_class) {
                case 0: name="行人"; break; case 1: name="自行车"; break;
                case 2: case 5: case 7: name="汽车"; break;
                case 3: name="摩托车"; break;
                case 9:
                    name = lightCaption(b);
                    break;
                case 11: name="停止标志"; break;
                }
                QString caption = text(name);
                const QFont font = [] { QFont f; f.setPixelSize(17); return f; }(); p.setFont(font);
                const int width = qMin(320,p.fontMetrics().horizontalAdvance(
                    b.signal ? text("暂未检出") : caption)+12);
                const int x = qBound(0,int(box.x()),1456-width);
                const int y = qBound(146,int(box.y())-25,940);
                QRect captionRect;
                const uint32_t vehicleId = vehicles ? vehicles->ids[i] : 0;
                auto available = [&](const QRect &candidate) {
                    if (candidate.left() < 0 || candidate.right() > 1455 ||
                        candidate.top() < 146 || candidate.bottom() > 964) return false;
                    for (const auto &placed : captions)
                        if (candidate.adjusted(-2,-1,2,1).intersects(placed)) return false;
                    return true;
                };
                const auto priorCaption = std::find_if(vehicleCaptions.begin(),vehicleCaptions.end(),
                    [&](const VehicleCaption &entry){ return entry.id == vehicleId; });
                if (vehicleId && priorCaption != vehicleCaptions.end()) {
                    const QRect candidate(QPoint(x,y)+priorCaption->offset,QSize(width,25));
                    if (available(candidate)) captionRect = candidate;
                }
                for (int column = 0; column < 3 && captionRect.isEmpty(); ++column) {
                    const int candidateX = qBound(0,x+(column == 1 ? -width-4 : column == 2 ? width+4 : 0),1456-width);
                    for (int offset = 0; offset < 64; ++offset) {
                        const int candidateY = y+(offset % 2 ? -1 : 1)*((offset+1)/2)*26;
                        const QRect candidate(candidateX,candidateY,width,25);
                        if (available(candidate)) { captionRect = candidate; break; }
                    }
                }
                if (captionRect.isEmpty()) continue;
                if (vehicleId) {
                    if (priorCaption != vehicleCaptions.end()) priorCaption->offset = captionRect.topLeft()-QPoint(x,y);
                    else vehicleCaptions.push_back({vehicleId,captionRect.topLeft()-QPoint(x,y)});
                }
                captions.push_back(captionRect);
                p.fillRect(captionRect,QColor(12,26,34,220));
                p.setPen(color);
                p.drawText(captionRect.adjusted(6,0,-6,0),Qt::AlignVCenter,caption);
                noteInk(p,captionRect,drawnInk);
            }
            p.restore();
        }
        if (pass == PaintOverlay) return;
        p.setRenderHint(QPainter::Antialiasing,false);
        p.fillRect(0,0,1920,88,Qt::white); p.fillRect(1456,88,464,992,Qt::white);
        p.fillRect(0,1024,1456,56,QColor("#203744"));
        p.setRenderHint(QPainter::Antialiasing,true);
        const char *sourceNames[] = {"CMOS1","CMOS2","HDMI","Ethernet","四分屏"};
        label(p,QRect(34,95,170,42),source >= 0 && source <= 4 ?
            text(sourceNames[source]) : text("未取图"),20,QColor("#d4e4ed"));
        const auto *behavior = driving ? &driving->output : testing ? &testing->output : nullptr;
        const QRect behaviorBanner((1456-610)/2,95,610,42);
        if (testing && testing->active && behavior && behavior->alarm_active)
            label(p,behaviorBanner,text(behavior->alarm_active & ADAS_DRIVE_PEDESTRIAN ?
                "前方行人，请停车让行" : behavior->alarm_active & ADAS_DRIVE_SOLID ?
                "请保持车道" : "红灯禁止行驶"),26,QColor("#ff243c"),Qt::AlignCenter);
        else if (pedestrian && behavior && behavior->evaluated &&
            (behavior->reasons & (ADAS_DRIVE_PEDESTRIAN | ADAS_DRIVE_PEDESTRIAN_NEARBY)))
            label(p,behaviorBanner,text(behavior->reasons & ADAS_DRIVE_PEDESTRIAN ?
                "请保持停止，等待行人" : "注意路边行人"),26,QColor("#f3c63e"),Qt::AlignCenter);
        label(p,QRect(34,0,170,88),text("道路感知"),28);
        label(p,QRect(245,0,70,88),text("视频"),23,muted);
        label(p,QRect(325,0,180,88),fps > 0 ? QString("%1FPS").arg(fps,0,'f',1) : text("—"),26);
        const auto *metrics = liveMetrics ? liveMetrics : detection;
        const bool yoloLoaded = metrics && metrics->state != DETECTION_NOT_LOADED;
        label(p,QRect(550,0,90,88),"YOLO",23,muted);
        label(p,QRect(650,0,170,88),yoloLoaded ?
            QString("%1FPS").arg(metrics->fps,0,'f',1) : text("未加载"),26);
        label(p,QRect(830,0,150,88),yoloLoaded ?
            QString("%1ms").arg(metrics->latency_ms,0,'f',0) : QString(),23,muted);
        const int sx = 1490, sw = 396;
        auto divider = [&](int y){ p.setPen(QPen(line,1)); p.drawLine(sx,y,sx+sw,y); };
        label(p,QRect(sx,0,250,88),text("识别开关"),23);
        button(p,QRect(1760,22,126,44),text(recognition ? "● 开" : "● 关"),recognition);
        label(p,QRect(sx,108,sw,32),text("交通灯颜色"),22);
        const auto *lampDetection = liveLights ? liveLights : detection;
        const bool lightsVisible = lampDetection && lampDetection->valid && recognition && bound && masks[0];
        for (unsigned i = 0; i < 2; ++i) {
            const QRect card(sx+int(i)*206,156,190,176); panel(p,card);
            const uint32_t color = !lightsVisible ? 0 : !liveLights && testing && testing->active ?
                directionColor(testing,observation,i) : previewDirectionColor(lampDetection,i);
            const QColor lamp = color == DETECTION_LIGHT_RED ? QColor("#ec354b") :
                color == DETECTION_LIGHT_YELLOW ? QColor("#bd8700") :
                color == DETECTION_LIGHT_GREEN ? QColor("#16865b") : QColor("#acb8bf");
            p.save(); p.translate(card.center().x(),220);
            QPolygon arrow;
            if (i == 0)
                arrow << QPoint(-31,-3) << QPoint(-6,-24) << QPoint(-6,-12)
                    << QPoint(18,-12) << QPoint(24,-6) << QPoint(24,28)
                    << QPoint(10,28) << QPoint(10,2) << QPoint(-6,2) << QPoint(-6,15);
            else
                arrow << QPoint(0,-28) << QPoint(25,-3) << QPoint(8,-3)
                    << QPoint(8,28) << QPoint(-8,28) << QPoint(-8,-3) << QPoint(-25,-3);
            p.setPen(Qt::NoPen); p.setBrush(lamp); p.drawPolygon(arrow); p.restore();
            label(p,QRect(card.x(),284,card.width(),30),text(i ? "直行灯" : "左转灯"),20,ink,Qt::AlignCenter);
        }
        divider(360); label(p,QRect(sx,368,250,42),text("暗光增强"),22);
        const bool enabled = dark.available && (dark.actual.status & VDMA_DARK_ENABLED);
        const bool busy = !dark.available || dark.phase != DARK_IDLE || (dark.actual.status & VDMA_DARK_PENDING);
        button(p,QRect(1760,368,126,42),dark.available ? text(enabled ? "● 开" : "● 关") : text("未知"),enabled,busy);
        label(p,QRect(sx,418,225,42),text("暗部阈值"),20);
        label(p,QRect(sx,468,225,42),text("增强强度"),20);
        for (int i = 0; i < 2; i++) {
            panel(p,fields[i]);
            if (focus == i) { p.setPen(QPen(accent,2)); p.setBrush(Qt::NoBrush); p.drawRoundedRect(fields[i],10,10); }
            label(p,fields[i].adjusted(20,0,-8,0),QString::fromLatin1(numbers[i].text)+(focus == i ? "|" : ""),20);
        }
        button(p,QRect(sx,520,190,46),text("应用参数"),!busy,busy);
        button(p,QRect(sx+206,520,190,46),text("恢复默认"),false);
        QString status;
        if (!dark.available) status = text(dark.error == -EAGAIN ? "实际状态：状态同步中" : "实际状态：未连接");
        else if (dark.phase == DARK_TIMEOUT) status = text("实际状态：超时，仍待确认");
        else if (dark.phase == DARK_FAULT) status = text("实际状态：失败，请核查");
        else if (busy) status = text("实际状态：应用中");
        else if (enabled && !bound) status = text("实际状态：已生效，等待新帧");
        else status = text(enabled ? "实际状态：已生效" : "实际状态：关闭");
        label(p,QRect(sx,578,sw,28),message.isEmpty() ? status : message,19,muted);
        const QString actual = dark.available ? QString::fromUtf8("生效：%1/%2  待用：%3/%4%5")
            .arg(dark.actual.active_threshold).arg(dark.actual.active_strength)
            .arg(dark.actual.shadow_threshold).arg(dark.actual.shadow_strength).arg(dirty ? " *" : "") : text("生效值：—");
        label(p,QRect(sx,610,sw,26),actual,18,muted);
        divider(646); label(p,QRect(sx,654,sw,32),text("OSD"),24);
        const char *maskNames[] = {"交通灯","车辆与行人","车道线与斑马线"};
        for (int i = 0; i < 3; i++) {
            label(p,QRect(sx,700+i*56,244,44),text(maskNames[i]),21);
            button(p,QRect(1760,700+i*56,126,44),text(masks[i] ? "● 开" : "● 关"),masks[i]);
        }
        const bool alertEnabled = alert && alert->enabled;
        const uint32_t alertLevel = alert ? alert->level : uint32_t(PED_ALERT_NONE);
        const QColor alertColor = alertLevel == PED_ALERT_YIELD ? QColor("#ff5266") :
            alertLevel == PED_ALERT_CAUTION ? QColor("#f3c63e") : QColor("#d4e4ed");
        QString alertText = text("已关闭");
        if (alertEnabled) {
            if (alert->status == PED_ALERT_RECOGNITION_OFF) alertText = text("等待识别开启");
            else if (alert->status == PED_ALERT_UNSUPPORTED) alertText = text("当前输入暂不可用");
            else if (alert->status == PED_ALERT_WAITING) alertText = text("等待新检测结果");
            else if (alert->status == PED_ALERT_UNAVAILABLE) alertText = text("当前无行人提示");
            else if (alertLevel == PED_ALERT_YIELD) alertText = text("前方行人，请停车让行");
            else if (alertLevel == PED_ALERT_CAUTION) alertText = text("请注意行人");
            else alertText = text("当前无行人提示");
        }
        label(p,QRect(850,95,572,42),alertText,22,alertColor,Qt::AlignRight | Qt::AlignVCenter);
        label(p,QRect(sx,902,sw,36),text("模拟驾驶输入"),24,muted);
        const char *intents[] = {"左变道","行驶","停止","右变道"};
        const uint32_t codes[] = {1,2,0,3};
        for (int i = 0; i < 4; i++) button(p,QRect(sx+i*101,954,93,60),text(intents[i]),
            intentValid && intent == codes[i],pedestrian && (i == 0 || i == 3));
        const uint32_t lane = driving ? driving->selected.ego_lane : 0;
        label(p,QRect(32,974,92,40),text("所在车道"),19,QColor("#d4e4ed"));
        const char *laneNames[] = {"左转车道","直行车道","右转车道"};
        for (int i = 0; i < 3; i++) button(p,QRect(128+i*156,974,146,40),text(laneNames[i]),
            lane == (1U << i),pedestrian);
        button(p,QRect(610,974,176,40),text(alertEnabled ? "行人预警 开" : "行人预警 关"),alertEnabled);
        button(p,QRect(990,974,180,40),text("清空测试"),false);
        button(p,QRect(1184,974,238,40),text(testing && testing->active ? "退出测试模式" : "进入测试模式"),
            testing && testing->active);
        QString judgment;
        QColor judgmentColor("#d4e4ed");
        if (driving) {
            const auto &d = driving->output;
            if (driving->selected.command != ADAS_DRIVE_UNKNOWN && d.evaluated &&
                d.control_generation == driving->selected.generation) {
                if (pedestrian && d.required_tasks == ADAS_DECISION_HAZARD) {
                    if (d.reasons & ADAS_DRIVE_USER_STOP)
                        judgment = text(d.reasons & ADAS_DRIVE_PEDESTRIAN ? "请保持停止，等待行人" :
                            d.reasons & ADAS_DRIVE_HAZARD_UNKNOWN ? "模拟停止 · 行人信息不足" :
                            d.reasons & ADAS_DRIVE_PEDESTRIAN_NEARBY ? "模拟停止 · 注意路边行人" : "模拟停止");
                    else if (d.reasons & ADAS_DRIVE_PEDESTRIAN)
                        judgment = text("前方行人，请停车让行");
                    else if (d.permission == ADAS_DRIVE_INSUFFICIENT)
                        judgment = text("行人信息不足");
                    else judgment = text(d.reasons & ADAS_DRIVE_PEDESTRIAN_NEARBY ?
                        "模拟行驶 · 注意路边行人" : "模拟行驶 · 当前区域未检出行人冲突");
                } else if (d.reasons & ADAS_DRIVE_USER_STOP)
                    judgment = text("模拟停止");
                else if (d.permission == ADAS_DRIVE_ALLOW)
                    judgment = text(d.action == ADAS_ACTION_CHANGE_LEFT ? "允许左变道" :
                        d.action == ADAS_ACTION_CHANGE_RIGHT ? "允许右变道" :
                        "允许行驶");
                else if (d.reasons & ADAS_DRIVE_RED) judgment = text("红灯禁止行驶");
                else if (d.reasons & ADAS_DRIVE_SOLID) judgment = text("请保持车道");
                else if (d.required_tasks == ADAS_DECISION_LINE &&
                    d.permission == ADAS_DRIVE_INSUFFICIENT) judgment = text("变道信息不足");
                else if (d.reasons & ADAS_DRIVE_PEDESTRIAN) judgment = text("行人路径冲突，请停止");
                else if (d.reasons & ADAS_DRIVE_VEHICLE) judgment = text("车辆路径冲突，请停止");
            }
            if (d.alarm_active) { judgment += text(" · 报警"); judgmentColor = QColor("#ff6b77"); }
        }
        label(p,QRect(32,1024,1392,56),judgment,21,judgmentColor,Qt::AlignRight|Qt::AlignVCenter);
        if (pointer) drawCursor(p);
    }
    void drawCursor(QPainter &p)
    {
        /* Painted cursor belongs to ARGB, not an extra DRM owner/plane. */
        QPolygon arrow; arrow << cursor << cursor+QPoint(0,25) << cursor+QPoint(7,19)
            << cursor+QPoint(13,31) << cursor+QPoint(18,28) << cursor+QPoint(12,16) << cursor+QPoint(22,16);
        p.setPen(QPen(Qt::black,2)); p.setBrush(Qt::white); p.drawPolygon(arrow);
    }
    void render(uint64_t nowNs = 0)
    {
        std::unique_lock<std::mutex> lock(ui.mutex,std::try_to_lock);
        if (!lock.owns_lock() || !ui.live) return;
        const auto dark = ui.dark; const double fps = ui.fps; const bool recognition = ui.recognition;
        const bool masks[] = {ui.masks[0],ui.masks[1],ui.masks[2]};
        const auto intent = ui.intent; const bool intentValid = ui.intent_valid, bound = ui.frame_bound;
        auto driving = ui.drive;
        const auto testing = ui.signal_test;
        const bool pedestrian = ui.pedestrian_mode;
        const auto observation = ui.test_observation;
        const auto testPicture = ui.test_picture;
        const int source = ui.frame.stream_epoch ? int(ui.frame.source_id) : -1;
        const auto road = ui.road;
        const auto roadRevision = ui.road_revision;
        const auto roadReceived = ui.road_received_ns;
        const auto alertControl = ui.alert_control_revision;
        const uint64_t renderStarted = driveNow();
        auto frame = ui.frame;
        const bool drawGeometry = testing.active || (!syncVideo.enabled && !ui.clean_presentation);
        const auto liveDetection = ui.detection;
        const auto liveLights = specialistDisplayLocked(frame,renderStarted);
        auto detection = residentDisplayLocked(liveDetection,frame,renderStarted);
        if (testing.active) {
            if (!specialistEnabled() && ui.frozen_test_image) frame = ui.frozen_test_image->identity;
            detection = ui.test_detection;
            if (residentLightEnabled() && !pedestrian) detection = liveLights;
        }
        const bool invalidated = ui.invalidate;
        /* Keep the display owner out of Qt formatting/tracking work. Only
         * immutable snapshots and buffer-state transitions need this lock. */
        lock.unlock();
        const auto alert = pedestrianAlertSnapshot();
        if (!nowNs) nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!testing.active && driving.output.permission == ADAS_DRIVE_ALLOW &&
            (!recognition || !bound || invalidated || nowNs >= driving.output.valid_until_ns)) {
            driving.output.permission = ADAS_DRIVE_INSUFFICIENT;
            driving.output.action = ADAS_ACTION_UNKNOWN;
            driving.output.reasons |= ADAS_DRIVE_NO_EVIDENCE;
            driving.output.valid_until_ns = 0;
        }
        const bool roadVisible = (!testing.active || specialistEnabled()) && road.valid && recognition && bound && masks[2] &&
            nowNs < road.valid_until_ns && road.stream_epoch == frame.stream_epoch &&
            road.source_id == frame.source_id && road.layout == frame.layout &&
            road.config_generation == frame.config_generation;
        const auto rawDetection = detection;
        if (!testing.active && !residentLightEnabled()) detection_light_tracker_update(&lightTracker,&rawDetection,&frame,nowNs,
            recognition && bound && masks[0],&detection);
        /* Hiding lights must not hide the independent vehicle overlay. */
        if (recognition && bound && !masks[0])
            detection.valid = detection_overlay_visible(&rawDetection,&frame,nowNs);
        QByteArray key(reinterpret_cast<const char *>(&dark.actual),sizeof(dark.actual));
        key += QByteArray::number(dark.phase)+":"+QByteArray::number(dark.available)+":"+QByteArray::number(dark.error);
        key += ":"+QByteArray::number(recognition);
        key += ":alert:"+QByteArray::number(qulonglong(alert.revision));
        key += ":"+QByteArray::number(alert.status)+":"+QByteArray::number(alert.level);
        if (specialistEnabled()) key += ":specialist:"+QByteArray::number(specialistProfile.load());
        key += clearGeometry ? ":clear" : ":raw";
        for (bool mask : masks) key += mask ? '1' : '0';
        key += ":"+QByteArray::number(intent)+":"+QByteArray::number(intentValid)+":"+QByteArray::number(bound)+":"+QByteArray::number(source);
        key += ":"+QByteArray::number(driving.selected.scene)+":"+QByteArray::number(driving.selected.ego_lane);
        key += ":"+QByteArray::number(qulonglong(driving.selected.generation));
        key += ":"+QByteArray::number(driving.output.evaluated)+":"+QByteArray::number(driving.output.permission);
        key += ":"+QByteArray::number(driving.output.action)+":"+QByteArray::number(driving.output.reasons);
        key += ":"+QByteArray::number(driving.output.alarm_active)+":"+QByteArray::number(driving.output.alarm_unresolved);
        key += ":"+QByteArray::number(driving.output.alarm_acknowledged);
        key += ":"+QByteArray::number(qulonglong(driving.output.control_generation));
        key += ":"+QByteArray::number(testing.active)+":"+QByteArray::number(qulonglong(testing.context));
        key += ":"+QByteArray::number(testing.readiness)+":"+QByteArray::number(testing.reason);
        key += pedestrian ? ":pedestrian" : ":signal";
        key += ":"+QByteArray::number(focus)+":"+QByteArray::number(dirty);
        key.append(':').append(numbers[0].text).append(':').append(numbers[1].text).append(':').append(message.toUtf8());
        const QByteArray controlsKey = key;
        const QByteArray videoFpsKey = QByteArray::number(fps,'f',1);
        const QByteArray yoloFpsKey = QByteArray::number(liveDetection.state == DETECTION_NOT_LOADED)+
            ":"+QByteArray::number(liveDetection.fps,'f',1)+":"+QByteArray::number(liveDetection.latency_ms,'f',0);
        const QByteArray statusKey = QByteArray::number(roadVisible && !syncVideo.enabled)+":"+QByteArray::number(detection.state)+
            ":"+QByteArray::number(detection.valid)+":"+QByteArray::number(detection.latency_ms,'f',0);
        const bool specialistDisplay = residentLightEnabled() && !pedestrian;
        const auto &lampDetection = specialistDisplay ? liveLights : detection;
        QByteArray lampKey = QByteArray::number(lampDetection.valid);
        for (unsigned i = 0; i < 2; ++i)
            lampKey += ":"+QByteArray::number(testing.active && !specialistDisplay ? directionColor(&testing,&observation,i) :
                previewDirectionColor(&lampDetection,i));
        if (residentLightEnabled() && !testing.active && lampKey != backgroundLampKey)
            std::printf("TRIAL_LIGHT_LAMPS now_ns=%llu left=%u straight=%u boxes=%u age_ms=%.3f\n",
                (unsigned long long)nowNs,
                recognition && bound && masks[0] ? previewDirectionColor(&lampDetection,0) : 0,
                recognition && bound && masks[0] ? previewDirectionColor(&lampDetection,1) : 0,
                lampDetection.count,lampDetection.valid ? double(nowNs-lampDetection.observed_ns)/1e6 : -1.);
        key += ":"+videoFpsKey+":"+yoloFpsKey+":"+statusKey+":"+lampKey;
        const QByteArray chromeKey = key;
        QByteArray nextGeometryKey = QByteArray::number(recognition)+":"+QByteArray::number(bound)+
            (clearGeometry ? ":clear" : ":raw");
        nextGeometryKey += ":"+QByteArray::number(testing.active);
        if (testing.active)
            nextGeometryKey += ":"+QByteArray::number(qulonglong(testing.context))+":"+
                QByteArray::number(qulonglong(driving.selected.generation));
        nextGeometryKey += ":"+QByteArray::number(driving.output.permission)+":"+
            QByteArray::number(driving.output.alarm_active);
        for (bool mask : masks) nextGeometryKey += mask ? '1' : '0';
        nextGeometryKey += ":"+QByteArray::number(qulonglong(roadVisible && drawGeometry &&
            !syncVideo.enabled ? roadRevision : 0));
        const uint32_t visibleCount = detection.valid && drawGeometry ? qMin(detection.count,uint32_t(DETECTION_OVERLAY_CAPACITY)) : 0;
        nextGeometryKey += ":"+QByteArray::number(visibleCount)+":"+QByteArray::number(detection.light_temporal_valid);
        nextGeometryKey.append(reinterpret_cast<const char *>(detection.boxes),int(visibleCount*sizeof(detection.boxes[0])));
        if (detection.light_temporal_valid) {
            nextGeometryKey.append(reinterpret_cast<const char *>(detection.track_ids),int(visibleCount*sizeof(detection.track_ids[0])));
            nextGeometryKey.append(reinterpret_cast<const char *>(detection.light_flags),int(visibleCount*sizeof(detection.light_flags[0])));
        }
        key += ":"+nextGeometryKey;
        if (key == lastKey && cursor == lastCursor) return;
        if (!lock.try_lock() || !ui.live) return;
        const int index = ui.clean_presentation ? ui.presentation.acquire() : vdma_ui_buffer_acquire(&ui.buffers);
        if (index < 0) return;
        ui.drawing = true;
        void *pixels = overlayPixels(index);
        const uint32_t pitch = overlayPitch(index);
        lock.unlock();
        const auto started = std::chrono::steady_clock::now();
        uint64_t stage = driveNow(), clearNs = 0, chromeNs = 0, overlayNs = 0, damageNs = 0;
        const bool full = key != backgroundKey;
        const bool chrome = full && chromeKey != backgroundChromeKey;
        if (full) {
            QRegion changed;
            const bool geometryChanged = nextGeometryKey != geometryKey;
            if (geometryChanged && !overlayInk.isEmpty()) {
                changed |= overlayInk;
                QPainter clear(&geometryLayer);
                clear.setClipRegion(overlayInk);
                clear.setCompositionMode(QPainter::CompositionMode_Source);
                clear.fillRect(QRect(0,146,1456,819),Qt::transparent);
            }
            clearNs = driveNow()-stage; stage = driveNow();
            /* Geometry/identity revisions do not invalidate unchanged controls. */
            if (chrome) {
                const bool controlsUnchanged = controlsKey == backgroundControlsKey;
                QRegion damage;
                /* Include adjacent antialiased panel edges at clip boundaries. */
                if (videoFpsKey != backgroundVideoFpsKey) damage |= QRect(325,0,180,88).adjusted(-2,-2,2,2);
                if (yoloFpsKey != backgroundYoloFpsKey) damage |= QRect(530,0,475,88).adjusted(-2,-2,2,2);
                if (lampKey != backgroundLampKey) damage |= QRect(1490,156,396,176).adjusted(-2,-2,2,2);
                paint(chromeLayer,dark,fps,recognition,masks,intent,intentValid,bound,source,false,roadVisible ? &road : nullptr,&detection,&driving,PaintChrome,
                    controlsUnchanged ? &damage : nullptr,nullptr,true,false,&testing,&testPicture,&observation,&liveDetection,pedestrian,&alert,
                    specialistDisplay ? &liveLights : nullptr);
                changed |= controlsUnchanged ? damage : QRegion(background.rect());
                backgroundChromeKey = chromeKey;
                backgroundControlsKey = controlsKey;
                backgroundVideoFpsKey = videoFpsKey;
                backgroundYoloFpsKey = yoloFpsKey;
                backgroundStatusKey = statusKey;
                backgroundLampKey = lampKey;
            }
            chromeNs = driveNow()-stage; stage = driveNow();
            if (geometryChanged) {
                overlayInk = QRegion();
                paint(geometryLayer,dark,fps,recognition,masks,intent,intentValid,bound,source,false,roadVisible && !syncVideo.enabled ? &road : nullptr,&detection,&driving,PaintOverlay,nullptr,&overlayInk,drawGeometry,false,&testing,&testPicture,&observation,&liveDetection,pedestrian);
                geometryKey = nextGeometryKey;
                changed |= overlayInk;
            }
            overlayNs = driveNow()-stage; stage = driveNow();
            /* Compose only changed pixels. The clean layer never receives ink. */
            dashboardComposeLayers(background,chromeLayer,geometryLayer,changed);
            /* Each scanout slot accumulates all changes while it is unavailable. */
            for (auto &pending : slotDamage) pending |= changed;
            backgroundKey = key;
            damageNs = driveNow()-stage;
        }
        if (drawGeometry) preview.offer(background,frame,detection,roadVisible ? &road : nullptr);
        const auto painted = std::chrono::steady_clock::now();
        QImage image(static_cast<uchar *>(pixels),1920,1080,pitch,QImage::Format_ARGB32_Premultiplied);
        const QRegion restore = (slotDamage[index] | QRegion(slotCursor[index])).intersected(image.rect());
        uint64_t copiedBytes = 0;
        for (const QRect &rect : restore) {
            for (int y = rect.top(); y <= rect.bottom(); y++)
                std::memcpy(image.scanLine(y)+rect.x()*4,background.constScanLine(y)+rect.x()*4,size_t(rect.width())*4);
            copiedBytes += uint64_t(rect.width())*rect.height()*4;
        }
        { QPainter painter(&image); painter.setRenderHint(QPainter::Antialiasing); drawCursor(painter); }
        slotDamage[index] = QRegion();
        slotCursor[index] = QRect(cursor-QPoint(3,3),QSize(29,38));
        lastKey = key; lastCursor = cursor;
        const uint64_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-started).count();
        lock.lock();
        if (alertControl != ui.alert_control_revision ||
            testing.context != ui.signal_test.context || testing.selection != ui.signal_test.selection ||
            pedestrian != ui.pedestrian_mode || driving.selected.generation != ui.drive.selected.generation) {
            if (ui.clean_presentation) ui.presentation.cancel(index);
            else vdma_ui_buffer_cancel_draw(&ui.buffers,index);
            slotDamage[index] = QRegion(background.rect());
            lastKey.clear();
            ui.drawing = false; ui.drawn.notify_all(); renderWake.request();
            return;
        }
        ++ui.revision;
        if (ui.clean_presentation) {
            if (!ui.presentation.cleanReady(index,overlayFramebuffer(index))) ui.presentation.cancel(index);
        } else vdma_ui_buffer_ready(&ui.buffers,index,ui.revision);
        auto &slot = ui.road_slots[index];
        slot.valid = roadVisible && drawGeometry; slot.context = frame; slot.sequence = road.frame_sequence;
        slot.observed = road.observed_ns; slot.finished = road.finished_ns;
        slot.received = roadReceived; slot.render = renderStarted;
        slot.ready = driveNow(); slot.until = road.valid_until_ns;
        ui.clear_ns += clearNs; ui.chrome_ns += chromeNs;
        ui.overlay_ns += overlayNs; ui.damage_ns += damageNs;
        if (roadVisible && drawGeometry) {
            ui.road_renders++;
            if (road.reference.confirmed_count || road.reference.cross_detected)
                ui.road_geometry_renders++;
        }
        ui.render_total_ns += elapsed; if (elapsed > ui.render_max_ns) ui.render_max_ns = elapsed;
        ui.chrome_renders += chrome;
        ui.argb_copy_bytes += copiedBytes;
        ui.paint_total_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(painted-started).count();
        ui.copy_total_ns += elapsed-std::chrono::duration_cast<std::chrono::nanoseconds>(painted-started).count();
        ui.full_renders += full;
        ui.drawing = false; ui.drawn.notify_all();
    }
};

static unsigned int paintSynchronized(void *pixels, const vdma_frame_identity &id,
    const road_overlay_geometry *road, const detection_overlay *detection)
{
    bool masks[3], recognition, bound;
    vdma_dark_async dark{};
    detection_overlay display{};
    {
        std::lock_guard<std::mutex> lock(ui.mutex);
        std::copy(ui.masks,ui.masks+3,masks);
        if (specialistEnabled() && ui.signal_test.active) masks[0] = false;
        recognition = ui.recognition; bound = ui.frame_bound && DashboardSync::context(id,ui.frame);
        dark = ui.dark;
        if (detection && residentLightEnabled()) {
            display = residentDisplayLocked(*detection,id,driveNow());
            detection = &display;
        }
    }
    /* A private view keeps all painter/font state on this worker. Its
     * overlay pass uses the same geometry and captions as the real-time UI. */
    static thread_local View view;
    QImage image(static_cast<uchar *>(pixels),1920,1080,3840,QImage::Format_RGB16);
    view.paint(image,dark,0,recognition,masks,0,false,bound,int(id.source_id),
        false,road,detection,nullptr,View::PaintOverlay,nullptr,nullptr,true,true);
    return recognition && bound ? ((road && masks[2] ? 1U : 0U) |
        (detection && (masks[0] || masks[1]) ? 2U : 0U)) : 0U;
}

static unsigned int paintExactTimed(void *memory, const vdma_frame_identity &id,
    const road_overlay_geometry *road, const detection_overlay *detection,
    uint64_t observed, uint64_t generation)
{
    static thread_local DashboardLaneDisplay lanes;
    static thread_local DashboardCrosswalkDisplay crosswalks;
    static thread_local DashboardCrosswalkScene crosswalkScene;
    static thread_local DashboardCrosswalkTrace crosswalkTrace;
    static thread_local DashboardVehicleDisplay vehicleDisplay;
    static thread_local DashboardVehicleTrace vehicleTrace;
    uint64_t controlRevision = 0;
    uint64_t vehicleRevision = 0;
    bool masks[3] = {true,true,true};
    detection_overlay filtered{};
    detection_overlay residentLights{};
    bool resident = false;
    if (ui.qt_exact) {
        std::lock_guard<std::mutex> lock(ui.mutex);
        if (!ui.recognition || !ui.frame_bound || ui.invalidate || id.source_id != 2 ||
            !DashboardSync::context(id,ui.frame)) {
            lanes.clear(); crosswalks.clear(); crosswalkScene.clear(); vehicleDisplay.clear(); return 0;
        }
        controlRevision = ui.crosswalk_control_revision;
        vehicleRevision = ui.vehicle_control_revision;
        std::copy(ui.masks,ui.masks+3,masks);
        if (specialistEnabled() && ui.signal_test.active) masks[0] = false;
        resident = residentLightEnabled();
        if (resident && masks[0] && !ui.signal_test.active)
            residentLights = residentLightSnapshotLocked(id,driveNow());
    }
    if (!road || !road->valid || !masks[2] || !DashboardSync::matches(*road,id))
        road = nullptr;
    DisplayedCrosswalk crosswalk;
    const char *crosswalkMode = std::getenv("VDMA_CROSSWALK_DISPLAY");
    const bool stabilize = observed && (!crosswalkMode || std::strcmp(crosswalkMode,"raw"));
    const char *vehicleMode = std::getenv("VDMA_VEHICLE_DISPLAY");
    const bool stableVehicles = ui.qt_exact && observed && (!vehicleMode || std::strcmp(vehicleMode,"raw"));
    bool sceneCut = false;
    if (stabilize || stableVehicles) sceneCut = crosswalkScene.cut(memory);
    if (stabilize) {
        const uint64_t started = driveNow();
        crosswalk = crosswalks.update(road,id,observed,controlRevision,generation,masks[2],sceneCut);
        crosswalkTrace.add(memory,id,observed,road,crosswalk,sceneCut,(driveNow()-started)/1e3);
    } else { crosswalks.clear(); if (!stableVehicles) crosswalkScene.clear(); }
    DisplayedVehicles vehicles;
    bool detectionInk = false;
    if (detection && detection->valid &&
        (syncVideo.holdYolo ? DashboardSync::holdMatches(*detection,id) : DashboardSync::matches(*detection,id))) {
        filtered = *detection;
        filtered.count = std::min(filtered.count,uint32_t(DETECTION_OVERLAY_CAPACITY));
        for (uint32_t i = 0; i < filtered.count; ++i) {
            auto &b = filtered.boxes[i];
            if (!masks[b.signal ? 0 : 1] || (resident && b.signal) || b.display_held ||
                (filtered.light_temporal_valid &&
                 (filtered.light_flags[i] & (DETECTION_OVERLAY_LIGHT_PREDICTED |
                    DETECTION_OVERLAY_LIGHT_PENDING))))
                b.x1 = b.x0;
        }
        detection = &filtered;
    } else detection = nullptr;
    if (stableVehicles) {
        const uint64_t started = driveNow();
        vehicles = vehicleDisplay.update(detection,id,observed,driveNow(),
            detection ? syncVideo.detectionDeadline(*detection) : 0,
            vehicleRevision,generation,masks[1],sceneCut);
        vehicleTrace.add(memory,id,observed,detection,vehicles,(driveNow()-started)/1e3,sceneCut);
        // A hidden vehicle layer must still pass original people/lights through.
        if (masks[1]) {
            filtered = vehicles.overlay;
            detection = filtered.valid ? &filtered : nullptr;
        }
    } else vehicleDisplay.clear();
    if (!memory) return 0;
    if (detection)
        for (unsigned i = 0; i < detection->count; ++i)
            detectionInk |= detection->boxes[i].x1 > detection->boxes[i].x0 &&
                detection->boxes[i].y1 > detection->boxes[i].y0;
    if (!detectionInk) detection = nullptr;
    const bool mainInk = detectionInk;
    if (residentLights.valid && residentLights.count) detectionInk = true;
    if (!road) lanes.clear();
    if (!road && !detection && !detectionInk) return 0;
    road_overlay_geometry displayed{};
    if (road && ui.qt_exact && syncVideo.holdYolo) {
        displayed = lanes.update(*road,id);
        road = &displayed;
    }
    // Keep original result identities even when display-only YOLO holding is enabled.
    static thread_local View view;
    view.clearGeometry = true;
    vdma_dark_async dark{};
    QImage image(static_cast<uchar *>(memory),1920,1080,3840,QImage::Format_RGB16);
    view.paint(image,dark,0,true,masks,0,false,true,int(id.source_id),
        false,road,detection,nullptr,View::PaintOverlay,nullptr,nullptr,true,true,
        nullptr,nullptr,nullptr,nullptr,false,nullptr,nullptr,stabilize ? &crosswalk : nullptr,
        stableVehicles ? &vehicles : nullptr);
    static thread_local View lightView;
    lightView.clearGeometry = true;
    if (residentLights.valid && residentLights.count)
        lightView.paint(image,dark,0,true,masks,0,false,true,int(id.source_id),
            false,nullptr,&residentLights,nullptr,View::PaintOverlay,nullptr,nullptr,true,true);
    static thread_local uint64_t lightReportAt = 0, lightFrames = 0, lightInk = 0, mainFrames = 0;
    if (resident) {
        ++lightFrames; lightInk += residentLights.valid && residentLights.count; mainFrames += mainInk;
        const uint64_t at = driveNow();
        if (!lightReportAt) lightReportAt = at;
        else if (at-lightReportAt >= UINT64_C(1000000000)) {
            std::printf("TRIAL_LIGHT_DISPLAY frames=%llu light=%llu main=%llu age_ms=%.3f ttl_ms=%.0f\n",
                (unsigned long long)lightFrames,(unsigned long long)lightInk,
                (unsigned long long)mainFrames,
                residentLights.valid ? double(at-residentLights.observed_ns)/1e6 : -1.,
                double(residentLightLifetimeNs())/1e6);
            lightFrames = lightInk = mainFrames = 0; lightReportAt = at;
        }
    }
    static thread_local uint64_t reportAt = 0, applications = 0, priorApplications = 0;
    static thread_local uint64_t priorSuppressed = 0, priorAccepted = 0;
    ++applications;
    const uint64_t stamp = driveNow();
    if (!reportAt) reportAt = stamp;
    else if (stamp-reportAt >= UINT64_C(5000000000)) {
        std::printf("VIDEO_INK_WINDOW applications=%llu lane_jump_suppressed=%llu "
            "lane_jump_accepted=%llu display_only=1 backend=qt-direct\n",
            (unsigned long long)(applications-priorApplications),
            (unsigned long long)(lanes.suppressed-priorSuppressed),
            (unsigned long long)(lanes.accepted-priorAccepted));
        priorApplications = applications;
        priorSuppressed = lanes.suppressed; priorAccepted = lanes.accepted; reportAt = stamp;
    }
    return (road ? 1U : 0U) | (detectionInk ? 2U : 0U);
}
// Existing callers that have no source observation time retain raw rendering.
static unsigned int paintExact(void *memory, const vdma_frame_identity &id,
    const road_overlay_geometry *road, const detection_overlay *detection)
{
    return paintExactTimed(memory,id,road,detection,0,0);
}
extern "C" int vdma_dashboard_sync_init(const int sources[4])
{
    const char *mode = std::getenv("VDMA_PRESENT_MODE");
    if (mode && (!std::strcmp(mode,"exact") || !std::strcmp(mode,"qt-exact"))) {
        const char *value = std::getenv("VDMA_EXACT_DELAY_MS");
        char *end = nullptr; unsigned long delay = value ? std::strtoul(value,&end,10) : 50;
        if ((value && (!*value || *end)) || delay < 20 || delay > 100) { errno = EINVAL; return -1; }
        const char *yoloOsd = std::getenv("VDMA_YOLO_OSD_MODE");
        const char *crosswalkMode = std::getenv("VDMA_CROSSWALK_DISPLAY");
        const char *vehicleMode = std::getenv("VDMA_VEHICLE_DISPLAY");
        if (vehicleMode && std::strcmp(vehicleMode,"stable") && std::strcmp(vehicleMode,"raw")) {
            errno = EINVAL; return -1;
        }
        std::printf("VEHICLE_DISPLAY mode=%s hold_source_ms=100 confirm_frames=2 capacity=32 "
            "minimum_hz=4 center_beta=80 scale_beta=1 raw_decisions=unchanged\n",
            vehicleMode ? vehicleMode : "stable");
        if (crosswalkMode && std::strcmp(crosswalkMode,"stable") && std::strcmp(crosswalkMode,"raw")) {
            errno = EINVAL; return -1;
        }
        std::printf("CROSSWALK_DISPLAY mode=%s minimum_hz=4 center_beta=80 scale_beta=4 "
            "initial_frames=2 jump_frames=3 missing_hold_ms=0 association_ms=80 raw_decisions=unchanged\n",
            crosswalkMode ? crosswalkMode : "stable");
        if (yoloOsd && std::strcmp(yoloOsd,"exact") && std::strcmp(yoloOsd,"hold")) {
            errno = EINVAL; return -1;
        }
        return syncVideo.start(ui.drm,sources,paintExact,unsigned(delay),false,true,
            yoloOsd && !std::strcmp(yoloOsd,"hold"),paintExactTimed) ? 0 : -1;
    }
    const char *clean = std::getenv("VDMA_CLEAN_VIDEO_DELAY_MS");
    if (clean && std::strcmp(clean,"0")) {
        char *end = nullptr; const unsigned long delay = std::strtoul(clean,&end,10);
        if (!ui.clean_presentation || !*clean || *end || delay < 80 || delay > 170) {
            errno = EINVAL; return -1;
        }
        return syncVideo.start(ui.drm,sources,nullptr,unsigned(delay),true) ? 0 : -1;
    }
    const char *value = std::getenv("VDMA_SAME_FRAME_DELAY_MS");
    if (!value || !std::strcmp(value,"0")) return 0;
    char *end = nullptr; const unsigned long delay = std::strtoul(value,&end,10);
    if (!*value || *end || delay < 120 || delay > 170) { errno = EINVAL; return -1; }
    return syncVideo.start(ui.drm,sources,paintSynchronized,unsigned(delay)) ? 0 : -1;
}
extern "C" int vdma_dashboard_sync_enabled() { return syncVideo.enabled; }
extern "C" int vdma_dashboard_sync_capture(const vdma_buffer *b,
    const vdma_frame_identity *id,uint64_t observed)
{ return syncVideo.capture(*b,*id,observed); }
extern "C" int vdma_dashboard_sync_holds(const vdma_buffer *b)
{ return syncVideo.enabled && syncVideo.holds(*b); }
extern "C" int vdma_dashboard_sync_peek(uint64_t stamp,uint32_t *fb,
    vdma_frame_identity *id,vdma_frame_header *header)
{ return syncVideo.peek(stamp,fb,id,header); }
extern "C" void vdma_dashboard_sync_cancel(int index) { syncVideo.cancel(index); }
extern "C" void vdma_dashboard_sync_submit(int index) { syncVideo.submit(index); }
extern "C" void vdma_dashboard_sync_complete(uint64_t stamp) { syncVideo.complete(stamp); }
extern "C" void vdma_dashboard_sync_stop() { syncVideo.stop(); }

static int inputOpen(const QString &path, View &view, bool mouse, QObject &parent)
{
    const int fd = open(path.toLocal8Bit().constData(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
    if (fd < 0) return -1;
    if (ioctl(fd,EVIOCGRAB,1)) { close(fd); return -1; }
    auto notifier = new QSocketNotifier(fd,QSocketNotifier::Read,&parent);
    QObject::connect(notifier,&QSocketNotifier::activated,&parent,[fd,&view,mouse,notifier](int){
        input_event events[32];
        for (;;) {
            const ssize_t bytes = read(fd,events,sizeof(events));
            if (bytes < 0 && (errno == EAGAIN || errno == EINTR)) return;
            if (bytes <= 0 || bytes % sizeof(input_event)) {
                std::lock_guard<std::mutex> lock(ui.mutex);
                resetDriveLocked(); notifier->setEnabled(false); return;
            }
            for (size_t i = 0; i < size_t(bytes)/sizeof(input_event); i++) {
                const auto &e = events[i];
                if (mouse && e.type == EV_REL) {
                    if (e.code == REL_X) view.cursor.setX(qBound(0,view.cursor.x()+e.value,1919));
                    if (e.code == REL_Y) view.cursor.setY(qBound(0,view.cursor.y()+e.value,1079));
                }
                if (e.type == EV_KEY && e.value) {
                    if (mouse && e.code == BTN_LEFT && e.value == 1) view.click();
                    else if (!mouse && e.value == 1) view.key(e.code);
                }
                if (e.type == EV_SYN && e.code == SYN_DROPPED) { /* Lost input must not issue commands. */
                    std::lock_guard<std::mutex> lock(ui.mutex); resetDriveLocked();
                    notifier->setEnabled(false); std::fprintf(stderr,"Input overrun: reconnect dashboard\n"); return;
                }
            }
        }
    });
    std::printf("DASHBOARD_INPUT path=%s kind=%s grabbed=1\n",path.toLocal8Bit().constData(),mouse ? "mouse" : "keyboard");
    return fd;
}

int main(int argc,char **argv)
{
    const char *laneNames[] = {"VDMA_LANE_REFERENCE_X","VDMA_LANE_REFERENCE_Y"};
    int32_t *laneValues[] = {&ui.lane_config.reference_x,&ui.lane_config.reference_y};
    const long limits[] = {1919,1079};
    for (unsigned i=0;i<2;++i) if (const char *value=std::getenv(laneNames[i])) {
        char *end=nullptr;
        errno=0;
        const long parsed=std::strtol(value,&end,10);
        if (errno || !*value || *end || parsed<1 || parsed>limits[i]) {
            std::fprintf(stderr,"%s expects 1..%ld\n",laneNames[i],limits[i]);return 1;
        }
        *laneValues[i]=int32_t(parsed);
    }
    std::printf("LANE_TEST_CONFIG reference=%d,%d minimum_span=%d maximum_width=%d ambiguity=%d scope=limited-front-view inputs=left,right association=fragments-v2\n",
        ui.lane_config.reference_x,ui.lane_config.reference_y,ui.lane_config.minimum_span,
        ui.lane_config.maximum_width,ui.lane_config.ambiguity_px);
    const char *resident = std::getenv("VDMA_LIGHT_RESIDENT");
    if (!resident) resident = std::getenv("VDMA_TRIAL_RESIDENT_LIGHT");
    if (resident && std::strcmp(resident,"0") && std::strcmp(resident,"1")) {
        std::fprintf(stderr,"VDMA_LIGHT_RESIDENT expects 0 or 1\n"); return 1;
    }
    if (residentLightEnabled() && !specialistEnabled()) {
        std::fprintf(stderr,"Resident light requires the specialist model\n"); return 1;
    }
    const char *period = std::getenv("VDMA_LIGHT_TASK_PERIOD_MS");
    if (period) {
        char *end = nullptr;
        errno = 0;
        const unsigned long value = std::strtoul(period,&end,10);
        if (errno || !*period || *end || value < 100 || value > 400) {
            std::fprintf(stderr,"VDMA_LIGHT_TASK_PERIOD_MS expects 100..400\n"); return 1;
        }
        specialistTaskPeriodMs=unsigned(value);
    }
    const char *profile = std::getenv("VDMA_LIGHT_PROFILE");
    if (profile) {
        char *end = nullptr;
        const unsigned long value = std::strtoul(profile,&end,10);
        if (!*profile || *end || value > 3) {
            std::fprintf(stderr,"VDMA_LIGHT_PROFILE expects 0..3\n"); return 1;
        }
        specialistProfile = unsigned(value);
    }
    const char *mode = std::getenv("VDMA_PRESENT_MODE");
    if (mode && std::strcmp(mode,"legacy") && std::strcmp(mode,"clean") &&
        std::strcmp(mode,"exact") && std::strcmp(mode,"qt-exact")) {
        std::fprintf(stderr,"VDMA_PRESENT_MODE expects legacy, clean, exact, or qt-exact\n"); return 1;
    }
    ui.qt_exact = mode && !std::strcmp(mode,"qt-exact");
    ui.clean_presentation = ui.qt_exact || (mode && !std::strcmp(mode,"clean"));
    ui.exact_presentation = mode && !std::strcmp(mode,"exact");
    const char *yoloOsd = std::getenv("VDMA_YOLO_OSD_MODE");
    if (yoloOsd && ((std::strcmp(yoloOsd,"exact") && std::strcmp(yoloOsd,"hold")) ||
        (!ui.exact_presentation && !ui.qt_exact))) {
        std::fprintf(stderr,"VDMA_YOLO_OSD_MODE expects exact or hold in exact/qt-exact presentation\n");
        return 1;
    }
#ifndef VDMA_HAVE_FPGA_ROAD
    if (ui.exact_presentation || ui.qt_exact) {
        std::fprintf(stderr,"Exact mode requires the FPGA-road production build\n"); return 1;
    }
#endif
    if (ui.exact_presentation || ui.qt_exact) {
        const char *value = std::getenv("VDMA_EXACT_DELAY_MS");
        char *end = nullptr; const unsigned long delay = value ? std::strtoul(value,&end,10) : 50;
        if ((value && (!*value || *end)) || delay < 20 || delay > 100) {
            std::fprintf(stderr,"VDMA_EXACT_DELAY_MS expects 20..100\n"); return 1;
        }
        ui.recognition = true;
    }
    const char *oldDelay = std::getenv("VDMA_SAME_FRAME_DELAY_MS");
    if (ui.clean_presentation && oldDelay && std::strcmp(oldDelay,"0")) {
        std::fprintf(stderr,"Clean presentation rejects the retired baked-video cache\n"); return 1;
    }
    const char *cleanDelay = std::getenv("VDMA_CLEAN_VIDEO_DELAY_MS");
    if (cleanDelay && std::strcmp(cleanDelay,"0")) {
        char *end = nullptr; const unsigned long delay = std::strtoul(cleanDelay,&end,10);
        if (!ui.clean_presentation || !*cleanDelay || *end || delay < 80 || delay > 170) {
            std::fprintf(stderr,"Clean video delay needs clean mode and 80..170 ms\n"); return 1;
        }
    }
    const char *history = std::getenv("VDMA_PRESENT_HISTORY");
    if (history && std::strcmp(history,"0") && std::strcmp(history,"1")) {
        std::fprintf(stderr,"VDMA_PRESENT_HISTORY expects 0 or 1\n"); return 1;
    }
    if (history && !std::strcmp(history,"1")) resultHistory.reset(new DashboardHistory);
    qputenv("QT_QPA_PLATFORM","offscreen");
    QGuiApplication app(argc,argv);
    std::unique_ptr<View> view;
    QString mouse,keyboard,render,validation,previewDirectory,recordPath;
    if (!ui.exact_presentation)
        view.reset(new View);
    const QByteArray fontFile = qgetenv("VDMA_FONT_FILE");
    if (!fontFile.isEmpty() &&
        QFontDatabase::addApplicationFont(QString::fromLocal8Bit(fontFile)) < 0) {
        std::fprintf(stderr,"Cannot load dashboard Chinese font\n"); return 1;
    }
    {
        QFont font("Noto Sans CJK SC"); font.setPixelSize(24);
        const QRawFont raw = QRawFont::fromFont(font);
        for (const QChar ch : text("道路感知识别开关交通灯状态模拟驾驶意图未知左右直停暗光增强阈值强度应用参数恢复默认实际生效车辆行人加载信息不足连接失败处理中确认整数车道线与斑马目标自行公交卡摩托车停止标志")) {
            if (!raw.supportsCharacter(ch)) { std::fprintf(stderr,"Dashboard Chinese glyph missing\n"); return 1; }
        }
    }
    std::vector<char *> args; args.push_back(argv[0]);
    bool validationRecognition = false;
    for (int i = 1; i < argc; i++) {
        const QString a = QString::fromLocal8Bit(argv[i]);
        if (a == "--validation-recognition" || a == "--recognition-on") { validationRecognition = true; continue; }
        if ((a == "--mouse" || a == "--keyboard" || a == "--render-only" || a == "--validation-dir" || a == "--preview-dir" || a == "--record") && i+1 < argc) {
            const QString value = QString::fromLocal8Bit(argv[++i]);
            if (a == "--mouse") mouse = value; else if (a == "--keyboard") keyboard = value;
            else if (a == "--validation-dir") validation = value; else if (a == "--preview-dir") previewDirectory = value;
            else if (a == "--record") recordPath = value; else render = value;
        } else args.push_back(argv[i]);
    }
    args.push_back(nullptr);
    if (!render.isEmpty() && !view) view.reset(new View);
    const char *workload = std::getenv("VDMA_PRESENT_TEST_WORKLOAD");
    if (workload && std::strcmp(workload,"full")) {
        const bool videoOnly = !std::strcmp(workload,"video");
        bool bounded = false;
        for (size_t i = 0; i+2 < args.size(); ++i)
            if (!std::strcmp(args[i],"--seconds")) {
                char *end = nullptr; const unsigned long seconds = std::strtoul(args[i+1],&end,10);
                bounded = *args[i+1] && !*end && seconds > 0 && seconds <= 1800;
            }
        if ((!videoOnly && std::strcmp(workload,"yolo")) || !ui.clean_presentation || !bounded) {
            std::fprintf(stderr,"Diagnostic workload needs clean mode and bounded --seconds\n"); return 1;
        }
        if (videoOnly) validationRecognition = false;
        ui.masks[2] = false;
        std::printf("PRESENT_TEST_WORKLOAD mode=%s diagnostic_only=1 inference_frequency_unchanged=1\n",workload);
    }
    if (!render.isEmpty()) {
        QImage image(1920,1080,QImage::Format_ARGB32_Premultiplied); const bool masks[] = {true,true,true};
        view->paint(image,ui.dark,0,false,masks,ui.intent,ui.intent_valid,false);
        return image.save(render) ? 0 : 1;
    }
    if (ui.exact_presentation && (!mouse.isEmpty() || !keyboard.isEmpty())) {
        std::fprintf(stderr,"Fullscreen exact mode does not accept dashboard input controls\n"); return 1;
    }
    if (!ui.exact_presentation && !evidence.start(validation)) {
        std::fprintf(stderr,"Cannot create validation directory\n"); return 1;
    }
    if (!ui.exact_presentation && !preview.start(previewDirectory)) { evidence.stop(); return 1; }
    if (!ui.exact_presentation) recorder.configure(recordPath);
    if (validationRecognition) { ui.recognition = true; ui.invalidate = true; std::puts("DASHBOARD recognition=ON startup_dark_command=NONE"); }
    QObject inputOwner; int mouseFd = -1,keyboardFd = -1;
    if (!mouse.isEmpty()) mouseFd = inputOpen(mouse,*view,true,inputOwner);
    if (!keyboard.isEmpty()) keyboardFd = inputOpen(keyboard,*view,false,inputOwner);
    if ((!mouse.isEmpty() && mouseFd < 0) || (!keyboard.isEmpty() && keyboardFd < 0)) {
        std::perror("Dashboard evdev input");
        if (mouseFd >= 0) close(mouseFd);
        if (keyboardFd >= 0) close(keyboardFd);
        if (!ui.exact_presentation) { evidence.stop(); preview.stop(); } return 1;
    }
    std::atomic<bool> done{false}; int result = 1;
    if (!ui.exact_presentation && !renderWake.start()) {
        std::perror("Dashboard render eventfd");
        if (mouseFd >= 0) close(mouseFd);
        if (keyboardFd >= 0) close(keyboardFd);
        if (!ui.exact_presentation) { evidence.stop(); preview.stop(); } return 1;
    }
    std::unique_ptr<QSocketNotifier> renderNotifier;
    if (!ui.exact_presentation) {
        renderNotifier.reset(new QSocketNotifier(renderWake.fd(),QSocketNotifier::Read,&app));
        QObject::connect(renderNotifier.get(),&QSocketNotifier::activated,&app,[&](int){
            renderWake.consume();
            if (done.load()) app.quit(); else view->render();
        });
    }
    std::thread owner([&]{
        result = vdma_display_main(int(args.size()-1),args.data());
        done.store(true);
        if (ui.exact_presentation)
            QMetaObject::invokeMethod(&app, "quit", Qt::QueuedConnection);
    });
    std::thread signalWorker;
    if (specialistEnabled()) signalWorker = std::thread([&]{
        vdma_snapshot_input input{};
        vdma_snapshot_output output{};
        while (!done.load()) {
            bool active;
            {
                std::lock_guard<std::mutex> lock(ui.mutex);
                active = specialistTaskNeededLocked() || residentTaskNeededLocked();
            }
            if (active) {
                const auto started=std::chrono::steady_clock::now();
                if (!processTestTask(nullptr,&input,&output)) processResidentLightTask();
                std::this_thread::sleep_until(started+
                    std::chrono::milliseconds(specialistTaskPeriodMs));
            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });
    QTimer timer;
    if (!ui.exact_presentation)
        QObject::connect(&timer,&QTimer::timeout,&app,[&]{ if (done.load()) app.quit(); else view->render(); });
    timer.setTimerType(Qt::PreciseTimer);
    if (!ui.exact_presentation) timer.start(16);
    app.exec(); owner.join();
    if (signalWorker.joinable()) signalWorker.join();
    if (mouseFd >= 0) close(mouseFd);
    if (keyboardFd >= 0) close(keyboardFd);
    if (!ui.exact_presentation) {
        if (!evidence.stop()) result = 1;
        if (!preview.stop()) result = 1;
        if (!recorder.stop() || recordingFailed) result = 1;
    }
    return result;
}
