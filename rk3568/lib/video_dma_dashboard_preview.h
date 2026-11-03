#ifndef VIDEO_DMA_DASHBOARD_PREVIEW_H
#define VIDEO_DMA_DASHBOARD_PREVIEW_H
#include <QPainter>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSaveFile>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#include <sys/ioctl.h>
#include "detection_overlay.h"
#include "video_dma_snapshot_uapi.h"
#include "road_overlay_geometry.h"

/* Validation-only software composition, never a DRM scanout screenshot.
 * At most one capture/second, two queued jobs and thirty captures per run. */
class DashboardPreview {
    struct Job {
        QImage ui; std::vector<unsigned char> video; vdma_frame_identity frame{};
        detection_overlay detection{}; uint64_t offered = 0, captured = 0, observed = 0; unsigned int number = 0;
        bool road_visible = false;
        uint32_t road_sequence = 0, road_raw = 0, road_confirmed = 0, road_cross = 0;
        uint64_t road_valid_until = 0;
    };
    QString directory; std::mutex mutex; std::condition_variable wake; std::deque<Job> jobs;
    Job offered; std::thread worker; bool stopping = false; unsigned int captured = 0;
    uint64_t lastOffer = 0; std::atomic<unsigned int> errors{0}, written{0}, skipped{0};
    static uint64_t nowNs() { return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count()); }
    void run() {
        for (;;) {
            std::unique_lock<std::mutex> lock(mutex); wake.wait(lock,[&]{ return stopping || !jobs.empty(); });
            if (jobs.empty()) return;
            Job job = std::move(jobs.front()); jobs.pop_front(); lock.unlock();
            QImage composite(1920,1080,QImage::Format_ARGB32_Premultiplied); composite.fill(Qt::black);
            QImage video(job.video.data(),1920,1080,3840,QImage::Format_RGB16);
            { QPainter p(&composite); p.drawImage(QRect(0,146,1456,819),video); p.drawImage(0,0,job.ui); }
            const QString prefix = directory+QString("/preview_%1").arg(job.number,3,10,QChar('0'));
            QJsonObject metadata{{"software_composite",true},{"physical_scanout_capture",false},
                {"epoch",int(job.frame.stream_epoch)},{"source",int(job.frame.source_id)},
                {"video_sequence",int(job.frame.frame_sequence)},{"config_generation",int(job.frame.config_generation)},
                {"detection_sequence",int(job.detection.frame_sequence)},{"detection_valid",bool(job.detection.valid)},
                {"yolo_fps",job.detection.fps},{"latency_ms",job.detection.latency_ms},
                {"video_observed_monotonic_ms",double(job.observed)/1e6},
                {"capture_monotonic_ms",double(job.captured)/1e6},{"ui_age_ms",double(job.captured-job.offered)/1e6}};
            QJsonArray lights;
            if (job.detection.valid) {
                for (uint32_t i = 0; i < job.detection.count && i < DETECTION_OVERLAY_CAPACITY; ++i) {
                    const auto &b = job.detection.boxes[i];
                    if (b.signal) lights.append(QJsonObject{{"x0",b.x0},{"y0",b.y0},{"x1",b.x1},{"y1",b.y1},
                        {"color",int(b.light_color)},{"geometry_held",bool(b.display_held)}});
                }
            }
            metadata.insert("display_light_boxes",lights);
            metadata.insert("detection_observed_monotonic_ms",double(job.detection.observed_ns)/1e6);
            metadata.insert("road_visible_at_render",job.road_visible);
            metadata.insert("road_sequence",int(job.road_sequence));
            metadata.insert("road_raw_count",int(job.road_raw));
            metadata.insert("road_confirmed_count",int(job.road_confirmed));
            metadata.insert("road_cross_detected",bool(job.road_cross));
            metadata.insert("road_valid_until_monotonic_ms",double(job.road_valid_until)/1e6);
            metadata.insert("road_fresh_at_capture",
                job.road_visible && job.captured < job.road_valid_until);
            QSaveFile file(prefix+".json"); const QByteArray json = QJsonDocument(metadata).toJson();
            if (composite.save(prefix+".png") && file.open(QIODevice::WriteOnly) && file.write(json) == json.size() && file.commit()) written++;
            else errors++;
        }
    }
public:
    bool start(const QString &path) {
        if (path.isEmpty()) return true;
        directory = QDir(path).absolutePath(); if (!QDir().mkpath(directory)) return false;
        worker = std::thread([this]{ run(); }); return true;
    }
    void offer(const QImage &image,const vdma_frame_identity &frame,const detection_overlay &detection,
               const road_overlay_geometry *road = nullptr) {
        if (directory.isEmpty()) return;
        const uint64_t now = nowNs(); if (now-lastOffer < UINT64_C(1000000000)) return;
        std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
        if (!lock.owns_lock()) return;
        if (captured >= 30 || jobs.size() >= 2 || !offered.ui.isNull()) return;
        offered.ui = image.copy(); offered.frame = frame; offered.detection = detection; offered.offered = now; lastOffer = now;
        offered.road_visible = road != nullptr;
        if (road) {
            offered.road_sequence = road->frame_sequence;
            offered.road_raw = road->reference.raw_count;
            offered.road_confirmed = road->reference.confirmed_count;
            offered.road_cross = road->reference.cross_detected;
            offered.road_valid_until = road->valid_until_ns;
        }
    }
    void owned(const vdma_frame_identity &frame,const void *pixels,uint64_t observedNs) {
        if (directory.isEmpty()) return;
        Job job; {
            std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
            if (!lock.owns_lock() || offered.ui.isNull() || jobs.size() >= 2 || captured >= 30) return;
            job = std::move(offered); offered = Job{};
        }
        const uint64_t now = nowNs();
        if (now < job.offered || now-job.offered > UINT64_C(200000000) ||
            frame.stream_epoch != job.frame.stream_epoch || frame.source_id != job.frame.source_id ||
            frame.layout != job.frame.layout || frame.config_generation != job.frame.config_generation ||
            (job.detection.valid && !detection_overlay_visible(&job.detection,&frame,now))) { skipped++; return; }
        if (!pixels) { errors++; return; }
        job.video.resize(VDMA_SNAPSHOT_FRAME_BYTES); std::memcpy(job.video.data(),pixels,VDMA_SNAPSHOT_FRAME_BYTES);
        job.frame = frame; job.captured = nowNs(); job.observed = observedNs;
        if (job.detection.valid && !detection_overlay_visible(&job.detection,&frame,job.captured)) { skipped++; return; }
        std::lock_guard<std::mutex> lock(mutex); job.number = ++captured; jobs.push_back(std::move(job)); wake.notify_one();
    }
    bool stop() {
        if (!worker.joinable()) return true;
        { std::lock_guard<std::mutex> lock(mutex); stopping = true; offered = Job{}; wake.notify_all(); }
        worker.join(); std::printf("DASHBOARD_PREVIEW written=%u skipped=%u errors=%u limit=30 rate_max_hz=1 software_composite=1\n",
            written.load(),skipped.load(),errors.load()); return !errors;
    }
};
#endif
