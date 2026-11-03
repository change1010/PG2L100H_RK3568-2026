#ifndef VIDEO_DMA_DASHBOARD_SYNC_H
#define VIDEO_DMA_DASHBOARD_SYNC_H
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>
#include <cerrno>
extern "C" {
#include "video_dma_sync_gpu.h"
#include "video_dma_uapi.h"
#include "video_dma_snapshot_uapi.h"
#include "road_overlay_geometry.h"
#include "detection_overlay.h"
}

/* The owner alone submits/completes scanout. The worker never releases a lease.
 * All slow copy/paint work runs outside mutex; state pins that destination. */
class DashboardSync {
public:
    enum State { Free, Queued, Copying, Waiting, Drawing, Ready, Reserved, Pending, Front };
    enum DetectionReason { NoCandidate, WrongContext, OutsideWindow, Expired, Empty, Hidden, Ink, ReasonCount };
    using Painter = unsigned int (*)(void *, const vdma_frame_identity &,
        const road_overlay_geometry *, const detection_overlay *);
    using TimedPainter = unsigned int (*)(void *, const vdma_frame_identity &,
        const road_overlay_geometry *, const detection_overlay *, uint64_t, uint64_t);
    static uint64_t now() {
        return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }
    static bool context(const vdma_frame_identity &a, const vdma_frame_identity &b) {
        return a.stream_epoch == b.stream_epoch && a.source_id == b.source_id &&
            a.layout == b.layout && a.config_generation == b.config_generation;
    }
    template<class T> static bool matches(const T &r, const vdma_frame_identity &f) {
        return r.valid && r.stream_epoch == f.stream_epoch && r.source_id == f.source_id &&
            r.layout == f.layout && r.config_generation == f.config_generation &&
            r.frame_sequence == f.frame_sequence;
    }
    struct Slot {
        State state = Free;
        vdma_frame_identity id{};
        vdma_frame_header header{};
        uint64_t observed = 0, order = 0, generation = 0, until = 0;
        bool roadInk = false, detectionInk = false, detectionHeld = false;
        DetectionReason detectionReason = NoCandidate;
        std::shared_ptr<const road_overlay_geometry> road;
        std::shared_ptr<const detection_overlay> detection;
    };
    std::array<Slot,VDMA_SYNC_SLOTS> slots{};
    std::mutex mutex;
    std::condition_variable wake;
    std::thread worker;
    void *gpu = nullptr;
    Painter painter = nullptr;
    TimedPainter timedPainter = nullptr;
    std::atomic<bool> enabled{false};
    bool cleanOnly = false;
    bool exactOnly = false;
    bool holdYolo = false;
    bool stopping = false;
    int error = 0, front = -1, pending = -1, reserved = -1;
    uint64_t generation = 1, nextOrder = 0, delayNs = 150000000, paintNs = 100000000;
    vdma_frame_identity latest{};
    uint64_t captured = 0, copied = 0, rendered = 0, presented = 0;
    uint64_t roadFrames = 0, detectionFrames = 0, missingRoad = 0, late = 0, full = 0;
    uint64_t detectionExactFrames = 0, detectionHeldFrames = 0, detectionLate = 0;
    uint64_t detectionHistoryClears = 0;
    uint64_t backlogRecoveries = 0, backlogDropped = 0;
    std::array<uint64_t,ReasonCount> detectionReasons{};
    uint64_t reportAt = 0, reportPresented = 0, reportRoad = 0, reportDetection = 0;
    uint64_t reportCopy = 0, reportDraw = 0, reportCopied = 0, reportRendered = 0, reportAge = 0;
    uint64_t reportDropped = 0;
    std::array<uint64_t,ReasonCount> reportReasons{};
    uint64_t copyNs = 0, copyMax = 0, drawNs = 0, drawMax = 0, ageNs = 0, ageMax = 0;
    uint64_t gaps = 0, repeat = 0, lastSequence = 0, maxOccupied = 0, resets = 0;
    road_overlay_geometry recentRoad{};
    detection_overlay recentDetection{};
    bool recentRoadValid = false, recentDetectionValid = false;
    std::array<std::shared_ptr<const detection_overlay>,16> detectionHistory{};
    size_t detectionNext = 0;
    bool sequenceValid = false;
    static bool holdMatches(const detection_overlay &d, const vdma_frame_identity &f) {
        const uint32_t distance = f.frame_sequence-d.frame_sequence;
        return d.valid && d.state == DETECTION_RUNNING && d.count <= DETECTION_OVERLAY_CAPACITY &&
            d.stream_epoch == f.stream_epoch && d.source_id == f.source_id &&
            d.layout == f.layout && d.config_generation == f.config_generation && distance <= 6;
    }
    uint64_t detectionDeadline(const detection_overlay &d) const {
        const uint64_t budget = delayNs+150000000;
        if (d.observed_ns > UINT64_MAX-budget) return 0;
        return std::min(d.valid_until_ns,d.observed_ns+budget);
    }
    std::shared_ptr<const detection_overlay> heldDetectionLocked(
        const vdma_frame_identity &id, uint64_t observed, uint64_t validAt,
        DetectionReason *reason = nullptr) const {
        std::shared_ptr<const detection_overlay> selected;
        uint32_t distance = UINT32_MAX;
        bool history = false, sameContext = false;
        for (const auto &d : detectionHistory) {
            if (!d) continue;
            history = true;
            sameContext |= d->stream_epoch == id.stream_epoch && d->source_id == id.source_id &&
                d->layout == id.layout && d->config_generation == id.config_generation;
            if (!holdMatches(*d,id) || !d->observed_ns ||
                d->observed_ns > observed || observed-d->observed_ns > 100000000)
                continue;
            const uint32_t delta = id.frame_sequence-d->frame_sequence;
            if (delta < distance) { selected = d; distance = delta; }
        }
        // An empty or expired newer result is a barrier; never revive older boxes.
        const bool expired = selected && validAt >= detectionDeadline(*selected);
        if (reason) *reason = selected ? expired ? Expired : selected->count ? Hidden : Empty :
            !history ? NoCandidate : !sameContext ? WrongContext : OutsideWindow;
        return selected && !expired ? selected : nullptr;
    }
    void dropBacklogLocked(uint64_t stamp) {
        if (!exactOnly || !holdYolo) return;
        uint64_t dropped = 0;
        // Completed copies can be retired before the ring becomes full.
        // Source leases and all active GPU/CPU/scanout owners stay pinned.
        for (auto &s : slots)
            if ((s.state == Waiting || s.state == Ready) &&
                s.generation == generation && stamp >= s.observed &&
                stamp-s.observed > delayNs+50000000) {
                s = Slot{}; ++dropped;
            }
        if (dropped) { ++backlogRecoveries; backlogDropped += dropped; }
    }
    int nextWorkLocked(uint64_t stamp, bool &copy) const {
        int queued = -1, waiting = -1;
        for (int i = 0; i < VDMA_SYNC_SLOTS; ++i) {
            const auto &s = slots[i];
            if (s.state == Queued && (queued < 0 || s.order < slots[queued].order)) queued = i;
            if (!stopping && s.state == Waiting && s.observed+paintNs <= stamp &&
                (waiting < 0 || s.order < slots[waiting].order)) waiting = i;
        }
        // Due frames must reach scanout instead of waiting behind every new copy.
        copy = waiting < 0 && queued >= 0;
        return waiting >= 0 ? waiting : queued;
    }
    void reportLocked(uint64_t stamp) {
        if (!reportAt) { reportAt = stamp; return; }
        if (stamp-reportAt < 5000000000ULL) return;
        const uint64_t frames = presented-reportPresented;
        const uint64_t copies = copied-reportCopied, draws = rendered-reportRendered;
        std::printf("SAME_FRAME_WINDOW seconds=%.3f frames=%llu fps=%.2f road=%llu detection=%llu "
            "age_ms=%.3f copy_ms=%.3f draw_ms=%.3f dropped=%llu "
            "no_candidate=%llu context=%llu window=%llu expired=%llu empty=%llu hidden=%llu ink=%llu\n",
            (stamp-reportAt)/1e9,(unsigned long long)frames,frames*1e9/(stamp-reportAt),
            (unsigned long long)(roadFrames-reportRoad),(unsigned long long)(detectionFrames-reportDetection),
            (ageNs-reportAge)/1e6/(frames ? frames : 1),
            (copyNs-reportCopy)/1e6/(copies ? copies : 1),(drawNs-reportDraw)/1e6/(draws ? draws : 1),
            (unsigned long long)(backlogDropped-reportDropped),
            (unsigned long long)(detectionReasons[NoCandidate]-reportReasons[NoCandidate]),
            (unsigned long long)(detectionReasons[WrongContext]-reportReasons[WrongContext]),
            (unsigned long long)(detectionReasons[OutsideWindow]-reportReasons[OutsideWindow]),
            (unsigned long long)(detectionReasons[Expired]-reportReasons[Expired]),
            (unsigned long long)(detectionReasons[Empty]-reportReasons[Empty]),
            (unsigned long long)(detectionReasons[Hidden]-reportReasons[Hidden]),
            (unsigned long long)(detectionReasons[Ink]-reportReasons[Ink]));
        std::fflush(stdout);
        reportAt = stamp; reportPresented = presented; reportRoad = roadFrames;
        reportDetection = detectionFrames; reportCopy = copyNs; reportDraw = drawNs;
        reportCopied = copied; reportRendered = rendered; reportAge = ageNs;
        reportDropped = backlogDropped; reportReasons = detectionReasons;
    }
    void resetLocked() {
        ++generation; ++resets; sequenceValid = false;
        recentRoadValid = false; recentDetectionValid = false;
        detectionHistory = {}; detectionNext = 0;
        for (auto &s : slots)
            if (s.state != Front && s.state != Pending && s.state != Reserved &&
                s.state != Copying && s.state != Drawing) s = Slot{};
        wake.notify_one();
    }
    void reset() { std::lock_guard<std::mutex> lock(mutex); resetLocked(); }
    bool start(int drm, const int sources[4], Painter draw, unsigned int delayMs,
               bool clean = false, bool exact = false, bool yoloHold = false,
               TimedPainter timed = nullptr) {
        gpu = vdma_sync_gpu_create(drm,sources);
        if (!gpu) return false;
        painter = draw; cleanOnly = clean; exactOnly = exact; holdYolo = yoloHold;
        timedPainter = timed;
        delayNs = uint64_t(delayMs)*1000000;
        paintNs = exactOnly ? delayNs-5000000 :
            delayNs > 50000000 ? delayNs-50000000 : 0;
        enabled = true;
        worker = std::thread([this]{ run(); });
        std::printf("SAME_FRAME_INIT delay_ms=%u paint_ms=%llu slots=%u bytes=%u road_no_result=clear default=off mode=%s "
            "yolo_osd=%s yolo_hold_ms=100 yolo_hold_frames=6 backlog=%s\n",
            delayMs,static_cast<unsigned long long>(paintNs/1000000),VDMA_SYNC_SLOTS,
            VDMA_SYNC_SLOTS*VDMA_SYNC_BYTES,exactOnly ? "exact" :
            cleanOnly ? "clean_copy" : "legacy_baked",holdYolo ? "hold" : "exact",
            exactOnly && holdYolo ? "recover" : "strict");
        return true;
    }
    int capture(const vdma_buffer &b, const vdma_frame_identity &id, uint64_t observed) {
        std::lock_guard<std::mutex> lock(mutex);
        if (error || stopping) { errno = error ? error : EPIPE; return -1; }
        if (!observed || id.slot >= 4 || id.slot != b.slot || id.token != b.token ||
            id.frame_sequence != b.header.sequence) { errno = EPROTO; return -1; }
        if (latest.stream_epoch && !context(latest,id)) resetLocked();
        latest = id;
        dropBacklogLocked(observed);
        unsigned int occupied = 0;
        for (const auto &s : slots) occupied += s.state != Free;
        if (occupied+1 > maxOccupied) maxOccupied = occupied+1;
        for (auto &s : slots) if (s.state == Free) {
            s.id = id; s.header = b.header; s.observed = observed;
            if (recentRoadValid && matches(recentRoad,id))
                s.road = std::make_shared<road_overlay_geometry>(recentRoad);
            if (recentDetectionValid && matches(recentDetection,id))
                s.detection = std::make_shared<detection_overlay>(recentDetection);
            s.order = ++nextOrder; s.generation = generation; s.state = Queued;
            ++captured; wake.notify_one(); return 0;
        }
        ++full; errno = ENOBUFS; return -1;
    }
    bool holds(const vdma_buffer &b) {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto &s : slots)
            if ((s.state == Queued || s.state == Copying) &&
                s.id.slot == b.slot && s.id.token == b.token) return true;
        return false;
    }
    void road(const road_overlay_geometry &r) {
        if (cleanOnly) return;
        std::lock_guard<std::mutex> lock(mutex);
        recentRoad = r; recentRoadValid = true;
        for (auto &s : slots) if (s.generation == generation && matches(r,s.id)) {
            if (s.state == Queued || s.state == Copying || s.state == Waiting) {
                if (s.road && s.road->observed_ns == r.observed_ns && s.road->finished_ns == r.finished_ns) return;
                s.road = std::make_shared<road_overlay_geometry>(r); wake.notify_one();
            } else if (!s.road) ++late;
            return;
        }
    }
    void detection(const detection_overlay &d) {
        if (cleanOnly) return;
        std::lock_guard<std::mutex> lock(mutex);
        if (holdYolo) {
            if (!d.valid || d.state != DETECTION_RUNNING || d.count > DETECTION_OVERLAY_CAPACITY) {
                ++detectionHistoryClears;
                detectionHistory = {}; detectionNext = 0;
                for (auto &s : slots)
                    if (s.state == Queued || s.state == Copying || s.state == Waiting) s.detection.reset();
            } else if (!latest.stream_epoch ||
                       (d.stream_epoch == latest.stream_epoch && d.source_id == latest.source_id &&
                        d.layout == latest.layout && d.config_generation == latest.config_generation)) {
                bool duplicate = false;
                for (const auto &entry : detectionHistory)
                    duplicate |= entry && entry->stream_epoch == d.stream_epoch &&
                        entry->source_id == d.source_id && entry->layout == d.layout &&
                        entry->config_generation == d.config_generation && entry->frame_sequence == d.frame_sequence;
                if (!duplicate) {
                    detectionHistory[detectionNext] = std::make_shared<const detection_overlay>(d);
                    detectionNext = (detectionNext+1)%detectionHistory.size();
                }
            }
        }
        recentDetection = d; recentDetectionValid = true;
        for (auto &s : slots) if (s.generation == generation && matches(d,s.id)) {
            if (s.state == Queued || s.state == Copying || s.state == Waiting) {
                if (s.detection && s.detection->observed_ns == d.observed_ns) return;
                s.detection = std::make_shared<detection_overlay>(d); wake.notify_one();
            } else if (!s.detection) ++detectionLate;
            return;
        }
    }
    /* Pin before entering DRM: the worker can reclaim Ready buffers while
     * the owner waits for the UI mutex or a slow atomic ioctl. */
    int peek(uint64_t stamp, uint32_t *fb, vdma_frame_identity *id, vdma_frame_header *header) {
        std::lock_guard<std::mutex> lock(mutex);
        if (error) { errno = error; return -1; }
        if (reserved >= 0 || pending >= 0) return 0;
        int oldest = -1;
        for (int i = 0; i < VDMA_SYNC_SLOTS; ++i) {
            const auto &s = slots[i];
            if (s.generation != generation || s.state == Free || s.state == Front || s.state == Pending) continue;
            if (oldest < 0 || s.order < slots[oldest].order) oldest = i;
        }
        if (oldest < 0 || slots[oldest].observed+delayNs > stamp) return 0;
        auto &s = slots[oldest];
        if (s.state != Ready) return 0;
        /* Ink was checked for its planned flip; a missed deadline cannot
         * silently display expired ink from an already composed buffer. */
        if (!exactOnly && s.until && stamp+20000000 >= s.until) { errno = ETIME; return -1; }
        *fb = vdma_sync_gpu_framebuffer(gpu,unsigned(oldest)); *id = s.id; *header = s.header;
        s.state = Reserved; reserved = oldest;
        return oldest+1;
    }
    void cancel(int index) {
        std::lock_guard<std::mutex> lock(mutex);
        if (index < 0 || index >= VDMA_SYNC_SLOTS || reserved != index ||
            slots[index].state != Reserved) {
            std::fprintf(stderr,"SAME_FRAME_STATE_ERROR stage=cancel slot=%d reserved=%d pending=%d\n",
                index,reserved,pending);
            error = EPROTO; return;
        }
        auto &s = slots[index];
        if (s.generation != generation || stopping) s = Slot{};
        else s.state = Ready;
        reserved = -1; wake.notify_one();
    }
    void submit(int index) {
        std::lock_guard<std::mutex> lock(mutex);
        if (index < 0 || index >= VDMA_SYNC_SLOTS || reserved != index ||
            slots[index].state != Reserved || pending >= 0) {
            std::fprintf(stderr,"SAME_FRAME_STATE_ERROR stage=submit slot=%d state=%d reserved=%d pending=%d\n",
                index,index >= 0 && index < VDMA_SYNC_SLOTS ? int(slots[index].state) : -1,
                reserved,pending);
            error = EPROTO; return;
        }
        slots[index].state = Pending; pending = index; reserved = -1;
    }
    void complete(uint64_t stamp) {
        std::lock_guard<std::mutex> lock(mutex);
        if (pending < 0) {
            std::fprintf(stderr,"SAME_FRAME_STATE_ERROR stage=complete reserved=%d pending=%d\n",reserved,pending);
            error = EPROTO; return;
        }
        if (front >= 0) slots[front] = Slot{};
        front = pending; pending = -1; auto &s = slots[front]; s.state = Front;
        ++presented; roadFrames += s.roadInk; detectionFrames += s.detectionInk;
        detectionHeldFrames += s.detectionInk && s.detectionHeld;
        detectionExactFrames += s.detectionInk && !s.detectionHeld;
        ++detectionReasons[s.detectionReason];
        missingRoad += !s.roadInk;
        const uint64_t age = stamp-s.observed; ageNs += age; if (age > ageMax) ageMax = age;
        if (sequenceValid) {
            const uint32_t delta = s.id.frame_sequence-uint32_t(lastSequence);
            if (!delta || delta >= 0x80000000U) ++repeat;
            else gaps += delta-1;
        }
        sequenceValid = true; lastSequence = s.id.frame_sequence;
        reportLocked(stamp);
    }
    void run() {
        std::unique_lock<std::mutex> lock(mutex);
        for (;;) {
            dropBacklogLocked(now());
            bool copy = false;
            const int index = nextWorkLocked(now(),copy);
            if (index < 0) {
                if (stopping || error) break;
                wake.wait_for(lock,std::chrono::milliseconds(1)); continue;
            }
            auto &s = slots[index]; s.state = copy ? Copying : Drawing;
            const auto id = s.id;
            const uint64_t observed = s.observed, drawingGeneration = s.generation;
            const auto road = s.road; auto detection = s.detection;
            const uint64_t validAt = holdYolo ?
                std::max(s.observed+delayNs+25000000,now()+20000000) :
                s.observed+delayNs+25000000;
            DetectionReason reason = detection ? Hidden : NoCandidate;
            if (!copy && holdYolo) detection = heldDetectionLocked(id,s.observed,validAt,&reason);
            const bool roadValid = !copy && road && matches(*road,id) &&
                road->observed_ns <= now() && (exactOnly || validAt < road->valid_until_ns);
            const bool detectionValid = !copy && detection && detection->count <= DETECTION_OVERLAY_CAPACITY &&
                (holdYolo ? holdMatches(*detection,id) : matches(*detection,id)) &&
                detection->observed_ns <= now() &&
                (holdYolo ? validAt < detectionDeadline(*detection) : exactOnly || validAt < detection->valid_until_ns);
            /* A held light box was propagated across input frames by a
             * visual tracker. Strict mode admits current detections only. */
            detection_overlay exact{};
            if (detectionValid) {
                exact = *detection;
                for (uint32_t i = 0; i < exact.count; ++i) {
                    if (exact.boxes[i].display_held || (exact.light_temporal_valid && (exact.light_flags[i] & 1U)))
                        exact.boxes[i].x1 = exact.boxes[i].x0;
                    if (exact.light_temporal_valid && exact.light_deadline_ns[i] <= validAt)
                        exact.boxes[i].light_color = DETECTION_LIGHT_UNKNOWN;
                }
            }
            lock.unlock();
            const uint64_t started = now();
            int rc = 0;
            unsigned int painted = 0;
            if (copy) rc = vdma_sync_gpu_copy(gpu,unsigned(index),id.slot);
            else if (roadValid || detectionValid) {
                rc = vdma_sync_gpu_access(gpu,unsigned(index),1);
                if (!rc) {
                    if (timedPainter)
                        painted = timedPainter(vdma_sync_gpu_pixels(gpu,unsigned(index)),id,
                            roadValid ? road.get() : nullptr,detectionValid ? &exact : nullptr,
                            observed,drawingGeneration);
                    else painted = painter(vdma_sync_gpu_pixels(gpu,unsigned(index)),id,
                            roadValid ? road.get() : nullptr,detectionValid ? &exact : nullptr);
                    rc = vdma_sync_gpu_access(gpu,unsigned(index),0);
                }
            } else if (timedPainter)
                timedPainter(nullptr,id,nullptr,nullptr,observed,drawingGeneration);
            const int saved = errno;
            const uint64_t elapsed = now()-started;
            lock.lock();
            if (rc) error = saved ? saved : EIO;
            if (copy) { ++copied; copyNs += elapsed; if (elapsed > copyMax) copyMax = elapsed; }
            else { ++rendered; drawNs += elapsed; if (elapsed > drawMax) drawMax = elapsed; }
            if (s.generation != generation || stopping) s = Slot{};
            else if (copy) s.state = cleanOnly ? Ready : Waiting;
            else {
                s.roadInk = roadValid && (painted & 1U); s.detectionInk = detectionValid && (painted & 2U);
                s.detectionReason = s.detectionInk ? Ink : reason;
                s.detectionHeld = s.detectionInk && detection->frame_sequence != id.frame_sequence;
                s.until = s.roadInk ? road->valid_until_ns : 0;
                if (s.detectionInk && (!s.until || exact.valid_until_ns < s.until)) s.until = exact.valid_until_ns;
                s.state = Ready;
            }
            wake.notify_all();
            if (error) break;
        }
    }
    void stop() {
        if (!worker.joinable()) return;
        { std::lock_guard<std::mutex> lock(mutex); stopping = true; wake.notify_all(); }
        worker.join();
    }
    void close(bool detached) {
        stop();
        if (!enabled) return;
        std::printf("SAME_FRAME_FINAL captured=%llu copied=%llu rendered=%llu presented=%llu "
            "road=%llu detection=%llu missing_road=%llu late=%llu full=%llu "
            "gaps=%llu repeats=%llu max_occupied=%llu resets=%llu "
            "copy_ms=%.3f copy_max_ms=%.3f draw_ms=%.3f draw_max_ms=%.3f "
            "age_ms=%.3f age_max_ms=%.3f error=%d detached=%d mode=%s "
            "yolo_osd=%s detection_exact=%llu detection_held=%llu detection_late=%llu detection_history_clears=%llu "
            "backlog_recoveries=%llu backlog_dropped=%llu\n",
            (unsigned long long)captured,(unsigned long long)copied,(unsigned long long)rendered,
            (unsigned long long)presented,(unsigned long long)roadFrames,(unsigned long long)detectionFrames,
            (unsigned long long)missingRoad,(unsigned long long)late,(unsigned long long)full,
            (unsigned long long)gaps,(unsigned long long)repeat,(unsigned long long)maxOccupied,
            (unsigned long long)resets,copyNs/1e6/(copied ? copied : 1),copyMax/1e6,
            drawNs/1e6/(rendered ? rendered : 1),drawMax/1e6,
            ageNs/1e6/(presented ? presented : 1),ageMax/1e6,error,int(detached),
            exactOnly ? "exact" : cleanOnly ? "clean_copy" : "legacy_baked",holdYolo ? "hold" : "exact",
            (unsigned long long)detectionExactFrames,(unsigned long long)detectionHeldFrames,
            (unsigned long long)detectionLate,(unsigned long long)detectionHistoryClears,
            (unsigned long long)backlogRecoveries,(unsigned long long)backlogDropped);
        if (detached) { vdma_sync_gpu_destroy(gpu); gpu = nullptr; }
        /* Failed detach intentionally retains GPU allocations until exit. */
        enabled = false;
    }
    ~DashboardSync() { stop(); }
};
#endif
