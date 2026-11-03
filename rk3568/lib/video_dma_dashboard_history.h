#ifndef VIDEO_DMA_DASHBOARD_HISTORY_H
#define VIDEO_DMA_DASHBOARD_HISTORY_H
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
extern "C" {
#include "video_dma_presentation.h"
#include "road_overlay_geometry.h"
#include "detection_overlay.h"
}

/* Only presentation owns this history. No mutation reaches the ADAS store.
 * Producer callbacks try the lock once; rendering never stalls a producer. */
class DashboardHistory {
    template<class T> using Ring = std::array<std::shared_ptr<const T>,32>;
    Ring<road_overlay_geometry> roads{};
    Ring<detection_overlay> detections{};
    size_t roadNext = 0, detectionNext = 0;
    std::mutex mutex;
    vdma_frame_identity context{};
    template<class T> bool matches(const T &r, const vdma_frame_identity &f) const {
        return r.stream_epoch == f.stream_epoch && r.source_id == f.source_id &&
            r.layout == f.layout && r.config_generation == f.config_generation;
    }
    template<class T> void publish(Ring<T> &ring, size_t &next, const T &value) {
        std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
        if (!lock.owns_lock()) { ++busy; return; }
        if (!context.stream_epoch || !matches(value,context) ||
            !value.observed_ns || value.valid_until_ns <= value.observed_ns) {
            ++rejected; return;
        }
        for (const auto &item : ring)
            if (item && item->frame_sequence == value.frame_sequence) { ++duplicates; return; }
        try {
            auto owned = std::make_shared<const T>(value);
            overwritten += bool(ring[next]);
            ring[next] = std::move(owned); next = (next+1)%ring.size(); ++accepted;
        } catch (const std::bad_alloc &) { ++allocationFailures; }
    }
    template<class T> std::shared_ptr<const T> exact(const Ring<T> &ring,
        const vdma_frame_identity &f,uint64_t effective,uint64_t hold) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!vdma_same_context(&context,&f) || hold > UINT64_MAX-effective) return {};
        for (const auto &item : ring)
            if (item && item->valid && matches(*item,f) &&
                item->frame_sequence == f.frame_sequence && item->observed_ns <= effective &&
                effective+hold < item->valid_until_ns) return item;
        return {};
    }
public:
    std::atomic<uint64_t> accepted{0}, rejected{0}, duplicates{0}, busy{0};
    std::atomic<uint64_t> overwritten{0}, resets{0}, allocationFailures{0};
    void setContext(const vdma_frame_identity &f) {
        std::lock_guard<std::mutex> lock(mutex);
        if (vdma_same_context(&context,&f)) return;
        roads = Ring<road_overlay_geometry>{}; detections = Ring<detection_overlay>{};
        context = f; roadNext = detectionNext = 0; ++resets;
    }
    void road(const road_overlay_geometry &r) { publish(roads,roadNext,r); }
    void detection(const detection_overlay &d) { publish(detections,detectionNext,d); }
    std::shared_ptr<const road_overlay_geometry> road(const vdma_frame_identity &f,
        uint64_t effective,uint64_t hold) { return exact(roads,f,effective,hold); }
    std::shared_ptr<const detection_overlay> detection(const vdma_frame_identity &f,
        uint64_t effective,uint64_t hold) { return exact(detections,f,effective,hold); }
};
#endif
