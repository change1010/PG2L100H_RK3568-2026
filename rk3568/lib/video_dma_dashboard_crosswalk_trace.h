#ifndef VIDEO_DMA_DASHBOARD_CROSSWALK_TRACE_H
#define VIDEO_DMA_DASHBOARD_CROSSWALK_TRACE_H
#include "video_dma_dashboard_crosswalk_display.h"
#include <array>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Small pre-OSD scene sample; no DMA owner, tracking, or frame retention.
class DashboardCrosswalkScene {
    std::array<unsigned char,576> previous{};
    bool valid = false;
public:
    void clear() { valid = false; }
    bool cut(const void *memory) {
        if (!memory) return false;
        const auto *pixels = static_cast<const uint16_t *>(memory);
        unsigned difference = 0, changed = 0, index = 0;
        for (unsigned y = 30; y < 1080; y += 60)
            for (unsigned x = 30; x < 1920; x += 60) {
                const uint16_t p = pixels[y*1920+x];
                const unsigned luma = (77*((p>>11)*255/31)+150*(((p>>5)&63)*255/63)+
                    29*((p&31)*255/31))/256;
                const unsigned d = unsigned(std::abs(int(luma)-previous[index]));
                difference += d; changed += d > 51;
                previous[index++] = static_cast<unsigned char>(luma);
            }
        const bool result = valid && difference > 576*64 && changed > 576*3/5;
        valid = true; return result;
    }
};

// Opt-in diagnostics are bounded (1200 frames / 3 small pre-OSD images).
// Disk I/O runs on a separate thread. Contention/full queue drops a record.
class DashboardCrosswalkTrace {
    struct Entry {
        vdma_frame_identity id{};
        uint64_t observed = 0;
        bool road = false, raw = false, scene = false;
        road_overlay_box box{};
        DisplayedCrosswalk displayed{};
        double us = 0;
        unsigned index = 0;
        std::vector<unsigned char> rgb;
    };
    std::string prefix;
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<Entry> queue;
    std::thread worker;
    bool stopping = false;
    unsigned submitted = 0, dropped = 0;
    void run() {
        FILE *csv = std::fopen((prefix+".csv").c_str(),"wx");
        if (!csv) return;
        std::fprintf(csv,"epoch,source,layout,config,sequence,observed_ns,road,raw,x0,y0,x1,y1,visible,dx0,dy0,dx1,dy1,reason,scene,update_us\n");
        for (;;) {
            Entry entry;
            {
                std::unique_lock<std::mutex> lock(mutex);
                wake.wait(lock,[this]{ return stopping || !queue.empty(); });
                if (queue.empty()) break;
                entry = std::move(queue.front()); queue.pop_front();
            }
            const auto &b = entry.box; const auto &d = entry.displayed.box;
            std::fprintf(csv,"%u,%u,%u,%u,%u,%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3f\n",
                entry.id.stream_epoch,entry.id.source_id,entry.id.layout,entry.id.config_generation,
                entry.id.frame_sequence,static_cast<unsigned long long>(entry.observed),
                entry.road,entry.raw,b.x0,b.y0,b.x1,b.y1,entry.displayed.visible,
                d.x0,d.y0,d.x1,d.y1,int(entry.displayed.reason),entry.scene,entry.us);
            if (!entry.rgb.empty()) {
                const std::string path = prefix+"-"+std::to_string(entry.id.frame_sequence)+".ppm";
                FILE *image = std::fopen(path.c_str(),"wx");
                if (image) {
                    std::fprintf(image,"P6\n480 270\n255\n");
                    std::fwrite(entry.rgb.data(),1,entry.rgb.size(),image); std::fclose(image);
                }
            }
            // At most 20 seconds of diagnostics; timely readback for validation.
            std::fflush(csv);
        }
        std::fclose(csv);
    }
public:
    DashboardCrosswalkTrace() {
        const char *path = std::getenv("VDMA_CROSSWALK_TRACE");
        if (path && *path) { prefix = path; worker = std::thread([this]{ run(); }); }
    }
    ~DashboardCrosswalkTrace() {
        if (!worker.joinable()) return;
        { std::lock_guard<std::mutex> lock(mutex); stopping = true; wake.notify_one(); }
        worker.join();
        std::printf("CROSSWALK_TRACE submitted=%u dropped=%u bounded=1200\n",submitted,dropped);
    }
    void add(const void *memory, const vdma_frame_identity &id, uint64_t observed,
        const road_overlay_geometry *raw, const DisplayedCrosswalk &displayed, bool scene, double us) {
        if (prefix.empty() || submitted >= 1200) return;
        Entry entry; entry.id = id; entry.observed = observed;
        entry.road = raw; entry.raw = raw && raw->zebra_count == 1;
        if (entry.raw) entry.box = raw->zebras[0];
        entry.displayed = displayed; entry.scene = scene; entry.us = us;
        entry.index = submitted++;
        if (memory && (entry.index == 0 || entry.index == 300 || entry.index == 600)) {
            entry.rgb.resize(480*270*3);
            const auto *pixels = static_cast<const uint16_t *>(memory);
            for (unsigned y = 0; y < 270; ++y)
                for (unsigned x = 0; x < 480; ++x) {
                    const uint16_t p = pixels[(y*4+2)*1920+x*4+2];
                    auto *dst = &entry.rgb[(y*480+x)*3];
                    dst[0] = (p>>11)*255/31; dst[1] = ((p>>5)&63)*255/63; dst[2] = (p&31)*255/31;
                }
        }
        std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
        if (!lock.owns_lock() || queue.size() >= 128) { ++dropped; return; }
        queue.push_back(std::move(entry)); wake.notify_one();
    }
};
#endif
