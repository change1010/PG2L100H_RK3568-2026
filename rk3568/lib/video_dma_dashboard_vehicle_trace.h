#ifndef VIDEO_DMA_DASHBOARD_VEHICLE_TRACE_H
#define VIDEO_DMA_DASHBOARD_VEHICLE_TRACE_H
#include "video_dma_dashboard_vehicle_display.h"
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/* Optional bounded trace. Pixels are sampled only from the existing owner's
 * pre-OSD CPU window. File I/O is never performed on the paint thread. */
class DashboardVehicleTrace {
    struct Entry {
        vdma_frame_identity id{};
        uint64_t observed = 0;
        detection_overlay raw{};
        DisplayedVehicles displayed{};
        double us = 0;
        bool cut = false;
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
        std::fprintf(csv,"kind,video_sequence,video_observed_ns,result_sequence,result_observed_ns,track,class,x0,y0,x1,y1,held,update_us,scene_cut\n");
        for (;;) {
            Entry e;
            {
                std::unique_lock<std::mutex> lock(mutex);
                wake.wait(lock,[this]{ return stopping || !queue.empty(); });
                if (queue.empty()) break;
                e = std::move(queue.front()); queue.pop_front();
            }
            std::fprintf(csv,"F,%u,%llu,%u,%llu,0,0,0,0,0,0,0,%.3f,%u\n",
                e.id.frame_sequence,(unsigned long long)e.observed,e.raw.frame_sequence,
                (unsigned long long)e.raw.observed_ns,e.us,unsigned(e.cut));
            for (unsigned pass = 0; pass < 2; ++pass) {
                const auto &overlay = pass ? e.displayed.overlay : e.raw;
                for (unsigned i = 0; i < std::min(overlay.count,uint32_t(DETECTION_OVERLAY_CAPACITY)); ++i) {
                    const auto &b = overlay.boxes[i];
                    if (!DashboardVehicleDisplay::vehicle(b) || b.x0 >= b.x1 || b.y0 >= b.y1) continue;
                    std::fprintf(csv,"%c,%u,%llu,%u,%llu,%u,%u,%d,%d,%d,%d,%u,%.3f,%u\n",
                        pass ? 'D' : 'R',e.id.frame_sequence,(unsigned long long)e.observed,
                        e.raw.frame_sequence,(unsigned long long)e.raw.observed_ns,
                        pass ? e.displayed.ids[i] : 0,b.model_class,b.x0,b.y0,b.x1,b.y1,
                        b.display_held,e.us,unsigned(e.cut));
                }
            }
            if (!e.rgb.empty()) {
                FILE *image = std::fopen((prefix+"-"+std::to_string(e.id.frame_sequence)+".ppm").c_str(),"wx");
                if (image) {
                    std::fprintf(image,"P6\n480 270\n255\n");
                    std::fwrite(e.rgb.data(),1,e.rgb.size(),image); std::fclose(image);
                }
            }
            std::fflush(csv);
        }
        std::fclose(csv);
    }
public:
    DashboardVehicleTrace() {
        const char *path = std::getenv("VDMA_VEHICLE_TRACE");
        if (path && *path) { prefix = path; worker = std::thread([this]{ run(); }); }
    }
    ~DashboardVehicleTrace() {
        if (!worker.joinable()) return;
        { std::lock_guard<std::mutex> lock(mutex); stopping = true; }
        wake.notify_one(); worker.join();
        std::printf("VEHICLE_TRACE submitted=%u dropped=%u limit=900\n",submitted,dropped);
    }
    void add(const void *memory, const vdma_frame_identity &id, uint64_t observed,
        const detection_overlay *raw, const DisplayedVehicles &displayed, double us, bool cut) {
        if (!worker.joinable() || submitted >= 900) return;
        std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
        if (!lock.owns_lock() || queue.size() >= 128) { ++dropped; return; }
        Entry e; e.id = id; e.observed = observed; e.displayed = displayed; e.us = us; e.cut = cut;
        if (raw) e.raw = *raw;
        if (memory && (submitted == 1 || submitted == 300 || submitted == 600)) {
            e.rgb.resize(480*270*3);
            const auto *pixels = static_cast<const uint16_t *>(memory);
            for (unsigned y = 0; y < 270; ++y) for (unsigned x = 0; x < 480; ++x) {
                const uint16_t p = pixels[(y*4+2)*1920+x*4+2];
                const unsigned j = (y*480+x)*3;
                e.rgb[j] = (p>>11)*255/31; e.rgb[j+1] = ((p>>5)&63)*255/63; e.rgb[j+2] = (p&31)*255/31;
            }
        }
        queue.push_back(std::move(e)); ++submitted; wake.notify_one();
    }
};
#endif
