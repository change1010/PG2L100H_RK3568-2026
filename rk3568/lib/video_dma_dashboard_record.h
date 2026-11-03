#ifndef VIDEO_DMA_DASHBOARD_RECORD_H
#define VIDEO_DMA_DASHBOARD_RECORD_H
#include <QFileInfo>
#include <QProcess>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#include <poll.h>
#include <unistd.h>
#include <sys/mman.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include "video_dma_osd_probe.h"

/* Capture the composed CRTC with the hardware writeback connector. Encoding
 * never borrows a scanout slot and a slow encoder only drops recording frames. */
class DashboardRecord {
    static constexpr unsigned width = 960, height = 540, hz = 10;
    int drm = -1;
    uint32_t crtc = 0, connector = 0, crtcProperty = 0, fbProperty = 0, fenceProperty = 0;
    uint32_t handle = 0, framebuffer = 0, pitch = 0;
    uint64_t bytes = 0;
    void *pixels = nullptr;
    QString path;
    std::atomic<bool> stopping{false}, failed{false};
    std::atomic<unsigned> captured{0}, encoded{0}, dropped{0}, busy{0}, flipRetries{0};
    std::thread captureWorker, encodeWorker;
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<std::vector<char>> queue;
    bool captureDone = false, encoderReady = false, reserved = false, inFlight = false;
    int fence = -1;
    vdma_osd_writeback writeback{};
    uint64_t nextCapture = 0;
    uint64_t firstCapture = 0, lastCapture = 0;

    static uint64_t nowNs()
    {
        return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }
    bool findConnector()
    {
        if (drmSetClientCap(drm, DRM_CLIENT_CAP_WRITEBACK_CONNECTORS, 1)) return false;
        drmModeRes *resources = drmModeGetResources(drm);
        if (!resources) return false;
        for (int i = 0; i < resources->count_connectors && !connector; ++i) {
            drmModeConnector *candidate = drmModeGetConnector(drm, resources->connectors[i]);
            if (!candidate) continue;
            if (candidate->connector_type == DRM_MODE_CONNECTOR_WRITEBACK) {
                drmModeObjectProperties *props = drmModeObjectGetProperties(
                    drm, candidate->connector_id, DRM_MODE_OBJECT_CONNECTOR);
                if (props) {
                    for (uint32_t j = 0; j < props->count_props; ++j) {
                        drmModePropertyRes *p = drmModeGetProperty(drm, props->props[j]);
                        if (!p) continue;
                        if (!std::strcmp(p->name, "CRTC_ID")) crtcProperty = p->prop_id;
                        if (!std::strcmp(p->name, "WRITEBACK_FB_ID")) fbProperty = p->prop_id;
                        if (!std::strcmp(p->name, "WRITEBACK_OUT_FENCE_PTR")) fenceProperty = p->prop_id;
                        drmModeFreeProperty(p);
                    }
                    drmModeFreeObjectProperties(props);
                }
                if (crtcProperty && fbProperty && fenceProperty) connector = candidate->connector_id;
            }
            drmModeFreeConnector(candidate);
        }
        drmModeFreeResources(resources);
        return connector != 0;
    }
    bool allocate()
    {
        drm_mode_create_dumb create{};
        create.width = width; create.height = height * 3 / 2; create.bpp = 8;
        if (drmIoctl(drm, DRM_IOCTL_MODE_CREATE_DUMB, &create)) return false;
        handle = create.handle; pitch = create.pitch; bytes = create.size;
        if (pitch < width || bytes < uint64_t(pitch) * height * 3 / 2) return false;
        drm_mode_map_dumb map{}; map.handle = handle;
        if (drmIoctl(drm, DRM_IOCTL_MODE_MAP_DUMB, &map)) return false;
        pixels = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, drm, map.offset);
        if (pixels == MAP_FAILED) { pixels = nullptr; return false; }
        uint32_t handles[4] = {handle, handle}, pitches[4] = {pitch, pitch};
        uint32_t offsets[4] = {0, pitch * height};
        return !drmModeAddFB2(drm, width, height, DRM_FORMAT_NV12,
                              handles, pitches, offsets, &framebuffer, 0);
    }
    void encode()
    {
        QProcess process;
        process.setProcessChannelMode(QProcess::ForwardedErrorChannel);
        process.start("/usr/bin/ffmpeg", {
            "-hide_banner", "-nostdin", "-n", "-loglevel", "warning",
            "-use_wallclock_as_timestamps", "1", "-f", "rawvideo", "-pixel_format", "nv12",
            "-video_size", "960x540", "-framerate", "10", "-i", "pipe:0",
            "-an", "-c:v", "libx264", "-threads", "1", "-preset", "ultrafast",
            "-crf", "23", "-pix_fmt", "yuv420p", "-vsync", "vfr", "-movflags", "+faststart", path});
        const bool started = process.waitForStarted(5000);
        {
            std::lock_guard<std::mutex> lock(mutex);
            encoderReady = true;
            if (!started) failed = true;
            wake.notify_all();
        }
        if (!started) return;
        for (;;) {
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait(lock, [&]{ return captureDone || !queue.empty(); });
            if (queue.empty()) break;
            auto frame = std::move(queue.front()); queue.pop_front(); lock.unlock();
            if (process.write(frame.data(), qint64(frame.size())) != qint64(frame.size())) {
                failed = true; break;
            }
            while (process.bytesToWrite() > 0)
                if (!process.waitForBytesWritten(2000)) { failed = true; break; }
            if (failed) break;
            encoded++;
        }
        process.closeWriteChannel();
        if (!process.waitForFinished(10000)) {
            process.kill(); process.waitForFinished(3000); failed = true;
        }
        if (process.exitStatus() != QProcess::NormalExit || process.exitCode()) failed = true;
    }
    void capture()
    {
        for (;;) {
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait(lock, [&]{ return stopping || failed || inFlight; });
            if (!inFlight) break;
            const int pending = fence;
            lock.unlock();
            pollfd wait{pending, POLLIN, 0};
            int ready;
            do { ready = poll(&wait, 1, 1000); } while (ready < 0 && errno == EINTR);
            if (ready != 1 || !(wait.revents & POLLIN)) {
                std::fprintf(stderr, "SCREEN_RECORD writeback fence timeout/error\n");
                if (pending >= 0) close(pending);
                failed = true; break;
            }
            close(pending);
            const uint64_t stamp = nowNs();
            std::vector<char> frame(width * height * 3 / 2);
            for (unsigned y = 0; y < height * 3 / 2; ++y)
                std::memcpy(frame.data() + y * width, static_cast<char *>(pixels) + y * pitch, width);
            if (!firstCapture) firstCapture = stamp;
            lastCapture = stamp; captured++;
            lock.lock();
            fence = -1; inFlight = false;
            queue.push_back(std::move(frame)); wake.notify_all();
        }
        std::lock_guard<std::mutex> lock(mutex); captureDone = true; wake.notify_all();
    }
public:
    void configure(const QString &output) { path = output; }
    const vdma_osd_writeback *prepare()
    {
        if (path.isEmpty() || drm < 0 || stopping || failed) return nullptr;
        std::lock_guard<std::mutex> lock(mutex);
        const uint64_t now = nowNs();
        if (reserved || inFlight || now < nextCapture) return nullptr;
        if (queue.size() >= 2) { dropped++; nextCapture = now + UINT64_C(100000000); return nullptr; }
        reserved = true; fence = -1; return &writeback;
    }
    void submitted(int result, int error)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!reserved) return;
        reserved = false;
        if (!result && fence >= 0) {
            inFlight = true; nextCapture = nowNs() + UINT64_C(1000000000) / hz;
            wake.notify_all(); return;
        }
        if (fence >= 0) close(fence);
        fence = -1;
        if (result && error == EBUSY) busy++;
        else failed = true;
    }
    bool retryFlip(int error)
    {
        if (path.isEmpty() || drm < 0 || error != EBUSY) return false;
        flipRetries++; return true;
    }
    bool start(int ownerDrm, uint32_t ownerCrtc)
    {
        if (path.isEmpty()) return true;
        if (QFileInfo::exists(path) || !QFileInfo("/usr/bin/ffmpeg").isExecutable()) return false;
        drm = ownerDrm; crtc = ownerCrtc;
        if (!findConnector() || !allocate()) { failed = true; return false; }
        writeback = {connector, crtcProperty, fbProperty, fenceProperty, framebuffer, &fence};
        nextCapture = nowNs() + UINT64_C(2000000000);
        encodeWorker = std::thread([this]{ encode(); });
        {
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait(lock, [&]{ return encoderReady; });
        }
        if (failed) return false;
        captureWorker = std::thread([this]{ capture(); });
        std::printf("SCREEN_RECORD_START physical_writeback=1 connector=%u crtc=%u size=960x540 target_hz=10 file=%s\n",
            connector, crtc, qPrintable(path));
        return true;
    }
    bool stop()
    {
        if (drm < 0) return true;
        stopping = true;
        wake.notify_all();
        if (captureWorker.joinable()) captureWorker.join();
        {
            std::lock_guard<std::mutex> lock(mutex); captureDone = true; wake.notify_all();
        }
        if (encodeWorker.joinable()) encodeWorker.join();
        if (connector) {
            drmModeAtomicReq *request = drmModeAtomicAlloc();
            if (request) {
                if (drmModeAtomicAddProperty(request, connector, crtcProperty, 0) < 0 ||
                    drmModeAtomicCommit(drm, request, DRM_MODE_ATOMIC_ALLOW_MODESET, nullptr))
                    failed = true;
                drmModeAtomicFree(request);
            } else failed = true;
        }
        if (framebuffer && drmModeRmFB(drm, framebuffer)) failed = true;
        if (pixels) munmap(pixels, bytes);
        if (handle) {
            drm_mode_destroy_dumb destroy{}; destroy.handle = handle;
            if (drmIoctl(drm, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy)) failed = true;
        }
        std::printf("SCREEN_RECORD_SUMMARY physical_writeback=1 captured=%u encoded=%u queue_drops=%u busy=%u flip_retries=%u failed=%u first_ns=%llu last_ns=%llu\n",
            captured.load(), encoded.load(), dropped.load(), busy.load(), flipRetries.load(),
            unsigned(failed.load()),
            static_cast<unsigned long long>(firstCapture), static_cast<unsigned long long>(lastCapture));
        drm = -1; return !failed && captured > 0;
    }
};
#endif
