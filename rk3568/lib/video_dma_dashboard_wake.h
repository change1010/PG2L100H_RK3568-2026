#ifndef VIDEO_DMA_DASHBOARD_WAKE_H
#define VIDEO_DMA_DASHBOARD_WAKE_H
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <sys/eventfd.h>
#include <unistd.h>

/* A bounded notification, never a frame queue. Drain before reading UI state:
 * a request merged during drain is covered by that next immutable snapshot. */
class DashboardWake {
    int descriptor = -1;
    std::atomic<bool> pending{false};
public:
    std::atomic<uint64_t> sent{0}, consumed{0}, errors{0};
    ~DashboardWake() { if (descriptor >= 0) close(descriptor); }
    bool start()
    {
        descriptor = eventfd(0,EFD_NONBLOCK|EFD_CLOEXEC);
        return descriptor >= 0;
    }
    int fd() const { return descriptor; }
    void request()
    {
        if (descriptor < 0 || pending.exchange(true)) return;
        const uint64_t value = 1;
        ssize_t bytes;
        do { bytes = write(descriptor,&value,sizeof(value)); } while (bytes < 0 && errno == EINTR);
        if (bytes == sizeof(value)) sent++;
        else { errors++; pending.store(false); }
    }
    bool consume()
    {
        uint64_t value;
        ssize_t bytes;
        do { bytes = read(descriptor,&value,sizeof(value)); } while (bytes < 0 && errno == EINTR);
        pending.store(false);
        if (bytes == sizeof(value)) { consumed++; return true; }
        if (bytes < 0 && errno == EAGAIN) return false;
        errors++; return false;
    }
};
#endif
