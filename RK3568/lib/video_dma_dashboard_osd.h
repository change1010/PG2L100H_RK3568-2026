#ifndef VIDEO_DMA_DASHBOARD_OSD_H
#define VIDEO_DMA_DASHBOARD_OSD_H
#include <array>
#include <cstdint>
extern "C" {
#include "video_dma_presentation.h"
}

/* Serialize outside this class. Published pixels are immutable until FREE.
 * The pinned clean layer can serve any target without retaining stale ink. */
class DashboardOsd {
public:
    enum State { Free, Drawing, Ready, Pending, Front };
    struct Slot {
        State state = Free;
        bool clean = false;
        uint64_t generation = 0, revision = 0;
        vdma_presentation presentation{};
    };
    std::array<Slot,4> slots{};
    int fallback = -1, front = -1, pending = -1;
    uint64_t generation = 1, revision = 0;
    uint64_t targets = 0, empty = 0, missed = 0, cancelled = 0;

    int acquire() {
        for (unsigned i = 0; i < slots.size(); ++i)
            if (slots[i].state == Free) {
                slots[i] = Slot{};
                slots[i].state = Drawing; slots[i].generation = generation;
                return int(i);
            }
        ++missed; return -1;
    }
    bool cancel(int i) {
        if (!is(i,Drawing)) return false;
        slots[i] = Slot{}; ++cancelled; return true;
    }
    bool cleanReady(int i, uint32_t framebuffer) {
        if (!drawable(i) || !framebuffer) return false;
        auto &s = slots[i];
        s.clean = true; s.revision = ++revision;
        s.presentation = vdma_presentation{};
        s.presentation.overlay_fb = framebuffer; s.state = Ready;
        const int previous = fallback;
        fallback = i;
        if (previous >= 0 && slots[previous].state == Ready) slots[previous] = Slot{};
        return true;
    }
    bool targetReady(int i, const vdma_presentation &p) {
        if (!drawable(i) || !vdma_presentation_valid(&p) ||
            (p.road.kind == VDMA_GEOMETRY_NONE && p.detection.kind == VDMA_GEOMETRY_NONE))
            return false;
        auto &s = slots[i];
        s.presentation = p; s.revision = ++revision; s.state = Ready;
        return true;
    }
    int select(uint32_t videoFb, const vdma_frame_identity &video,
               uint64_t effective, uint64_t hold) const {
        int chosen = -1;
        for (unsigned i = 0; i < slots.size(); ++i) {
            const auto &s = slots[i];
            if (s.clean || s.generation != generation ||
                (s.state != Ready && s.state != Front)) continue;
            auto p = s.presentation; p.effective_ns = effective; p.hold_ns = hold;
            if (p.video_fb == videoFb && vdma_same_frame(&p.video,&video) &&
                vdma_presentation_valid(&p) &&
                (chosen < 0 || s.revision > slots[chosen].revision)) chosen = int(i);
        }
        return chosen >= 0 ? chosen : fallback;
    }
    bool submit(int i) {
        if (pending >= 0 || i < 0 || i >= int(slots.size()) ||
            (slots[i].state != Ready && slots[i].state != Front)) return false;
        slots[i].state = Pending; pending = i;
        if (slots[i].clean) ++empty; else ++targets;
        return true;
    }
    bool complete() {
        if (pending < 0) return false;
        if (front >= 0 && front != pending) {
            if (front == fallback) slots[front].state = Ready;
            else slots[front] = Slot{};
        }
        front = pending; pending = -1; slots[front].state = Front; return true;
    }
    void reset() {
        ++generation;
        for (unsigned i = 0; i < slots.size(); ++i)
            if (int(i) != fallback && slots[i].state == Ready) slots[i] = Slot{};
    }
    void retire(uint32_t videoFb, const vdma_frame_identity &video, uint64_t effective) {
        for (unsigned i = 0; i < slots.size(); ++i) {
            auto &s = slots[i];
            if (int(i) == fallback || s.state != Ready) continue;
            const auto &p = s.presentation;
            const uint32_t delta = video.frame_sequence-p.video.frame_sequence;
            const bool passed = delta > 0 && delta < UINT32_C(0x80000000);
            if (s.generation != generation || !vdma_same_context(&p.video,&video) ||
                passed || (vdma_same_frame(&p.video,&video) && p.video_fb != videoFb) ||
                (p.road.kind && effective >= p.road.valid_until_ns) ||
                (p.detection.kind && effective >= p.detection.valid_until_ns)) s = Slot{};
        }
    }
private:
    bool is(int i, State state) const {
        return i >= 0 && i < int(slots.size()) && slots[i].state == state;
    }
    bool drawable(int i) {
        if (!is(i,Drawing)) return false;
        if (slots[i].generation != generation) { cancel(i); return false; }
        return true;
    }
};
#endif
