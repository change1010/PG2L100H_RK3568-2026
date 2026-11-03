#ifndef VIDEO_DMA_SPECIALIST_BATCH_H
#define VIDEO_DMA_SPECIALIST_BATCH_H
#include "adas_signal_test.h"

/* One entry batch supplies both direction caches before or during lane selection.
 * Lane changes never restart its task budget. */
class DashboardSpecialistBatch {
    uint64_t context = 0, perception = 0, first = 0, previous = 0, sequence = 0;
    adas_drive_frame frame{};
    adas_signal_test_observation last{};
    unsigned count = 0;
    void bind(const adas_signal_test &s) {
        if (context == s.context && perception == s.perception &&
            adas_lane_test_same_frame(&frame,&s.frame)) return;
        *this = DashboardSpecialistBatch{};
        context = s.context; perception = s.perception; frame = s.frame;
    }
public:
    bool needed(const adas_signal_test &s, uint64_t now) {
        bind(s);
        return s.active && s.completed_tasks < s.config.stable_tasks &&
            now >= s.started_ns && now-s.started_ns < s.config.wait_ns;
    }
    void observe(const adas_signal_test &s, const adas_signal_test_observation &o) {
        bind(s);
        if (!s.active || o.context != context || o.perception != perception ||
            !o.selection || o.selection > s.selection ||
            !adas_lane_test_same_frame(&o.frame,&frame) ||
            !o.sequence || o.sequence <= sequence) return;
        sequence = o.sequence;
        const unsigned expected = o.layout == ADAS_TEST_THREE_ARROW_CIRCLES ? 3 :
            (o.layout == ADAS_TEST_TWO_ARROW_CIRCLE || o.layout == ADAS_TEST_TWO_CIRCLES) ? 2 : 0;
        bool valid = !o.reason && o.groups_valid == 1 && o.group_signature &&
            expected && o.group_count == expected && o.started_ns >= s.started_ns &&
            o.finished_ns >= o.started_ns;
        if (s.layout_signature) {
            valid &= o.group_signature == s.layout_signature && o.layout == s.layout &&
                o.group_count == s.group_count;
            for (unsigned i = 0; i < expected; ++i) valid &= o.groups[i].id == s.group_ids[i];
        }
        bool same = last.group_signature == o.group_signature && last.layout == o.layout &&
            last.group_count == o.group_count;
        for (unsigned i = 0; i < expected; ++i) {
            const auto &g = o.groups[i], &old = last.groups[i];
            valid &= g.identity_valid == 1 && g.id && g.color_valid == 1 &&
                g.color >= 1 && g.color <= 3;
            for (unsigned j = 0; j < i; ++j) valid &= g.id != o.groups[j].id;
            same &= g.id == old.id && g.color == old.color && g.color_valid == old.color_valid;
        }
        if (!valid) { count = 0; first = previous = 0; last = {}; return; }
        if (!previous || !same || o.finished_ns < previous ||
            o.finished_ns-previous > s.config.maximum_gap_ns) {
            count = 0; first = o.finished_ns;
        }
        if (count < s.config.stable_tasks) ++count;
        previous = o.finished_ns; last = o;
    }
    unsigned stableCount() const { return count; }
};
#endif
