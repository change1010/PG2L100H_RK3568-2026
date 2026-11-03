#ifndef VIDEO_DMA_DASHBOARD_CROSSWALK_DISPLAY_H
#define VIDEO_DMA_DASHBOARD_CROSSWALK_DISPLAY_H
#include "road_overlay_geometry.h"
#include "video_dma_snapshot_uapi.h"
#include "video_dma_dashboard_one_euro.h"
#include <algorithm>
#include <cmath>

struct DisplayedCrosswalk {
    enum Reason { Disabled, Missing, Empty, Invalid, Pending, NewTarget, Filtered,
                  SupportedIntersection, Replaced, Duplicate, OldFrame, SceneReset };
    bool visible = false;
    road_overlay_box box{};
    vdma_frame_identity displayed_frame{}, contributing_frame{};
    uint64_t observed_ns = 0;
    Reason reason = Missing;
};

/* Single observed crosswalk only. Raw publications never change. No output on
 * missing/empty input; geometry memory exists for association, not bridging. */
class DashboardCrosswalkDisplay {
    using Box = road_overlay_box;
    vdma_frame_identity prior{};
    uint64_t time = 0, measured = 0, pendingAt = 0, control = 0, owner = 0;
    Box shown{}, previousRaw{}, candidate{};
    bool bound = false, tracking = false, hasPending = false, lastPresent = false;
    unsigned votes = 0;
    DashboardOneEuro filters[4];
    DisplayedCrosswalk last{};
    static double area(const Box &b) {
        return double(std::max(0,b.x1-b.x0))*std::max(0,b.y1-b.y0);
    }
    static Box intersect(const Box &a, const Box &b) {
        return {std::max(a.x0,b.x0),std::max(a.y0,b.y0),
                std::min(a.x1,b.x1),std::min(a.y1,b.y1)};
    }
    static double delta(const Box &a, const Box &b) {
        return std::max(std::max(std::abs(double(a.x0)-b.x0),std::abs(double(a.x1)-b.x1)),
            std::max(std::abs(double(a.y0)-b.y0),std::abs(double(a.y1)-b.y1)));
    }
    static bool context(const vdma_frame_identity &a, const vdma_frame_identity &b) {
        return a.stream_epoch == b.stream_epoch && a.source_id == b.source_id &&
            a.layout == b.layout && a.config_generation == b.config_generation &&
            a.canvas_width == b.canvas_width && a.canvas_height == b.canvas_height;
    }
    static bool same(const road_overlay_geometry &r, const vdma_frame_identity &id) {
        return r.valid && r.stream_epoch == id.stream_epoch && r.source_id == id.source_id &&
            r.layout == id.layout && r.config_generation == id.config_generation &&
            r.frame_sequence == id.frame_sequence;
    }
    static void vector(const Box &b, double v[4]) {
        v[0] = (b.x0+double(b.x1))/3840; v[1] = (b.y0+double(b.y1))/2160;
        v[2] = std::log((b.x1-double(b.x0))/1920);
        v[3] = std::log((b.y1-double(b.y0))/1080);
    }
    void seed(const Box &b) {
        shown = previousRaw = b; tracking = true;
        double v[4]; vector(b,v);
        for (int i = 0; i < 4; ++i) filters[i].reset(v[i]);
    }
    bool confirm(const Box &b, uint64_t ns, unsigned required) {
        const Box common = intersect(candidate,b);
        const double small = std::min(area(candidate),area(b));
        const bool initialNested = required == 2 && hasPending && ns-pendingAt <= 100000000 &&
            delta(candidate,b) > 12 && small > 0 && area(common)/small >= .9;
        if (initialNested) {
            candidate = common;
            if (++votes < required) return false;
            hasPending = false; votes = 0; return true;
        }
        if (!hasPending || ns-pendingAt > 100000000 || delta(candidate,b) > 12) {
            candidate = b; pendingAt = ns; votes = 0; hasPending = true;
        }
        candidate = b;
        if (++votes < required) return false;
        hasPending = false; votes = 0; return true;
    }
public:
    // Normalized units; frozen after raw/EMA/One-Euro development replay.
    double minimum = 4, centerBeta = 80, scaleBeta = 4;
    uint64_t updates = 0, suppressed = 0, replaced = 0, resets = 0;
    void clear() {
        bound = tracking = hasPending = lastPresent = false;
        time = measured = pendingAt = 0; votes = 0; last = DisplayedCrosswalk{};
        ++resets;
    }
    DisplayedCrosswalk update(const road_overlay_geometry *raw, const vdma_frame_identity &id,
        uint64_t ns, uint64_t revision, uint64_t generation, bool enabled = true, bool sceneCut = false) {
        DisplayedCrosswalk out; out.displayed_frame = id; out.observed_ns = ns;
        if (!enabled) { clear(); out.reason = DisplayedCrosswalk::Disabled; return out; }
        if (sceneCut || (bound && (!context(prior,id) || control != revision || owner != generation))) clear();
        if (!ns || (bound && (ns < time ||
            uint32_t(id.frame_sequence-prior.frame_sequence) >= 0x80000000U))) {
            out.reason = DisplayedCrosswalk::OldFrame; return out;
        }
        bool present = raw && same(*raw,id) && raw->zebra_count == 1;
        if (bound && id.frame_sequence == prior.frame_sequence) {
            // An absent result is a barrier even if it repeats the last identity.
            if (!present) { hasPending = false; lastPresent = false; last.visible = false; }
            if (present && lastPresent) {
                out = last; out.reason = DisplayedCrosswalk::Duplicate;
            } else out.reason = DisplayedCrosswalk::Duplicate;
            return out;
        }
        if (bound && (ns <= time || ns-time > 100000000)) clear();
        if (tracking && ns-measured > 80000000) { tracking = hasPending = false; votes = 0; }
        const double dt = measured && ns > measured ? (ns-measured)/1e9 : 0;
        prior = id; time = ns; control = revision; owner = generation; bound = true; ++updates;
        if (!present) {
            hasPending = false; votes = 0; lastPresent = false;
            out.reason = !raw ? DisplayedCrosswalk::Missing :
                !raw->zebra_count ? DisplayedCrosswalk::Empty : DisplayedCrosswalk::Invalid;
            last = out; return out;
        }
        const auto &r = raw->zebras[0];
        // Reject malformed/inverted/absurd coordinates before arithmetic.
        if (r.x0 < 0 || r.x0 > 1919 || r.y0 < 0 || r.y0 > 1079 ||
            r.x1 < 0 || r.x1 > 1920 || r.y1 < 0 || r.y1 > 1080 ||
            r.x1-r.x0 < 4 || r.y1-r.y0 < 4) {
            hasPending = false; lastPresent = false;
            out.reason = DisplayedCrosswalk::Invalid; last = out; return out;
        }
        out.contributing_frame = id; lastPresent = true;
        if (!tracking) {
            if (confirm(r,ns,2)) { seed(candidate); measured = ns; out.visible = true; out.box = shown; out.reason = DisplayedCrosswalk::NewTarget; }
            else out.reason = DisplayedCrosswalk::Pending;
        } else {
            const Box common = intersect(shown,r);
            const double small = std::min(area(shown),area(r));
            const double iom = small > 0 ? area(common)/small : 0;
            const double ratio = std::max(area(shown),area(r))/std::max(1.0,small);
            const double dx = std::abs((r.x0+double(r.x1)-previousRaw.x0-previousRaw.x1)*.5);
            const double dy = std::abs((r.y0+double(r.y1)-previousRaw.y0-previousRaw.y1)*.5);
            const bool ordinary = ratio <= 2 && dx <= std::max(24.0,(previousRaw.x1-previousRaw.x0)*.25) &&
                dy <= std::max(24.0,(previousRaw.y1-previousRaw.y0)*.25) &&
                (iom >= .65 || area(common)/(area(shown)+area(r)-area(common)) >= .25);
            if (!ordinary) {
                if (confirm(r,ns,3)) {
                    seed(r); measured = ns; out.visible = true; out.box = shown;
                    out.reason = DisplayedCrosswalk::Replaced; ++replaced;
                } else if (iom >= .9 && area(common) >= 16) {
                    // Only show a region supported by THIS frame. Never retain
                    // the unsupported outer part of an older, larger rectangle.
                    seed(common);
                    out.visible = true; out.box = common;
                    out.reason = DisplayedCrosswalk::SupportedIntersection; ++suppressed;
                    measured = ns;
                } else { out.reason = DisplayedCrosswalk::Pending; ++suppressed; }
            } else {
                hasPending = false; votes = 0;
                double v[4]; vector(r,v);
                for (int i = 0; i < 4; ++i)
                    v[i] = filters[i].update(v[i],dt,minimum,i < 2 ? centerBeta : scaleBeta);
                const double w = std::min(1920.0,1920*std::exp(v[2]));
                const double h = std::min(1080.0,1080*std::exp(v[3]));
                shown = {int32_t(std::lround(std::max(0.0,v[0]*1920-w*.5))),
                    int32_t(std::lround(std::max(0.0,v[1]*1080-h*.5))),
                    int32_t(std::lround(std::min(1920.0,v[0]*1920+w*.5))),
                    int32_t(std::lround(std::min(1080.0,v[1]*1080+h*.5)))};
                previousRaw = r; measured = ns;
                out.box = shown; out.visible = area(shown) > 0; out.reason = DisplayedCrosswalk::Filtered;
            }
        }
        last = out; return out;
    }
};
#endif
