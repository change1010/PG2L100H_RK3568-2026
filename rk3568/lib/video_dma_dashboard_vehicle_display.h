#ifndef VIDEO_DMA_DASHBOARD_VEHICLE_DISPLAY_H
#define VIDEO_DMA_DASHBOARD_VEHICLE_DISPLAY_H
#include "detection_overlay.h"
#include "video_dma_dashboard_one_euro.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

struct DisplayedVehicles {
    detection_overlay overlay{};
    std::array<uint32_t,DETECTION_OVERLAY_CAPACITY> ids{};
    uint64_t revision = 0;
    unsigned measured = 0, held = 0;
};

/* Display state only: original detections and pedestrian decisions are never
 * changed. Source observation time, not the number of repaints, ages a track. */
class DashboardVehicleDisplay {
    using Box = detection_overlay_box;
    static const unsigned Capacity = 32;
    static const uint64_t HoldNs = 100000000;
    struct Track {
        bool active = false, confirmed = false, seen = false;
        uint32_t id = 0;
        unsigned hits = 0, jumpVotes = 0;
        uint64_t born = 0, measured = 0, deadline = 0, jumpAt = 0;
        Box raw{}, shown{}, jump{};
        DashboardOneEuro filters[4];
    };
    std::array<Track,Capacity> tracks{};
    vdma_frame_identity contextId{};
    uint64_t control = 0, owner = 0, sourceTime = 0, videoTime = 0, revision = 1;
    uint32_t sequence = 0, videoSequence = 0, nextId = 1;
    bool bound = false, haveResult = false;
    static bool sane(const Box &b) {
        return !b.signal && !b.display_held && b.confidence_q10000 <= 10000 &&
            b.x0 >= 0 && b.y0 >= 0 && b.x0 < b.x1 && b.y0 < b.y1 &&
            b.x1 <= 1920 && b.y1 <= 1080;
    }
    static bool sameContext(const vdma_frame_identity &a, const vdma_frame_identity &b) {
        return a.stream_epoch == b.stream_epoch && a.source_id == b.source_id &&
            a.layout == b.layout && a.config_generation == b.config_generation &&
            a.canvas_width == b.canvas_width && a.canvas_height == b.canvas_height;
    }
    static double width(const Box &b) { return double(b.x1)-b.x0; }
    static double height(const Box &b) { return double(b.y1)-b.y0; }
    static double overlap(const Box &a, const Box &b) {
        const double intersection = double(std::max(0,std::min(a.x1,b.x1)-std::max(a.x0,b.x0))) *
            std::max(0,std::min(a.y1,b.y1)-std::max(a.y0,b.y0));
        return intersection/std::max(1.0,width(a)*height(a)+width(b)*height(b)-intersection);
    }
    static double cost(const Box &a, const Box &b) {
        if (a.model_class != b.model_class) return 4;
        const double wr = width(b)/width(a), hr = height(b)/height(a);
        if (wr < .5 || wr > 2 || hr < .5 || hr > 2) return 4;
        const double dx = (a.x0+double(a.x1)-b.x0-b.x1)*.5;
        const double dy = (a.y0+double(a.y1)-b.y0-b.y1)*.5;
        const double distance = std::hypot(dx,dy)/std::hypot(width(a),height(a));
        const double iou = overlap(a,b);
        if (iou < .3 && (distance > .15 || wr < .67 || wr > 1.5 || hr < .67 || hr > 1.5))
            return 4;
        return .7*(1-iou)+.2*std::min(1.0,distance/.15)+
            .1*std::min(1.0,std::abs(std::log(wr))+std::abs(std::log(hr)));
    }
    static void vector(const Box &b, double value[4]) {
        value[0] = (b.x0+double(b.x1))/3840;
        value[1] = (b.y0+double(b.y1))/2160;
        value[2] = std::log(width(b)/1920);
        value[3] = std::log(height(b)/1080);
    }
    static void seed(Track &t, const Box &b) {
        t.shown = b;
        double value[4]; vector(b,value);
        for (unsigned i = 0; i < 4; ++i) t.filters[i].reset(value[i]);
    }
    void measure(Track &t, const Box &b, uint64_t ns, uint64_t deadline) {
        const double dt = t.measured && ns > t.measured ? (ns-t.measured)/1e9 : 0;
        if (!t.measured) seed(t,b);
        else {
            Box accepted = b;
            const bool jump = std::abs(std::log(width(b)/width(t.shown))) > std::log(1.2) ||
                std::abs(std::log(height(b)/height(t.shown))) > std::log(1.2);
            if (jump) {
                if (!t.jumpVotes || ns-t.jumpAt > HoldNs || overlap(t.jump,b) < .8) {
                    t.jump = b; t.jumpAt = ns; t.jumpVotes = 1;
                } else ++t.jumpVotes;
                if (t.jumpVotes < 2) {
                    // Keep the current center; only the suspicious size waits.
                    const double cx = (b.x0+double(b.x1))*.5, cy = (b.y0+double(b.y1))*.5;
                    const double w = width(t.shown), h = height(t.shown);
                    accepted.x0 = int32_t(std::lround(std::max(0.0,cx-w*.5)));
                    accepted.y0 = int32_t(std::lround(std::max(0.0,cy-h*.5)));
                    accepted.x1 = int32_t(std::lround(std::min(1920.0,cx+w*.5)));
                    accepted.y1 = int32_t(std::lround(std::min(1080.0,cy+h*.5)));
                    ++suppressed;
                } else { seed(t,b); t.jumpVotes = 0; }
            } else t.jumpVotes = 0;
            double value[4]; vector(accepted,value);
            for (unsigned i = 0; i < 4; ++i)
                value[i] = t.filters[i].update(value[i],dt,minimum,i < 2 ? centerBeta : scaleBeta);
            const double w = std::min(1920.0,1920*std::exp(value[2]));
            const double h = std::min(1080.0,1080*std::exp(value[3]));
            t.shown = b;
            t.shown.x0 = int32_t(std::lround(std::max(0.0,value[0]*1920-w*.5)));
            t.shown.y0 = int32_t(std::lround(std::max(0.0,value[1]*1080-h*.5)));
            t.shown.x1 = int32_t(std::lround(std::min(1920.0,value[0]*1920+w*.5)));
            t.shown.y1 = int32_t(std::lround(std::min(1080.0,value[1]*1080+h*.5)));
        }
        t.raw = b; t.measured = ns; t.deadline = deadline; t.seen = true;
        if (++t.hits >= 2 && ns-t.born < HoldNs) t.confirmed = true;
        ++measurements;
    }
    /* Rectangular shortest augmenting-path assignment with one private dummy
     * column per track. A forbidden match always costs more than leaving it. */
    static void assign(const double costs[Capacity][DETECTION_OVERLAY_CAPACITY+Capacity],
        unsigned rows, unsigned columns, int matches[Capacity]) {
        double u[Capacity+1]{}, v[DETECTION_OVERLAY_CAPACITY+Capacity+1]{};
        unsigned p[DETECTION_OVERLAY_CAPACITY+Capacity+1]{};
        unsigned way[DETECTION_OVERLAY_CAPACITY+Capacity+1]{};
        for (unsigned i = 1; i <= rows; ++i) {
            p[0] = i;
            unsigned j0 = 0;
            double minimumCost[DETECTION_OVERLAY_CAPACITY+Capacity+1];
            std::fill(std::begin(minimumCost),std::end(minimumCost),1e9);
            bool used[DETECTION_OVERLAY_CAPACITY+Capacity+1]{};
            do {
                used[j0] = true;
                const unsigned i0 = p[j0];
                unsigned j1 = 0; double delta = 1e9;
                for (unsigned j = 1; j <= columns; ++j) if (!used[j]) {
                    const double candidate = costs[i0-1][j-1]-u[i0]-v[j];
                    if (candidate < minimumCost[j]) { minimumCost[j] = candidate; way[j] = j0; }
                    if (minimumCost[j] < delta) { delta = minimumCost[j]; j1 = j; }
                }
                for (unsigned j = 0; j <= columns; ++j) {
                    if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                    else minimumCost[j] -= delta;
                }
                j0 = j1;
            } while (p[j0]);
            do { const unsigned j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
        }
        std::fill(matches,matches+Capacity,-1);
        for (unsigned j = 1; j <= columns; ++j)
            if (p[j]) matches[p[j]-1] = int(j-1);
    }
public:
    double minimum = 4, centerBeta = 80, scaleBeta = 1;
    uint64_t updates = 0, measurements = 0, suppressed = 0, expired = 0, resets = 0;
    static bool vehicle(const Box &b) {
        return !b.signal && (b.model_class == 1 || b.model_class == 2 ||
            b.model_class == 3 || b.model_class == 5 || b.model_class == 7);
    }
    void clear() {
        tracks = {}; bound = haveResult = false; sourceTime = videoTime = 0;
        ++revision; ++resets;
    }
    DisplayedVehicles update(const detection_overlay *raw, const vdma_frame_identity &id,
        uint64_t observed, uint64_t now, uint64_t deadline, uint64_t controlRevision,
        uint64_t ownerGeneration, bool enabled = true, bool sceneCut = false) {
        DisplayedVehicles out;
        if (enabled && !sceneCut && bound && sameContext(contextId,id) &&
            control == controlRevision && owner == ownerGeneration &&
            (observed < videoTime || uint32_t(id.frame_sequence-videoSequence) >= 0x80000000U))
            return out;
        if (!enabled || !observed || !now || !raw || !raw->valid ||
            raw->state != DETECTION_RUNNING || raw->count > DETECTION_OVERLAY_CAPACITY ||
            raw->source_id != id.source_id || raw->layout != id.layout ||
            raw->stream_epoch != id.stream_epoch || raw->config_generation != id.config_generation ||
            !raw->observed_ns || raw->observed_ns > observed || raw->observed_ns > now ||
            observed-raw->observed_ns >= HoldNs || now >= deadline ||
            now >= raw->valid_until_ns || uint32_t(id.frame_sequence-raw->frame_sequence) > 6) {
            clear(); return out;
        }
        if (sceneCut || (bound && (!sameContext(contextId,id) ||
            control != controlRevision || owner != ownerGeneration))) clear();
        if (bound && (observed < videoTime ||
            uint32_t(id.frame_sequence-videoSequence) >= 0x80000000U)) return out;
        if (bound && observed-videoTime > HoldNs) clear();
        if (haveResult && (raw->observed_ns < sourceTime ||
            uint32_t(raw->frame_sequence-sequence) >= 0x80000000U)) return out;
        if (haveResult && ((raw->frame_sequence == sequence) != (raw->observed_ns == sourceTime))) {
            clear(); return out;
        }
        contextId = id; control = controlRevision; owner = ownerGeneration;
        videoTime = observed; videoSequence = id.frame_sequence; bound = true;
        for (auto &t : tracks)
            if (t.active && (observed < t.measured || observed-t.measured >= HoldNs || now >= t.deadline ||
                (!t.confirmed && observed-t.born >= HoldNs))) {
                t = Track{}; ++expired;
            }
        if (!haveResult || raw->frame_sequence != sequence) {
            unsigned boxes[DETECTION_OVERLAY_CAPACITY]{}, active[Capacity]{}, count = 0, size = 0;
            for (unsigned i = 0; i < raw->count; ++i)
                if (vehicle(raw->boxes[i]) && sane(raw->boxes[i])) boxes[count++] = i;
            // Stable geometry order also resolves exact assignment ties.
            std::sort(boxes,boxes+count,[&](unsigned a, unsigned b) {
                const auto &x = raw->boxes[a]; const auto &y = raw->boxes[b];
                if (x.model_class != y.model_class) return x.model_class < y.model_class;
                if (x.x0 != y.x0) return x.x0 < y.x0;
                if (x.y0 != y.y0) return x.y0 < y.y0;
                if (x.x1 != y.x1) return x.x1 < y.x1;
                return x.y1 < y.y1;
            });
            for (unsigned i = 0; i < Capacity; ++i) {
                tracks[i].seen = false;
                if (tracks[i].active) active[size++] = i;
            }
            double costs[Capacity][DETECTION_OVERLAY_CAPACITY+Capacity]{};
            for (unsigned i = 0; i < size; ++i)
                for (unsigned j = 0; j < count+size; ++j)
                    costs[i][j] = j < count ? cost(tracks[active[i]].raw,raw->boxes[boxes[j]]) : 1;
            int matched[Capacity]; assign(costs,size,count+size,matched);
            bool used[DETECTION_OVERLAY_CAPACITY]{};
            for (unsigned i = 0; i < size; ++i)
                if (matched[i] >= 0 && unsigned(matched[i]) < count && costs[i][matched[i]] < 1) {
                    used[matched[i]] = true;
                    measure(tracks[active[i]],raw->boxes[boxes[matched[i]]],raw->observed_ns,deadline);
                }
            for (unsigned j = 0; j < count; ++j) if (!used[j]) {
                auto place = std::find_if(tracks.begin(),tracks.end(),[](const Track &t){ return !t.active; });
                if (place == tracks.end()) {
                    // Current measurements take priority over unmatched history.
                    place = std::min_element(tracks.begin(),tracks.end(),[](const Track &a, const Track &b) {
                        if (a.seen != b.seen) return !a.seen;
                        return a.measured < b.measured;
                    });
                    if (place->seen) break;
                }
                *place = Track{}; place->active = true; place->id = nextId++;
                if (!nextId) { nextId = 1; ++revision; }
                place->born = raw->observed_ns;
                measure(*place,raw->boxes[boxes[j]],raw->observed_ns,deadline);
            }
            sequence = raw->frame_sequence; sourceTime = raw->observed_ns; haveResult = true; ++updates;
        }
        out.overlay = *raw; out.overlay.count = 0; out.revision = revision;
        std::array<uint32_t,DETECTION_OVERLAY_CAPACITY> labels{};
        for (unsigned i = 0; i < raw->count; ++i) if (!vehicle(raw->boxes[i])) {
            const unsigned j = out.overlay.count++;
            out.overlay.boxes[j] = raw->boxes[i];
            out.overlay.track_ids[j] = raw->track_ids[i];
            out.overlay.light_flags[j] = raw->light_flags[i];
            out.overlay.light_deadline_ns[j] = raw->light_deadline_ns[i];
        }
        std::array<unsigned,Capacity> order{};
        unsigned size = 0;
        for (unsigned i = 0; i < Capacity; ++i)
            if (tracks[i].active && tracks[i].confirmed) order[size++] = i;
        std::sort(order.begin(),order.begin()+size,[&](unsigned a, unsigned b) {
            if (tracks[a].seen != tracks[b].seen) return tracks[a].seen;
            return tracks[a].id < tracks[b].id;
        });
        for (unsigned i = 0; i < size && out.overlay.count < DETECTION_OVERLAY_CAPACITY; ++i) {
            const auto &t = tracks[order[i]];
            bool covered = false;
            if (!t.seen)
                for (unsigned j = 0; j < raw->count; ++j)
                    if (vehicle(raw->boxes[j]) && sane(raw->boxes[j]) &&
                        overlap(t.shown,raw->boxes[j]) >= .5) covered = true;
            if (covered) continue;
            const unsigned j = out.overlay.count++;
            out.overlay.boxes[j] = t.shown;
            out.overlay.boxes[j].display_held = !t.seen;
            out.overlay.track_ids[j] = out.overlay.light_flags[j] = 0;
            out.overlay.light_deadline_ns[j] = 0;
            out.ids[j] = t.id;
            labels[j] = t.id;
            if (t.seen) ++out.measured; else ++out.held;
            out.overlay.valid_until_ns = std::min(out.overlay.valid_until_ns,t.deadline);
        }
        // Keep caption priority by persistent ID even when a measured track
        // becomes held. Capacity admission above still prefers measurements.
        for (unsigned i = 0; i < out.overlay.count; ++i)
            for (unsigned j = i+1; j < out.overlay.count; ++j)
                if (labels[i] && labels[j] && labels[j] < labels[i]) {
                    std::swap(out.overlay.boxes[i],out.overlay.boxes[j]);
                    std::swap(out.ids[i],out.ids[j]); std::swap(labels[i],labels[j]);
                }
        return out;
    }
};
#endif
