#ifndef VIDEO_DMA_DASHBOARD_LANE_DISPLAY_H
#define VIDEO_DMA_DASHBOARD_LANE_DISPLAY_H
#include "video_dma_dashboard_paint.h"
#include "road_overlay_geometry.h"
#include "video_dma_snapshot_uapi.h"
#include <vector>

/* Display coordinates only. Caller keeps the raw publication for decisions.
 * Association uses shape/overlap, never the detector's array order. */
class DashboardLaneDisplay {
    struct Track { QPolygonF shown, pending; unsigned confirmations = 0; };
    std::vector<Track> tracks;
    vdma_frame_identity prior{};
    bool bound = false;
    static double distance(const QPolygonF &a, const QPolygonF &b) {
        double value = 0;
        for (int i = 0; i < 4; ++i)
            value = qMax(value,qMax(std::abs(a[i].x()-b[i].x()),std::abs(a[i].y()-b[i].y())));
        return value;
    }
    static double score(const QPolygonF &a, const QPolygonF &b) {
        const auto ab = a.boundingRect(), bb = b.boundingRect(), overlap = ab.intersected(bb);
        const double area = qMin(ab.width()*ab.height(),bb.width()*bb.height());
        if (area <= 0 || overlap.width()*overlap.height() < area*.2) return 1e9;
        const QPointF aBottom = (a[2]+a[3])*.5, bBottom = (b[2]+b[3])*.5;
        const QPointF aTop = (a[0]+a[1])*.5, bTop = (b[0]+b[1])*.5;
        const double dx = std::abs(aBottom.x()-bBottom.x()), dy = std::abs(aBottom.y()-bBottom.y());
        const double slopeA = (aBottom.x()-aTop.x())/qMax(1.0,aBottom.y()-aTop.y());
        const double slopeB = (bBottom.x()-bTop.x())/qMax(1.0,bBottom.y()-bTop.y());
        if (dx > 100 || dy > 100 || std::abs(slopeA-slopeB) > .4) return 1e9;
        return dx+dy+80*std::abs(slopeA-slopeB);
    }
public:
    uint64_t suppressed = 0, accepted = 0, filtered = 0;
    void clear() { tracks.clear(); bound = false; }
    road_overlay_geometry update(const road_overlay_geometry &raw, const vdma_frame_identity &id) {
        const uint32_t gap = id.frame_sequence-prior.frame_sequence;
        if (!bound || prior.stream_epoch != id.stream_epoch || prior.source_id != id.source_id ||
            prior.layout != id.layout || prior.config_generation != id.config_generation ||
            gap > 6) clear();
        const bool repeated = bound && !gap;
        prior = id; bound = true;
        auto out = raw;
        auto &ref = out.reference;
        if (!raw.valid || !ref.present || ref.confirmed_count > ROAD_REF_MAX_SEGMENTS ||
            ref.point_count > ROAD_REF_MAX_POINTS) { clear(); return out; }
        std::vector<QPolygonF> shapes;
        for (unsigned i = 0; i < ref.confirmed_count; ++i) {
            const auto &s = ref.confirmed[i];
            QPolygonF polygon;
            if (s.point_count >= 3 && s.first_point <= ref.point_count &&
                s.point_count <= ref.point_count-s.first_point) {
                for (unsigned j = 0; j < s.point_count; ++j) {
                    const auto &p = ref.points[s.first_point+j];
                    polygon << QPointF(p.x,p.y);
                }
                polygon = displayLaneQuad(polygon);
            }
            shapes.push_back(polygon);
        }
        std::vector<int> matches(shapes.size(),-1);
        std::vector<bool> used(tracks.size(),false);
        // Global greedy association prevents a reordered array from stealing a closer marking.
        for (size_t n = 0; n < shapes.size(); ++n) {
            double best = 1e9; int a = -1, b = -1;
            for (size_t i = 0; i < shapes.size(); ++i) if (matches[i] < 0 && shapes[i].size() == 4)
                for (size_t j = 0; j < tracks.size(); ++j) if (!used[j]) {
                    const double value = score(shapes[i],tracks[j].shown);
                    if (value < best) { best = value; a = int(i); b = int(j); }
                }
            if (a < 0) break;
            matches[a] = b; used[b] = true;
        }
        std::vector<Track> next;
        ref.point_count = 0;
        for (size_t i = 0; i < shapes.size(); ++i) {
            auto polygon = shapes[i];
            if (polygon.size() == 4) {
                Track track;
                if (matches[i] >= 0) track = tracks[matches[i]];
                else track.shown = polygon;
                const double delta = distance(track.shown,polygon);
                if (delta <= 8) {
                    track.confirmations = 0; track.pending.clear();
                    if (!repeated && delta > 2) {
                        for (int p = 0; p < 4; ++p) track.shown[p] = track.shown[p]*.65+polygon[p]*.35;
                        ++filtered;
                    }
                } else if (!repeated) {
                    if (track.pending.size() != 4 || distance(track.pending,polygon) > 12)
                        track.confirmations = 0;
                    track.pending = polygon;
                    if (++track.confirmations >= 3) {
                        track.shown = polygon; track.confirmations = 0; ++accepted;
                    } else ++suppressed;
                }
                polygon = track.shown; next.push_back(track);
            }
            auto &s = ref.confirmed[i];
            s.first_point = ref.point_count;
            s.point_count = unsigned(polygon.size());
            if (s.point_count > ROAD_REF_MAX_POINTS-ref.point_count) s.point_count = 0;
            for (unsigned p = 0; p < s.point_count; ++p)
                ref.points[ref.point_count++] = {qRound(polygon[int(p)].x()),qRound(polygon[int(p)].y())};
        }
        tracks = std::move(next); // Explicitly absent markings never persist.
        return out;
    }
};
#endif
