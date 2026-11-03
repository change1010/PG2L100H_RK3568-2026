#ifndef ROAD_REF_GEOMETRY_H
#define ROAD_REF_GEOMETRY_H
#include <stdint.h>

/* Owned application metadata, never a DMA ABI. Polygons retain detector order.
 * Overflow drops whole polygons and sets truncated; it never simplifies them. */
#define ROAD_REF_MAX_SEGMENTS 64U
#define ROAD_REF_MAX_POINTS 8192U
struct road_ref_point { int32_t x, y; };
struct road_ref_polygon {
    uint32_t line_type, first_point, point_count;
    double area_px, visible_span_px;
};
struct road_ref_geometry {
    uint32_t present, truncated, raw_count, confirmed_count, point_count;
    uint32_t raw_observed, confirmed_observed;
    struct road_ref_polygon raw[ROAD_REF_MAX_SEGMENTS];
    struct road_ref_polygon confirmed[ROAD_REF_MAX_SEGMENTS];
    struct road_ref_point points[ROAD_REF_MAX_POINTS];
    uint32_t cross_detected, cross_stable;
    int32_t cross_y_begin, cross_y_end, cross_stripes;
    double cross_score, cross_pitch;
};
#endif
