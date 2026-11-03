#ifndef ROAD_OVERLAY_GEOMETRY_H
#define ROAD_OVERLAY_GEOMETRY_H
#include <stdint.h>
#include "road_ref_geometry.h"

/* Owned C/C++ drawing snapshot; no atomics, borrowed buffers or DRM owner.
 * Coordinates are canvas pixel centres in 1920x1080. */
#define ROAD_OVERLAY_MAX_POINTS 96U
#define ROAD_OVERLAY_MAX_ZEBRAS 8U
#define ROAD_OVERLAY_LEFT_COLOR 0xFF00FF50U
#define ROAD_OVERLAY_RIGHT_COLOR 0xFFFFA000U
struct road_overlay_point { int32_t x, y; };
struct road_overlay_lane {
    uint32_t count;
    struct road_overlay_point points[ROAD_OVERLAY_MAX_POINTS];
    uint32_t measured_connection[(ROAD_OVERLAY_MAX_POINTS + 31U) / 32U];
};
struct road_overlay_box { int32_t x0, y0, x1, y1; };
struct road_overlay_geometry {
    uint32_t valid;
    uint32_t stream_epoch, source_id, layout, config_generation, frame_sequence;
    uint64_t publication_sequence, valid_until_ns;
    uint64_t observed_ns, finished_ns;
    struct road_overlay_lane lanes[2];
    uint32_t zebra_count;
    struct road_overlay_box zebras[ROAD_OVERLAY_MAX_ZEBRAS];
    struct road_ref_geometry reference;
};
#endif
