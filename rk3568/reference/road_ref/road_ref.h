#ifndef ROAD_REF_H
#define ROAD_REF_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define ROAD_REF_API __declspec(dllexport)
#else
#define ROAD_REF_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define ROAD_REF_ABI 1u
typedef struct RoadRefHandle RoadRefHandle;
enum { ROAD_REF_BGR8 = 1, ROAD_REF_GRAY8 = 2 };
enum {
    ROAD_REF_CROSS_GRAY8 = 1,
    ROAD_REF_LANE_GRAY8 = 2,
    ROAD_REF_LANE_SPREAD8 = 3
};
enum { ROAD_REF_OK = 0, ROAD_REF_INVALID = -1, ROAD_REF_EXCEPTION = -2 };
enum { ROAD_REF_SOLID = 1, ROAD_REF_DASHED = 2 };

typedef struct {
    uint32_t size, abi;
    int32_t lane_width, strict_geometry, confirm_frames, max_missed;
    int32_t crosswalk_stride, crosswalk_win, vote_window, vote_need;
    double horizon, bottom, min_white_area, bright_threshold, pitch_ratio;
} RoadRefConfig;

typedef struct {
    uint32_t size, format;
    int32_t width, height;
    size_t stride, data_bytes;
    const uint8_t *data;
    uint64_t sequence, timestamp_ns, source_id;
} RoadRefFrame;

/* Local borrowed views, not a serialized DMA descriptor. Plane order is free.
 * Dimensions must match the configured lane working size and source size.
 * No flags are defined yet; flags must be zero. */
typedef struct {
    uint32_t kind;
    int32_t width, height;
    size_t stride, data_bytes;
    const uint8_t *data;
} RoadRefPlane;

typedef struct {
    uint32_t size, abi;
    int32_t source_width, source_height;
    uint32_t source_epoch, preprocess_id, plane_count, flags;
    uint64_t sequence, timestamp_ns, source_id;
    RoadRefPlane planes[3];
} RoadRefPlanesFrame;

typedef struct { int32_t x, y; } RoadRefPoint;
typedef struct {
    int32_t line_type;
    uint32_t point_count;
    const RoadRefPoint *points;
    double area_px, visible_span_px;
} RoadRefSegment;

typedef struct {
    int32_t detected, stable, y_begin, y_end, n;
    int32_t n_bands_total, n_ac_pass, n_stereo_pass, empty_candidates;
    double score, pitch, duty, gap_mad, ac, resid, slope;
} RoadRefCrosswalk;

/* Arrays belong to the handle and remain valid until process/reset/destroy.
 * The caller owns the input, which is used only during process().
 * One handle per stream/worker; concurrent calls on one handle are unsupported.
 */
typedef struct {
    uint32_t size, raw_count, confirmed_count;
    const RoadRefSegment *raw_segments, *confirmed_segments;
    RoadRefCrosswalk crosswalk;
    uint64_t sequence, timestamp_ns, source_id;
    int32_t state_reset;
} RoadRefResult;

typedef struct {
    uint32_t size;
    /* Concurrent algorithm wall times overlap; their sum is not frame time. */
    uint64_t lane_ns, crosswalk_ns, temporal_ns;
} RoadRefTimings;
ROAD_REF_API int road_ref_last_timings(RoadRefHandle *handle, RoadRefTimings *out);
/* Lane: threshold, morphology/components, integral/local, tracks, polygons.
 * Crosswalk: normalization, profiles/FFT, consensus, adaptive/verify, geometry. */
typedef struct {
    uint32_t size;
    uint64_t lane_ns[5], crosswalk_ns[5];
} RoadRefDetailTimings;
ROAD_REF_API int road_ref_detail_timings(RoadRefHandle *handle, RoadRefDetailTimings *out);

ROAD_REF_API void road_ref_default_config(RoadRefConfig *config);
ROAD_REF_API RoadRefHandle *road_ref_create(const RoadRefConfig *config);
ROAD_REF_API int road_ref_process(RoadRefHandle *handle,
                                 const RoadRefFrame *frame,
                                 RoadRefResult *result);
ROAD_REF_API int road_ref_process_planes(RoadRefHandle *handle,
                                        const RoadRefPlanesFrame *frame,
                                        RoadRefResult *result);
ROAD_REF_API void road_ref_reset(RoadRefHandle *handle);
ROAD_REF_API void road_ref_destroy(RoadRefHandle *handle);
/* Diagnostics are optional and serialized only on request. */
ROAD_REF_API const char *road_ref_debug_json(RoadRefHandle *handle);
/* plane: 0 raw white mask, 1 cleaned mask, 2 scan mask, 3 ROI.
 * Query dimensions with dst=NULL. Non-NULL dst requires bytes >= stride*height.
 */
ROAD_REF_API int road_ref_copy_mask(RoadRefHandle *handle, int plane,
                                   uint8_t *dst, size_t stride, size_t bytes,
                                   int32_t *width, int32_t *height);
/* Optional exact float32 diagnostics: 0 profile, 1 filtered signal,
 * 2 autocorrelation, 3 adaptive signal. Returns length or INVALID.
 * Query with dst=NULL; otherwise capacity is in float elements.
 */
ROAD_REF_API int road_ref_copy_band(RoadRefHandle *handle, int band, int plane,
                                   float *dst, size_t capacity);
ROAD_REF_API const char *road_ref_opencv_version(void);
/* Offline reproduction of track_pitch_threads defaults. Future frames can
 * change earlier flags. This utility must not be used as the live stable flag.
 * All arrays have count elements; output may not alias an input.
 */
ROAD_REF_API int road_ref_offline_pitch_threads(const uint8_t *detected,
                                              const double *pitches,
                                              size_t count, uint8_t *output);

#ifdef __cplusplus
}
#endif
#endif
