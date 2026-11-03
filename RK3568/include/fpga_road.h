#ifndef FPGA_ROAD_H
#define FPGA_ROAD_H
#include "adas_result.h"
#include "fpga_road_uapi.h"
#include "video_dma_feature_uapi.h"
#include "video_dma_road_ref_uapi.h"

struct fpga_road;
struct fpga_road_stats {
    uint64_t received, published, unmatched, rejected;
    uint32_t dropped, last_epoch, last_sequence;
};
struct fpga_road *fpga_road_create(struct adas_store *store);
void fpga_road_destroy(struct fpga_road *road);
void fpga_road_clear(struct fpga_road *road);
struct adas_producer fpga_road_registration(void);
/* Owner-thread API. Observe while the video slot/token lease is still held.
 * Hardware results can arrive before video; both histories are bounded. */
/* Pure transport boundary, also exercised by host fixtures. */
int fpga_road_bind(struct fpga_road *road, const struct vdma_frame_identity *display,
    const struct vdma_native_identity *native, uint64_t observed_ns);
int fpga_road_accept(struct fpga_road *road, const struct vdma_fpga_road *packet,
    uint64_t now_ns);
int fpga_road_flush(struct fpga_road *road, uint64_t now_ns);
int fpga_road_decode(const struct vdma_fpga_road *packet, struct adas_road *out);
void fpga_road_get_stats(const struct fpga_road *road, struct fpga_road_stats *stats);
#endif
