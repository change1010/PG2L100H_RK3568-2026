#ifndef VIDEO_DMA_FEATURES_H
#define VIDEO_DMA_FEATURES_H

#include <stddef.h>
#include <stdint.h>

/* All multibyte wire fields are little endian. Query CAPS before configuring. */
#define VDMA_FEATURE_MAGIC UINT32_C(0x31544656) /* bytes: VFT1 */
#define VDMA_FEATURE_VERSION 1U
#define VDMA_FEATURE_QUAD_VERSION 2U
#define VDMA_FEATURE_COLOR_PACKET_BYTES 20912U
#define VDMA_FEATURE_QUAD_REGIONS 4U
#define VDMA_FEATURE_MAX_BYTES (48U * 1024U)
#define VDMA_FEATURE_COLOR 1U
#define VDMA_FEATURE_LANE 2U
#define VDMA_FEATURE_ZEBRA 4U
#define VDMA_FEATURE_MASK 7U
#define VDMA_FEATURE_ROAD_LANE_CANDIDATE VDMA_FEATURE_LANE
/* v5 second-section tag. It must not collide with VDMA_FEATURE_ZEBRA (4): that
 * value already names the legacy colour zebra section inside version 1/2
 * packets, while this names a ROAD candidate section inside version 5 packets.
 * Keeping them distinct stops one enum from making two grammars look alike. */
#define VDMA_FEATURE_ROAD_DENSE_CANDIDATE 3U
/* The ROAD wire moved to v4 (see road_wire.h). Producers emit v4, but a board
 * whose bitstream and kernel were upgraded in the wrong order still delivers v3
 * packets, and the difference is invisible on screen: the FPGA keeps sending,
 * the host rejects every packet, and the symptom is "no crosswalk" rather than
 * "version mismatch". Consumers therefore accept both and only the encoder
 * writes the current version. */
#ifndef VDMA_FEATURE_ROAD_PACKET_VERSION
#define VDMA_FEATURE_ROAD_PACKET_VERSION 5U
#endif
#ifndef VDMA_FEATURE_ROAD_PACKET_VERSION_MIN
#define VDMA_FEATURE_ROAD_PACKET_VERSION_MIN 3U
#endif
#ifndef VDMA_FEATURE_ROAD_MAX_PACKET_BYTES
#define VDMA_FEATURE_ROAD_MAX_PACKET_BYTES 32768U
#endif
#define VDMA_FEATURE_ROAD_MAX_CANDIDATES 1536U
/* v3 used flags[3:0]; v4 adds flags[4] as the synthetic (dense-run) marker and
 * keeps [7:5] reserved. Both versions pass this mask because v3 never sets it. */
#define VDMA_FEATURE_ROAD_FLAG_MASK 0x1fU
#define VDMA_FEATURE_ROAD_FLAG_MANDATORY 0x07U
#define VDMA_FEATURE_ROAD_FLAG_TRUNCATED 0x08U
#define VDMA_FEATURE_ROAD_FLAG_SYNTHETIC 0x10U
/* Per-row wire bound. The RTL currently ships 8 (96 x 8 = 768 candidates); the
 * wire allows 16 and the stores accept it, but nothing produces 16 yet. */
#define VDMA_FEATURE_ROAD_MAX_PER_ROW 16U
#define VDMA_FEATURE_ROAD_SHIPPED_PER_ROW 8U
#define VDMA_FEATURE_TRUNCATED 1U
#define VDMA_FEATURE_OVERFLOW 2U
#define VDMA_FEATURE_INPUT_INVALID 4U
/* S2: a sampled row overflowed the elastic candidate FIFO. The packet is
 * still valid; the affected row is simply incomplete. */
#define VDMA_FEATURE_LINE_OVERFLOW 64U
#define VDMA_FEATURE_COLOR_COLS 32U
#define VDMA_FEATURE_COLOR_ROWS 18U
#define VDMA_FEATURE_LANE_ROWS 96U
#define VDMA_FEATURE_LANE_PER_ROW 8U

struct vdma_feature_header {
    uint32_t magic, version, header_bytes, packet_bytes;
    uint32_t feature_mask, status_flags, stream_epoch, frame_sequence;
    uint32_t source_id, layout, config_generation, canvas_width;
    uint32_t canvas_height, section_count, packet_generation, feature_dropped;
    uint32_t section_offset[3];
    union {
        uint32_t reserved[13];
        struct {
            uint32_t region_index, region_count, required_region_mask, batch_id;
            uint32_t reserved[9];
        } quad;
    };
};

/* type equals one feature mask bit. bytes includes header and zero padding
 * to a 16-byte boundary.
 * Coordinates are canvas pixels with half-open ROI bounds.
 * COLOR: dimension0=columns, dimension1=rows; count=cols*rows*3.
 * LANE: dimension0=sampled rows, dimension1=max candidates per row.
 * ZEBRA: dimension0=3 partitions, dimension1=1; count=ROI height.
 */
struct vdma_feature_section {
    uint32_t type, bytes, element_bytes, count;
    uint32_t x0, y0, x1, y1;
    uint32_t dimension0, dimension1, flags, reserved;
};

/* Row-major cells, then RED/GREEN/YELLOW within each cell. */
struct vdma_color_cell {
    uint32_t count, sum_x, sum_y;
};

struct vdma_lane_candidate {
    uint16_t y, x0, x1, width;
    uint32_t gradient_sum;
    uint16_t color, flags; /* color: 3=yellow, 4=white; flags reserved=0 */
};

struct vdma_zebra_row {
    uint32_t left, center, right;
};

#ifdef __cplusplus
#define VDMA_FEATURE_ASSERT static_assert
#else
#define VDMA_FEATURE_ASSERT _Static_assert
#endif
VDMA_FEATURE_ASSERT(sizeof(struct vdma_feature_header) == 128, "feature header layout");
VDMA_FEATURE_ASSERT(offsetof(struct vdma_feature_header, section_offset) == 64, "section offsets");
VDMA_FEATURE_ASSERT(sizeof(struct vdma_feature_section) == 48, "section layout");
VDMA_FEATURE_ASSERT(sizeof(struct vdma_color_cell) == 12, "color layout");
VDMA_FEATURE_ASSERT(sizeof(struct vdma_lane_candidate) == 16, "lane layout");
VDMA_FEATURE_ASSERT(sizeof(struct vdma_zebra_row) == 12, "zebra layout");
#undef VDMA_FEATURE_ASSERT

/* Validates framing, capacities and ROI; does not establish detection validity.
 * Accepts unaligned input. Never casts wire data to host structs.
 */
int vdma_feature_packet_valid(const void *packet, size_t bytes);
/* ROAD v3 is validated separately so COLOR v1/v2 callers keep their exact ABI. */
int vdma_road_packet_valid(const void *packet, size_t bytes);

struct vdma_color_identity {
    uint32_t stream_epoch, frame_sequence, source_id, layout, config_generation;
};

/* No video slot ownership crosses this interface. Pixels stay with the caller. */
int vdma_feature_matches_video(const void *packet, size_t bytes,
                              const struct vdma_color_identity *video);

#define VDMA_COLOR_GROUP_TIMEOUT_NS UINT64_C(300000000)
#define VDMA_COLOR_IDENTITY_CACHE 64U
struct vdma_color_group {
    struct vdma_color_identity identity;
    uint32_t batch_id, received_mask;
    uint64_t first_received_ns, complete_received_ns;
    unsigned char packets[VDMA_FEATURE_QUAD_REGIONS][VDMA_FEATURE_COLOR_PACKET_BYTES];
};
struct vdma_color_group_key {
    uint32_t epoch, sequence, config, batch;
};
struct vdma_color_assembler {
    struct vdma_color_group groups[2];
    struct vdma_color_identity videos[VDMA_COLOR_IDENTITY_CACHE];
    struct vdma_color_group_key closed[16];
    unsigned int used[2], video_count, video_cursor, closed_count, closed_cursor;
    uint32_t epoch, config;
    uint32_t closed_sequence;
    int active, closed_valid;
    uint64_t completed, complete_inputs, incomplete, rejected, stale;
};
void vdma_color_assembler_init(struct vdma_color_assembler *assembler);
/* Bind an acknowledged profile and observed epoch without inventing a video frame. */
void vdma_color_assembler_session(struct vdma_color_assembler *assembler,
                                  uint32_t epoch, uint32_t config, int active);
void vdma_color_assembler_video(struct vdma_color_assembler *assembler,
                                const struct vdma_color_identity *identity);
void vdma_color_assembler_expire(struct vdma_color_assembler *assembler, uint64_t now_ns);
int vdma_color_assembler_push(struct vdma_color_assembler *assembler,
                             const void *packet, size_t bytes, uint64_t now_ns);
int vdma_color_assembler_take(struct vdma_color_assembler *assembler,
                             struct vdma_color_group *result, uint64_t now_ns);

#endif
