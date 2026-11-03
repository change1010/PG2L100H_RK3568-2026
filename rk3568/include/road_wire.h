#ifndef ROAD_WIRE_H
#define ROAD_WIRE_H

#include <stdint.h>

/* ROAD candidate packet contract that crosses FPGA -> PCIe BAR -> rk3568.
 *
 * This header is the single source of truth for the wire numbers. Every
 * constant here is mirrored at least once outside it (RTL parameters, uapi,
 * video_dma_features.h, road_feature_consumer.h, road_features_reference.py),
 * and those mirrors already drifted once: three different default configs
 * disagree on version, enable_mask and reserved[2]. scripts/road_wire_check.py
 * derives the capacity implied by the RTL storage instead of trusting the
 * written number, then fails on any disagreement, so a mirror that drifts is
 * caught before a frame is silently dropped.
 *
 * v3 (shipped): 96 rows x 8 per row, packet <= 16384 B, 88-bit RTL record.
 * v4 (shipped): 96 rows x 16 per row, packet <= 24752 B, 96-bit record.
 * v5 (this header): two sections in one packet, lane 96x8 + dense 96x12.
 *
 * v4 merged both front ends into a single row list ordered by
 * (gradient desc, width desc, x0 asc, x1 asc). That forced one shared
 * candidates_per_row, so a row of Sobel fragments could crowd out the dense
 * white runs the zebra detector needs. v5 gives each source its own section
 * with its own per-row budget, and the packet carries both sections back to
 * back. The sections are physically separate byte ranges, so neither source
 * can starve the other and each detector reads only its own candidates.
 *
 * Why two sections instead of two packets: the transport publishes one packet
 * per frame, and the measured PCIe/lease cadence tops out at 30 Hz. Sending one
 * packet per source would halve that to 15 Hz per source, below the 20 fps
 * target. Two sections in one packet keep both sources at the full cadence.
 */

#define ROAD_WIRE_VERSION 5U

#define ROAD_WIRE_HEADER_BYTES 128U
#define ROAD_WIRE_SECTION_BYTES 48U
#define ROAD_WIRE_RECORD_BYTES 16U
#define ROAD_WIRE_SECTION_COUNT 2U
#define ROAD_WIRE_FIRST_RECORD (ROAD_WIRE_HEADER_BYTES + ROAD_WIRE_SECTION_BYTES)

#define ROAD_WIRE_MAX_ROWS 96U
/* Largest per-row budget of any single section. Per-row is a u8 field, so this
 * is also the bound the staging buffer in road_cv.h must satisfy. */
#define ROAD_WIRE_MAX_PER_ROW 16U
/* Per-section budgets. Field data from 24 captured frames showed 73-91 of the
 * 96 sample rows hitting the 8-candidate lane cap, which is what kept the line
 * fitter from ever assembling a cluster. Dense recall measured identical at 8
 * and 12 runs per row (only the reported box width differs), so the packet
 * budget is moved from dense to lane: 96x13 + 96x8 still fits 32768 B. */
#define ROAD_WIRE_LANE_MAX_PER_ROW 13U
#define ROAD_WIRE_DENSE_MAX_PER_ROW 8U
#define ROAD_WIRE_MAX_CANDIDATES (ROAD_WIRE_MAX_ROWS * ROAD_WIRE_MAX_PER_ROW)

/* Section sizes are derived from the row budget, never chosen independently. */
#define ROAD_WIRE_LANE_CANDIDATES (ROAD_WIRE_MAX_ROWS * ROAD_WIRE_LANE_MAX_PER_ROW)
#define ROAD_WIRE_DENSE_CANDIDATES (ROAD_WIRE_MAX_ROWS * ROAD_WIRE_DENSE_MAX_PER_ROW)
#define ROAD_WIRE_SECTION_A_BYTES (ROAD_WIRE_LANE_CANDIDATES * ROAD_WIRE_RECORD_BYTES)
#define ROAD_WIRE_SECTION_B_OFFSET (ROAD_WIRE_FIRST_RECORD + ROAD_WIRE_SECTION_A_BYTES)
#define ROAD_WIRE_SECTION_B_FIRST_RECORD (ROAD_WIRE_SECTION_B_OFFSET + ROAD_WIRE_SECTION_BYTES)

/* Declared transport capacity. It is derived, not chosen: road_packet_bank.v
 * keeps one packet per bank, the bank is half of packet_mem (4096 x 128 bit in
 * v4), and read_beat addresses 16-byte beats, so a bank holds
 * (4096 / 2) * 16 = 32768 B. v3 was (2048 / 2) * 16 = 16384 B for the same
 * reason, which is why the old number was never a protocol decision.
 *
 * With 96 rows this caps per_row at 21 (96 * 21 * 16 + 176 = 32432 B); 22
 * overflows. v4 takes 16 and leaves the remainder as headroom. */
#define ROAD_WIRE_CAPACITY_BYTES 32768U
#define ROAD_WIRE_MAX_PACKET_BYTES ROAD_WIRE_CAPACITY_BYTES

/* RTL record width. v3 packed {y,x0,x1,width,gradient,color,flags[3:0]} into
 * 88 bit. v4 widens flags to 8 bit and pads the record to 96 bit so sixteen
 * records fill one row word with no bit arithmetic at the bank boundary. */
#define ROAD_WIRE_RECORD_BITS 96U

/* flags: [2:0] must read as 111 (both validators enforce it), [3] = the row was
 * truncated by the segment store, [4] = synthesized dense run, [7:5] reserved.
 * There is no spare bit below 5: bit3 is load bearing for truncation, so the
 * dense marker could not be added inside v3 without redefining a used bit. */
#define ROAD_WIRE_FLAG_MANDATORY 0x07U
#define ROAD_WIRE_FLAG_TRUNCATED 0x08U
#define ROAD_WIRE_FLAG_SYNTHETIC 0x10U
#define ROAD_WIRE_FLAG_VALID_MASK 0x1fU

#define ROAD_WIRE_COLOR_YELLOW 3U
#define ROAD_WIRE_COLOR_WHITE 4U

/* Candidate provenance. The bank tags every accepted candidate with the front
 * end that produced it, so v5 can carry both sources in one packet and the
 * consumer can route each segment to the right detector. 2 and 3 are reserved
 * and rejected by the bank, making a mis-wired front end a hard failure rather
 * than a silent slot collision. Must match ROAD_SOURCE_* in
 * fpga/video_dma_hdmi/rtl/features/road_feature_channel.v. */
#define ROAD_WIRE_SOURCE_SOBEL 0U
#define ROAD_WIRE_SOURCE_DENSE 1U
#define ROAD_WIRE_SOURCE_COUNT 2U

/* Per-row staging is owned by road_cv.h (ROAD_CV_MAX_SEGMENTS); test_road_wire.c
 * and road_wire_check.py keep it at least as large as ROAD_WIRE_MAX_PER_ROW. */

_Static_assert(ROAD_WIRE_MAX_ROWS * ROAD_WIRE_MAX_PER_ROW == ROAD_WIRE_MAX_CANDIDATES,
               "candidate capacity must equal rows times per-row slots");
_Static_assert(ROAD_WIRE_MAX_PER_ROW <= 255U, "candidates_per_row is a u8 field");
_Static_assert(ROAD_WIRE_MAX_ROWS <= 96U, "sample_rows field and RTL MAX_ROWS are 96");
_Static_assert(ROAD_WIRE_FIRST_RECORD == 176U, "candidate body starts after 128 + 48 bytes");
_Static_assert(ROAD_WIRE_FIRST_RECORD + ROAD_WIRE_MAX_CANDIDATES * ROAD_WIRE_RECORD_BYTES
                   <= ROAD_WIRE_MAX_PACKET_BYTES,
               "worst-case packet must fit the declared capacity");
_Static_assert((ROAD_WIRE_FIRST_RECORD + ROAD_WIRE_MAX_CANDIDATES * ROAD_WIRE_RECORD_BYTES) % 16U == 0,
               "driver requires packet_bytes aligned to 16");
_Static_assert(ROAD_WIRE_CAPACITY_BYTES % ROAD_WIRE_RECORD_BYTES == 0, "beats are 16 bytes");
_Static_assert(ROAD_WIRE_RECORD_BITS == 96U, "record width matches road_candidate_segments");

/* v5 two-section layout. The offsets are derived from the row budgets so a
 * change to either budget breaks the build instead of silently shifting the
 * second section header. */
_Static_assert(ROAD_WIRE_LANE_CANDIDATES == 1248U, "lane section holds 96 rows x 13");
_Static_assert(ROAD_WIRE_DENSE_CANDIDATES == 768U, "dense section holds 96 rows x 8");
_Static_assert(ROAD_WIRE_LANE_MAX_PER_ROW <= ROAD_WIRE_MAX_PER_ROW,
               "lane budget must fit the staging bound");
_Static_assert(ROAD_WIRE_DENSE_MAX_PER_ROW <= ROAD_WIRE_MAX_PER_ROW,
               "dense budget must fit the staging bound");
_Static_assert(ROAD_WIRE_SECTION_B_OFFSET == 20144U,
               "second section header sits at 176 + 96*13*16");
_Static_assert(ROAD_WIRE_SECTION_B_FIRST_RECORD == 20192U,
               "second section body starts 48 bytes after its header");
_Static_assert(ROAD_WIRE_SECTION_B_FIRST_RECORD
                   + ROAD_WIRE_DENSE_CANDIDATES * ROAD_WIRE_RECORD_BYTES == 32480U,
               "worst-case v5 packet is 32480 bytes");
_Static_assert(ROAD_WIRE_SECTION_B_FIRST_RECORD
                   + ROAD_WIRE_DENSE_CANDIDATES * ROAD_WIRE_RECORD_BYTES
                   <= ROAD_WIRE_MAX_PACKET_BYTES,
               "worst-case v5 packet must fit the declared capacity");

/* Full packet length for a single-section packet (v3/v4).
 * Kept because v3 and v4 packets still decode. */
static inline uint32_t road_wire_packet_bytes(uint32_t candidates)
{
    return ROAD_WIRE_FIRST_RECORD + candidates * ROAD_WIRE_RECORD_BYTES;
}

/* Byte offset of the second section header. It depends on how many lane
 * candidates the producer actually emitted, so a consumer must read the first
 * section header (record 8) to find where section B starts rather than assume
 * the worst-case offset. */
static inline uint32_t road_wire_section_b_offset(uint32_t lane_candidates)
{
    return ROAD_WIRE_FIRST_RECORD + lane_candidates * ROAD_WIRE_RECORD_BYTES;
}

/* Full v5 packet length for two sections. */
static inline uint32_t road_wire_packet_bytes_two(uint32_t lane_candidates,
                                                 uint32_t dense_candidates)
{
    return road_wire_section_b_offset(lane_candidates) + ROAD_WIRE_SECTION_BYTES
         + dense_candidates * ROAD_WIRE_RECORD_BYTES;
}

/* True for a flags word a v4 consumer must accept. */
static inline int road_wire_flags_valid(uint32_t flags)
{
    return (flags & ROAD_WIRE_FLAG_MANDATORY) == ROAD_WIRE_FLAG_MANDATORY &&
           (flags & ~ROAD_WIRE_FLAG_VALID_MASK) == 0;
}

/* True when the candidate was synthesized by the dense-run engine. */
static inline int road_wire_flags_synthetic(uint32_t flags)
{
    return (flags & ROAD_WIRE_FLAG_SYNTHETIC) != 0;
}

#endif /* ROAD_WIRE_H */
