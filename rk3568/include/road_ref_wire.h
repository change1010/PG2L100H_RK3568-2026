#ifndef ROAD_REF_WIRE_H
#define ROAD_REF_WIRE_H

#ifdef __KERNEL__
#include <linux/types.h>
typedef __le32 rr_wire_u32;
#else
#include <stdint.h>
typedef uint32_t rr_wire_u32;
#endif

/* All fields are little endian on the wire. No process pointers or size_t.
 * Producer is a hint; commit must equal the expected serial before dma_rmb().
 * Slot is not reusable until the consumer releases that serial and session. */
#define RR_WIRE_ABI 1u
#define RR_WIRE_MAGIC 0x31503352u
#define RR_PAYLOAD_BYTES 3916800u
#define RR_FRAME_BEATS 122400u
#define RR_DDR_BASE_BYTES 0x06000000u
#define RR_DDR_SLOT_BYTES 0x00400000u
#define RR_HOST_SLOT_COUNT 3u
#define RR_COMPLETION_COUNT 4u
#define RR_RECORD_BYTES 256u
#define RR_FENCE_OFFSET 1024u
#define RR_COMPLETION_ALLOC_BYTES 4096u
#define RR_PREPROCESS_ID 1u
#define RR_TIMESTAMP_HZ 148500000u
#define RR_FLAG_COMPLETE 1u
#define RR_FLAG_CRC32 2u
#define RR_FENCE_MAGIC 0x31463352u

/* BAR0 byte offsets. Writes use the first dword of each 16-byte word. */
#define RR_REG_CAPABILITY 0x0e00u
#define RR_REG_CONTROL 0x0e10u
#define RR_REG_PERIOD 0x0e20u
#define RR_REG_FORMAT 0x0e30u
#define RR_REG_HOST0_LO 0x0e40u
#define RR_REG_HOST0_HI 0x0e50u
#define RR_REG_HOST1_LO 0x0e60u
#define RR_REG_HOST1_HI 0x0e70u
#define RR_REG_HOST2_LO 0x0e80u
#define RR_REG_HOST2_HI 0x0e90u
#define RR_REG_COMPLETION_LO 0x0ea0u
#define RR_REG_COMPLETION_HI 0x0eb0u
#define RR_REG_CONSUMER 0x0ec0u
#define RR_REG_COUNTERS 0x0ed0u
#define RR_REG_SESSION 0x0ee0u
#define RR_REG_STOP 0x0ef0u
#define RR_REG_STOP_SENT 0x0f00u
#define RR_REG_MEMORY_STATUS 0x0f10u
#define RR_REG_PIXEL_STATUS 0x0f20u
#define RR_REG_DDR_LAYOUT 0x0f30u
#define RR_REG_NATIVE_IDENTITY 0x0f40u
#define RR_REG_NATIVE_LEFT_CHECK 0x0f50u
#define RR_REG_NATIVE_LEFT_COUNTS 0x0f60u
#define RR_REG_IMPLEMENTATION 0x0f70u
/* Optional diagnostic: magic 0x31505252, control revision 2, then two zeros.
 * Older ABI1 images return zero. This does not change frame descriptors. */

/* LEFT_CHECK dwords: native sequence, epoch, readback CRC32, capture CRC32.
 * Covers the first 128 RGB565 pixels of every native row, little endian.
 * LEFT_COUNTS dwords: completed full-resolution checks, CRC mismatches.
 * Quad is excluded. Counters are diagnostics, never DMA completion fences. */

typedef struct {
    rr_wire_u32 kind, width, height, stride_bytes;
    rr_wire_u32 offset_lo, offset_hi, length_lo, length_hi;
} RoadRefWirePlane;

typedef struct {
    rr_wire_u32 commit, session, slot, flags;
    rr_wire_u32 magic, abi, header_bytes, payload_bytes;
    rr_wire_u32 source_id, source_epoch, sequence_lo, preprocess_id;
    rr_wire_u32 timestamp_lo, timestamp_hi, timestamp_hz, plane_count;
    rr_wire_u32 source_width, source_height, total_beats, sequence_hi;
    RoadRefWirePlane planes[3];
    rr_wire_u32 crc32[3];
    rr_wire_u32 reserved[17];
} RoadRefWireRecord;

/* One final ordered MWR after the stream is quiescent. This is independent
 * of a frame commit and must be observed in host memory before freeing DMA
 * targets. RO/IDO are disabled, same requester and TC as payload writes. */
typedef struct {
    rr_wire_u32 token, session, magic, reserved;
} RoadRefWireFence;

#if defined(__cplusplus)
static_assert(sizeof(RoadRefWireRecord) == RR_RECORD_BYTES, "wire record size");
#else
_Static_assert(sizeof(RoadRefWireRecord) == RR_RECORD_BYTES, "wire record size");
#endif

#endif
