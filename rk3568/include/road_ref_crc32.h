#ifndef ROAD_REF_CRC32_H
#define ROAD_REF_CRC32_H
#include <stdint.h>
#include <string.h>
#include <zlib.h>

#if defined(__aarch64__) && defined(__linux__) && defined(__GNUC__) && \
    defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#include <arm_acle.h>
#include <asm/hwcap.h>
#include <sys/auxv.h>
#define ROAD_REF_CRC32_ARM 1

__attribute__((target("+crc")))
static inline uint32_t road_ref_crc32_arm(const unsigned char *data, uint32_t bytes)
{
    uint32_t crc = UINT32_MAX;
    while (bytes >= 8) {
        uint64_t word;
        memcpy(&word, data, sizeof(word));
        crc = __crc32d(crc, word);
        data += 8; bytes -= 8;
    }
    while (bytes--) crc = __crc32b(crc, *data++);
    return ~crc;
}
#endif

static inline int road_ref_crc32_accelerated(void)
{
#ifdef ROAD_REF_CRC32_ARM
    return (getauxval(AT_HWCAP) & HWCAP_CRC32) != 0;
#else
    return 0;
#endif
}

/* The wire protocol uses IEEE CRC32, not the CRC32C instruction variant. */
static inline uint32_t road_ref_crc32(const unsigned char *data, uint32_t bytes)
{
#ifdef ROAD_REF_CRC32_ARM
    if (road_ref_crc32_accelerated()) return road_ref_crc32_arm(data, bytes);
#endif
    return (uint32_t)crc32(0, data, bytes);
}
#endif
