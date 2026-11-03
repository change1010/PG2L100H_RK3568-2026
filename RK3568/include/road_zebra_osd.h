#ifndef ROAD_ZEBRA_OSD_H
#define ROAD_ZEBRA_OSD_H
#include <stdint.h>
#include "adas_result.h"
#include "road_overlay_geometry.h"

/* Overlay drawing for the owner-managed ARGB8888 scanout buffer that
 * vdma_osd_probe hands out. Pixels are DRM_FORMAT_ARGB8888 with
 * "Pre-multiplied" blend, so every colour below uses alpha 0xFF and therefore
 * needs no premultiplication. Everything is clipped to the canvas; the video
 * plane is never touched.
 */

#define ROAD_ZEBRA_OSD_BOX_COLOR 0xFF18E018U
#define ROAD_ZEBRA_OSD_INK_COLOR 0xFF041004U

/* Zeros the overlay so the next composite shows only the video plane. */
void road_zebra_osd_clear(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height);

/* Clears then draws observed lanes and zebra boxes of a VALID road snapshot.
 * Returns the number of lane/box primitives drawn, or -1 on bad arguments. They are drawn
 * only when the branch is ADAS_VALID, so a stale or unknown result erases the
 * previous frames instead of leaving them on screen. */
int road_zebra_osd_draw(uint32_t *argb, uint32_t pitch, uint32_t width, uint32_t height,
                        const struct adas_road *road, uint32_t thickness);

void road_zebra_osd_geometry(struct road_overlay_geometry *out, const struct adas_road *road);

/* Writes one raw ARGB8888 frame (pitch * height * 4 bytes, no header) so the
 * on-screen composite can be reproduced off-board. */
int road_zebra_osd_dump(const uint32_t *argb, uint32_t pitch, uint32_t height, const char *path);

#endif
