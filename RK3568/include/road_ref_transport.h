#ifndef ROAD_REF_TRANSPORT_H
#define ROAD_REF_TRANSPORT_H
#include "video_dma_snapshot.h"

struct road_ref_transport;
struct road_ref_transport *road_ref_transport_create(int fd, unsigned int period);
/* One copy/CRC worker; the display owner still acquires and pairs identities. */
int road_ref_transport_start_async(struct road_ref_transport *transport);
/* Join the producer before stopping/destroying its snapshot runtime. */
int road_ref_transport_stop(struct road_ref_transport *transport);
/* Releases local mappings; a failed STOP is returned to the application. */
int road_ref_transport_destroy(struct road_ref_transport *transport);
int road_ref_transport_observe(struct road_ref_transport *transport,
    const struct vdma_frame_identity *identity, uint64_t observed_ns);
int road_ref_transport_drain(struct road_ref_transport *transport,
    struct vdma_snapshot_runtime *runtime, int enabled);
/* Decoder checks the frozen layout and all plane CRCs before borrowing views. */
int road_ref_transport_decode(const struct vdma_rr_frame *frame,
    const unsigned char *payload, size_t bytes, RoadRefPlanesFrame *out);
void road_ref_transport_report(struct road_ref_transport *transport, FILE *out);
#endif
