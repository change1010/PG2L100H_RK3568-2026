#ifndef ROAD_CV_ADAS_H
#define ROAD_CV_ADAS_H

#include "road_cv.h"

/* Analyze one identity-bound raw packet and publish one coherent ADAS ROAD
 * snapshot. The caller owns the token and must have captured the matching
 * frame identity before calling this function. */
int road_cv_publish_adas(struct adas_store *store, const struct adas_token *token,
                         const struct vdma_road_result *input,
                         const struct road_cv_config *config, uint64_t now_ns);

/* Same publication from a result the caller already analysed, so a caller that
 * post-processes the geometry (temporal filtering, fusion) does not analyse the
 * packet twice and does not have to re-derive it. */
int road_cv_publish_analyzed(struct adas_store *store, const struct adas_token *token,
                             const struct vdma_road_result *input,
                             const struct road_cv_output *analyzed, uint64_t now_ns);

#endif
