#include "traffic_light_color.h"

int light_specialist_sample_color(const unsigned char *pixels, const int box[4],
    unsigned *color, unsigned *evidence)
{
    struct light_color_workspace work;
    struct light_color_result result;
    const struct adas_box roi = {box[0],box[1],box[2],box[3]};
    int rc = traffic_light_color_rgb565(pixels,1920u*1080u*2u,3840,1920,1080,
                                       &roi,&work,&result);
    *color = rc ? ADAS_COLOR_UNKNOWN : result.color;
    *evidence = rc ? 0 : result.evidence_permille;
    return rc;
}
