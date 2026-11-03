#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "video_dma_ui_layout.h"
#include "cJSON.h"

int vdma_ui_layout_validate(const struct vdma_ui_layout *l)
{
    if (!l || l->version != 1 || l->output_width != 1920 || l->output_height != 1080 ||
        l->source_width != 1920 || l->source_height != 1080 ||
        !l->video.width || !l->video.height || l->video.x > 1456 || l->video.y < 88 ||
        l->video.y > 1024 || l->video.width > 1456 - l->video.x ||
        l->video.height > 1024 - l->video.y ||
        (uint64_t)l->video.width * 1080 != (uint64_t)l->video.height * 1920) {
        errno = EINVAL; return -1;
    }
    return 0;
}

static int number(const cJSON *value, uint32_t *out)
{
    if (!cJSON_IsNumber(value) || !isfinite(value->valuedouble) || value->valuedouble < 0 ||
        value->valuedouble > UINT32_MAX || value->valuedouble != (uint32_t)value->valuedouble)
        return -1;
    *out = (uint32_t)value->valuedouble; return 0;
}

int vdma_ui_layout_load(struct vdma_ui_layout *layout, const char *path)
{
    if (!layout || !path) { errno = EINVAL; return -1; }
    FILE *file = fopen(path, "rb");
    if (!file) return -1;
    char data[2048]; size_t size = fread(data, 1, sizeof(data) - 1, file);
    int invalid = ferror(file) || fgetc(file) != EOF;
    if (fclose(file)) invalid = 1;
    if (invalid) { errno = EINVAL; return -1; }
    data[size] = 0;
    const char *end = NULL;
    cJSON *json = cJSON_ParseWithLengthOpts(data, size + 1, &end, 1);
    struct vdma_ui_layout candidate = {0};
    uint32_t *fields[] = {&candidate.version, &candidate.output_width, &candidate.output_height,
        &candidate.source_width, &candidate.source_height, &candidate.video.x, &candidate.video.y,
        &candidate.video.width, &candidate.video.height};
    const char *names[] = {"version", "output_width", "output_height", "source_width", "source_height",
        "video_x", "video_y", "video_width", "video_height"};
    unsigned int count = 0;
    if (!cJSON_IsObject(json) || end != data + size) invalid = 1;
    for (const cJSON *item = json ? json->child : NULL; item; item = item->next) {
        unsigned int n;
        for (n = 0; n < 9; n++) if (!strcmp(item->string, names[n])) break;
        if (n == 9 || (count & (1U << n)) || number(item, fields[n])) { invalid = 1; break; }
        count |= 1U << n;
    }
    if (count != 511 || vdma_ui_layout_validate(&candidate)) invalid = 1;
    cJSON_Delete(json);
    if (invalid) { errno = EINVAL; return -1; }
    *layout = candidate; return 0;
}
