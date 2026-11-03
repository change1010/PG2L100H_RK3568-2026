#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include "video_dma_osd_probe.h"

static int select_plane(struct vdma_osd_probe *osd)
{
    drmModeRes *resources = drmModeGetResources(osd->fd);
    if (!resources) return -1;
    uint32_t bit = 0;
    for (int i = 0; i < resources->count_crtcs && i < 32; i++)
        if (resources->crtcs[i] == osd->crtc) bit = 1U << i;
    drmModeFreeResources(resources);
    if (!bit) { errno = ENODEV; return -1; }
    drmModePlaneRes *planes = drmModeGetPlaneResources(osd->fd);
    if (!planes) return -1;
    /* RK3568 Esmart is exposed as Cursor but accepts full-size linear ARGB.
     * Cluster Overlay advertises ARGB yet needs AFBC; select a cursor first.
     * Actual atomic commit is required to validate this configuration. */
    for (uint32_t i = 0; i < planes->count_planes; i++) {
        drmModePlane *plane = drmModeGetPlane(osd->fd, planes->planes[i]);
        if (!plane) continue;
        int format = 0;
        for (uint32_t f = 0; f < plane->count_formats; f++)
            if (plane->formats[f] == DRM_FORMAT_ARGB8888) format = 1;
        uint64_t type = UINT64_MAX;
        uint32_t property_ids[3] = {0};
        uint64_t values[3] = {0};
        const char *names[3] = {"alpha", "pixel blend mode", "zpos"};
        int premultiplied = 0;
        uint32_t geometry[10] = {0};
        uint64_t geometry_values[10] = {0};
        const char *geometry_names[10] = {"FB_ID", "CRTC_ID", "CRTC_X", "CRTC_Y",
            "CRTC_W", "CRTC_H", "SRC_X", "SRC_Y", "SRC_W", "SRC_H"};
        drmModeObjectProperties *props = drmModeObjectGetProperties(osd->fd,
            plane->plane_id, DRM_MODE_OBJECT_PLANE);
        if (props) {
            for (uint32_t p = 0; p < props->count_props; p++) {
                drmModePropertyRes *prop = drmModeGetProperty(osd->fd, props->props[p]);
                if (!prop) continue;
                if (!strcmp(prop->name, "type")) type = props->prop_values[p];
                for (unsigned int n = 0; n < 10; n++)
                    if (!strcmp(prop->name, geometry_names[n])) {
                        geometry[n] = prop->prop_id; geometry_values[n] = props->prop_values[p];
                    }
                for (unsigned int n = 0; n < 3; n++) {
                    if (!strcmp(prop->name, names[n])) {
                        property_ids[n] = prop->prop_id; values[n] = props->prop_values[p];
                        if (n == 1) {
                            for (int e = 0; e < prop->count_enums; e++)
                                if (!strcmp(prop->enums[e].name, "Pre-multiplied") &&
                                    prop->enums[e].value == 0) premultiplied = 1;
                        }
                    }
                }
                drmModeFreeProperty(prop);
            }
            drmModeFreeObjectProperties(props);
        }
        if ((plane->possible_crtcs & bit) && type == DRM_PLANE_TYPE_PRIMARY &&
            plane->crtc_id == osd->crtc) {
            osd->primary = plane->plane_id; osd->primary_fb_property = geometry[0];
            memcpy(osd->primary_geometry, geometry, sizeof(geometry));
            memcpy(osd->primary_previous, geometry_values, sizeof(geometry_values));
        }
        if ((plane->possible_crtcs & bit) && !plane->crtc_id && !plane->fb_id &&
            format && type == DRM_PLANE_TYPE_CURSOR && property_ids[0] &&
            property_ids[1] && property_ids[2] && premultiplied) {
            osd->plane = plane->plane_id;
            memcpy(osd->property, property_ids, sizeof(property_ids));
            memcpy(osd->previous, values, sizeof(values));
            memcpy(osd->geometry, geometry, sizeof(geometry));
        }
        drmModeFreePlane(plane);
    }
    drmModeFreePlaneResources(planes);
    if (osd->plane && osd->primary && osd->primary_fb_property) {
        for (unsigned int n = 0; n < 10; n++)
            if (!osd->geometry[n]) { errno = ENOTSUP; return -1; }
        return 0;
    }
    errno = ENOTSUP; return -1;
}

static int init_common(struct vdma_osd_probe *osd, int fd, uint32_t crtc,
                        const char *path)
{
    memset(osd, 0, sizeof(*osd)); osd->fd = fd; osd->crtc = crtc;
    if (drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1)) return -1;
    if (drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1)) return -1;
    if (select_plane(osd)) return -1;
    struct drm_mode_create_dumb create = {.width = 1920, .height = 1080, .bpp = 32};
    if (drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create)) return -1;
    osd->handle = create.handle; osd->pitch = create.pitch; osd->bytes = create.size;
    if (create.pitch < 1920U * 4 || create.size < (uint64_t)create.pitch * 1080) {
        errno = EPROTO; return -1;
    }
    struct drm_mode_map_dumb map = {.handle = create.handle};
    if (drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map)) return -1;
    osd->pixels = mmap(NULL, create.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, map.offset);
    if (osd->pixels == MAP_FAILED) { osd->pixels = NULL; return -1; }
    memset(osd->pixels, 0, create.size);
    FILE *image = path ? fopen(path, "rb") : NULL;
    size_t transparent = 0, opaque = 0, translucent = 0;
    if (path) {
    if (!image) return -1;
    int valid = 1;
    for (unsigned int y = 0; y < 1080; y++) {
        uint32_t *row = (uint32_t *)((unsigned char *)osd->pixels + (size_t)y * create.pitch);
        if (fread(row, 4, 1920, image) != 1920) { valid = 0; break; }
        for (unsigned int x = 0; x < 1920; x++) {
            unsigned int a = row[x] >> 24;
            if ((row[x] & 255) > a || ((row[x] >> 8) & 255) > a ||
                ((row[x] >> 16) & 255) > a) valid = 0;
            transparent += a == 0; opaque += a == 255; translucent += a > 0 && a < 255;
        }
    }
    if (fgetc(image) != EOF || ferror(image)) valid = 0;
    if (fclose(image)) valid = 0;
    if (!valid || !transparent || !opaque || !translucent) { errno = EINVAL; return -1; }
    }
    uint32_t handles[4] = {create.handle}, pitches[4] = {create.pitch}, offsets[4] = {0};
    if (drmModeAddFB2(fd, 1920, 1080, DRM_FORMAT_ARGB8888, handles, pitches,
                      offsets, &osd->framebuffer, 0)) return -1;
    uint64_t settings[3] = {65535, 0, 7};
    for (unsigned int p = 0; p < 3; p++) {
        if (drmModeObjectSetProperty(fd, osd->plane, DRM_MODE_OBJECT_PLANE,
                                    osd->property[p], settings[p])) return -1;
        osd->changed++;
    }
    printf("OSD_PROBE plane=%u crtc=%u format=ARGB8888 blend=premultiplied zpos=7 "
           "pitch=%u bytes=%llu transparent=%zu opaque=%zu translucent=%zu\n",
           osd->plane, crtc, osd->pitch, (unsigned long long)osd->bytes,
           path ? transparent : 0, path ? opaque : 0, path ? translucent : 0);
    return 0;
}

int vdma_osd_probe_init(struct vdma_osd_probe *osd, int fd, uint32_t crtc, const char *path)
{
    if (!path) { errno = EINVAL; return -1; }
    return init_common(osd, fd, crtc, path);
}
int vdma_osd_probe_init_blank(struct vdma_osd_probe *osd, int fd, uint32_t crtc)
{ return init_common(osd, fd, crtc, NULL); }

int vdma_osd_probe_show(struct vdma_osd_probe *osd, int visible)
{
    if (!!visible == osd->active && (visible || !osd->attached)) return 0;
    int result = visible ? drmModeSetPlane(osd->fd, osd->plane, osd->crtc,
        osd->framebuffer, 0, 0, 0, 1920, 1080, 0, 0, 1920U << 16, 1080U << 16) :
        drmModeSetPlane(osd->fd, osd->plane, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (result) return -1;
    osd->attached = !!visible;
    if (!!visible != osd->active) osd->toggles++;
    osd->active = !!visible;
    printf("OSD_PROBE visible=%d toggles=%llu\n", osd->active,
           (unsigned long long)osd->toggles); fflush(stdout);
    return 0;
}

int vdma_osd_probe_close(struct vdma_osd_probe *osd)
{
    int result = 0;
    if (osd->attached && vdma_osd_probe_show(osd, 0)) return -1;
    /* Caller has disabled CRTC (including the final pending flip). Restore
     * source/destination geometry without resurrecting an old FB/CRTC binding. */
    if (osd->layout_attached) {
        drmModeAtomicReq *request = drmModeAtomicAlloc();
        if (!request) return -1;
        for (unsigned int n = 2; n < 10; n++) {
            if (drmModeAtomicAddProperty(request, osd->primary, osd->primary_geometry[n],
                                         osd->primary_previous[n]) < 0) {
                drmModeAtomicFree(request); return -1;
            }
        }
        int status = drmModeAtomicCommit(osd->fd, request, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
        drmModeAtomicFree(request);
        if (status) return -1;
        osd->layout_attached = 0;
    }
    while (osd->changed) {
        unsigned int p = --osd->changed;
        if (drmModeObjectSetProperty(osd->fd, osd->plane, DRM_MODE_OBJECT_PLANE,
                                    osd->property[p], osd->previous[p])) result = -1;
    }
    /* Restore failure is safe: the plane was detached before freeing memory. */
    if (osd->pixels) munmap(osd->pixels, osd->bytes);
    if (osd->framebuffer && drmModeRmFB(osd->fd, osd->framebuffer)) result = -1;
    if (osd->handle) {
        struct drm_mode_destroy_dumb destroy = {.handle = osd->handle};
        if (drmIoctl(osd->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy)) result = -1;
    }
    osd->pixels = NULL; osd->handle = osd->framebuffer = 0;
    return result;
}

static int submit(struct vdma_osd_probe *osd, uint32_t framebuffer,
                  int visible, void *event_data, int test_only, uint32_t overlay,
                  const struct vdma_osd_writeback *writeback)
{
    drmModeAtomicReq *request = drmModeAtomicAlloc();
    if (!request) { errno = ENOMEM; return -1; }
    int result = -1;
    if (drmModeAtomicAddProperty(request, osd->primary, osd->primary_fb_property,
                                 framebuffer) < 0) goto out;
    if (osd->layout_enabled && !osd->layout_attached) {
        const struct vdma_ui_rect *r = &osd->layout.video;
        const uint64_t primary_values[9] = {osd->crtc, r->x, r->y, r->width, r->height,
            0, 0, 1920U << 16, 1080U << 16};
        for (unsigned int n = 1; n < 10; n++)
            if (drmModeAtomicAddProperty(request, osd->primary, osd->primary_geometry[n],
                                         primary_values[n-1]) < 0) goto out;
    }
    if (!osd->attached) {
        uint64_t values[10] = {overlay ? overlay : osd->framebuffer, osd->crtc,
            0, 0, 1920, 1080, 0, 0, 1920U << 16, 1080U << 16};
        for (unsigned int n = 0; n < 10; n++)
            if (drmModeAtomicAddProperty(request, osd->plane, osd->geometry[n], values[n]) < 0)
                goto out;
    } else if (overlay) {
        if (drmModeAtomicAddProperty(request, osd->plane, osd->geometry[0], overlay) < 0) goto out;
    }
    /* Keep geometry/buffer bound during the test; hide with global alpha.
     * Re-enabling Esmart incurs visible refresh stalls on this driver. */
    if (!osd->attached || !!visible != osd->active)
        if (drmModeAtomicAddProperty(request, osd->plane, osd->property[0],
                                     visible ? 65535 : 0) < 0) goto out;
    if (writeback) {
        if (drmModeAtomicAddProperty(request, writeback->connector,
                                     writeback->crtc_property, osd->crtc) < 0 ||
            drmModeAtomicAddProperty(request, writeback->connector,
                                     writeback->fb_property, writeback->framebuffer) < 0 ||
            drmModeAtomicAddProperty(request, writeback->connector,
                                     writeback->fence_property, (uintptr_t)writeback->fence) < 0)
            goto out;
    }
    result = drmModeAtomicCommit(osd->fd, request, test_only ? DRM_MODE_ATOMIC_TEST_ONLY :
        DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT |
        (writeback ? DRM_MODE_ATOMIC_ALLOW_MODESET : 0), test_only ? NULL : event_data);
    if (!result && test_only) osd->layout_tested = 1;
    if (!result && !test_only) {
        osd->attached = 1;
        if (osd->layout_enabled) osd->layout_attached = 1;
    }
    if (!result && !test_only && !!visible != osd->active) {
        osd->active = !!visible; osd->toggles++;
        printf("OSD_PROBE visible=%d toggles=%llu atomic=1\n", osd->active,
            (unsigned long long)osd->toggles); fflush(stdout);
    }
out:
    drmModeAtomicFree(request);
    return result;
}

int vdma_osd_probe_set_layout(struct vdma_osd_probe *osd, const struct vdma_ui_layout *layout)
{
    if (osd->attached || osd->layout_attached) { errno = EBUSY; return -1; }
    if (vdma_ui_layout_validate(layout)) return -1;
    for (unsigned int n = 1; n < 10; n++)
        if (!osd->primary_geometry[n]) { errno = ENOTSUP; return -1; }
    osd->layout = *layout; osd->layout_enabled = 1; osd->layout_tested = 0;
    return 0;
}

int vdma_osd_probe_test_layout(struct vdma_osd_probe *osd, uint32_t framebuffer)
{
    if (!osd->layout_enabled || osd->layout_attached) { errno = EINVAL; return -1; }
    return submit(osd, framebuffer, 1, NULL, 1, 0, NULL);
}

int vdma_osd_probe_flip(struct vdma_osd_probe *osd, uint32_t framebuffer,
                        int visible, void *event_data)
{
    if (osd->layout_enabled && !osd->layout_tested) { errno = EPROTO; return -1; }
    return submit(osd, framebuffer, visible, event_data, 0, 0, NULL);
}

int vdma_osd_probe_flip_buffer(struct vdma_osd_probe *osd, uint32_t video_fb,
                               uint32_t argb_fb, void *event_data, int test_only)
{
    if (!argb_fb || (osd->layout_enabled && !test_only && !osd->layout_tested)) {
        errno = EPROTO; return -1;
    }
    return submit(osd, video_fb, 1, event_data, test_only, argb_fb, NULL);
}

int vdma_osd_probe_flip_buffer_record(struct vdma_osd_probe *osd, uint32_t video_fb,
                                      uint32_t argb_fb, void *event_data,
                                      const struct vdma_osd_writeback *writeback)
{
    if (!argb_fb || (osd->layout_enabled && !osd->layout_tested)) {
        errno = EPROTO; return -1;
    }
    return submit(osd, video_fb, 1, event_data, 0, argb_fb, writeback);
}
