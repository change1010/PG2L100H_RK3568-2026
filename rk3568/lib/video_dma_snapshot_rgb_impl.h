/* RGB leases are drained by the video owner; cache pairing holds r->mutex. */
static bool rgb_matches(const struct vdma_rgb_frame *d,
                        const struct vdma_frame_identity *id)
{
    return d->epoch == id->stream_epoch && d->sequence == id->frame_sequence &&
        d->config == id->config_generation && d->layout == id->layout &&
        id->source_id == id->layout && id->source_id < 4 &&
        id->canvas_width == 1920 && id->canvas_height == 1080;
}

static void rgb_pair_locked(struct vdma_snapshot_runtime *r, uint64_t now)
{
    for (int j = 0; j < VDMA_SNAPSHOT_BUFFERS; ++j) {
        struct vdma_snapshot_image *image = &r->images[j];
        if (image->state != SNAP_WAIT_RGB) continue;
        int found = -1;
        for (int i = 0; i < 8; ++i)
            if (r->rgb_cache[i].valid &&
                rgb_matches(&r->rgb_cache[i].descriptor, &image->identity)) {
                found = i; break;
            }
        if (found >= 0) {
            unsigned char *spare = image->small_rgb;
            image->small_rgb = r->rgb_cache[found].pixels;
            r->rgb_cache[found].pixels = spare;
            image->rgb_descriptor = r->rgb_cache[found].descriptor;
            r->rgb_cache[found].valid = false;
            r->rgb_stats.paired++;
            r->rgb_stats.wait_ns += now - image->wait_started_ns;
            queue_locked(r, j);
        } else if (now - image->wait_started_ns >= UINT64_C(100000000)) {
            image->state = SNAP_FREE;
            r->rgb_stats.missing++;
        }
    }
}

int vdma_snapshot_enable_rgb(struct vdma_snapshot_runtime *r, int fd, unsigned int mode)
{
    struct vdma_rgb_caps c = {.version = 1, .size = sizeof(c)};
    struct vdma_rgb_control control = {1, sizeof(control), 1, 0};
    if (!r || !r->initialized || r->bound || r->rgb_mode || r->plane_enabled ||
        r->rga_reference || mode < 1 || mode > 3) {
        errno = EINVAL; return -1;
    }
    if (ioctl(fd, VDMA_RGB_CAPS, &c)) return -1;
    if (c.version != 1 || c.size != sizeof(c) || c.abi != 4 ||
        c.width != 640 || c.height != 360 || c.stride != 1920 ||
        c.bytes != VDMA_RGB_BYTES || c.format != 2 || c.preprocessing != 1 ||
        c.slots != 3 || c.slot_bytes != VDMA_RGB_SLOT_BYTES ||
        c.mmap_page != VDMA_RGB_MMAP_PAGE || c.mmap_bytes != VDMA_RGB_MAP_BYTES ||
        c.reserved[0] || c.reserved[1] || c.reserved[2]) {
        errno = EPROTO; return -1;
    }
    r->rgb_mapping = mmap(NULL, VDMA_RGB_MAP_BYTES, PROT_READ, MAP_SHARED, fd,
                         (off_t)VDMA_RGB_MMAP_PAGE * 4096);
    if (r->rgb_mapping == MAP_FAILED) { r->rgb_mapping = NULL; return -1; }
    for (int i = 0; i < 8; ++i) {
        r->rgb_cache[i].pixels = calloc(1, VDMA_RGB_BYTES);
        if (!r->rgb_cache[i].pixels) return -1;
    }
    for (int i = 0; i < VDMA_SNAPSHOT_BUFFERS; ++i) {
        r->images[i].small_rgb = calloc(1, VDMA_RGB_BYTES);
        if (!r->images[i].small_rgb) return -1;
    }
    if (mode == 3) {
        r->rgb_reference = malloc(VDMA_RGB_BYTES);
        if (!r->rgb_reference) return -1;
    }
    const char *path = getenv("VDMA_RGB_AUDIT");
    if (path && *path) {
        r->rgb_audit = fopen(path, "wx");
        if (!r->rgb_audit) return -1;
        setvbuf(r->rgb_audit, NULL, _IOFBF, 65536);
    }
    r->rgb_fd = fd;
    /* Failed ARM can have written addresses; cleanup still disables sampling. */
    r->rgb_mode = mode;
    if (ioctl(fd, VDMA_RGB_CONTROL, &control)) return -1;
    return 0;
}

int vdma_snapshot_rgb_drain(struct vdma_snapshot_runtime *r)
{
    if (!r || !r->rgb_mode) return 0;
    for (int attempt = 0; attempt < 3; ++attempt) {
        struct vdma_rgb_frame f = {.version = 1, .size = sizeof(f)};
        if (ioctl(r->rgb_fd, VDMA_RGB_ACQUIRE, &f)) {
            if (errno != EAGAIN && errno != EINTR) return -1;
            break;
        }
        int error = f.version != 1 || f.size != sizeof(f) || f.host_slot >= 3 ||
            f.width != 640 || f.height != 360 || f.bytes != VDMA_RGB_BYTES ||
            f.stride != 1920 || f.format != 2 || f.preprocessing != 1 ||
            f.layout > 4 || f.reserved[0] || f.reserved[1] ? EPROTO : 0;
        pthread_mutex_lock(&r->mutex);
        unsigned int i = r->rgb_next;
        uint64_t start = snapshot_now_ns();
        if (!error)
            memcpy(r->rgb_cache[i].pixels,
                (const unsigned char *)r->rgb_mapping + f.host_slot * VDMA_RGB_SLOT_BYTES,
                VDMA_RGB_BYTES);
        r->rgb_stats.copy_ns += snapshot_now_ns() - start;
        if (ioctl(r->rgb_fd, VDMA_RGB_RELEASE, &f)) error = errno;
        if (!error) {
            if (r->rgb_cache[i].valid) r->rgb_stats.evicted++;
            r->rgb_cache[i].descriptor = f;
            r->rgb_cache[i].valid = true;
            r->rgb_next = (i + 1) % 8;
            r->rgb_stats.received++;
            r->rgb_phase = f; r->rgb_phase_valid = true;
            rgb_pair_locked(r, snapshot_now_ns());
        } else {
            r->rgb_stats.errors++; r->rgb_cache[i].valid = false;
        }
        pthread_mutex_unlock(&r->mutex);
        if (error) { errno = error; return -1; }
    }
    pthread_mutex_lock(&r->mutex);
    rgb_pair_locked(r, snapshot_now_ns());
    pthread_mutex_unlock(&r->mutex);
    return 0;
}

void vdma_snapshot_rgb_get_stats(struct vdma_snapshot_runtime *r,
    struct vdma_snapshot_rgb_stats *stats)
{
    pthread_mutex_lock(&r->mutex);
    *stats = r->rgb_stats;
    pthread_mutex_unlock(&r->mutex);
}

static int snapshot_rgb_verify(struct vdma_snapshot_runtime *r,
                                const struct vdma_snapshot_input *in)
{
    pthread_mutex_lock(&r->mutex);
    uint64_t count = r->rgb_stats.verified;
    pthread_mutex_unlock(&r->mutex);
    if (r->rgb_mode != 3 || count >= 30) return 0;
    if (!in->pixels || !in->small_rgb || !rgb_matches(&in->rgb_descriptor, &in->identity))
        return EPROTO;
    for (unsigned y = 0; y < 360; ++y) for (unsigned x = 0; x < 640; ++x) {
        const unsigned char *p = in->pixels + (3 * y + 1) * 3840 + (3 * x + 1) * 2;
        unsigned v = p[0] | ((unsigned)p[1] << 8);
        unsigned r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
        unsigned index = (y * 640 + x) * 3;
        r->rgb_reference[index] = (r5 << 3) | (r5 >> 2);
        r->rgb_reference[index + 1] = (g6 << 2) | (g6 >> 4);
        r->rgb_reference[index + 2] = (b5 << 3) | (b5 >> 2);
    }
    size_t different = 0, first = VDMA_RGB_BYTES;
    unsigned maximum = 0;
    for (size_t i = 0; i < VDMA_RGB_BYTES; ++i) {
        unsigned a = r->rgb_reference[i], b = in->small_rgb[i];
        unsigned difference = a > b ? a - b : b - a;
        if (difference) {
            if (first == VDMA_RGB_BYTES) first = i;
            different++;
            if (difference > maximum) maximum = difference;
        }
    }
    pthread_mutex_lock(&r->mutex);
    r->rgb_stats.verified++;
    if (different) r->rgb_stats.mismatched++;
    pthread_mutex_unlock(&r->mutex);
    if (r->rgb_audit) {
        fprintf(r->rgb_audit, "{\"type\":\"pixels\",\"epoch\":%u,\"sequence\":%u,"
            "\"config\":%u,\"layout\":%u,\"bytes\":%u,\"different\":%zu,"
            "\"first_byte\":%zu,\"max_difference\":%u}\n",
            in->identity.stream_epoch, in->identity.frame_sequence,
            in->identity.config_generation, in->identity.layout, VDMA_RGB_BYTES,
            different, first, maximum);
        fflush(r->rgb_audit);
    }
    return different ? EILSEQ : 0;
}
