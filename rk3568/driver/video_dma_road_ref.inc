/* Included by video_dma.c; all entry points hold the existing device lock. */
static int vdma_rr_present(struct vdma_device *v)
{
    int rc = vdma_check_link_locked(v);
    if (rc) return rc;
    if (readl(v->bar + RR_REG_CAPABILITY) != RR_WIRE_MAGIC) return -EOPNOTSUPP;
    if (readl(v->bar + RR_REG_CAPABILITY + 4) != RR_RECORD_BYTES ||
        readl(v->bar + RR_REG_CAPABILITY + 8) != RR_PAYLOAD_BYTES ||
        readl(v->bar + RR_REG_CAPABILITY + 12) != RR_WIRE_ABI) return -EPROTO;
    return 0;
}

static int vdma_rr_stop_locked(struct vdma_device *v)
{
    RoadRefWireFence *fence;
    unsigned long deadline;
    int rc;
    if (!v->rr_need_fence) return 0;
    rc = vdma_rr_present(v);
    if (rc) return rc;
    writel(0, v->bar + RR_REG_CONTROL);
    readl(v->bar + RR_REG_CONTROL);
    v->rr_running = false;
    fence = v->rr_completion + RR_FENCE_OFFSET;
    if (!v->rr_stop_cookie) {
        v->rr_stop_cookie = readl(v->bar + RR_REG_STOP_SENT) + 1;
        if (!v->rr_stop_cookie) v->rr_stop_cookie = 1;
        memset(fence, 0, sizeof(*fence));
        dma_wmb();
    }
    /* A retry keeps the same token and retained targets. A command readback
     * is diagnostic only; only the ordered memory fence permits cleanup. */
    if (readl(v->bar + RR_REG_STOP) != v->rr_stop_cookie) {
        writel(v->rr_stop_cookie, v->bar + RR_REG_STOP);
        if (readl(v->bar + RR_REG_STOP) != v->rr_stop_cookie)
            dev_err(&v->pdev->dev,
                "reference STOP command not acknowledged: token=%u readback=%u control=%#x\n",
                v->rr_stop_cookie, readl(v->bar + RR_REG_STOP),
                readl(v->bar + RR_REG_CONTROL));
    }
    deadline = jiffies + msecs_to_jiffies(3000);
    do {
        if (le32_to_cpu(READ_ONCE(fence->token)) == v->rr_stop_cookie) {
            dma_rmb();
            if (le32_to_cpu(READ_ONCE(fence->session)) != v->rr_session ||
                le32_to_cpu(READ_ONCE(fence->magic)) != RR_FENCE_MAGIC ||
                READ_ONCE(fence->reserved)) return -EPROTO;
            /* The ordered fence covers all earlier payload/commit writes.
             * Discard remaining unleased records before reconfiguration. */
            v->rr_consumer = readl(v->bar + RR_REG_COUNTERS);
            writel(v->rr_consumer, v->bar + RR_REG_CONSUMER);
            readl(v->bar + RR_REG_CONSUMER);
            v->rr_acquired = false;
            v->rr_need_fence = false;
            v->rr_stop_cookie = 0;
            return 0;
        }
        usleep_range(500, 1000);
    } while (time_before(jiffies, deadline));
    dev_err(&v->pdev->dev,
        "reference STOP fence timeout: token=%u stop=%u sent=%u control=%#x fence=%u/%u/%#x\n",
        v->rr_stop_cookie, readl(v->bar + RR_REG_STOP),
        readl(v->bar + RR_REG_STOP_SENT), readl(v->bar + RR_REG_CONTROL),
        le32_to_cpu(READ_ONCE(fence->token)),
        le32_to_cpu(READ_ONCE(fence->session)),
        le32_to_cpu(READ_ONCE(fence->magic)));
    return -ETIMEDOUT;
}

static int vdma_rr_allocate(struct vdma_device *v)
{
    unsigned int i;
    if (!v->rr_completion) {
        v->rr_completion = dma_alloc_coherent(&v->pdev->dev, RR_COMPLETION_ALLOC_BYTES,
            &v->rr_completion_dma, GFP_KERNEL);
        if (!v->rr_completion) return -ENOMEM;
    }
    for (i = 0; i < RR_HOST_SLOT_COUNT; ++i) {
        if (!v->rr_slots[i])
            v->rr_slots[i] = dma_alloc_coherent(&v->pdev->dev, PAGE_ALIGN(RR_PAYLOAD_BYTES),
                &v->rr_slot_dma[i], GFP_KERNEL);
        if (!v->rr_slots[i]) return -ENOMEM;
        if (v->rr_slot_dma[i] & 4095) return -EINVAL;
    }
    return (v->rr_completion_dma & 4095) ? -EINVAL : 0;
}

static int vdma_rr_start_locked(struct vdma_device *v, struct vdma_rr_control *c)
{
    unsigned long deadline;
    unsigned int i;
    int rc = vdma_rr_present(v);
    if (rc) return rc;
    if (c->version != VDMA_RR_VERSION || c->size != sizeof(*c) ||
        c->period < 1 || c->period > 15 || c->reserved ||
        c->session || c->producer || c->consumer || c->status) return -EINVAL;
    if (v->rr_running || v->rr_acquired) return -EBUSY;
    if (v->rr_need_fence) {
        rc = vdma_rr_stop_locked(v);
        if (rc) return rc;
    }
    deadline = jiffies + msecs_to_jiffies(250);
    while (readl(v->bar + RR_REG_CONTROL) & 2) {
        if (time_after_eq(jiffies, deadline)) return -EBUSY;
        usleep_range(100, 200);
    }
    /* A new driver must not overwrite DMA addresses left by another owner. */
    if (!v->rr_configured && (readl(v->bar + RR_REG_COMPLETION_LO) ||
        readl(v->bar + RR_REG_COMPLETION_HI))) return -EBUSY;
    rc = vdma_rr_allocate(v);
    if (rc) return rc;
    v->rr_session = readl(v->bar + RR_REG_SESSION) + 1;
    if (!v->rr_session) v->rr_session = 1;
    v->rr_consumer = readl(v->bar + RR_REG_COUNTERS);
    memset(v->rr_completion, 0, RR_COMPLETION_ALLOC_BYTES);
    for (i = 0; i < RR_COMPLETION_COUNT; ++i) {
        RoadRefWireRecord *r = v->rr_completion + i * RR_RECORD_BYTES;
        r->commit = cpu_to_le32(v->rr_consumer);
    }
    for (i = 0; i < RR_HOST_SLOT_COUNT; ++i)
        vdma_address(v, RR_REG_HOST0_LO + i * 32, v->rr_slot_dma[i]);
    vdma_address(v, RR_REG_COMPLETION_LO, v->rr_completion_dma);
    writel(v->rr_session, v->bar + RR_REG_SESSION);
    writel(c->period, v->bar + RR_REG_PERIOD);
    writel(1, v->bar + RR_REG_FORMAT);
    writel(v->rr_consumer, v->bar + RR_REG_CONSUMER);
    dma_wmb();
    v->rr_configured = true;
    v->rr_need_fence = true;
    writel(1, v->bar + RR_REG_CONTROL);
    deadline = jiffies + msecs_to_jiffies(250);
    do {
        c->status = readl(v->bar + RR_REG_CONTROL);
        if (c->status & 0x20) { rc = -EPROTO; break; }
        if (c->status & 1) {
            v->rr_running = true; c->session = v->rr_session;
            c->producer = c->consumer = v->rr_consumer;
            return 0;
        }
        usleep_range(100, 200);
    } while (time_before(jiffies, deadline));
    if (!rc) rc = -ETIMEDOUT;
    /* Keep need_fence set even on an enable timeout. */
    return rc;
}

static int vdma_rr_record_valid(const RoadRefWireRecord *r, u32 token, u32 session)
{
    unsigned int i;
    const u32 widths[] = {1920, 1280, 1280}, heights[] = {1080, 720, 720};
    const u32 offsets[] = {0, 2073600, 2995200}, lengths[] = {2073600, 921600, 921600};
#define RR_VALUE(field) le32_to_cpu(r->field)
    if (RR_VALUE(commit) != token || RR_VALUE(session) != session ||
        RR_VALUE(slot) >= RR_HOST_SLOT_COUNT || RR_VALUE(flags) != 3 ||
        RR_VALUE(magic) != RR_WIRE_MAGIC || RR_VALUE(abi) != RR_WIRE_ABI ||
        RR_VALUE(header_bytes) != RR_RECORD_BYTES || RR_VALUE(payload_bytes) != RR_PAYLOAD_BYTES ||
        RR_VALUE(source_id) != VDMA_LAYOUT_HDMI || RR_VALUE(preprocess_id) != RR_PREPROCESS_ID ||
        RR_VALUE(timestamp_hz) != RR_TIMESTAMP_HZ || RR_VALUE(plane_count) != 3 ||
        RR_VALUE(source_width) != 1920 || RR_VALUE(source_height) != 1080 ||
        RR_VALUE(total_beats) != RR_FRAME_BEATS) return -EPROTO;
#undef RR_VALUE
    for (i = 0; i < 3; ++i) {
        const RoadRefWirePlane *p = &r->planes[i];
        if (le32_to_cpu(p->kind) != i + 1 || le32_to_cpu(p->width) != widths[i] ||
            le32_to_cpu(p->height) != heights[i] || le32_to_cpu(p->stride_bytes) != widths[i] ||
            le32_to_cpu(p->offset_lo) != offsets[i] || p->offset_hi ||
            le32_to_cpu(p->length_lo) != lengths[i] || p->length_hi) return -EPROTO;
    }
    for (i = 0; i < ARRAY_SIZE(r->reserved); ++i)
        if (r->reserved[i]) return -EPROTO;
    return 0;
}

static int vdma_rr_export_locked(struct file *file, unsigned long arg)
{
    struct vdma_device *v = file->private_data;
    struct vdma_rr_export request;
    struct vdma_exported_slot *exported;
    struct dma_buf *dmabuf;
    DEFINE_DMA_BUF_EXPORT_INFO(info);
    int descriptor;

    if (copy_from_user(&request, (void __user *)arg, sizeof(request))) return -EFAULT;
    if (request.version != VDMA_RR_VERSION || request.size != sizeof(request) ||
        request.slot >= RR_HOST_SLOT_COUNT || request.reserved) return -EINVAL;
    if (!v->rr_configured || !v->rr_slots[request.slot]) return -EPIPE;
    if (v->rr_acquired) return -EBUSY;
    descriptor = get_unused_fd_flags(O_CLOEXEC);
    if (descriptor < 0) return descriptor;
    exported = kzalloc(sizeof(*exported), GFP_KERNEL);
    if (!exported) {
        put_unused_fd(descriptor);
        return -ENOMEM;
    }
    exported->owner = get_file(file);
    exported->slot = request.slot;
    exported->road_ref = true;
    info.ops = &vdma_dmabuf_ops;
    info.size = PAGE_ALIGN(RR_PAYLOAD_BYTES);
    info.flags = O_RDONLY;
    info.priv = exported;
    dmabuf = dma_buf_export(&info);
    if (IS_ERR(dmabuf)) {
        fput(exported->owner);
        kfree(exported);
        put_unused_fd(descriptor);
        return PTR_ERR(dmabuf);
    }
    fd_install(descriptor, dmabuf->file);
    return descriptor;
}

static long vdma_rr_ioctl_locked(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct vdma_device *v = file->private_data;
    void __user *user = (void __user *)arg;
    int rc = vdma_rr_present(v);
    if (rc) return rc;
    if (cmd == VDMA_RR_CAPS) {
        struct vdma_rr_caps c;
        if (copy_from_user(&c, user, sizeof(c))) return -EFAULT;
        if (c.version != 1 || c.size != sizeof(c) || c.reserved) return -EINVAL;
        c = (struct vdma_rr_caps){1, sizeof(c), RR_HOST_SLOT_COUNT,
            PAGE_ALIGN(RR_PAYLOAD_BYTES), RR_PAYLOAD_BYTES, VDMA_RR_MMAP_PAGE, RR_RECORD_BYTES, 0};
        return copy_to_user(user, &c, sizeof(c)) ? -EFAULT : 0;
    }
    if (!(file->f_mode & FMODE_WRITE)) return -EACCES;
    if (cmd == VDMA_RR_EXPORT) return vdma_rr_export_locked(file, arg);
    if (cmd == VDMA_RR_STOP) {
        if (v->rr_acquired) return -EBUSY;
        return vdma_rr_stop_locked(v);
    }
    if (cmd == VDMA_RR_START) {
        struct vdma_rr_control c;
        if (copy_from_user(&c, user, sizeof(c))) return -EFAULT;
        rc = vdma_rr_start_locked(v, &c);
        if (rc) return rc;
        return copy_to_user(user, &c, sizeof(c)) ? -EFAULT : 0;
    }
    if (cmd == VDMA_RR_ACQUIRE || cmd == VDMA_RR_RELEASE) {
        struct vdma_rr_frame f;
        u32 producer, expected = v->rr_consumer + 1;
        if (copy_from_user(&f, user, sizeof(f))) return -EFAULT;
        if (f.version != 1 || f.size != sizeof(f) ||
            f.reserved[0] || f.reserved[1] || f.reserved[2]) return -EINVAL;
        if (cmd == VDMA_RR_RELEASE) {
            if (!v->rr_acquired || f.token != expected || f.session != v->rr_session ||
                f.slot != v->rr_acquired_slot) return -ESTALE;
            dma_wmb();
            writel(expected, v->bar + RR_REG_CONSUMER);
            readl(v->bar + RR_REG_CONSUMER);
            v->rr_consumer = expected; v->rr_acquired = false;
            return 0;
        }
        if (!v->rr_running) return -EPIPE;
        if (v->rr_acquired) return -EBUSY;
        producer = readl(v->bar + RR_REG_COUNTERS);
        if ((u32)(producer - v->rr_consumer) > 3) return -EOVERFLOW;
        {
            RoadRefWireRecord *record = v->rr_completion +
                (v->rr_consumer & 3) * RR_RECORD_BYTES;
            if (le32_to_cpu(READ_ONCE(record->commit)) != expected) return -EAGAIN;
            dma_rmb();
            memcpy(&f.record, record, sizeof(f.record));
        }
        rc = vdma_rr_record_valid(&f.record, expected, v->rr_session);
        if (rc) return rc;
        f.token = expected; f.session = v->rr_session; f.slot = le32_to_cpu(f.record.slot);
        if (copy_to_user(user, &f, sizeof(f))) return -EFAULT;
        v->rr_acquired_slot = f.slot; v->rr_acquired = true;
        return 0;
    }
    return -ENOTTY;
}

static long vdma_native_identity_locked(struct vdma_device *v, unsigned long arg)
{
    struct vdma_native_identity n;
    struct vdma_frame_identity id;
    __le32 *record;
    int rc = vdma_check_link_locked(v);
    if (!rc && readl(v->bar + FPGA_ROAD_CAP) != FPGA_ROAD_MAGIC)
        rc = vdma_rr_present(v);
    if (rc) return rc;
    if (copy_from_user(&n, (void __user *)arg, sizeof(n))) return -EFAULT;
    if (n.version != 1 || n.size != sizeof(n)) return -EINVAL;
    memset(&id, 0, sizeof(id));
    id.version = 1; id.size = sizeof(id); id.slot = n.slot; id.token = n.token;
    rc = vdma_frame_identity_locked(v, &id);
    if (rc) return rc;
    if (id.layout != VDMA_LAYOUT_HDMI) return -EOPNOTSUPP;
    record = v->completion + n.slot * 64;
    n.native_sequence = le32_to_cpu(READ_ONCE(record[12]));
    n.native_epoch = le32_to_cpu(READ_ONCE(record[13]));
    n.timestamp_lo = le32_to_cpu(READ_ONCE(record[14]));
    n.timestamp_hi = le32_to_cpu(READ_ONCE(record[15]));
    return copy_to_user((void __user *)arg, &n, sizeof(n)) ? -EFAULT : 0;
}
