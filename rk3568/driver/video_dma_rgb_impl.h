/* Existing small-RGB ABI4 implementation; callers hold video->lock. */
static int vdma_rgb_allocate(struct vdma_device *video)
{
    struct pci_dev *pdev = video->pdev;
    if (readl(video->bar + 0xc00) != 0x52474231)
        return 0;
    if (readl(video->bar + 0xc04) != 4 ||
        readl(video->bar + 0xc08) != VDMA_RGB_BYTES ||
        readl(video->bar + 0xc0c) != 1 ||
        readl(video->bar + 0xc34) != ((360U << 16) | 640U) ||
        readl(video->bar + 0xc38) != 1920 ||
        readl(video->bar + 0xccc) != 3)
        return -EPROTO;
    if (readl(video->bar + 0xc10) || readl(video->bar + 0xc40) ||
        readl(video->bar + 0xc50) || readl(video->bar + 0xc60) ||
        readl(video->bar + 0xc70) || readl(video->bar + 0xc90))
        return -EBUSY;
    video->rgb = dma_alloc_coherent(&pdev->dev, VDMA_RGB_MAP_BYTES,
                                    &video->rgb_dma, GFP_KERNEL);
    video->rgb_completion = dma_alloc_coherent(&pdev->dev, 4096,
                                    &video->rgb_completion_dma, GFP_KERNEL);
    if (!video->rgb || !video->rgb_completion)
        return -ENOMEM;
    if ((video->rgb_dma & 4095) || (video->rgb_completion_dma & 4095))
        return -EINVAL;
    memset(video->rgb, 0, VDMA_RGB_MAP_BYTES);
    memset(video->rgb_completion, 0, 4096);
    for (unsigned int i = 0; i < 4; ++i)
        WRITE_ONCE(*(__le32 *)(video->rgb_completion + i * 64),
                   cpu_to_le32(0xfffffffcU));
    video->rgb_present = true;
    dev_info(&pdev->dev, "RGB ABI4 ready, disabled: payload=%pad completion=%pad\n",
             &video->rgb_dma, &video->rgb_completion_dma);
    return 0;
}

static void vdma_rgb_disable(struct vdma_device *video)
{
    if (video->rgb_present && video->rgb_armed) {
        writel(0, video->bar + 0xc10);
        readl(video->bar + 0xc10);
    }
}

static long vdma_rgb_ioctl(struct vdma_device *video, unsigned int command,
                           unsigned long argument)
{
    void __user *user = (void __user *)argument;
    int rc = vdma_check_link_locked(video);
    if (rc)
        return rc;
    if (!video->rgb_present)
        return -EOPNOTSUPP;
    if (command == VDMA_RGB_CAPS) {
        struct vdma_rgb_caps c;
        if (copy_from_user(&c, user, sizeof(c)))
            return -EFAULT;
        if (c.version != 1 || c.size != sizeof(c) ||
            c.reserved[0] || c.reserved[1] || c.reserved[2])
            return -EINVAL;
        c = (struct vdma_rgb_caps){1, sizeof(c), 4, 640, 360, 1920,
            VDMA_RGB_BYTES, 2, 1, VDMA_RGB_SLOT_BYTES, 3,
            VDMA_RGB_MMAP_PAGE, VDMA_RGB_MAP_BYTES, {0, 0, 0}};
        return copy_to_user(user, &c, sizeof(c)) ? -EFAULT : 0;
    }
    if (command == VDMA_RGB_CONTROL) {
        struct vdma_rgb_control c;
        if (copy_from_user(&c, user, sizeof(c)))
            return -EFAULT;
        if (c.version != 1 || c.size != sizeof(c) || c.enable > 1 || c.reserved)
            return -EINVAL;
        if (!c.enable) {
            vdma_rgb_disable(video);
            return 0;
        }
        if (!video->rgb_armed) {
            writel(2, video->bar + 0xc20);
            writel(2, video->bar + 0xc30);
            vdma_address(video, 0xc40, video->rgb_dma);
            vdma_address(video, 0xc60, video->rgb_completion_dma);
            if (readl(video->bar + 0xc30) != 2 ||
                readl(video->bar + 0xc20) != 2)
                return -EPROTO;
            /* Sampling disable has no DMA fence. Retain the existing policy. */
            __module_get(THIS_MODULE);
            video->rgb_armed = true;
            dma_wmb();
        } else if (readl(video->bar + 0xc40) != lower_32_bits(video->rgb_dma) ||
                   readl(video->bar + 0xc50) != upper_32_bits(video->rgb_dma) ||
                   readl(video->bar + 0xc60) != lower_32_bits(video->rgb_completion_dma) ||
                   readl(video->bar + 0xc70) != upper_32_bits(video->rgb_completion_dma))
            return -ESTALE;
        writel(1, video->bar + 0xc10);
        return (readl(video->bar + 0xc10) & 3) == 3 ? 0 : -EIO;
    }
    if (command == VDMA_RGB_ACQUIRE) {
        struct vdma_rgb_frame f;
        __u32 words[16];
        __u32 serial = video->rgb_consumer + 1;
        __le32 *record = video->rgb_completion +
            (video->rgb_consumer & 3) * 64;
        if (copy_from_user(&f, user, sizeof(f)))
            return -EFAULT;
        if (f.version != 1 || f.size != sizeof(f) || f.reserved[0] || f.reserved[1])
            return -EINVAL;
        if (!video->rgb_armed)
            return -EPIPE;
        if (video->rgb_acquired)
            return -EBUSY;
        if (le32_to_cpu(READ_ONCE(record[0])) != serial)
            return -EAGAIN;
        dma_rmb();
        for (unsigned int i = 0; i < 16; ++i)
            words[i] = le32_to_cpu(READ_ONCE(record[i]));
        if (vdma_rgb_decode(words, serial, &f))
            return -EPROTO;
        if (copy_to_user(user, &f, sizeof(f)))
            return -EFAULT;
        video->rgb_lease = f;
        video->rgb_acquired = true;
        return 0;
    }
    if (command == VDMA_RGB_RELEASE) {
        struct vdma_rgb_frame f;
        if (copy_from_user(&f, user, sizeof(f)))
            return -EFAULT;
        if (!video->rgb_acquired || memcmp(&f, &video->rgb_lease, sizeof(f)))
            return -ESTALE;
        mb();
        video->rgb_consumer = f.serial;
        writel(f.serial, video->bar + 0xc80);
        readl(video->bar + 0xc80);
        video->rgb_acquired = false;
        return 0;
    }
    if (command == VDMA_RGB_STATS) {
        struct vdma_rgb_stats s = {0};
        s.version = 1; s.size = sizeof(s);
        s.producer = readl(video->bar + 0xc90);
        s.consumer = video->rgb_consumer;
        s.dropped = readl(video->bar + 0xc94);
        s.committed = readl(video->bar + 0xca0);
        s.writer_rejected = readl(video->bar + 0xca4);
        s.mailbox_skipped = readl(video->bar + 0xca8);
        s.memory_status = readl(video->bar + 0xcac);
        s.capture_rejected = readl(video->bar + 0xcb0);
        s.control = readl(video->bar + 0xc10);
        s.acquired = video->rgb_acquired;
        return copy_to_user(user, &s, sizeof(s)) ? -EFAULT : 0;
    }
    return -ENOTTY;
}
