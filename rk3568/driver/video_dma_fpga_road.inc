/* Existing device mutex protects the stable, non-overwriting FPGA FIFO head.
 * Copy to the client before POP: EFAULT must not consume its result. No DMA
 * memory or new fences are involved in this bounded PIO operation.
 */
static long vdma_fpga_road_status_locked(struct vdma_device *v, unsigned long arg)
{
    struct vdma_fpga_road_status s;
    int rc = vdma_check_link_locked(v);
    if (rc) return rc;
    if (copy_from_user(&s, (void __user *)arg, sizeof(s))) return -EFAULT;
    if (s.version != 1 || s.size != sizeof(s) || s.reserved) return -EINVAL;
    memset(&s, 0, sizeof(s));
    s.version = 1; s.size = sizeof(s);
    s.magic = readl(v->bar + FPGA_ROAD_CAP);
    s.period = readl(v->bar + FPGA_ROAD_CAP + 4);
    s.record_bytes = readl(v->bar + FPGA_ROAD_CAP + 8);
    s.abi = readl(v->bar + FPGA_ROAD_CAP + 12);
    s.head_valid = readl(v->bar + FPGA_ROAD_STATUS + 4);
    s.dropped = readl(v->bar + FPGA_ROAD_STATUS + 8);
    s.consumed = readl(v->bar + FPGA_ROAD_STATUS + 12);
    return copy_to_user((void __user *)arg, &s, sizeof(s)) ? -EFAULT : 0;
}

static long vdma_fpga_road_read_locked(struct vdma_device *v, unsigned long arg)
{
    struct vdma_fpga_road r;
    unsigned int i;
    int rc = vdma_check_link_locked(v);
    if (rc) return rc;
    if (readl(v->bar + FPGA_ROAD_CAP) != FPGA_ROAD_MAGIC) return -EOPNOTSUPP;
    /* Cadence is capability data, not a packet ABI revision. Accept legacy
     * period=3 candidates as well as the new every-frame implementation. */
    i = readl(v->bar + FPGA_ROAD_CAP + 4);
    if (i < 1 || i > 255 ||
        readl(v->bar + FPGA_ROAD_CAP + 8) != FPGA_ROAD_RECORD_BYTES ||
        readl(v->bar + FPGA_ROAD_CAP + 12) != 1) return -EPROTO;
    if (copy_from_user(&r, (void __user *)arg, sizeof(r))) return -EFAULT;
    if (r.version != 1 || r.size != sizeof(r) || r.reserved) return -EINVAL;
    if (!(readl(v->bar + FPGA_ROAD_STATUS + 4) & 1)) return -EAGAIN;
    r.dropped = readl(v->bar + FPGA_ROAD_STATUS + 8);
    for (i = 0; i < 48; ++i)
        r.record[i] = cpu_to_le32(readl(v->bar + FPGA_ROAD_DATA + i * 4));
    if (le32_to_cpu(r.record[0]) != FPGA_ROAD_MAGIC ||
        le32_to_cpu(r.record[1]) != 1) return -EPROTO;
    if (copy_to_user((void __user *)arg, &r, sizeof(r))) return -EFAULT;
    writel(le32_to_cpu(r.record[3]), v->bar + FPGA_ROAD_POP);
    readl(v->bar + FPGA_ROAD_STATUS);
    return 0;
}
