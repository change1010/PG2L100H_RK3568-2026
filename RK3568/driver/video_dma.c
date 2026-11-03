#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/file.h>
#include <linux/scatterlist.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/jiffies.h>
#include <linux/sched/signal.h>
#include <linux/compat.h>
#include "video_dma_uapi.h"
#include "video_dma_feature_regs.h"
#include "video_dma_feature_validate.h"
#include "video_dma_road_only.h"
#include "video_dma_snapshot_validate.h"
#include "video_dma_dark_core.h"
#include "video_dma_road_ref_uapi.h"
#include "video_dma_rgb_uapi.h"
#include "fpga_road_uapi.h"

#define FRAME_BYTES 4147200U
#define SLOT_BYTES PAGE_ALIGN(FRAME_BYTES)
/*
 * 平面像素区的 mmap 页偏移：紧跟 4 个显示槽之后。显示槽用 pgoff 0/1/2/3
 * 各选一槽（步长 = SLOT_BYTES），这里用第 5 段，不新增 ioctl 就能区分。
 */
#define PLANE_MMAP_PAGE_OFFSET (4 * (SLOT_BYTES >> PAGE_SHIFT))
#define COMPLETION_BYTES 4096U

/*
 * default CMA 池不足时，dma_alloc_coherent() 内部会回退到 alloc_pages()，驱动拿到的仍是
 * 非 NULL 地址，只有内核 mm/cma.c 打一条 "cma_alloc: reserved: alloc failed" 告警。
 * 于是"四个槽都分配成功"并不能推出"CMA 够用"。
 * 内核 5.9 起 struct cma 已私有化（公开头文件里只有前置声明），驱动无法直接读 base_pfn，
 * 因此这里只打印每个槽的 DMA 地址；来源判定放在用户态，与 dmesg 里
 * "cma: Reserved <N> MiB at <addr>" 报出的区间做比较即可，无需依赖内核内部结构。
 */
#define REG_MAGIC 0x1000
#define REG_VERSION 0x1010
#define REG_CONTROL 0x1020
#define REG_STATUS 0x1030
#define REG_CONSUMER 0x1040
#define REG_PRODUCER 0x1050
#define REG_ERRORS 0x1070
#define REG_STOP 0x1080
#define REG_STOP_SENT 0x1090
#define REG_CAPTURED 0x10a0
#define REG_DROPPED 0x10b0
#define REG_SLOTS 0x1100
#define REG_COMPLETION 0x1180
#define REG_CONTROL_MAGIC 0x1200
#define REG_CONTROL_VERSION 0x1210
#define REG_CONTROL_GENERATION 0x1220
#define REG_CONTROL_FLAGS 0x1230
#define REG_CONTROL_EVENTS 0x1240
#define REG_CONTROL_ALARMS 0x1250
#define REG_CONTROL_CANVAS 0x1260
/* Lane plane block, BAR0 register 0x127..0x13F (byte offset = reg << 4, same
 * convention as the video block above). Kept separate from REG_* on purpose:
 * these are decoded by road_plane_ctl, not by the video endpoint. */
#define PLANE_MAGIC 0x504c414eU
#define PLANE_REG(reg) ((u32)(reg) << 4)
#define PLANE_REG_CTRL PLANE_REG(0x127)
#define PLANE_REG_STAT PLANE_REG(0x128)
#define PLANE_REG_SEQ PLANE_REG(0x129)
#define PLANE_REG_GEN PLANE_REG(0x12a)
#define PLANE_REG_BASE PLANE_REG(0x12b)
#define PLANE_REG_BEATS PLANE_REG(0x12c)
#define PLANE_REG_SLOTS PLANE_REG(0x12d)
#define PLANE_REG_GEOM PLANE_REG(0x12e)
#define PLANE_REG_FMT PLANE_REG(0x12f)
#define PLANE_REG_INDEX PLANE_REG(0x130)
#define PLANE_REG_DROP PLANE_REG(0x131)
#define PLANE_REG_MAGIC PLANE_REG(0x132)
#define PLANE_REG_VER PLANE_REG(0x133)
#define PLANE_REG_HADDR_LO PLANE_REG(0x134)
#define PLANE_REG_HADDR_HI PLANE_REG(0x135)
#define PLANE_REG_CMPL_LO PLANE_REG(0x136)
#define PLANE_REG_CMPL_HI PLANE_REG(0x137)
#define PLANE_REG_CONSUMER PLANE_REG(0x138)
#define PLANE_REG_REV PLANE_REG(0x139)
#define PLANE_VERSION_SOURCE_ID 2U

/*
 * Plane payloads supported by the driver.  The current production RTL uses
 * one 640x360 Y8 plane followed by one 640x360 C8 plane.  Keep the legacy
 * GRAY8 constants for old bitstreams and old ioctl clients.
 */
#define PLANE_GRAY_BYTES 230400U
#define PLANE_YC_BYTES 460800U
#define PLANE_COMPLETION_BYTES 4096U
#define PLANE_GRAY_KIND 2U
#define PLANE_YC_KIND 3U
#define PLANE_YC_METADATA_VERSION 3U
#define PLANE_GEOMETRY ((360U << 16) | 640U)

enum vdma_io_mode {
    VDMA_MODE_NONE,
    VDMA_MODE_READ,
    VDMA_MODE_MMAP,
    VDMA_MODE_DISPLAY,
};

struct vdma_device {
    struct pci_dev *pdev;
    struct miscdevice misc;
    struct mutex lock;
    struct kref refs;
    void __iomem *bar;
    void *slots[4];
    dma_addr_t slot_dma[4];
    void *completion;
    dma_addr_t completion_dma;
    void *plane;
    dma_addr_t plane_dma;
    size_t plane_slot_bytes;
    u32 plane_payload_bytes;
    u32 plane_y_offset;
    u32 plane_y_length;
    u32 plane_c_offset;
    u32 plane_c_length;
    u32 plane_stride;
    u32 plane_kind;
    u32 plane_metadata_version;
    u32 plane_format;
    u32 plane_hardware_version;
    u32 plane_rtl_revision;
    bool plane_yc;
    bool plane_present;
    bool plane_started;
    void *rgb, *rgb_completion;
    dma_addr_t rgb_dma, rgb_completion_dma;
    u32 rgb_consumer;
    bool rgb_present, rgb_armed, rgb_acquired;
    struct vdma_rgb_frame rgb_lease;
    void *plane_completion;
    dma_addr_t plane_completion_dma;
    u32 plane_consumer;
    u32 plane_sequence;
    bool plane_acquired;
    void *rr_slots[RR_HOST_SLOT_COUNT], *rr_completion;
    dma_addr_t rr_slot_dma[RR_HOST_SLOT_COUNT], rr_completion_dma;
    u32 rr_session, rr_consumer, rr_stop_cookie, rr_acquired_slot;
    bool rr_running, rr_need_fence, rr_acquired, rr_configured;
    u32 consumer;
    u32 dequeue;
    u32 exported_mask;
    u32 stop_cookie;
    bool opened;
    bool dead;
    bool running;
    bool need_fence;
    bool quarantine;
    bool acquired;
    bool feature_acquired;
    struct vdma_feature_lease feature_lease;
    enum vdma_io_mode io_mode;
    char name[48];
};

static int vdma_rr_stop_locked(struct vdma_device *video);

struct vdma_exported_slot {
    struct file *owner;
    unsigned int slot;
    bool rgb;
    bool road_ref;
};

static struct sg_table *vdma_dmabuf_map(struct dma_buf_attachment *attachment,
                                      enum dma_data_direction direction)
{
    struct vdma_exported_slot *exported = attachment->dmabuf->priv;
    struct vdma_device *video = exported->owner->private_data;
    struct sg_table *table;
    int result;

    if (direction != DMA_TO_DEVICE && direction != DMA_BIDIRECTIONAL)
        return ERR_PTR(-EACCES);
    table = kzalloc(sizeof(*table), GFP_KERNEL);
    if (!table)
        return ERR_PTR(-ENOMEM);
    mutex_lock(&video->lock);
    if (video->dead) {
        result = -ENODEV;
        goto unlock;
    }
    if (exported->road_ref) {
        result = dma_get_sgtable(&video->pdev->dev, table,
            video->rr_slots[exported->slot], video->rr_slot_dma[exported->slot],
            PAGE_ALIGN(RR_PAYLOAD_BYTES));
    } else if (exported->rgb) {
        size_t offset = exported->slot * VDMA_RGB_SLOT_BYTES;
        result = dma_get_sgtable(&video->pdev->dev, table,
            video->rgb + offset, video->rgb_dma + offset, VDMA_RGB_SLOT_BYTES);
    } else {
        result = dma_get_sgtable(&video->pdev->dev, table, video->slots[exported->slot],
                                video->slot_dma[exported->slot], SLOT_BYTES);
    }
    if (result)
        goto unlock;
    result = dma_map_sgtable(attachment->dev, table, direction, 0);
    if (result)
        sg_free_table(table);
unlock:
    mutex_unlock(&video->lock);
    if (result) {
        kfree(table);
        return ERR_PTR(result);
    }
    return table;
}

static void vdma_dmabuf_unmap(struct dma_buf_attachment *attachment,
                             struct sg_table *table, enum dma_data_direction direction)
{
    dma_unmap_sgtable(attachment->dev, table, direction, 0);
    sg_free_table(table);
    kfree(table);
}

static int vdma_dmabuf_cpu_access(struct dma_buf *dmabuf, enum dma_data_direction direction)
{
    return -EOPNOTSUPP;
}

static void vdma_dmabuf_release(struct dma_buf *dmabuf)
{
    struct vdma_exported_slot *exported = dmabuf->priv;

    fput(exported->owner);
    kfree(exported);
}

static const struct dma_buf_ops vdma_dmabuf_ops = {
    .map_dma_buf = vdma_dmabuf_map,
    .unmap_dma_buf = vdma_dmabuf_unmap,
    .begin_cpu_access = vdma_dmabuf_cpu_access,
    .end_cpu_access = vdma_dmabuf_cpu_access,
    .release = vdma_dmabuf_release,
};

static int vdma_export_locked(struct file *file, unsigned long argument)
{
    struct vdma_device *video = file->private_data;
    struct vdma_export request;
    struct vdma_exported_slot *exported;
    struct dma_buf *dmabuf;
    DEFINE_DMA_BUF_EXPORT_INFO(info);
    int descriptor;

    if (copy_from_user(&request, (void __user *)argument, sizeof(request)))
        return -EFAULT;
    if (request.slot >= 4 || request.reserved)
        return -EINVAL;
    if (video->running || video->need_fence ||
        (video->io_mode != VDMA_MODE_NONE && video->io_mode != VDMA_MODE_DISPLAY))
        return -EBUSY;
    descriptor = get_unused_fd_flags(O_CLOEXEC);
    if (descriptor < 0)
        return descriptor;
    exported = kzalloc(sizeof(*exported), GFP_KERNEL);
    if (!exported) {
        put_unused_fd(descriptor);
        return -ENOMEM;
    }
    exported->owner = get_file(file);
    exported->slot = request.slot;
    info.ops = &vdma_dmabuf_ops;
    info.size = SLOT_BYTES;
    info.flags = O_RDONLY;
    info.priv = exported;
    dmabuf = dma_buf_export(&info);
    if (IS_ERR(dmabuf)) {
        fput(exported->owner);
        kfree(exported);
        put_unused_fd(descriptor);
        return PTR_ERR(dmabuf);
    }
    video->io_mode = VDMA_MODE_DISPLAY;
    video->exported_mask |= BIT(request.slot);
    fd_install(descriptor, dmabuf->file);
    return descriptor;
}

static int vdma_rgb_export_locked(struct file *file, unsigned long argument)
{
    struct vdma_device *video = file->private_data;
    struct vdma_rgb_export request;
    struct vdma_exported_slot *exported;
    struct dma_buf *dmabuf;
    DEFINE_DMA_BUF_EXPORT_INFO(info);
    int descriptor;

    if (copy_from_user(&request, (void __user *)argument, sizeof(request)))
        return -EFAULT;
    if (request.version != 1 || request.size != sizeof(request) ||
        request.slot >= VDMA_RGB_SLOTS || request.reserved)
        return -EINVAL;
    if (!video->rgb_present)
        return -EOPNOTSUPP;
    if (video->rgb_acquired)
        return -EBUSY;
    descriptor = get_unused_fd_flags(O_CLOEXEC);
    if (descriptor < 0)
        return descriptor;
    exported = kzalloc(sizeof(*exported), GFP_KERNEL);
    if (!exported) {
        put_unused_fd(descriptor);
        return -ENOMEM;
    }
    exported->owner = get_file(file);
    exported->slot = request.slot;
    exported->rgb = true;
    info.ops = &vdma_dmabuf_ops;
    info.size = VDMA_RGB_SLOT_BYTES;
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

static void vdma_destroy(struct kref *refs)
{
    struct vdma_device *video = container_of(refs, struct vdma_device, refs);
    unsigned int index;

    if (video->quarantine || video->plane_started || video->rr_need_fence || video->rgb_armed) {
        dev_err(&video->pdev->dev, "DMA stop unconfirmed: coherent memory quarantined until reboot; do not rebind\n");
        return;
    }
    for (index = 0; index < 4; index++)
        if (video->slots[index])
            dma_free_coherent(&video->pdev->dev, SLOT_BYTES, video->slots[index], video->slot_dma[index]);
    if (video->completion)
        dma_free_coherent(&video->pdev->dev, COMPLETION_BYTES, video->completion, video->completion_dma);
    if (video->rgb)
        dma_free_coherent(&video->pdev->dev, VDMA_RGB_MAP_BYTES, video->rgb, video->rgb_dma);
    if (video->rgb_completion)
        dma_free_coherent(&video->pdev->dev, 4096, video->rgb_completion, video->rgb_completion_dma);
    if (video->plane) {
        dma_free_coherent(&video->pdev->dev, video->plane_slot_bytes,
                          video->plane, video->plane_dma);
    }
    if (video->plane_completion)
        dma_free_coherent(&video->pdev->dev, PLANE_COMPLETION_BYTES, video->plane_completion, video->plane_completion_dma);
    for (index = 0; index < RR_HOST_SLOT_COUNT; ++index)
        if (video->rr_slots[index])
            dma_free_coherent(&video->pdev->dev, PAGE_ALIGN(RR_PAYLOAD_BYTES),
                video->rr_slots[index], video->rr_slot_dma[index]);
    if (video->rr_completion)
        dma_free_coherent(&video->pdev->dev, RR_COMPLETION_ALLOC_BYTES,
            video->rr_completion, video->rr_completion_dma);
    pci_dev_put(video->pdev);
    kfree(video);
}

static void vdma_address(struct vdma_device *video, unsigned int offset, dma_addr_t address)
{
    writel(lower_32_bits(address), video->bar + offset);
    writel(upper_32_bits(address), video->bar + offset + 16);
}

static int vdma_check_link_locked(struct vdma_device *video)
{
    u32 identity;
    u32 status;

    if (pci_read_config_dword(video->pdev, PCI_VENDOR_ID, &identity) != PCIBIOS_SUCCESSFUL ||
        identity != 0x07550755)
        return -ENODEV;
    status = readl(video->bar + REG_STATUS);
    if (status == U32_MAX || !(status & 8) ||
        readl(video->bar + REG_MAGIC) != 0x56444d41)
        return -ENODEV;
    return 0;
}

static int vdma_stop_locked(struct vdma_device *video)
{
    unsigned long deadline;
    __le32 *fence = (__le32 *)(video->completion + 256);

    if (!video->need_fence)
        return 0;
    if (vdma_check_link_locked(video)) {
        video->running = false;
        dev_err(&video->pdev->dev, "PCIe inaccessible: stop unconfirmed; buffers retained\n");
        return -ENODEV;
    }
    writel(0, video->bar + REG_CONTROL);
    readl(video->bar + REG_CONTROL);
    video->running = false;
    if (!video->stop_cookie) {
        u32 previous = readl(video->bar + REG_STOP_SENT);

        video->stop_cookie = previous + 1;
        if (!video->stop_cookie)
            video->stop_cookie = 1;
        WRITE_ONCE(*fence, cpu_to_le32(previous));
        dma_wmb();
        writel(video->stop_cookie, video->bar + REG_STOP);
        readl(video->bar + REG_STOP);
    }
    deadline = jiffies + msecs_to_jiffies(3000);
    do {
        if (le32_to_cpu(READ_ONCE(*fence)) == video->stop_cookie) {
            dma_rmb();
            video->need_fence = false;
            video->stop_cookie = 0;
            return 0;
        }
        usleep_range(500, 1000);
    } while (time_before(jiffies, deadline));
    dev_err(&video->pdev->dev, "stop fence timeout; buffers remain allocated\n");
    return -ETIMEDOUT;
}

#include "video_dma_rgb_impl.h"

static int vdma_feature_present_locked(struct vdma_device *video)
{
    int result = vdma_check_link_locked(video);
    u32 version, mask;

    if (result)
        return result;
    if (pci_resource_len(video->pdev, 0) < VFT_BAR_BYTES ||
        readl(video->bar + VFT_REG_MAGIC) != 0x31544656)
        return -ENOTTY;
    version = readl(video->bar + VFT_REG_VERSION);
    mask = readl(video->bar + VFT_REG_MASK);
    if ((version == 1 || version == 2) && mask == VDMA_FEATURE_COLOR_BIT)
        return 0;
    if (version >= 3 && (mask & (VDMA_FEATURE_COLOR_BIT | VDMA_FEATURE_ROAD_BIT)) &&
        !(mask & ~(VDMA_FEATURE_COLOR_BIT | VDMA_FEATURE_ROAD_BIT)))
        return 0;
    return -ENOTTY;
}

static int vdma_feature_matches_locked(struct vdma_device *video,
                                       u32 lease, u32 generation, u32 epoch)
{
    if (!video->feature_acquired || lease != video->feature_lease.lease_id ||
        generation != video->feature_lease.packet_generation ||
        epoch != video->feature_lease.stream_epoch)
        return -ESTALE;
    if (!(readl(video->bar + VFT_REG_STATUS) & VFT_STATUS_LEASE) ||
        readl(video->bar + VFT_REG_LEASE) != lease ||
        readl(video->bar + VFT_REG_PACKET_GENERATION) != generation ||
        readl(video->bar + VFT_REG_EPOCH) != epoch) {
        video->feature_acquired = false;
        return -ESTALE;
    }
    return 0;
}

/* ROAD commands execute in the pixel domain. BAR reads themselves remain
 * fixed-latency; bit 3 stays set until a coherent execution snapshot arrives.
 */
static int vdma_feature_wait_commands_locked(struct vdma_device *video)
{
    unsigned long deadline = jiffies + msecs_to_jiffies(250);

    do {
        u32 status = readl(video->bar + VFT_REG_STATUS);

        if (status == U32_MAX)
            return -ENODEV;
        if (!(status & VFT_STATUS_CONFIG_PENDING))
            return 0;
        if (signal_pending(current))
            return -ERESTARTSYS;
        usleep_range(50, 100);
    } while (time_before(jiffies, deadline));
    return -ETIMEDOUT;
}

static int vdma_feature_release_locked(struct vdma_device *video)
{
    const struct vdma_feature_lease *lease = &video->feature_lease;
    int result;

    if (!video->feature_acquired)
        return 0;
    result = vdma_feature_present_locked(video);
    if (result)
        goto clear;
    result = vdma_feature_matches_locked(video, lease->lease_id,
                                         lease->packet_generation, lease->stream_epoch);
    if (result)
        goto clear;
    writel(lease->lease_id, video->bar + VFT_REG_EXPECTED_LEASE);
    writel(lease->packet_generation, video->bar + VFT_REG_EXPECTED_GENERATION);
    writel(lease->stream_epoch, video->bar + VFT_REG_EXPECTED_EPOCH);
    writel(2, video->bar + VFT_REG_COMMAND);
    result = vdma_feature_wait_commands_locked(video);
    if (result)
        goto clear;
    if (readl(video->bar + VFT_REG_COMMAND) != 2)
        result = -ESTALE;
clear:
    video->feature_acquired = false;
    return result;
}

static int vdma_feature_config_locked(struct vdma_device *video,
                                      struct vdma_feature_config *config)
{
    unsigned long deadline;
    u32 words[32];
    u32 expected_generation;
    unsigned int index;
    int result;

    if ((config->version != 3 && config->version != 4 && config->version != 5) ||
        config->enable_mask != VDMA_FEATURE_ROAD_BIT)
        return -EOPNOTSUPP;
    if (!vdma_road_only_config_valid(config))
        return -EINVAL;
    if (readl(video->bar + VFT_REG_VERSION) != VDMA_ROAD_ONLY_TRANSPORT_VERSION ||
        readl(video->bar + VFT_REG_MASK) != VDMA_FEATURE_ROAD_BIT)
        return -EOPNOTSUPP;
    if (readl(video->bar + VFT_REG_STATUS) & VFT_STATUS_CONFIG_PENDING)
        return -EBUSY;
    result = vdma_feature_release_locked(video);
    if (result && result != -ESTALE) return result;
    memcpy(words, config, sizeof(words));
    for (index = 0; index < ARRAY_SIZE(words); index++) {
        writel(index, video->bar + VFT_REG_CONFIG_INDEX);
        writel(words[index], video->bar + VFT_REG_CONFIG_DATA);
    }
    /* ROAD-only: wait for shadow writes before issuing APPLY. */
    result = vdma_feature_wait_commands_locked(video);
    if (result)
        return result;
    expected_generation = readl(video->bar + VFT_REG_CONFIG_GENERATION) + 1;
    writel(config->request_id, video->bar + VFT_REG_CONFIG_APPLY);
    readl(video->bar + VFT_REG_CONFIG_APPLY);
    deadline = jiffies + msecs_to_jiffies(250);
    do {
        u32 error = readl(video->bar + VFT_REG_CONFIG_ERROR);
        u32 status = readl(video->bar + VFT_REG_STATUS);

        if (status == U32_MAX || error == U32_MAX)
            return -ENODEV;
        if (error)
            return error == 2 ? -EBUSY : -EINVAL;
        if (!(status & VFT_STATUS_CONFIG_PENDING) &&
            readl(video->bar + VFT_REG_CONFIG_REQUEST) == config->request_id &&
            readl(video->bar + VFT_REG_CONFIG_GENERATION) == expected_generation) {
            config->active_generation = readl(video->bar + VFT_REG_CONFIG_GENERATION);
            config->applied_request_id = config->request_id;
            return 0;
        }
        if (signal_pending(current))
            return -ERESTARTSYS;
        usleep_range(500, 1000);
    } while (time_before(jiffies, deadline));
    /* A timeout is not cancellation. Return observed ACK fields for diagnosis. */
    config->active_generation = readl(video->bar + VFT_REG_CONFIG_GENERATION);
    config->applied_request_id = readl(video->bar + VFT_REG_CONFIG_REQUEST);
    return -ETIMEDOUT;
}

static int vdma_feature_acquire_locked(struct vdma_device *video,
                                       struct vdma_feature_lease *lease)
{
    unsigned int attempt;
    u32 requested_type = lease->feature_mask;
    int result;

    if (!vdma_road_only_type_valid(requested_type)) return -EOPNOTSUPP;
    if (readl(video->bar + VFT_REG_VERSION) != VDMA_ROAD_ONLY_TRANSPORT_VERSION ||
        readl(video->bar + VFT_REG_MASK) != VDMA_FEATURE_ROAD_BIT) return -EOPNOTSUPP;
    if (video->feature_acquired) {
        result = vdma_feature_matches_locked(video, video->feature_lease.lease_id,
                video->feature_lease.packet_generation, video->feature_lease.stream_epoch);
        if (!result)
            return -EBUSY;
    }
    result = vdma_feature_wait_commands_locked(video);
    if (result) return result;
    if (requested_type) {
        writel(requested_type, video->bar + VFT_REG_TYPE);
        if (readl(video->bar + VFT_REG_TYPE) != requested_type) return -EBUSY;
    }
    for (attempt = 0; attempt < 8; attempt++) {
        u32 generation = readl(video->bar + VFT_REG_DESCRIPTOR_GENERATION);

        if (!readl(video->bar + VFT_REG_READY))
            return -EAGAIN;
        lease->version = 1;
        lease->size = sizeof(*lease);
        lease->packet_generation = readl(video->bar + VFT_REG_PACKET_GENERATION);
        lease->stream_epoch = readl(video->bar + VFT_REG_EPOCH);
        if (generation != readl(video->bar + VFT_REG_DESCRIPTOR_GENERATION))
            continue;
        writel(0, video->bar + VFT_REG_EXPECTED_LEASE);
        writel(lease->packet_generation, video->bar + VFT_REG_EXPECTED_GENERATION);
        writel(lease->stream_epoch, video->bar + VFT_REG_EXPECTED_EPOCH);
        writel(1, video->bar + VFT_REG_COMMAND);
        result = vdma_feature_wait_commands_locked(video);
        if (result)
            return result;
        if (readl(video->bar + VFT_REG_COMMAND) != 1)
            continue;
        lease->lease_id = readl(video->bar + VFT_REG_LEASE);
        lease->frame_sequence = readl(video->bar + VFT_REG_SEQUENCE);
        lease->packet_bytes = readl(video->bar + VFT_REG_BYTES);
        lease->feature_mask = readl(video->bar + VFT_REG_FEATURES);
        lease->status_flags = readl(video->bar + VFT_REG_FLAGS);
        video->feature_lease = *lease;
        video->feature_acquired = true;
        /* Claim used expected_lease=0. Authorize packet-window reads with
         * the assigned lease before entering READ; ROAD RAM rejects zero.
         */
        writel(lease->lease_id, video->bar + VFT_REG_EXPECTED_LEASE);
        result = vdma_feature_wait_commands_locked(video);
        if (result) {
            vdma_feature_release_locked(video);
            return result;
        }
        if (!lease->lease_id || lease->feature_mask != VDMA_FEATURE_ROAD_BIT ||
            (requested_type && lease->feature_mask != requested_type) ||
            !lease->packet_bytes || lease->packet_bytes > VDMA_FEATURE_ROAD_MAX_PACKET_BYTES ||
            lease->packet_bytes < 176 || (lease->packet_bytes & 15U) ||
            lease->status_flags & ~0x3fU) {
            vdma_feature_release_locked(video);
            return -EPROTO;
        }
        result = vdma_feature_matches_locked(video, lease->lease_id,
                                             lease->packet_generation, lease->stream_epoch);
        if (!result)
            return 0;
    }
    return -EAGAIN;
}

static int vdma_feature_read_locked(struct vdma_device *video,
                                    struct vdma_feature_read *request)
{
    unsigned int copied = 0;
    u32 last_page = U32_MAX;
    int result;

    if (request->version != 1 || request->size != sizeof(*request) || request->reserved ||
        !request->bytes || request->bytes > VDMA_FEATURE_READ_BYTES)
        return -EINVAL;
    result = vdma_feature_matches_locked(video, request->lease_id,
                                         request->packet_generation, request->stream_epoch);
    if (result)
        return result;
    if (!vdma_feature_read_valid(request, video->feature_lease.packet_bytes))
        return -EINVAL;
    while (copied < request->bytes) {
        u32 absolute = request->offset + copied;
        u32 page = absolute / VDMA_FEATURE_READ_BYTES;
        u32 offset = absolute % VDMA_FEATURE_READ_BYTES;
        u32 value;

        if (page != last_page) {
            writel(page, video->bar + VFT_REG_PAGE);
            if (readl(video->bar + VFT_REG_PAGE) != page)
                return -EIO;
            last_page = page;
        }
        value = readl(video->bar + VFT_REG_WINDOW + (offset & ~3U));
        do {
            request->data[copied++] = value >> ((offset & 3U) * 8);
            offset++;
        } while ((offset & 3U) && copied < request->bytes);
    }
    return vdma_feature_matches_locked(video, request->lease_id,
                                       request->packet_generation, request->stream_epoch);
}

static int vdma_frame_identity_locked(struct vdma_device *video,
                                      struct vdma_frame_identity *identity)
{
    __le32 *record;
    u32 cursor = identity->token - 1;

    if (identity->version != 1 || identity->size != sizeof(*identity) || identity->slot >= 4)
        return -EINVAL;
    if (identity->slot != (cursor & 3) ||
        (video->io_mode == VDMA_MODE_DISPLAY ?
         (u32)(cursor - video->consumer) >= (u32)(video->dequeue - video->consumer) :
         video->io_mode != VDMA_MODE_MMAP || !video->acquired || cursor != video->consumer))
        return -ESTALE;
    record = (__le32 *)(video->completion + identity->slot * 64);
    if (le32_to_cpu(READ_ONCE(record[0])) != identity->token)
        return -ESTALE;
    dma_rmb();
    if (le32_to_cpu(READ_ONCE(record[10])) != VFT_IDENTITY_MAGIC)
        return -ENOTTY;
    identity->stream_epoch = le32_to_cpu(READ_ONCE(record[8]));
    identity->frame_sequence = le32_to_cpu(READ_ONCE(record[4]));
    identity->layout = le32_to_cpu(READ_ONCE(record[5]));
    identity->source_id = le32_to_cpu(READ_ONCE(record[11]));
    identity->config_generation = le32_to_cpu(READ_ONCE(record[9]));
    identity->canvas_width = 1920;
    identity->canvas_height = 1080;
    identity->flags = 0;
    if (identity->source_id != identity->layout || identity->layout > VDMA_LAYOUT_MAX)
        return -EPROTO;
    return 0;
}

static int vdma_open(struct inode *inode, struct file *file)
{
    struct vdma_device *video = container_of(file->private_data, struct vdma_device, misc);
    int result = 0;

    mutex_lock(&video->lock);
    if (video->dead)
        result = -ENODEV;
    else if (video->opened)
        result = -EBUSY;
    else if (video->need_fence)
        result = vdma_stop_locked(video);
    if (!result) {
        video->opened = true;
        video->acquired = false;
        video->feature_acquired = false;
        video->dequeue = video->consumer;
        video->exported_mask = 0;
        video->io_mode = VDMA_MODE_NONE;
        kref_get(&video->refs);
        file->private_data = video;
    }
    mutex_unlock(&video->lock);
    return result;
}

static int vdma_release(struct inode *inode, struct file *file)
{
    struct vdma_device *video = file->private_data;

    mutex_lock(&video->lock);
    if (!video->dead)
        vdma_stop_locked(video);
    if (!video->dead)
        vdma_feature_release_locked(video);
    if (!video->dead && vdma_rr_stop_locked(video))
        video->quarantine = true;
    if (!video->dead) {
        vdma_rgb_disable(video);
        if (video->rgb_acquired) {
            mb();
            video->rgb_consumer = video->rgb_lease.serial;
            writel(video->rgb_consumer, video->bar + 0xc80);
            readl(video->bar + 0xc80);
            video->rgb_acquired = false;
        }
    }
    video->acquired = false;
    video->opened = false;
    mutex_unlock(&video->lock);
    kref_put(&video->refs, vdma_destroy);
    return 0;
}

static int vdma_wait_frame_locked(struct file *file, u32 cursor,
                                  struct vdma_frame_header *header)
{
    struct vdma_device *video = file->private_data;
    __le32 *record = (__le32 *)(video->completion + (cursor & 3) * 64);
    unsigned long deadline = jiffies + msecs_to_jiffies(3000);

    if (video->dead || !video->running)
        return video->dead ? -ENODEV : -EPIPE;
    while (le32_to_cpu(READ_ONCE(record[0])) != cursor + 1) {
        if (file->f_flags & O_NONBLOCK)
            return -EAGAIN;
        if (signal_pending(current))
            return -ERESTARTSYS;
        if (time_after_eq(jiffies, deadline))
            return -ETIMEDOUT;
        usleep_range(500, 1000);
    }
    dma_rmb();
    if (le32_to_cpu(READ_ONCE(record[6])) != FRAME_BYTES ||
        le32_to_cpu(READ_ONCE(record[7])) != 1)
        return -EIO;
    header->magic = 0x5646524d;
    header->sequence = le32_to_cpu(READ_ONCE(record[4]));
    header->width = 1920;
    header->height = 1080;
    header->stride = 3840;
    header->format = 0x36314752;
    header->layout = le32_to_cpu(READ_ONCE(record[5]));
    header->bytes = FRAME_BYTES;
    return 0;
}

static void vdma_consume_locked(struct vdma_device *video)
{
    mb();
    video->consumer++;
    writel(video->consumer, video->bar + REG_CONSUMER);
}

static int vdma_control_status_locked(struct vdma_device *video,
                                      struct vdma_control_status *status)
{
    unsigned int attempt;
    u32 generation;

    if (pci_resource_len(video->pdev, 0) < REG_CONTROL_CANVAS + sizeof(u32))
        return -EOPNOTSUPP;
    if (readl(video->bar + REG_CONTROL_MAGIC) != 0x53543143 ||
        readl(video->bar + REG_CONTROL_VERSION) != 1)
        return -EOPNOTSUPP;
    for (attempt = 0; attempt < 8; attempt++) {
        generation = readl(video->bar + REG_CONTROL_GENERATION);
        if (!generation)
            return -EAGAIN;
        status->version = 1;
        status->generation = generation;
        status->flags = readl(video->bar + REG_CONTROL_FLAGS);
        status->event_sequence = readl(video->bar + REG_CONTROL_EVENTS);
        status->alarm_reset_sequence = readl(video->bar + REG_CONTROL_ALARMS);
        status->canvas_sequence = readl(video->bar + REG_CONTROL_CANVAS);
        if (generation == readl(video->bar + REG_CONTROL_GENERATION)) {
            if ((status->flags & 7) > VDMA_LAYOUT_MAX ||
                ((status->flags >> 3) & 7) > 4 ||
                ((status->flags >> 7) & 3) > 2 || status->flags & ~0x1ffU)
                return -EPROTO;
            return 0;
        }
    }
    return -EAGAIN;
}

static long vdma_frame_identity_ioctl_locked(struct file *file,
                                             unsigned long argument)
{
    struct vdma_device *video = file->private_data;
    void __user *user = (void __user *)argument;
    struct vdma_frame_identity identity;
    int result = vdma_check_link_locked(video);

    if (result)
        return result;
    if (copy_from_user(&identity, user, sizeof(identity)))
        return -EFAULT;
    if (identity.stream_epoch || identity.frame_sequence || identity.source_id ||
        identity.layout || identity.config_generation || identity.canvas_width ||
        identity.canvas_height || identity.flags)
        return -EINVAL;
    /* Completion identity remains available after the old ROAD BAR is retired. */
    result = vdma_frame_identity_locked(video, &identity);
    if (!result && copy_to_user(user, &identity, sizeof(identity)))
        return -EFAULT;
    return result;
}

static long vdma_feature_ioctl_locked(struct file *file, unsigned int command,
                                      unsigned long argument)
{
    struct vdma_device *video = file->private_data;
    void __user *user = (void __user *)argument;
    struct vdma_feature_caps caps;
    struct vdma_feature_config config;
    struct vdma_feature_lease lease;
    struct vdma_feature_read *request;
    unsigned int index;
    int result = vdma_feature_present_locked(video);

    if (result)
        return result;
    switch (command) {
    case VDMA_FEATURE_CAPS:
        if (copy_from_user(&caps, user, sizeof(caps)))
            return -EFAULT;
        if ((caps.version != 1 && caps.version != 2) || caps.size != sizeof(caps) || caps.reserved)
            return -EINVAL;
        index = caps.version;
        memset(&caps, 0, sizeof(caps));
        caps.version = index;
        caps.size = sizeof(caps);
        caps.feature_mask = readl(video->bar + VFT_REG_MASK);
        if (readl(video->bar + VFT_REG_VERSION) != VDMA_ROAD_ONLY_TRANSPORT_VERSION ||
            caps.feature_mask != VDMA_FEATURE_ROAD_BIT)
            return -EOPNOTSUPP;
        caps.max_packet_bytes = readl(video->bar + VFT_REG_CAPACITY);
        if (caps.max_packet_bytes != VDMA_FEATURE_ROAD_MAX_PACKET_BYTES)
            return -EPROTO;
        if (caps.version == 2) {
            caps.profile_mask = readl(video->bar + VFT_REG_PROFILES);
            if (caps.profile_mask != VDMA_FEATURE_SINGLE_PROFILES)
                return -EPROTO;
        }
        caps.max_lane_rows = 96;
        caps.max_lane_per_row = 13;
        caps.packet_banks = 2;
        caps.aperture_bytes = VDMA_FEATURE_READ_BYTES;
        return copy_to_user(user, &caps, sizeof(caps)) ? -EFAULT : 0;
    case VDMA_FEATURE_CONFIG:
        if (copy_from_user(&config, user, sizeof(config)))
            return -EFAULT;
        result = vdma_feature_config_locked(video, &config);
        if ((!result || result == -ETIMEDOUT) && copy_to_user(user, &config, sizeof(config)))
            return -EFAULT;
        return result;
    case VDMA_FEATURE_ACQUIRE:
    case VDMA_FEATURE_RELEASE:
        if (copy_from_user(&lease, user, sizeof(lease)))
            return -EFAULT;
        if (lease.version != 1 || lease.size != sizeof(lease))
            return -EINVAL;
        for (index = 0; index < ARRAY_SIZE(lease.reserved); index++)
            if (lease.reserved[index])
                return -EINVAL;
        if (command == VDMA_FEATURE_RELEASE) {
            result = vdma_feature_matches_locked(video, lease.lease_id,
                                                 lease.packet_generation, lease.stream_epoch);
            return result ? result : vdma_feature_release_locked(video);
        }
        /* feature_mask is the ACQUIRE input selector, not only an output.
         * Preserve it while clearing descriptor fields supplied by userspace. */
        {
            u32 requested_type = lease.feature_mask;

            if (!vdma_road_only_type_valid(requested_type))
                return -EOPNOTSUPP;
            memset(&lease, 0, sizeof(lease));
            lease.feature_mask = requested_type;
        }
        result = vdma_feature_acquire_locked(video, &lease);
        if (!result && copy_to_user(user, &lease, sizeof(lease))) {
            vdma_feature_release_locked(video);
            result = -EFAULT;
        }
        return result;
    case VDMA_FEATURE_READ:
        request = kzalloc(sizeof(*request), GFP_KERNEL);
        if (!request)
            return -ENOMEM;
        if (copy_from_user(request, user, sizeof(*request))) {
            kfree(request);
            return -EFAULT;
        }
        memset(request->data, 0, sizeof(request->data));
        result = vdma_feature_read_locked(video, request);
        if (!result && copy_to_user(user, request, sizeof(*request)))
            result = -EFAULT;
        kfree(request);
        return result;
    default:
        return -ENOTTY;
    }
}

/* Called with video->lock held: RELEASE/STOP cannot race the CPU copy.
 * FPGA cannot reuse a held slot until ordered consumer advancement.
 */
static int vdma_snapshot_copy_locked(struct file *file, unsigned long argument)
{
    struct vdma_device *video = file->private_data;
    struct vdma_snapshot_copy request;
    int result;

    if (copy_from_user(&request, (void __user *)argument, sizeof(request)))
        return -EFAULT;
    result = vdma_snapshot_request_valid(&request);
    if (result)
        return result;
    if (video->io_mode != VDMA_MODE_DISPLAY)
        return -EBUSY;
    result = vdma_check_link_locked(video);
    if (result)
        return result;
    if (!video->running)
        return -EPIPE;
    request.actual = request.expected;
    result = vdma_frame_identity_locked(video, &request.actual);
    if (result)
        return result;
    if (!vdma_snapshot_identity_equal(&request.expected, &request.actual))
        return -ESTALE;
    result = vdma_wait_frame_locked(file, request.expected.token - 1, &request.header);
    if (result)
        return result;
    if (request.header.sequence != request.actual.frame_sequence ||
        request.header.layout != request.actual.layout)
        return -EIO;
    if (copy_to_user(u64_to_user_ptr(request.destination),
                     video->slots[request.expected.slot], FRAME_BYTES))
        return -EFAULT;
    request.copied_bytes = FRAME_BYTES;
    if (copy_to_user((void __user *)argument, &request, sizeof(request)))
        return -EFAULT;
    return 0;
}

static u32 vdma_dark_mmio_read(void *context, u32 offset)
{
    struct vdma_device *video = context;
    return readl(video->bar + offset);
}

static void vdma_dark_mmio_write(void *context, u32 offset, u32 value)
{
    struct vdma_device *video = context;
    writel(value, video->bar + offset);
}

static long vdma_dark_ioctl_locked(struct file *file, unsigned int command,
                                   unsigned long argument)
{
    struct vdma_device *video = file->private_data;
    const struct vdma_dark_io io = {video, vdma_dark_mmio_read, vdma_dark_mmio_write};
    int result;

    BUILD_BUG_ON(sizeof(struct vdma_dark_status) != 104);
    BUILD_BUG_ON(sizeof(struct vdma_dark_command) != 48);
    if (pci_resource_len(video->pdev, 0) < DARK_BAR_BYTES)
        return -EOPNOTSUPP;
    result = vdma_check_link_locked(video);
    if (result)
        return result;
    if (command == VDMA_DARK_STATUS) {
        struct vdma_dark_status s;
        if (copy_from_user(&s, (void __user *)argument, sizeof(s)))
            return -EFAULT;
        if (s.version != VDMA_DARK_API_VERSION || s.size != sizeof(s) ||
            !vdma_dark_zero(s.reserved, 4))
            return -EINVAL;
        result = vdma_dark_read_status(&io, &s);
        if (result)
            return result;
        return copy_to_user((void __user *)argument, &s, sizeof(s)) ? -EFAULT : 0;
    } else {
        struct vdma_dark_command request;
        if (!(file->f_mode & FMODE_WRITE))
            return -EACCES;
        if (copy_from_user(&request, (void __user *)argument, sizeof(request)))
            return -EFAULT;
        result = vdma_dark_submit(&io, &request);
        if (result)
            return result;
        /* EFAULT here does not roll back a command accepted by the hardware. */
        return copy_to_user((void __user *)argument, &request, sizeof(request)) ? -EFAULT : 0;
    }
}

/*
 * 平面帧只租一块宿主窗口（road_plane_tx 的载荷地址是 cfg_host_addr + byte_offset，
 * 与槽号无关），所以正确性完全依赖 CONSUMER 的推进时机：ACQUIRE 时不推进、RELEASE 时才推进。
 * 配合 0x12d SLOTS=1，inflight 只可能是 0 或 1，用户在读完之前 FPGA 不会覆写窗口。
 */
static int vdma_plane_acquire_locked(struct vdma_device *video, void __user *user, bool full_identity)
{
    struct vdma_plane_frame_v2 descriptor;
    struct vdma_plane_frame *frame = &descriptor.frame;
    const __le32 *record;
    unsigned int slot;
    u32 producer;
    int result = 0;

    if (video->dead) { result = -ENODEV; goto out; }
    if (!video->plane_present) { result = -EOPNOTSUPP; goto out; }
    if (video->plane_yc) { result = -EPROTO; goto out; }
    if (video->plane_acquired) { result = -EBUSY; goto out; }
    if (vdma_check_link_locked(video)) { result = -EIO; goto out; }
    producer = readl(video->bar + PLANE_REG_SEQ);
    if (producer == video->plane_consumer) { result = -EAGAIN; goto out; }
    dma_rmb();
    slot = (producer - 1) & 3U;
    record = (const __le32 *)(video->plane_completion + slot * 64);
    /*
     * COMMIT 包是整帧的最后一笔写，它的 dword0 就是提交序号（RTL 写
     * producer+1，随后 producer 自增到同一值，所以与 0x129 相等）。
     * d1/d2 是常量（有效字节数 / KIND），任何槽都能通过，无法判断
     * 新鲜度——只查它们会让陈旧槽静默过关，因此必须先比 dword0。
     */
    if (le32_to_cpu(record[0]) != producer) {
        result = -EAGAIN;
        goto out;
    }
    if (le32_to_cpu(record[5]) != PLANE_GRAY_BYTES ||
        le32_to_cpu(record[6]) != PLANE_GRAY_KIND) {
        dev_err(&video->pdev->dev,
                "plane record rejected: seq=%u word0=%u bytes=%u kind=%u\n",
                producer, le32_to_cpu(record[0]), le32_to_cpu(record[5]),
                le32_to_cpu(record[6]));
        result = -EIO; goto out;
    }
    memset(&descriptor, 0, sizeof(descriptor));
    frame->version = full_identity ? 2U : 1U;
    frame->size = full_identity ? sizeof(descriptor) : sizeof(*frame);
    frame->sequence = producer;
    frame->generation = readl(video->bar + PLANE_REG_GEN);
    frame->geometry = readl(video->bar + PLANE_REG_GEOM);
    frame->format = readl(video->bar + PLANE_REG_FMT);
    /*
     * 用户态把这个值直接当 mmap 的 offset 用，落到平面像素区。
     * 不能用 0：offset=0 选中的是显示槽 0，会读到显示画面而不是灰度平面。
     */
    frame->offset = (u32)(PLANE_MMAP_PAGE_OFFSET << PAGE_SHIFT);
    frame->length = PLANE_GRAY_BYTES;
    if (full_identity && readl(video->bar + PLANE_REG_VER) == PLANE_VERSION_SOURCE_ID) {
        if (le32_to_cpu(record[4]) != PLANE_VERSION_SOURCE_ID) {
            result = -EPROTO;
            goto out;
        }
        descriptor.source_epoch = le32_to_cpu(record[9]);
        descriptor.source_sequence = le32_to_cpu(record[8]);
        descriptor.source_layout = le32_to_cpu(record[7]);
        descriptor.source_config_generation = le32_to_cpu(record[10]);
        if (descriptor.source_layout > VDMA_LAYOUT_MAX) {
            result = -EPROTO;
            goto out;
        }
        frame->flags |= VDMA_PLANE_SOURCE_ID;
    }
    if (copy_to_user(user, &descriptor, full_identity ? sizeof(descriptor) : sizeof(*frame))) { result = -EFAULT; goto out; }
    video->plane_acquired = true;
    video->plane_sequence = producer;
out:
    return result;
}

static int vdma_plane_acquire_v3_locked(struct vdma_device *video, void __user *user)
{
    struct vdma_plane_frame_v3 descriptor;
    struct vdma_plane_frame *frame = &descriptor.frame;
    const __le32 *record;
    unsigned int slot;
    u32 producer;
    int result = 0;

    if (video->dead) { result = -ENODEV; goto out; }
    if (!video->plane_present) { result = -EOPNOTSUPP; goto out; }
    if (!video->plane_yc) { result = -EPROTO; goto out; }
    if (video->plane_acquired) { result = -EBUSY; goto out; }
    if (vdma_check_link_locked(video)) { result = -EIO; goto out; }
    producer = readl(video->bar + PLANE_REG_SEQ);
    if (producer == video->plane_consumer) { result = -EAGAIN; goto out; }
    dma_rmb();
    slot = (producer - 1) & 3U;
    record = (const __le32 *)(video->plane_completion + slot * 64);
    if (le32_to_cpu(record[0]) != producer) { result = -EAGAIN; goto out; }
    if (le32_to_cpu(record[4]) != PLANE_YC_METADATA_VERSION ||
        le32_to_cpu(record[5]) != PLANE_YC_BYTES ||
        le32_to_cpu(record[6]) != PLANE_YC_KIND) {
        dev_err(&video->pdev->dev,
                "YC plane record rejected: seq=%u meta=%u bytes=%u kind=%u\n",
                producer, le32_to_cpu(record[4]), le32_to_cpu(record[5]),
                le32_to_cpu(record[6]));
        result = -EIO;
        goto out;
    }
    memset(&descriptor, 0, sizeof(descriptor));
    frame->version = 3U;
    frame->size = sizeof(descriptor);
    frame->sequence = producer;
    frame->generation = readl(video->bar + PLANE_REG_GEN);
    frame->geometry = readl(video->bar + PLANE_REG_GEOM);
    frame->format = readl(video->bar + PLANE_REG_FMT);
    frame->offset = (u32)(PLANE_MMAP_PAGE_OFFSET << PAGE_SHIFT);
    frame->length = PLANE_YC_BYTES;
    frame->flags = VDMA_PLANE_SOURCE_ID | VDMA_PLANE_YC;
    descriptor.y_offset = 0;
    descriptor.y_length = PLANE_GRAY_BYTES;
    descriptor.c_offset = PLANE_GRAY_BYTES;
    descriptor.c_length = PLANE_GRAY_BYTES;
    descriptor.stride = 640;
    descriptor.metadata_version = le32_to_cpu(record[4]);
    descriptor.kind = le32_to_cpu(record[6]);
    descriptor.rtl_revision = readl(video->bar + PLANE_REG_REV);
    descriptor.source_sequence = le32_to_cpu(record[8]);
    descriptor.source_epoch = le32_to_cpu(record[9]);
    descriptor.source_layout = le32_to_cpu(record[7]);
    descriptor.source_config_generation = le32_to_cpu(record[10]);
    if (frame->format != VDMA_PLANE_FORMAT_YC8 ||
        frame->geometry != PLANE_GEOMETRY ||
        descriptor.source_layout > VDMA_LAYOUT_MAX) {
        result = -EPROTO;
        goto out;
    }
    if (copy_to_user(user, &descriptor, sizeof(descriptor))) {
        result = -EFAULT;
        goto out;
    }
    video->plane_acquired = true;
    video->plane_sequence = producer;
out:
    return result;
}

static int vdma_plane_release_locked(struct vdma_device *video, struct vdma_plane_frame __user *user)
{
    struct vdma_plane_frame frame;
    u32 live;
    int result = 0;

    if (video->dead) { result = -ENODEV; goto out; }
    if (!video->plane_acquired) { result = -EINVAL; goto out; }
    if (copy_from_user(&frame, user, sizeof(frame))) { result = -EFAULT; goto out; }
    if (frame.sequence != video->plane_sequence) { result = -EINVAL; goto out; }
    /*
     * 只接受"已经提交过"的序号。CONSUMER 一旦超过 producer，TX 里
     * inflight = producer - consumer 会下溢成 0xFFFFFFFF，room 永久为 0，
     * 平面通路会自锁到复位为止。这里挡住用户态传入未来序号。
     */
    live = readl(video->bar + PLANE_REG_SEQ);
    if ((u32)(live - frame.sequence) > 1U) {
        dev_err(&video->pdev->dev,
                "plane release rejected: seq=%u ahead of producer=%u\n",
                frame.sequence, live);
        result = -EINVAL; goto out;
    }
    writel(frame.sequence, video->bar + PLANE_REG_CONSUMER);
    readl(video->bar + PLANE_REG_CONSUMER);
    video->plane_consumer = frame.sequence;
    video->plane_acquired = false;
out:
    return result;
}

#include "video_dma_road_ref.inc"
#include "video_dma_fpga_road.inc"

static long vdma_ioctl(struct file *file, unsigned int command, unsigned long argument)
{
    struct vdma_device *video = file->private_data;
    struct vdma_stats stats;
    struct vdma_control_status control_status;
    struct vdma_mmap_info info = { 1, 4, SLOT_BYTES, FRAME_BYTES };
    struct vdma_buffer buffer;
    struct vdma_release_buffer released;
    int result = 0;

    if (mutex_lock_interruptible(&video->lock))
        return -ERESTARTSYS;
    if (video->dead) {
        result = -ENODEV;
        goto out;
    }
    switch (command) {
    case VDMA_FPGA_ROAD_STATUS:
        result = vdma_fpga_road_status_locked(video, argument);
        break;
    case VDMA_FPGA_ROAD_READ:
        result = vdma_fpga_road_read_locked(video, argument);
        break;
    case VDMA_RGB_EXPORT:
        result = vdma_check_link_locked(video);
        if (!result)
            result = vdma_rgb_export_locked(file, argument);
        break;
    case VDMA_RGB_CAPS:
    case VDMA_RGB_CONTROL:
    case VDMA_RGB_ACQUIRE:
    case VDMA_RGB_RELEASE:
    case VDMA_RGB_STATS:
        result = vdma_rgb_ioctl(video, command, argument);
        break;
    case VDMA_RR_CAPS:
    case VDMA_RR_START:
    case VDMA_RR_STOP:
    case VDMA_RR_ACQUIRE:
    case VDMA_RR_RELEASE:
    case VDMA_RR_EXPORT:
        result = vdma_rr_ioctl_locked(file, command, argument);
        break;
    case VDMA_NATIVE_IDENTITY:
        result = vdma_native_identity_locked(video, argument);
        break;
    case VDMA_DARK_STATUS:
    case VDMA_DARK_COMMAND:
        result = vdma_dark_ioctl_locked(file, command, argument);
        break;
    case VDMA_SNAPSHOT_CAPS: {
        struct vdma_snapshot_caps caps;
        if (copy_from_user(&caps, (void __user *)argument, sizeof(caps))) {
            result = -EFAULT;
            break;
        }
        if (caps.version != VDMA_SNAPSHOT_VERSION || caps.size != sizeof(caps) ||
            caps.reserved[0] || caps.reserved[1] || caps.reserved[2] || caps.reserved[3]) {
            result = -EINVAL;
            break;
        }
        memset(&caps, 0, sizeof(caps));
        caps.version = VDMA_SNAPSHOT_VERSION;
        caps.size = sizeof(caps);
        caps.capabilities = VDMA_SNAPSHOT_COPY_HELD;
        caps.frame_bytes = FRAME_BYTES;
        caps.format = 0x36314752;
        caps.width = 1920;
        caps.height = 1080;
        caps.stride = 3840;
        if (copy_to_user((void __user *)argument, &caps, sizeof(caps)))
            result = -EFAULT;
        break;
    }
    case VDMA_COPY_HELD_FRAME:
        result = vdma_snapshot_copy_locked(file, argument);
        break;
    case VDMA_FRAME_IDENTITY:
        result = vdma_frame_identity_ioctl_locked(file, argument);
        break;
    case VDMA_FEATURE_CAPS:
    case VDMA_FEATURE_CONFIG:
    case VDMA_FEATURE_ACQUIRE:
    case VDMA_FEATURE_READ:
    case VDMA_FEATURE_RELEASE:
        result = vdma_feature_ioctl_locked(file, command, argument);
        break;
    case VDMA_CONTROL_STATUS:
        result = vdma_check_link_locked(video);
        if (result)
            break;
        result = vdma_control_status_locked(video, &control_status);
        if (!result && copy_to_user((void __user *)argument, &control_status,
                                    sizeof(control_status)))
            result = -EFAULT;
        break;
    case VDMA_START:
        result = vdma_check_link_locked(video);
        if (result)
            break;
        if (video->running)
            break;
        if (video->acquired || (video->io_mode == VDMA_MODE_DISPLAY &&
                               video->dequeue != video->consumer)) {
            result = -EBUSY;
            break;
        }
        if (video->need_fence) {
            result = vdma_stop_locked(video);
            if (result)
                break;
        }
        video->consumer = readl(video->bar + REG_PRODUCER);
        video->dequeue = video->consumer;
        writel(video->consumer, video->bar + REG_CONSUMER);
        dma_wmb();
        video->need_fence = true;
        writel(1, video->bar + REG_CONTROL);
        if (readl(video->bar + REG_CONTROL) != 1) {
            result = -EIO;
            break;
        }
        video->running = true;
        video->need_fence = true;
        break;
    case VDMA_STOP:
        result = vdma_stop_locked(video);
        if (!result)
            vdma_feature_release_locked(video);
        break;
    case VDMA_STATS:
        stats.producer = readl(video->bar + REG_PRODUCER);
        stats.consumer = video->consumer;
        stats.captured = readl(video->bar + REG_CAPTURED);
        stats.dropped = readl(video->bar + REG_DROPPED);
        stats.errors = readl(video->bar + REG_ERRORS);
        stats.status = readl(video->bar + REG_STATUS);
        if (stats.status == U32_MAX || stats.errors == U32_MAX) {
            result = -ENODEV;
            break;
        }
        if (copy_to_user((void __user *)argument, &stats, sizeof(stats)))
            result = -EFAULT;
        break;
    case VDMA_DENSE_DEBUG: {
        /* ROAD dense probe at register 0x15f (bar offset 0x15f0). It sits outside
           the 0x140..0x15b status window on purpose: REG_FLAGS doubles as the
           packet lease status word that this driver validates against 0x3f, so a
           probe parked there failed every acquire with -EPROTO. */
        __u32 value = readl(video->bar + 0x15f0U);
        if (copy_to_user((void __user *)argument, &value, sizeof(value)))
            result = -EFAULT;
        break;
    }
    case VDMA_PLANE_INFO: {
        struct vdma_plane_info plane;

        result = vdma_check_link_locked(video);
        if (result)
            break;
        memset(&plane, 0, sizeof(plane));
        plane.magic = readl(video->bar + PLANE_REG_MAGIC);
        if (plane.magic != PLANE_MAGIC) {
            /* Old bitstream: 0x127..0x13F is not decoded by any block yet.
               Returning zeros here would look like a valid idle channel. */
            result = -EOPNOTSUPP;
            break;
        }
        plane.version = readl(video->bar + PLANE_REG_VER);
        plane.size = sizeof(plane);
        plane.control = readl(video->bar + PLANE_REG_CTRL);
        plane.status = readl(video->bar + PLANE_REG_STAT);
        plane.sequence = readl(video->bar + PLANE_REG_SEQ);
        plane.generation = readl(video->bar + PLANE_REG_GEN);
        plane.base_beat = readl(video->bar + PLANE_REG_BASE);
        plane.slot_beats = readl(video->bar + PLANE_REG_BEATS);
        plane.slot_count = readl(video->bar + PLANE_REG_SLOTS);
        plane.geometry = readl(video->bar + PLANE_REG_GEOM);
        plane.format = readl(video->bar + PLANE_REG_FMT);
        plane.slot_index = readl(video->bar + PLANE_REG_INDEX);
        plane.dropped = readl(video->bar + PLANE_REG_DROP);
        if (copy_to_user((void __user *)argument, &plane, sizeof(plane)))
            result = -EFAULT;
        break;
    }
    case VDMA_PLANE_CAPS: {
        struct vdma_plane_caps caps;

        result = vdma_check_link_locked(video);
        if (result)
            break;
        if (!video->plane_present) {
            result = -EOPNOTSUPP;
            break;
        }
        if (copy_from_user(&caps, (void __user *)argument, sizeof(caps))) {
            result = -EFAULT;
            break;
        }
        if (caps.version != VDMA_PLANE_CAPS_VERSION ||
            caps.size != sizeof(caps) || caps.reserved[0] ||
            caps.reserved[1] || caps.reserved[2] || caps.reserved[3]) {
            result = -EINVAL;
            break;
        }
        memset(&caps, 0, sizeof(caps));
        caps.version = VDMA_PLANE_CAPS_VERSION;
        caps.size = sizeof(caps);
        caps.hardware_version = video->plane_hardware_version;
        caps.format = video->plane_format;
        caps.kind = video->plane_kind;
        caps.metadata_version = video->plane_metadata_version;
        caps.geometry = PLANE_GEOMETRY;
        caps.stride = video->plane_stride;
        caps.payload_bytes = video->plane_payload_bytes;
        caps.y_offset = video->plane_y_offset;
        caps.y_length = video->plane_y_length;
        caps.c_offset = video->plane_c_offset;
        caps.c_length = video->plane_c_length;
        caps.mmap_bytes = video->plane_slot_bytes;
        caps.rtl_revision = video->plane_rtl_revision;
        caps.flags = video->plane_yc ? VDMA_PLANE_YC : 0;
        if (copy_to_user((void __user *)argument, &caps, sizeof(caps)))
            result = -EFAULT;
        break;
    }
    case VDMA_PLANE_ACQUIRE:
        result = vdma_plane_acquire_locked(video, (void __user *)argument, false);
        break;
    case VDMA_PLANE_ACQUIRE_V2:
        result = vdma_plane_acquire_locked(video, (void __user *)argument, true);
        break;
    case VDMA_PLANE_ACQUIRE_V3:
        result = vdma_plane_acquire_v3_locked(video, (void __user *)argument);
        break;
    case VDMA_PLANE_RELEASE:
        result = vdma_plane_release_locked(video, (struct vdma_plane_frame __user *)argument);
        break;
    case VDMA_MMAP_INFO:
        if (copy_to_user((void __user *)argument, &info, sizeof(info)))
            result = -EFAULT;
        break;
    case VDMA_ACQUIRE:
        if (video->io_mode != VDMA_MODE_MMAP) {
            result = -EINVAL;
            break;
        }
        if (video->acquired) {
            result = -EBUSY;
            break;
        }
        result = vdma_wait_frame_locked(file, video->consumer, &buffer.header);
        if (result)
            break;
        buffer.slot = video->consumer & 3;
        buffer.token = video->consumer + 1;
        if (copy_to_user((void __user *)argument, &buffer, sizeof(buffer))) {
            result = -EFAULT;
            break;
        }
        video->acquired = true;
        break;
    case VDMA_RELEASE:
        if (copy_from_user(&released, (void __user *)argument, sizeof(released))) {
            result = -EFAULT;
            break;
        }
        if ((video->io_mode == VDMA_MODE_DISPLAY ? video->dequeue == video->consumer :
             !video->acquired) || released.slot != (video->consumer & 3) ||
            released.token != video->consumer + 1) {
            result = -EINVAL;
            break;
        }
        if (!video->running && video->need_fence) {
            result = -EBUSY;
            break;
        }
        vdma_consume_locked(video);
        video->acquired = false;
        break;
    case VDMA_EXPORT:
        result = vdma_export_locked(file, argument);
        break;
    case VDMA_ACQUIRE_NEXT:
        if (video->io_mode != VDMA_MODE_DISPLAY || video->exported_mask != 15) {
            result = -EINVAL;
            break;
        }
        if ((u32)(video->dequeue - video->consumer) >= 4) {
            result = -EBUSY;
            break;
        }
        result = vdma_wait_frame_locked(file, video->dequeue, &buffer.header);
        if (result)
            break;
        buffer.slot = video->dequeue & 3;
        buffer.token = video->dequeue + 1;
        if (copy_to_user((void __user *)argument, &buffer, sizeof(buffer))) {
            result = -EFAULT;
            break;
        }
        video->dequeue++;
        break;
    default:
        result = -ENOTTY;
    }
out:
    mutex_unlock(&video->lock);
    return result;
}

static ssize_t vdma_read(struct file *file, char __user *buffer, size_t count, loff_t *position)
{
    struct vdma_device *video = file->private_data;
    struct vdma_frame_header header;
    unsigned int slot;
    ssize_t result;

    if (count < sizeof(header) + FRAME_BYTES)
        return -EMSGSIZE;
    if (mutex_lock_interruptible(&video->lock))
        return -ERESTARTSYS;
    if (video->io_mode == VDMA_MODE_MMAP || video->io_mode == VDMA_MODE_DISPLAY) {
        result = -EBUSY;
        goto out;
    }
    video->io_mode = VDMA_MODE_READ;
    result = vdma_wait_frame_locked(file, video->consumer, &header);
    if (result)
        goto out;
    slot = video->consumer & 3;
    if (copy_to_user(buffer, &header, sizeof(header)) ||
        copy_to_user(buffer + sizeof(header), video->slots[slot], FRAME_BYTES)) {
        result = -EFAULT;
        goto out;
    }
    vdma_consume_locked(video);
    result = sizeof(header) + FRAME_BYTES;
out:
    mutex_unlock(&video->lock);
    return result;
}

static int vdma_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct vdma_device *video = file->private_data;
    unsigned long slot_pages = SLOT_BYTES >> PAGE_SHIFT;
    unsigned long requested_offset = vma->vm_pgoff;
    unsigned long length = vma->vm_end - vma->vm_start;
    unsigned long slot = requested_offset / slot_pages;
    bool is_plane = (requested_offset == PLANE_MMAP_PAGE_OFFSET);
    int result;
    if (requested_offset == VDMA_RGB_MMAP_PAGE) {
        if (length != VDMA_RGB_MAP_BYTES || !(vma->vm_flags & VM_MAYSHARE) ||
            (vma->vm_flags & (VM_WRITE | VM_EXEC))) return -EACCES;
        if (mutex_lock_interruptible(&video->lock)) return -ERESTARTSYS;
        if (video->dead) result = -ENODEV;
        else if (!video->rgb_present) result = -EOPNOTSUPP;
        else {
            vma->vm_flags &= ~(VM_MAYWRITE | VM_MAYEXEC);
            vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
            vma->vm_pgoff = 0;
            result = dma_mmap_coherent(&video->pdev->dev, vma, video->rgb,
                                       video->rgb_dma, VDMA_RGB_MAP_BYTES);
            vma->vm_pgoff = requested_offset;
        }
        mutex_unlock(&video->lock);
        return result;
    }
    if (requested_offset >= VDMA_RR_MMAP_PAGE) {
        unsigned long rr_pages = PAGE_ALIGN(RR_PAYLOAD_BYTES) >> PAGE_SHIFT;
        unsigned long relative = requested_offset - VDMA_RR_MMAP_PAGE;
        unsigned long rr_slot = relative / rr_pages;
        if (relative % rr_pages || rr_slot >= RR_HOST_SLOT_COUNT ||
            length != PAGE_ALIGN(RR_PAYLOAD_BYTES)) return -EINVAL;
        if (!(vma->vm_flags & VM_MAYSHARE) || (vma->vm_flags & (VM_WRITE | VM_EXEC)))
            return -EACCES;
        if (mutex_lock_interruptible(&video->lock)) return -ERESTARTSYS;
        if (video->dead) result = -ENODEV;
        else if (!video->rr_slots[rr_slot]) result = -EOPNOTSUPP;
        else {
            vma->vm_flags &= ~(VM_MAYWRITE | VM_MAYEXEC);
            vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
            vma->vm_pgoff = 0;
            result = dma_mmap_coherent(&video->pdev->dev, vma, video->rr_slots[rr_slot],
                video->rr_slot_dma[rr_slot], PAGE_ALIGN(RR_PAYLOAD_BYTES));
            vma->vm_pgoff = requested_offset;
        }
        mutex_unlock(&video->lock);
        return result;
    }

    /*
     * 平面像素区和显示槽共用同一个 mmap 入口，用页偏移区分：
     *   0 .. 4*slot_pages-1   → 显示帧槽（每 slot_pages 一槽）
     *   PLANE_MMAP_PAGE_OFFSET → 平面载荷（单槽，按能力页对齐）
     * 平面是只读旁路，不占用 io_mode，因此显示模式下也能同时映射。
     */
    if (is_plane) {
        if (length != video->plane_slot_bytes)
            return -EINVAL;
    } else if (slot >= 4 || requested_offset % slot_pages || length != SLOT_BYTES) {
        return -EINVAL;
    }
    if (!(vma->vm_flags & VM_MAYSHARE) || (vma->vm_flags & (VM_WRITE | VM_EXEC)))
        return -EACCES;
    if (mutex_lock_interruptible(&video->lock))
        return -ERESTARTSYS;
    if (video->dead) {
        result = -ENODEV;
        goto out;
    }
    if (is_plane && !video->plane) {
        /* 位流没有平面通路，或 probe 时 CMA 分配失败。 */
        result = -EOPNOTSUPP;
        goto out;
    }
    if (!is_plane &&
        (video->io_mode == VDMA_MODE_READ || video->io_mode == VDMA_MODE_DISPLAY)) {
        result = -EBUSY;
        goto out;
    }
    vma->vm_flags &= ~(VM_MAYWRITE | VM_MAYEXEC);
    vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
    vma->vm_pgoff = 0;
    if (is_plane)
        result = dma_mmap_coherent(&video->pdev->dev, vma, video->plane,
                                   video->plane_dma, video->plane_slot_bytes);
    else
        result = dma_mmap_coherent(&video->pdev->dev, vma, video->slots[slot],
                                   video->slot_dma[slot], SLOT_BYTES);
    vma->vm_pgoff = requested_offset;
    if (!result && !is_plane)
        video->io_mode = VDMA_MODE_MMAP;
out:
    mutex_unlock(&video->lock);
    return result;
}

static const struct file_operations vdma_fops = {
    .owner = THIS_MODULE,
    .open = vdma_open,
    .release = vdma_release,
    .unlocked_ioctl = vdma_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = compat_ptr_ioctl,
#endif
    .read = vdma_read,
    .mmap = vdma_mmap,
    .llseek = no_llseek,
};

static int vdma_probe(struct pci_dev *pdev, const struct pci_device_id *identity)
{
    struct vdma_device *video;
    unsigned int index;
    u32 device_id = 0xffffffff;
    u32 magic;
    u32 version;
    u32 status;
    int result;

    result = pci_read_config_dword(pdev, PCI_VENDOR_ID, &device_id);
    if (result != PCIBIOS_SUCCESSFUL || device_id != 0x07550755) {
        dev_err(&pdev->dev, "PCIe config inaccessible or unexpected: id=%#010x result=%d; check endpoint power/reset and link before probing\n",
                device_id, result);
        return -ENODEV;
    }
    result = pci_enable_device_mem(pdev);
    if (result) {
        dev_err(&pdev->dev, "pci_enable_device_mem failed: %d\n", result);
        return result;
    }
    result = pci_request_regions(pdev, "video_dma");
    if (result) {
        dev_err(&pdev->dev, "pci_request_regions failed: %d\n", result);
        goto disable;
    }
    result = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (result)
        result = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (result) {
        dev_err(&pdev->dev, "No supported coherent DMA mask: %d\n", result);
        goto regions;
    }
    video = kzalloc(sizeof(*video), GFP_KERNEL);
    if (!video) { result = -ENOMEM; goto regions; }
    video->pdev = pci_dev_get(pdev);
    kref_init(&video->refs);
    mutex_init(&video->lock);
    if (pci_resource_len(pdev, 0) < PLANE_REG_REV + 16) {
        dev_err(&pdev->dev, "BAR0 is smaller than the video/plane register window\n");
        result = -ENODEV; goto destroy;
    }
    video->bar = pci_iomap(pdev, 0, 0);
    if (!video->bar) {
        dev_err(&pdev->dev, "BAR0 mapping failed\n");
        result = -ENOMEM; goto destroy;
    }
    magic = readl(video->bar + REG_MAGIC);
    version = readl(video->bar + REG_VERSION);
    status = readl(video->bar + REG_STATUS);
    dev_info(&pdev->dev, "BAR0 video ABI: magic=%#010x version=%#010x status=%#010x\n",
             magic, version, status);
    if (magic != 0x56444d41 || version != 0x00020001) {
        dev_err(&pdev->dev, "Video ABI mismatch: expected magic=0x56444d41 version=0x00020001; verify FPGA image and BAR0 reads\n");
        result = -ENODEV; goto unmap;
    }
    if (!(status & 1)) {
        dev_err(&pdev->dev, "Video engine is not stopped: status=%#010x\n", status);
        result = -EBUSY; goto unmap;
    }
    result = vdma_rgb_allocate(video);
    if (result) {
        dev_err(&pdev->dev, "small RGB allocation/contract failed: %d\n", result);
        goto unmap;
    }
    /*
     * Read the plane contract before allocating/mapping the host window.
     * Reject unknown combinations instead of guessing a smaller DMA window.
     */
    video->plane_hardware_version = readl(video->bar + PLANE_REG_VER);
    video->plane_format = readl(video->bar + PLANE_REG_FMT);
    video->plane_rtl_revision = readl(video->bar + PLANE_REG_REV);
    video->plane_present = readl(video->bar + PLANE_REG_MAGIC) == PLANE_MAGIC;
    if (video->plane_present &&
        ((readl(video->bar + PLANE_REG_CTRL) & 1U) ||
         readl(video->bar + PLANE_REG_HADDR_LO) ||
         readl(video->bar + PLANE_REG_HADDR_HI))) {
        dev_err(&pdev->dev, "plane retains previous DMA state: reset endpoint/reboot before rebind\n");
        result = -EBUSY; goto unmap;
    }
    if (video->plane_present && video->plane_hardware_version == 3 &&
        video->plane_format == VDMA_PLANE_FORMAT_YC8) {
        video->plane_yc = true;
        video->plane_payload_bytes = PLANE_YC_BYTES;
        video->plane_y_offset = 0;
        video->plane_y_length = PLANE_GRAY_BYTES;
        video->plane_c_offset = PLANE_GRAY_BYTES;
        video->plane_c_length = PLANE_GRAY_BYTES;
        video->plane_stride = 640;
        video->plane_kind = PLANE_YC_KIND;
        video->plane_metadata_version = PLANE_YC_METADATA_VERSION;
    } else if (video->plane_present &&
               (video->plane_hardware_version == 1 ||
                video->plane_hardware_version == 2) &&
               video->plane_format == VDMA_PLANE_FORMAT_GRAY8) {
        video->plane_yc = false;
        video->plane_format = VDMA_PLANE_FORMAT_GRAY8;
        video->plane_payload_bytes = PLANE_GRAY_BYTES;
        video->plane_y_offset = 0;
        video->plane_y_length = PLANE_GRAY_BYTES;
        video->plane_c_offset = 0;
        video->plane_c_length = 0;
        video->plane_stride = 640;
        video->plane_kind = PLANE_GRAY_KIND;
        video->plane_metadata_version = video->plane_hardware_version;
    } else if (video->plane_present) {
        dev_err(&pdev->dev, "unsupported plane ABI: hw=%u format=%u\n",
                video->plane_hardware_version, video->plane_format);
        result = -EPROTO; goto unmap;
    }
    if (video->plane_present && readl(video->bar + PLANE_REG_GEOM) != PLANE_GEOMETRY) {
        result = -EPROTO; goto unmap;
    }
    video->plane_slot_bytes = PAGE_ALIGN(video->plane_payload_bytes);
    dev_info(&pdev->dev,
             "plane contract: hw=%u format=%u payload=%u mmap=%zu kind=%u meta=%u rev=%#x\n",
             video->plane_hardware_version, video->plane_format,
             video->plane_payload_bytes, video->plane_slot_bytes,
             video->plane_kind, video->plane_metadata_version,
             video->plane_rtl_revision);
    for (index = 0; index < 4; index++) {
        video->slots[index] = dma_alloc_coherent(&pdev->dev, SLOT_BYTES, &video->slot_dma[index], GFP_KERNEL);
        if (!video->slots[index]) {
            dev_err(&pdev->dev, "Coherent frame slot %u allocation failed (%lu bytes)\n",
                    index, (unsigned long)SLOT_BYTES);
            result = -ENOMEM; goto unmap;
        }
        if (video->slot_dma[index] & 4095) { result = -EINVAL; goto unmap; }
        memset(video->slots[index], 0, SLOT_BYTES);
        dev_info(&pdev->dev, "frame slot %u dma=%pad bytes=%lu\n",
                 index, &video->slot_dma[index], (unsigned long)SLOT_BYTES);
    }
    video->completion = dma_alloc_coherent(&pdev->dev, COMPLETION_BYTES, &video->completion_dma, GFP_KERNEL);
    if (!video->completion) {
        dev_err(&pdev->dev, "Coherent completion allocation failed\n");
        result = -ENOMEM; goto unmap;
    }
    memset(video->completion, 0, COMPLETION_BYTES);
    if (!video->plane_present)
        goto video_addresses;
    /*
     * 平面窗口只有一块：road_plane_tx 的载荷地址是 cfg_host_addr + byte_offset，与槽号无关，
     * 所以开多块宿主缓冲并不能防撕裂。防撕裂靠 0x138 CONSUMER 与 0x12D SLOTS 的背压，
     * 由 PLANE_ACQUIRE/RELEASE 维护（见 vdma_plane_acquire）。
     */
    video->plane = dma_alloc_coherent(&pdev->dev, video->plane_slot_bytes,
                                      &video->plane_dma, GFP_KERNEL);
    if (!video->plane) {
        dev_err(&pdev->dev, "Coherent plane window allocation failed (%lu bytes)\n",
                (unsigned long)video->plane_slot_bytes);
        result = -ENOMEM; goto unmap;
    }
    memset(video->plane, 0, video->plane_slot_bytes);
    video->plane_completion = dma_alloc_coherent(&pdev->dev, PLANE_COMPLETION_BYTES,
                                                 &video->plane_completion_dma, GFP_KERNEL);
    if (!video->plane_completion) {
        dev_err(&pdev->dev, "Coherent plane completion allocation failed\n");
        result = -ENOMEM; goto unmap;
    }
    memset(video->plane_completion, 0, PLANE_COMPLETION_BYTES);
    dev_info(&pdev->dev, "plane window dma=%pad bytes=%lu completion=%pad\n",
             &video->plane_dma, (unsigned long)video->plane_slot_bytes,
             &video->plane_completion_dma);
    /*
     * SLOTS 取 1 是刻意的：宿主只有一块窗口，inflight 必须被压到 0/1 才不会撕裂。
     * period_div 保留原值（bits[7:4]），只动 bit0。
     */
    video->plane_consumer = readl(video->bar + PLANE_REG_SEQ);
    writel(video->plane_consumer, video->bar + PLANE_REG_CONSUMER);
    for (index = 0; index < 4; index++)
        WRITE_ONCE(*(__le32 *)(video->plane_completion + index * 64),
                   cpu_to_le32(video->plane_consumer - 4));
    writel(1, video->bar + PLANE_REG_SLOTS);
    if (video->plane_yc) {
        /* road_plane_ctl requires explicit format selection before enable. */
        writel(VDMA_PLANE_FORMAT_YC8, video->bar + PLANE_REG_FMT);
        if (readl(video->bar + PLANE_REG_FMT) != VDMA_PLANE_FORMAT_YC8) {
            dev_err(&pdev->dev, "YC plane format selection was not accepted\n");
            result = -EPROTO; goto unmap;
        }
    }
video_addresses:
    video->consumer = readl(video->bar + REG_PRODUCER);
    for (index = 0; index < 4; index++) {
        WRITE_ONCE(*(__le32 *)(video->completion + index * 64), cpu_to_le32(video->consumer - 4));
        vdma_address(video, REG_SLOTS + index * 32, video->slot_dma[index]);
    }
    vdma_address(video, REG_COMPLETION, video->completion_dma);
    if (video->plane_present) {
        vdma_address(video, PLANE_REG_HADDR_LO, video->plane_dma);
        vdma_address(video, PLANE_REG_CMPL_LO, video->plane_completion_dma);
        readl(video->bar + PLANE_REG_CMPL_HI);
    }
    writel(video->consumer, video->bar + REG_CONSUMER);
    readl(video->bar + REG_CONSUMER);
    dma_wmb();
    pci_set_master(pdev);
    /* Enable only after format, host address and completion address are live. */
    if (video->plane_present) {
        video->plane_started = true; /* No plane TX fence: retain buffers until reboot. */
        writel(readl(video->bar + PLANE_REG_CTRL) | 1U, video->bar + PLANE_REG_CTRL);
        if (!(readl(video->bar + PLANE_REG_CTRL) & 1U)) {
            dev_err(&pdev->dev, "plane enable was not accepted\n");
            result = -EPROTO; goto clear_master;
        }
    }
    snprintf(video->name, sizeof(video->name), "video_dma_%s", pci_name(pdev));
    video->misc.minor = MISC_DYNAMIC_MINOR;
    video->misc.name = video->name;
    video->misc.fops = &vdma_fops;
    video->misc.parent = &pdev->dev;
    video->misc.mode = 0600;
    result = misc_register(&video->misc);
    if (result) {
        dev_err(&pdev->dev, "Device node registration failed: %d\n", result);
        goto clear_master;
    }
    pci_set_drvdata(pdev, video);
    dev_info(&pdev->dev, "Ready: /dev/%s; DMA remains stopped until START\n", video->name);
    return 0;
clear_master:
    if (video->plane_present) {
        writel(readl(video->bar + PLANE_REG_CTRL) & ~1U, video->bar + PLANE_REG_CTRL);
        readl(video->bar + PLANE_REG_CTRL);
    }
    pci_clear_master(pdev);
unmap:
    if (video->bar) {
        pci_iounmap(pdev, video->bar);
        video->bar = NULL;
    }
destroy:
    kref_put(&video->refs, vdma_destroy);
regions:
    pci_release_regions(pdev);
disable:
    pci_disable_device(pdev);
    return result;
}

static void vdma_remove(struct pci_dev *pdev)
{
    struct vdma_device *video = pci_get_drvdata(pdev);

    misc_deregister(&video->misc);
    mutex_lock(&video->lock);
    vdma_rgb_disable(video);
    if (vdma_stop_locked(video))
        video->quarantine = true;
    if (vdma_rr_stop_locked(video))
        video->quarantine = true;
    if (video->plane_started) {
        /* CTRL stops new accepts only. There is no plane PCIe fence yet;
         * quarantine the allocations rather than free an in-flight target. */
        writel(readl(video->bar + PLANE_REG_CTRL) & ~1U, video->bar + PLANE_REG_CTRL);
        readl(video->bar + PLANE_REG_CTRL);
        video->quarantine = true;
    }
    video->dead = true;
    pci_clear_master(pdev);
    pci_iounmap(pdev, video->bar);
    video->bar = NULL;
    mutex_unlock(&video->lock);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    kref_put(&video->refs, vdma_destroy);
}

static void vdma_shutdown(struct pci_dev *pdev)
{
    struct vdma_device *video = pci_get_drvdata(pdev);

    if (!video)
        return;
    mutex_lock(&video->lock);
    if (!vdma_check_link_locked(video)) {
        vdma_rgb_disable(video);
        if (video->plane_present) {
            writel(readl(video->bar + PLANE_REG_CTRL) & ~1U,
                   video->bar + PLANE_REG_CTRL);
            readl(video->bar + PLANE_REG_CTRL);
        }
        vdma_stop_locked(video);
        vdma_rr_stop_locked(video);
    }
    pci_clear_master(pdev);
    mutex_unlock(&video->lock);
}

static const struct pci_device_id vdma_ids[] = {
    { PCI_DEVICE(0x0755, 0x0755) },
    { 0 }
};
MODULE_DEVICE_TABLE(pci, vdma_ids);
static struct pci_driver vdma_driver = {
    .name = "video_dma",
    .id_table = vdma_ids,
    .probe = vdma_probe,
    .remove = vdma_remove,
    .shutdown = vdma_shutdown,
};
module_pci_driver(vdma_driver);
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_DESCRIPTION("Final RGB565 video DMA receiver, ABI 2.1");
MODULE_VERSION("2026.11.01-road-every-frame1");
