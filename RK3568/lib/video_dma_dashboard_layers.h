#ifndef VIDEO_DMA_DASHBOARD_LAYERS_H
#define VIDEO_DMA_DASHBOARD_LAYERS_H
#include <QImage>
#include <QRegion>
#include <cstring>

/* Chrome is transparent inside this viewport; geometry is empty outside.
 * Selecting the owning layer avoids blending two full ARGB surfaces. */
static inline void dashboardComposeLayers(QImage &target, const QImage &chrome,
                                         const QImage &geometry, const QRegion &damage)
{
    const QRegion clipped = damage.intersected(target.rect());
    for (const QRect &rect : clipped)
        for (int y = rect.top(); y <= rect.bottom(); ++y) {
            uchar *out = target.scanLine(y);
            const uchar *clean = chrome.constScanLine(y);
            const int end = rect.right()+1;
            if (y < 146 || y >= 965 || rect.left() >= 1456) {
                std::memcpy(out+rect.left()*4,clean+rect.left()*4,size_t(rect.width())*4);
                continue;
            }
            const int videoEnd = qMin(end,1456);
            std::memcpy(out+rect.left()*4,geometry.constScanLine(y)+rect.left()*4,
                        size_t(videoEnd-rect.left())*4);
            if (end > videoEnd)
                std::memcpy(out+videoEnd*4,clean+videoEnd*4,size_t(end-videoEnd)*4);
        }
}
#endif
