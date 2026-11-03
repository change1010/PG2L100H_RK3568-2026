#ifndef VIDEO_DMA_DASHBOARD_PAINT_H
#define VIDEO_DMA_DASHBOARD_PAINT_H
#include <QPainter>
#include <QFont>
#include <QPolygonF>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

/* Fit each marking independently; detector geometry and temporal matching
 * keep the original polygon. Curved/degenerate contours keep their outline. */
static QPolygonF displayLaneQuad(const QPolygonF &polygon)
{
    if (polygon.size() < 3) return polygon;
    const QRectF bounds = polygon.boundingRect();
    if (bounds.height() < 4 || bounds.width() < 1 ||
        bounds.left() < -1920 || bounds.right() > 3840 ||
        bounds.top() < -1080 || bounds.bottom() > 2160) return polygon;
    std::array<double,16> left{}, right{}, ys{};
    double sumY = 0, sumYY = 0, sumL = 0, sumR = 0, sumYL = 0, sumYR = 0;
    for (unsigned i = 0; i < ys.size(); ++i) {
        const double y = bounds.top()+bounds.height()*(i+0.5)/ys.size();
        double lo = std::numeric_limits<double>::max(), hi = -lo;
        for (int j = 0; j < polygon.size(); ++j) {
            const auto &a = polygon[j], &b = polygon[(j+1)%polygon.size()];
            if ((a.y() <= y && b.y() > y) || (b.y() <= y && a.y() > y)) {
                const double x = a.x()+(y-a.y())*(b.x()-a.x())/(b.y()-a.y());
                lo = std::min(lo,x); hi = std::max(hi,x);
            }
        }
        if (hi <= lo) return polygon;
        ys[i] = y-bounds.top(); left[i] = lo; right[i] = hi;
        sumY += ys[i]; sumYY += ys[i]*ys[i]; sumL += lo; sumR += hi;
        sumYL += ys[i]*lo; sumYR += ys[i]*hi;
    }
    const double count = ys.size(), denominator = count*sumYY-sumY*sumY;
    if (denominator <= 0) return polygon;
    const double slopeL = (count*sumYL-sumY*sumL)/denominator;
    const double slopeR = (count*sumYR-sumY*sumR)/denominator;
    const double offsetL = (sumL-slopeL*sumY)/count, offsetR = (sumR-slopeR*sumY)/count;
    for (unsigned i = 0; i < ys.size(); ++i)
        if (std::abs(offsetL+slopeL*ys[i]-left[i]) > 8 ||
            std::abs(offsetR+slopeR*ys[i]-right[i]) > 8) return polygon;
    const double bottomL = offsetL+slopeL*bounds.height(), bottomR = offsetR+slopeR*bounds.height();
    if (offsetL > offsetR || bottomL > bottomR) return polygon;
    QPolygonF quad;
    quad << QPointF(offsetL,bounds.top()) << QPointF(offsetR,bounds.top())
         << QPointF(bottomR,bounds.bottom()) << QPointF(bottomL,bounds.bottom());
    return quad;
}
static QString text(const char *value) { return QString::fromUtf8(value); }
static const QColor ink("#203747"), muted("#637b8b"), line("#d6e2e9"), accent("#008673");
static const QColor surface("#eef3f7"), red("#d24353"), green("#008758");

static void label(QPainter &p, const QRect &r, const QString &value, int size = 24,
                  QColor color = ink, int align = Qt::AlignLeft | Qt::AlignVCenter)
{
    if (p.hasClipping() && !p.clipRegion().intersects(r)) return;
    QFont font("Noto Sans CJK SC"); font.setPixelSize(size); p.setFont(font);
    p.setPen(color); p.drawText(r, align, value);
}

static void panel(QPainter &p, const QRect &r, QColor fill = surface)
{
    if (p.hasClipping() && !p.clipRegion().intersects(r.adjusted(-1,-1,1,1))) return;
    p.setBrush(fill); p.setPen(QPen(line, 1)); p.drawRoundedRect(r, 10, 10);
}

static void button(QPainter &p, const QRect &r, const QString &value, bool on, bool disabled = false)
{
    if (p.hasClipping() && !p.clipRegion().intersects(r.adjusted(-1,-1,1,1))) return;
    panel(p, r, on ? accent : surface);
    label(p, r, value, 23, on ? Qt::white : disabled ? QColor("#98a7b2") : muted, Qt::AlignCenter);
}

#endif
