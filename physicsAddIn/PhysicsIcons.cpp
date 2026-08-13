#include "PhysicsIcons.h"

#include <QApplication>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QPixmap>

namespace {

constexpr int kSource = 64;

// The box every shape is drawn inside, in source pixels. Inset, so a stroke centred on the edge is not
// half clipped - which is what a beam drawn flush to the boundary looks like at 16 px.
constexpr qreal kInset = 6.0;

QColor strokeColour() {
    const QColor text = qApp ? qApp->palette().color(QPalette::WindowText) : QColor(40, 40, 40);
    return text;
}

constexpr QColor kRailColour(0x4a, 0x90, 0xd9);      // steel blue: a beam
constexpr QColor kRobotColour(0xe0, 0x7b, 0x39);     // amber: an arm, and the colour arms are painted
constexpr QColor kToolColour(0x8e, 0x7c, 0xc3);      // violet: a gripper
constexpr QColor kAccessoryColour(0x5c, 0xb8, 0x5c); // green: a deck things travel along
constexpr QColor kDeleteColour(0xd0, 0x53, 0x4f);    // red: the one destructive verb

struct Canvas {
    QPixmap pixmap;
    QPainter painter;

    Canvas() : pixmap(kSource, kSource) {
        pixmap.fill(Qt::transparent);
        painter.begin(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
    }
    ~Canvas() { painter.end(); }

    QPen pen(qreal width) const {
        QPen p(strokeColour());
        p.setWidthF(width);
        p.setJoinStyle(Qt::RoundJoin);
        p.setCapStyle(Qt::RoundCap);
        return p;
    }
    QIcon finish() {
        painter.end();
        return QIcon(pixmap);
    }
};

// A beam running across with a carriage sitting on it: the two things a rail is, and the two an operator
// is about to move. The carriage is off centre so the icon says which way the axis runs.
QIcon railIcon() {
    Canvas canvas;
    const qreal left = kInset;
    const qreal right = kSource - kInset;
    const qreal beamTop = kSource * 0.58;
    const qreal beamHeight = kSource * 0.14;
    canvas.painter.setPen(canvas.pen(3.0));
    canvas.painter.setBrush(kRailColour);
    canvas.painter.drawRect(QRectF(left, beamTop, right - left, beamHeight));
    const qreal carriageWidth = kSource * 0.30;
    const qreal carriageLeft = left + (right - left - carriageWidth) * 0.62;
    canvas.painter.setBrush(kRailColour.lighter(140));
    canvas.painter.drawRect(QRectF(carriageLeft, beamTop - kSource * 0.16, carriageWidth,
                                   beamHeight + kSource * 0.20));
    canvas.painter.setBrush(kRailColour.darker(130));
    const qreal footWidth = kSource * 0.10;
    const qreal footTop = beamTop + beamHeight;
    canvas.painter.drawRect(QRectF(left + footWidth * 0.4, footTop, footWidth, kSource * 0.13));
    canvas.painter.drawRect(QRectF(right - footWidth * 1.4, footTop, footWidth, kSource * 0.13));
    return canvas.finish();
}

// A two-segment arm on a base. Not a realistic arm - at 16 px nothing is - but a shoulder, an elbow and a
// flange, which is the silhouette that says "this one has joints".
QIcon robotIcon() {
    Canvas canvas;
    const QPointF base(kSource * 0.30, kSource - kInset - kSource * 0.06);
    const QPointF shoulder(kSource * 0.30, kSource * 0.58);
    const QPointF elbow(kSource * 0.44, kSource * 0.26);
    const QPointF flange(kSource * 0.78, kSource * 0.34);

    canvas.painter.setPen(canvas.pen(3.0));
    canvas.painter.setBrush(kRobotColour.darker(130));
    canvas.painter.drawRect(QRectF(kSource * 0.16, base.y() - kSource * 0.06, kSource * 0.28,
                                   kSource * 0.12));

    QPen limb = canvas.pen(kSource * 0.13);
    limb.setColor(kRobotColour);
    canvas.painter.setPen(limb);
    canvas.painter.drawLine(shoulder, elbow);
    canvas.painter.drawLine(elbow, flange);
    canvas.painter.setPen(canvas.pen(2.0));
    canvas.painter.setBrush(kRobotColour.lighter(150));
    const qreal joint = kSource * 0.07;
    canvas.painter.drawEllipse(shoulder, joint, joint);
    canvas.painter.drawEllipse(elbow, joint, joint);
    return canvas.finish();
}

// Two jaws about to close on something. The gap is the whole idea, so it is wide.
QIcon toolIcon() {
    Canvas canvas;
    canvas.painter.setPen(canvas.pen(3.0));
    canvas.painter.setBrush(kToolColour);
    // The wrist it hangs off.
    canvas.painter.drawRect(QRectF(kSource * 0.38, kInset, kSource * 0.24, kSource * 0.22));
    // Two jaws, each an L reaching down and inward.
    const qreal jawTop = kSource * 0.30;
    const qreal jawThick = kSource * 0.12;
    canvas.painter.setBrush(kToolColour.lighter(140));
    canvas.painter.drawRect(QRectF(kSource * 0.20, jawTop, jawThick, kSource * 0.42));
    canvas.painter.drawRect(QRectF(kSource * 0.68, jawTop, jawThick, kSource * 0.42));
    // The bar they hang from, so the two jaws are one gripper.
    canvas.painter.setBrush(kToolColour);
    canvas.painter.drawRect(QRectF(kSource * 0.20, jawTop, kSource * 0.60, jawThick * 0.8));
    return canvas.finish();
}

// A deck with rollers under it: a conveyor, and by extension anything generated from parameters.
QIcon accessoryIcon() {
    Canvas canvas;
    const qreal left = kInset;
    const qreal right = kSource - kInset;
    const qreal deckTop = kSource * 0.34;
    canvas.painter.setPen(canvas.pen(3.0));
    canvas.painter.setBrush(kAccessoryColour);
    canvas.painter.drawRect(QRectF(left, deckTop, right - left, kSource * 0.12));
    // Three rollers, evenly spaced, which is what a roller bed is.
    canvas.painter.setBrush(kAccessoryColour.lighter(145));
    const qreal radius = kSource * 0.085;
    const qreal centreY = deckTop + kSource * 0.12 + radius;
    for (int roller = 0; roller < 3; ++roller) {
        const qreal t = (roller + 0.5) / 3.0;
        canvas.painter.drawEllipse(QPointF(left + (right - left) * t, centreY), radius, radius);
    }
    return canvas.finish();
}

// A plain box: a workpiece, and what a package with no mechanism in it amounts to.
QIcon boxIcon() {
    Canvas canvas;
    const qreal size = kSource - 2 * kInset - kSource * 0.10;
    const qreal top = kInset + kSource * 0.10;
    canvas.painter.setPen(canvas.pen(3.0));
    canvas.painter.setBrush(QColor(0x9a, 0x9a, 0x9a));
    canvas.painter.drawRect(QRectF(kInset, top, size, size));
    QPainterPath lid;
    lid.moveTo(kInset, top);
    lid.lineTo(kInset + kSource * 0.10, kInset);
    lid.lineTo(kInset + size + kSource * 0.10, kInset);
    lid.lineTo(kInset + size, top);
    lid.closeSubpath();
    canvas.painter.setBrush(QColor(0xbe, 0xbe, 0xbe));
    canvas.painter.drawPath(lid);
    return canvas.finish();
}

QIcon crossIcon() {
    Canvas canvas;
    QPen pen = canvas.pen(kSource * 0.14);
    pen.setColor(kDeleteColour);
    canvas.painter.setPen(pen);
    const qreal low = kInset + kSource * 0.08;
    const qreal high = kSource - kInset - kSource * 0.08;
    canvas.painter.drawLine(QPointF(low, low), QPointF(high, high));
    canvas.painter.drawLine(QPointF(high, low), QPointF(low, high));
    return canvas.finish();
}

} // namespace

QIcon libraryCategoryIcon(librarycatalogue::Category category) {
    switch (category) {
    case librarycatalogue::Category::Robot: return robotIcon();
    case librarycatalogue::Category::LinearRail: return railIcon();
    case librarycatalogue::Category::Tool: return toolIcon();
    case librarycatalogue::Category::Accessory: return accessoryIcon();
    }
    return boxIcon();
}

QIcon categoryIcon(placedmechanism::AxisKind kind) {
    switch (kind) {
    case placedmechanism::AxisKind::Rail: return railIcon();
    case placedmechanism::AxisKind::Robot: return robotIcon();
    case placedmechanism::AxisKind::None: break;
    }
    // Driven by nothing, so it is a thing that stands somewhere: a pedestal, a table, a workpiece.
    return boxIcon();
}

QIcon conveyorIcon() { return accessoryIcon(); }

QIcon deleteIcon() { return crossIcon(); }
