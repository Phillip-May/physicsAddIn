#pragma once

// Qt conversions for the CadNode types, for the Qt applications only.

#include "CadNode.h"

#include <QColor>
#include <QVector3D>

inline QColor toQColor(const CADNodeColor& color) {
    return QColor::fromRgbF(color.r, color.g, color.b, color.a);
}

inline QVector3D toQVector3D(const CadVec3& vector) {
    return QVector3D(static_cast<float>(vector.x),
                     static_cast<float>(vector.y),
                     static_cast<float>(vector.z));
}

inline CadVec3 toCadVec3(const QVector3D& vector) {
    return CadVec3(vector.x(), vector.y(), vector.z());
}
