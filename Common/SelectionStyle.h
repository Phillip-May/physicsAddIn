#ifndef SELECTIONSTYLE_H
#define SELECTIONSTYLE_H

#include "CadNode.h"

#include <algorithm>

namespace selectionstyle {

// The outline purple.
inline CADNodeColor outlineColor() { return CADNodeColor(0.72f, 0.38f, 1.0f, 1.0f); }

inline double outlineThicknessMm(double viewDistanceMm) {
    return std::max(0.5, viewDistanceMm * 0.003);
}

inline double outlineExpansion(double halfExtent, double thicknessMm) {
    return 1.0 + thicknessMm / std::max(halfExtent, thicknessMm);
}

} // namespace selectionstyle

#endif // SELECTIONSTYLE_H
