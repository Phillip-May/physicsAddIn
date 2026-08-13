#include "PlacementSession.h"

#include <utility>

namespace placementsession {

void Session::arm(std::vector<mountingsnap::Interface> sourceInterfaces, bool toolPackage) {
    *this = Session();
    m_sourceInterfaces = std::move(sourceInterfaces);
    m_toolPackage = toolPackage;
}

void Session::setQuarterTurn(int quarterTurn) {
    m_quarterTurn = quarterTurn % 4;
    if (m_quarterTurn < 0) m_quarterTurn += 4;
}

const mountingsnap::Interface* Session::activeInterface() const {
    if (m_last.sourceInterface < 0 ||
        m_last.sourceInterface >= static_cast<int>(m_sourceInterfaces.size())) {
        return nullptr;
    }
    return &m_sourceInterfaces[static_cast<size_t>(m_last.sourceInterface)];
}

const mountingsnap::Result& Session::moved(double cursorX, double cursorY,
                                           const std::vector<mountingsnap::Interface>& targets,
                                           const mountingsnap::View& view,
                                           int viewportWidthPx, int viewportHeightPx,
                                           const CadNode* floor, double snapScreenPercent) {
    if (!armed()) return m_last;
    mountingsnap::Request request;
    request.sourceInterfaces = &m_sourceInterfaces;
    request.targets = &targets;
    request.floor = floor;
    request.toolPackage = m_toolPackage;
    request.cursorX = cursorX;
    request.cursorY = cursorY;
    request.viewportWidthPx = viewportWidthPx;
    request.viewportHeightPx = viewportHeightPx;
    request.snapScreenPercent = snapScreenPercent;
    request.quarterTurn = m_quarterTurn;
    request.retainSnapAfterRotation = m_retainSnapAfterRotation;
    request.retainedTarget = m_last.targetNode;
    request.retainedSourceInterface = m_last.sourceInterface;

    m_last = mountingsnap::solve(request, view);
    m_retainSnapAfterRotation = false;
    return m_last;
}

void Session::rotated(float wheelDelta) {
    if (!armed() || wheelDelta == 0.0f) return;
    m_wheelAccumulator += wheelDelta;
    int steps = 0;
    while (m_wheelAccumulator >= 1.0f) {
        ++steps;
        m_wheelAccumulator -= 1.0f;
    }
    while (m_wheelAccumulator <= -1.0f) {
        --steps;
        m_wheelAccumulator += 1.0f;
    }
    if (steps == 0) return;
    int quarterStep = 1;
    if (m_last.snapped) {
        if (const mountingsnap::Interface* mated = activeInterface()) {
            quarterStep = mountingsnap::rotationQuarterStep(*mated);
            m_retainSnapAfterRotation = true;
        }
    }
    setQuarterTurn(m_quarterTurn + steps * quarterStep);
}

} // namespace placementsession
