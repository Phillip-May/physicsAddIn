#ifndef PLACEMENTSESSION_H
#define PLACEMENTSESSION_H

#include "MountingSnap.h"

// The state between arming a placement and committing it.
namespace placementsession {

class Session {
public:
    void arm(std::vector<mountingsnap::Interface> sourceInterfaces, bool toolPackage = false);
    bool armed() const { return !m_sourceInterfaces.empty(); }
    void cancel() { *this = Session(); }
    void setQuarterTurn(int quarterTurn);

    const mountingsnap::Result& moved(double cursorX, double cursorY,
                                      const std::vector<mountingsnap::Interface>& targets,
                                      const mountingsnap::View& view,
                                      int viewportWidthPx, int viewportHeightPx,
                                      const CadNode* floor, double snapScreenPercent);
    // One wheel notch, or part of one - a trackpad delivers fractions, so they accumulate here rather
    // than each being rounded to a turn of its own.
    void rotated(float wheelDelta);

    const mountingsnap::Result& last() const { return m_last; }
    int quarterTurn() const { return m_quarterTurn; }
    bool toolPackage() const { return m_toolPackage; }
    const std::vector<mountingsnap::Interface>& sourceInterfaces() const {
        return m_sourceInterfaces;
    }
    const mountingsnap::Interface* activeInterface() const;

private:
    std::vector<mountingsnap::Interface> m_sourceInterfaces;
    mountingsnap::Result m_last;
    bool m_toolPackage = false;
    int m_quarterTurn = 0;
    float m_wheelAccumulator = 0.0f;
    // Set by `rotated` and cleared by the solve that reads it: exactly one frame holds the current
    // mate while the wheel selects another symmetry of the same complete pattern.
    bool m_retainSnapAfterRotation = false;
};

} // namespace placementsession

#endif // PLACEMENTSESSION_H
