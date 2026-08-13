#ifndef PLACEDITEMAXES_H
#define PLACEDITEMAXES_H

#include "PlacedMechanismSchema.h"
#include "RobotRuntime.h"

#include <QString>

#include <vector>

// Kinematics for a plugin-drawn robot or rail.
class PlacedItemAxes {
public:
    enum class Kind { None, Robot, Rail };

    Kind bind(CadNode* root);
    Kind kind() const { return m_kind; }
    // Drag chains still require updates when no axis is directly exposed.
    bool drives() const { return m_kind != Kind::None || !m_chains.empty(); }
    int axisCount() const;

    // Robot values are degrees; rail values are millimetres.
    std::vector<double> values() const;
    bool setValues(const std::vector<double>& values, QString* error = nullptr);
    std::vector<placedmechanism::AxisField> fields() const;
    void apply();

    // Identity when the bound tree is not the corresponding mechanism type.
    CadTransform flangeInArmBase() const;
    CadTransform armBaseInTree() const;
    CadTransform railCarriageInTree() const;
    // RoboDK translation mechanisms move along local +Z.
    CadTransform railBaseInTree() const;

    const RobotPoseController& robot() const { return m_robot; }
    int dragChainCount() const { return static_cast<int>(m_chains.size()); }

private:
    Kind m_kind = Kind::None;
    // Non-owning; controllers and the tree share a lifetime.
    CadNode* m_root = nullptr;
    RobotPoseController m_robot;
    GantryPoseController m_rail;
    std::vector<DragChainPoseController> m_chains;
};

#endif // PLACEDITEMAXES_H
