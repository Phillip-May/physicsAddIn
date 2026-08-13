#pragma once

#include "CadNode.h"
#include "RobotMotionCore.h"
#include "StringUtil.h"

#include <array>
#include <memory>
#include <utility>
#include <string>
#include <vector>


CadNode* findOPW6RobotNode(CadNode* root);
const CadNode* findOPW6RobotNode(const CadNode* root);

CadNode* findGantryMechanismNode(CadNode* root);
const CadNode* findGantryMechanismNode(const CadNode* root);
std::vector<CadNode*> collectGantryMechanismNodes(CadNode* root);
std::vector<CadNode*> collectDragChainMechanismNodes(CadNode* root);

std::vector<CadNode*> collectRobotNodes(CadNode* root);

// Gives every arm in the tree a base frame standing over it: a Transform node in the arm's slot
// carrying whatever placement the arm was loaded with, with the arm itself at identity beneath.
int ensureRobotBaseFrames(CadNode* root);

// The node holding this arm's placement: its base frame if it has one, otherwise the arm itself.
CadNode* robotBaseFrameNode(CadNode* robotNode);
const CadNode* robotBaseFrameNode(const CadNode* robotNode);

std::vector<CadNode*> collectRobotLinks(CadNode* robotNode);
std::vector<CadNode*> collectRobotTools(CadNode* robotNode);

ConvexHullData makeBoxHullFromBounds(double xmin, double ymin, double zmin,
                                     double xmax, double ymax, double zmax);
bool bakeRobotCollisionHulls(CadNode* root, std::string* errorMessage = nullptr);
bool validateRobotPackage(CadNode* root, std::string* errorMessage = nullptr);
std::vector<std::string> describeRobotJointAxes(CadNode* root);

struct RobotCollisionPair {
    int linkA = -1;
    int linkB = -1;
};

// Value snapshot of the geometric forward kinematics that RobotPoseController::toolPose() uses.
struct RobotKinematicSnapshot {
    std::array<std::array<double, 3>, 6> axisOrigin{};
    std::array<std::array<double, 3>, 6> axisDirection{};
    std::array<double, 6> qHome{};
    CadTransform toolBindPose;
    CadTransform baseWorldTransform;
    bool valid = false;
};

// Robot-local: the TCP in the arm's own base frame. What the planner, the statistics and the
// trajectory CSV mean by a TCP pose.
CadTransform toolPoseFromKinematicSnapshot(const RobotKinematicSnapshot& snapshot,
                                           const std::array<double, 6>& q);

// World: the same pose stood where the arm stands. What anything drawn in the scene wants -
// path previews, gimbals, a readout quoting a position in the cell.
CadTransform worldToolPoseFromKinematicSnapshot(const RobotKinematicSnapshot& snapshot,
                                                const std::array<double, 6>& q);

// Drives one linear gantry without making it a seventh arm joint. The moving frame may carry any
// subtree; mounting an arm directly beneath it requires no intermediate transform.
class GantryPoseController {
public:
    bool bind(CadNode* root, std::string* errorMessage = nullptr);
    bool bindToGantry(CadNode* gantryNode, std::string* errorMessage = nullptr);
    bool isBound() const { return m_gantryNode != nullptr && m_data != nullptr && m_movingFrame != nullptr; }

    CadNode* gantryNode() const { return m_gantryNode; }
    CadNode* movingFrame() const { return m_movingFrame; }
    GantryMechanismData* gantryData() const { return m_data; }
    double positionMm() const { return m_positionMm; }

    void setPositionMm(double positionMm);
    void resetHome();

private:
    CadNode* m_gantryNode = nullptr;
    CadNode* m_movingFrame = nullptr;
    GantryMechanismData* m_data = nullptr;
    CadTransform m_movingFrameBindLoc;
    std::array<double, 3> m_axis{{1.0, 0.0, 0.0}};
    double m_positionMm = 0.0;
};

// Supplies deterministic poses for every chain member and is also the pose seed/fallback for the
// desktop PhysX implementation. The carrier lies in the plane normal to hingeAxis; its U bend
// moves at half carriage speed as the two straight runs exchange length.
class DragChainPoseController {
public:
    bool bindToDragChain(CadNode* chainNode, std::string* errorMessage = nullptr);
    bool isBound() const { return m_chainNode && m_data && m_data->movingFrame && !m_data->linkFrames.empty(); }
    void update();

    CadNode* chainNode() const { return m_chainNode; }
    DragChainMechanismData* chainData() const { return m_data; }

private:
    CadNode* m_chainNode = nullptr;
    DragChainMechanismData* m_data = nullptr;
};

class RobotPoseController {
public:
    static constexpr int kMaxToolPoseSolutions = 12;

    RobotKinematicSnapshot kinematicSnapshot() const;

    // bind() takes the first robot in the tree, which is what every single-arm caller wants and
    // what the CLI commands have no way to improve on. bindToRobot() names the arm, for a scene
    // holding more than one.
    bool bind(CadNode* root, std::string* errorMessage = nullptr);
    bool bindToRobot(CadNode* robotNode, std::string* errorMessage = nullptr);
    // Recomputes only the flange-to-active-TCP transform from the immutable tool binding captured
    // by bindToRobot(). It deliberately does not rebuild or apply link bindings, so selecting a
    // tool while the arm is posed cannot compound transforms and tear the model apart.
    bool refreshActiveToolBind(std::string* errorMessage = nullptr);
    // Adds a newly attached tool without rebinding the already-posed robot. The node arrives in its
    // current link-6 pose; this converts it back to an immutable home bind before registering it.
    bool registerAttachedTool(CadNode* toolNode, std::string* errorMessage = nullptr);
    bool isBound() const { return m_robotNode != nullptr && m_robotData != nullptr; }

    OPW6RobotData* robotData() const { return m_robotData; }
    CadNode* robotNode() const { return m_robotNode; }
    const std::array<double, 6>& joints() const { return m_q; }
    CadTransform toolPose() const;
    CadTransform baseWorldTransform() const;
    CadTransform worldToolPose() const { return baseWorldTransform() * toolPose(); }
    RobotMotionCore::RobotModel motionModel() const;

    void setJoints(const std::array<double, 6>& q);
    bool setToolPose(const CadTransform& targetPose, std::string* errorMessage = nullptr);
    bool setToolPoseSolution(const CadTransform& targetPose, int solutionIndex, std::string* errorMessage = nullptr);
    int toolPoseSolutions(const CadTransform& targetPose,
                          std::array<double, 6>* solutions,
                          int solutionCapacity,
                          std::string* errorMessage = nullptr) const;
    void resetHome();
    void setVisualMeshesVisible(bool visible);
    void setJointAxesVisible(bool visible);
    void setHullOverlaysVisible(bool visible);

private:
    struct Binding {
        CadNode* node = nullptr;
        CadTransform bindLoc;
        int linkIndex = 0;
    };
    struct JointAxis {
        std::array<double, 3> origin{{0.0, 0.0, 0.0}};
        std::array<double, 3> direction{{0.0, 0.0, 0.0}};
        bool valid = false;
    };

    CadNode* m_robotNode = nullptr;
    OPW6RobotData* m_robotData = nullptr;
    std::array<double, 6> m_q{};
    std::array<JointAxis, 6> m_jointAxes{};
    std::vector<Binding> m_linkBindings;
    std::vector<Binding> m_geometryBindings;
    std::vector<Binding> m_axisBindings;
    CadTransform m_toolBindPose;

    void applyPose();
    bool solveToolPose(const CadTransform& targetPose, std::array<double, 6>& solvedQ, std::string* errorMessage = nullptr) const;
};

class RobotCollisionModel {
public:
    bool bind(CadNode* root, std::string* errorMessage = nullptr);
    bool bindToRobot(CadNode* robotNode, std::string* errorMessage = nullptr);
    std::vector<RobotCollisionPair> selfCollisions(const std::array<double, 6>& q) const;

private:
    struct HullBinding {
        int linkIndex = 0;
        ConvexHullData hull;
    };
    struct JointAxis {
        std::array<double, 3> origin{{0.0, 0.0, 0.0}};
        std::array<double, 3> direction{{0.0, 0.0, 0.0}};
        bool valid = false;
    };

    OPW6RobotData* m_robotData = nullptr;
    std::array<JointAxis, 6> m_jointAxes{};
    std::vector<HullBinding> m_hulls;

    bool isIgnored(int a, int b) const;
};
