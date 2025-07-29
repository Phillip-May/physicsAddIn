#pragma once

#include <QVector3D>
#include <QVector4D>
#include <vector>
#include <memory>
#include <QDebug>
#include <cmath>
#include <functional>

// PhysX includes
#include "PxPhysicsAPI.h"
#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"

// Forward declarations
class CadOpenGLWidget;

// Define ConnectionPathSegment structure to avoid circular dependency
struct ConnectionPathSegment {
    QVector3D start;
    QVector3D end;
    QVector4D color;
    float width;
    bool isBend;
    
    ConnectionPathSegment(const QVector3D& s, const QVector3D& e, const QVector4D& c = QVector4D(1, 0, 0, 1), float w = 2.0f, bool bend = false)
        : start(s), end(e), color(c), width(w), isBend(bend) {}
};

// Core drag chain segment - now using PhysX transforms
struct DragChainSegment {
    QVector3D startPoint;
    QVector3D endPoint;
    QVector3D direction;
    double length;           // Fixed pitch length
    double bendAngle;        // Angle from previous segment (radians)
    QVector3D bendAxis;      // Rotation axis for bend
    bool isBend;
    double bendRadius;
    
    // PhysX specific data
    physx::PxRigidDynamic* rigidBody;
    physx::PxTransform localPose;
    
    DragChainSegment(const QVector3D& start, const QVector3D& end, double segLength, 
                     double angle = 0.0, const QVector3D& axis = QVector3D(0,0,0), 
                     bool bend = false, double radius = 0.0)
        : startPoint(start), endPoint(end), direction((end - start).normalized()), 
          length(segLength), bendAngle(angle), bendAxis(axis), isBend(bend), bendRadius(radius),
          rigidBody(nullptr) {}
};

// PhysX-based constraint structure
struct PhysXDragChainConstraint {
    enum Type {
        StartPoint,          // Must start at specific point
        EndPoint,           // Must end at specific point  
        StartDirection,      // First segment must have specific direction
        EndDirection,        // Last segment must have specific direction
        MaxBendRadius,       // Bend radius must be <= maximum
        FixedPitchLength,    // All segments must have exact pitch length
        RigidBodyLimit,      // Rigid body position/rotation limits
        DriveTarget         // Target position for rigid body drive
    };
    
    Type type;
    physx::PxVec3 value;    // Constraint value (point, direction, etc.)
    double tolerance;        // Constraint tolerance
    
    PhysXDragChainConstraint(Type t, const physx::PxVec3& val, double tol = 0.1)
        : type(t), value(val), tolerance(tol) {}
};

class DragChainConstraintSolver {
public:
    DragChainConstraintSolver();
    ~DragChainConstraintSolver();
    
    // Initialize PhysX foundation and physics
    bool initializePhysX();
    void cleanupPhysX();
    
    // Main solving function using PhysX rigid bodies
    std::vector<DragChainSegment> solveDragChainPath(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& startDirection,
        const QVector3D& endDirection,
        double pitchLength,
        double maxBendRadius,
        int segmentCount,
        bool startLocked,
        bool endLocked,
        const QVector3D& startPlaneNormal = QVector3D(0, 0, 1),
        const QVector3D& endPlaneNormal = QVector3D(0, 0, 1)
    );
    
    // Convert solved path to visualization segments
    std::vector<ConnectionPathSegment> convertToVisualizationSegments(
        const std::vector<DragChainSegment>& segments,
        const QVector4D& mainColor = QVector4D(1, 0, 0, 0.8f),
        const QVector4D& indicatorColor = QVector4D(0, 1, 0, 1.0f)
    );
    
    // Get path statistics from solved segments
    struct PathStatistics {
        double totalLength;
        int totalSegments;
        int straightSegments;
        int bendSegments;
        double averageSegmentLength;
        double maxSegmentLength;
        double minSegmentLength;
        bool isValid;
        
        PathStatistics() : totalLength(0.0), totalSegments(0), straightSegments(0), bendSegments(0),
                          averageSegmentLength(0.0), maxSegmentLength(0.0), minSegmentLength(0.0), isValid(false) {}
    };
    
    PathStatistics calculatePathStatistics(const std::vector<DragChainSegment>& segments);
    
    // Solver statistics structure
    struct SolverStatistics {
        int iterationsUsed;
        double finalDistanceToTarget;
        bool converged;
        double simulationTime;
        int rigidBodiesCreated;
        int jointsCreated;
        double minSegmentLength;
        double maxSegmentLength;
        double averageSegmentLength;
        
        SolverStatistics() : iterationsUsed(0), finalDistanceToTarget(0.0), converged(false), 
                           simulationTime(0.0), rigidBodiesCreated(0), jointsCreated(0),
                           minSegmentLength(0.0), maxSegmentLength(0.0), averageSegmentLength(0.0) {}
        
        // Copy constructor
        SolverStatistics(const SolverStatistics& other) 
            : iterationsUsed(other.iterationsUsed), finalDistanceToTarget(other.finalDistanceToTarget),
              converged(other.converged), simulationTime(other.simulationTime),
              rigidBodiesCreated(other.rigidBodiesCreated), jointsCreated(other.jointsCreated),
              minSegmentLength(other.minSegmentLength), maxSegmentLength(other.maxSegmentLength),
              averageSegmentLength(other.averageSegmentLength) {}
        
        // Assignment operator
        SolverStatistics& operator=(const SolverStatistics& other) {
            if (this != &other) {
                iterationsUsed = other.iterationsUsed;
                finalDistanceToTarget = other.finalDistanceToTarget;
                converged = other.converged;
                simulationTime = other.simulationTime;
                rigidBodiesCreated = other.rigidBodiesCreated;
                jointsCreated = other.jointsCreated;
                minSegmentLength = other.minSegmentLength;
                maxSegmentLength = other.maxSegmentLength;
                averageSegmentLength = other.averageSegmentLength;
            }
            return *this;
        }
    };
    
    SolverStatistics getLastSolverStatistics() const { 
        qDebug() << "[PhysXDragChainSolver] getLastSolverStatistics() called";
        qDebug() << "[PhysXDragChainSolver]   Current stats - Distance to target:" << m_lastSolverStatistics.finalDistanceToTarget << "mm";
        qDebug() << "[PhysXDragChainSolver]   Current stats - Converged:" << m_lastSolverStatistics.converged;
        qDebug() << "[PhysXDragChainSolver]   Current stats - Iterations:" << m_lastSolverStatistics.iterationsUsed;
        qDebug() << "[PhysXDragChainSolver]   Current stats - Min segment length:" << m_lastSolverStatistics.minSegmentLength << "mm";
        qDebug() << "[PhysXDragChainSolver]   Current stats - Max segment length:" << m_lastSolverStatistics.maxSegmentLength << "mm";
        qDebug() << "[PhysXDragChainSolver]   Current stats - Avg segment length:" << m_lastSolverStatistics.averageSegmentLength << "mm";
        SolverStatistics stats = m_lastSolverStatistics;
        return stats;
    }
    
    // Get real-time statistics during solving
    SolverStatistics getCurrentSolverStatistics() const { 
        return m_lastSolverStatistics;
    }
    
    // Update solver statistics in real-time
    void updateSolverStatistics(double distanceToTarget, bool converged, int additionalIterations);
    
    // Calculate segment length statistics from current rigid body positions
    void calculateSegmentLengthStatistics();
    
    // Reset solver statistics
    void resetSolverStatistics() { m_lastSolverStatistics = SolverStatistics(); }
    
    // Set callback function for real-time statistics updates
    void setStatisticsCallback(std::function<void(const SolverStatistics&)> callback) {
        m_statisticsCallback = callback;
    }
    
    // Test function
    bool testConstraintSolver();
    
    // Example usage function
    void exampleUsage();
    
    // Solver parameters
    void setMaxIterations(int iterations) { m_maxIterations = iterations; }
    void setConvergenceTolerance(double tolerance) { m_convergenceTolerance = tolerance; }
    void setBendRadiusTolerance(double tolerance) { m_bendRadiusTolerance = tolerance; }
    void setSimulationTimeStep(double timeStep) { m_simulationTimeStep = timeStep; }
    void setMaxSimulationTime(double maxTime) { m_maxSimulationTime = maxTime; }
    
    // Get solver statistics
    int getMaxIterations() const { return m_maxIterations; }
    double getConvergenceTolerance() const { return m_convergenceTolerance; }
    double getBendRadiusTolerance() const { return m_bendRadiusTolerance; }
    double getSimulationTimeStep() const { return m_simulationTimeStep; }
    double getMaxSimulationTime() const { return m_maxSimulationTime; }

private:
    // PhysX core components
    physx::PxFoundation* m_foundation;
    physx::PxPhysics* m_physics;
    physx::PxScene* m_scene;
    physx::PxMaterial* m_material;
    
    // Rigid body system
    std::vector<physx::PxRigidDynamic*> m_rigidBodies;
    std::vector<physx::PxTransform> m_targetPoses;
    std::vector<physx::PxJoint*> m_joints;
    
    // Fixed joints for start and end points
    physx::PxFixedJoint* m_startFixedJoint;
    physx::PxFixedJoint* m_endFixedJoint;
    physx::PxRigidStatic* m_startAnchor;
    physx::PxRigidStatic* m_endAnchor;
    
    // Core PhysX-based solving functions
    bool createRigidBodyChain(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& startDirection,
        double pitchLength,
        int segmentCount,
        double maxBendRadius
    );
    
    bool solveRigidBodyConstraints(
        const QVector3D& targetEndPoint,
        const QVector3D& targetEndDirection,
        bool startLocked,
        bool endLocked,
        double maxBendRadius,
        double pitchLength,
        int segmentCount
    );
    
    // PhysX utility functions
    physx::PxTransform qVector3DToPxTransform(const QVector3D& position, const QVector3D& direction);
    QVector3D pxTransformToQVector3D(const physx::PxTransform& transform);
    physx::PxVec3 qVector3DToPxVec3(const QVector3D& vec);
    QVector3D pxVec3ToQVector3D(const physx::PxVec3& vec);
    physx::PxQuat qVector3DToPxQuat(const QVector3D& axis, double angle);
    
    // Constraint solving with PhysX
    bool applyStartPointConstraint(const QVector3D& startPoint);
    bool applyEndPointConstraint(const QVector3D& endPoint);
    bool applyDirectionConstraints(const QVector3D& startDirection, const QVector3D& endDirection);
    bool applyBendRadiusConstraints(double maxBendRadius, double pitchLength);
    bool applyPitchLengthConstraints(double pitchLength);
    bool applyExcessSegmentForces(const QVector3D& targetEndPoint, double pitchLength);
    std::vector<QVector3D> calculateOptimalChainConfiguration(const QVector3D& startPoint, const QVector3D& endPoint, double pitchLength, int segmentCount);
    
    // Fixed joint management
    bool createFixedJoints(const QVector3D& startPoint, const QVector3D& endPoint);
    void cleanupFixedJoints();
    
    // Progressive segment addition with distributed bends
    bool addSegmentProgressively(const QVector3D& targetEndPoint, double pitchLength, double maxBendRadius, int segmentCount);
    bool addSingleSegmentWithDistributedBend(const QVector3D& targetEndPoint, double pitchLength, double maxBendRadius, int segmentIndex, double optimalSag);
    bool stabilizeChain(double timeStep, int maxSteps, const QVector3D& endPoint);
    bool stabilizeChainWithDistributedBends(const QVector3D& targetEndPoint, double pitchLength, double maxBendRadius, double optimalSag);
    
    // Distributed bend calculation methods
    double calculateOptimalSag(double totalDistance, double pitchLength, double maxBendRadius);
    double calculateBendDistributionFactor(int segmentIndex, int totalSegments);
    QVector3D calculateDistributedBendOffset(const QVector3D& direction, double maxBendRadius, double distributionFactor, double optimalSag);
    QVector3D calculateSagOffset(const QVector3D& currentPos, const QVector3D& startPos, const QVector3D& endPos, double optimalSag, double distributionFactor);
    
    // PhysX simulation helpers
    void simulateRigidBodies(double timeStep, int maxSteps, const QVector3D& endPoint);
    bool checkConvergence(const QVector3D& targetEndPoint, double tolerance);
    void updateSegmentPositions(std::vector<DragChainSegment>& segments);
    
    // Geometric calculations (adapted for PhysX)
    double calculateBendRadius(double angle, double segmentLength);
    double calculateMaxBendAngle(double segmentLength, double maxBendRadius);
    physx::PxVec3 calculatePerpendicularAxis(const physx::PxVec3& direction);
    physx::PxQuat calculateRotationBetweenVectors(const physx::PxVec3& from, const physx::PxVec3& to);
    
    // Visualization helpers
    std::vector<QVector3D> generateBendPoints(const QVector3D& start, const QVector3D& end, const QVector3D& direction, double radius, int numPoints = 8);
    QVector3D calculateBendCenter(const QVector3D& start, const QVector3D& end, const QVector3D& direction, double radius);
    
    // Joint visualization
    std::vector<ConnectionPathSegment> generateJointSpheres(const std::vector<DragChainSegment>& segments, double jointRadius = 2.0f);
    
    // Constraint validation
    bool validateBendRadiusConstraints(const std::vector<DragChainSegment>& segments, double maxBendRadius);
    bool validateDirectionConstraints(const std::vector<DragChainSegment>& segments, const QVector3D& startDirection, const QVector3D& endDirection);
    
private:
    int m_maxIterations;
    double m_convergenceTolerance;
    double m_bendRadiusTolerance;
    double m_simulationTimeStep;
    double m_maxSimulationTime;
    bool m_physXInitialized;
    
    // Solver statistics
    SolverStatistics m_lastSolverStatistics;
    
    // Callback function for real-time statistics updates
    std::function<void(const SolverStatistics&)> m_statisticsCallback;
}; 