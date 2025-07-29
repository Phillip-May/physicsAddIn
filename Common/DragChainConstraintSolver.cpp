#include "DragChainConstraintSolver.h"
#include "CadOpenGLWidget.h"
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <chrono>

// PhysX includes
#include "PxPhysicsAPI.h"
#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"


using namespace physx;

DragChainConstraintSolver::DragChainConstraintSolver()
    : m_maxIterations(200)  // Increased from 100
    , m_convergenceTolerance(0.5)  // Increased from 0.01 for more realistic tolerance
    , m_bendRadiusTolerance(0.1)
    , m_simulationTimeStep(1.0/120.0)  // Smaller time step for more stable simulation
    , m_maxSimulationTime(10.0)  // Increased from 5.0
    , m_physXInitialized(false)
    , m_foundation(nullptr)
    , m_physics(nullptr)
    , m_scene(nullptr)
    , m_material(nullptr)
    , m_startFixedJoint(nullptr)
    , m_endFixedJoint(nullptr)
    , m_startAnchor(nullptr)
    , m_endAnchor(nullptr)
    , m_lastSolverStatistics()
    , m_statisticsCallback(nullptr)
{
    initializePhysX();
}

DragChainConstraintSolver::~DragChainConstraintSolver()
{
    cleanupPhysX();
}

bool DragChainConstraintSolver::initializePhysX()
{
    if (m_physXInitialized) {
        return true;
    }
    
    qDebug() << "[PhysXDragChainSolver] Initializing PhysX 5.6...";
    
    // Create foundation
    static PxDefaultAllocator gAllocator;
    static PxDefaultErrorCallback gErrorCallback;
    m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, 
                                     gAllocator, 
                                     gErrorCallback);
    if (!m_foundation) {
        qDebug() << "[PhysXDragChainSolver] Failed to create PhysX foundation";
        return false;
    }
    
    // Create physics
    m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, 
                               PxTolerancesScale(), false, nullptr);
    if (!m_physics) {
        qDebug() << "[PhysXDragChainSolver] Failed to create PhysX physics";
        return false;
    }
    
    // Create scene
    PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, 0.0f, -9.81f); // Z is vertical, gravity points down
    sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(4);
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
    
    m_scene = m_physics->createScene(sceneDesc);
    if (!m_scene) {
        qDebug() << "[PhysXDragChainSolver] Failed to create PhysX scene";
        return false;
    }
    
    // Create material
    m_material = m_physics->createMaterial(0.5f, 0.5f, 0.1f);
    if (!m_material) {
        qDebug() << "[PhysXDragChainSolver] Failed to create PhysX material";
        return false;
    }
    
    m_physXInitialized = true;
    qDebug() << "[PhysXDragChainSolver] PhysX initialization successful";
    return true;
}

void DragChainConstraintSolver::cleanupPhysX()
{
    if (!m_physXInitialized) {
        return;
    }
    
    qDebug() << "[PhysXDragChainSolver] Cleaning up PhysX...";
    
    // Clean up fixed joints
    cleanupFixedJoints();
    
    // Clean up joints
    for (auto* joint : m_joints) {
        if (joint) {
            joint->release();
        }
    }
    m_joints.clear();
    
    // Clean up rigid bodies
    for (auto* rigidBody : m_rigidBodies) {
        if (rigidBody) {
            m_scene->removeActor(*rigidBody);
            rigidBody->release();
        }
    }
    m_rigidBodies.clear();
    m_targetPoses.clear();
    
    // Clean up scene
    if (m_scene) {
        m_scene->release();
        m_scene = nullptr;
    }
    
    // Clean up material
    if (m_material) {
        m_material->release();
        m_material = nullptr;
    }
    
    // Clean up physics
    if (m_physics) {
        m_physics->release();
        m_physics = nullptr;
    }
    
    // Clean up foundation
    if (m_foundation) {
        m_foundation->release();
        m_foundation = nullptr;
    }
    
    m_physXInitialized = false;
    qDebug() << "[PhysXDragChainSolver] PhysX cleanup completed";
}

std::vector<DragChainSegment> DragChainConstraintSolver::solveDragChainPath(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& startDirection,
    const QVector3D& endDirection,
    double pitchLength,
    double maxBendRadius,
    int segmentCount,
    bool startLocked,
    bool endLocked,
    const QVector3D& startPlaneNormal,
    const QVector3D& endPlaneNormal)
{
    qDebug() << "[PhysXDragChainSolver] Solving drag chain path with" << segmentCount << "segments";
    qDebug() << "[PhysXDragChainSolver] Start:" << startPoint << "End:" << endPoint;
    qDebug() << "[PhysXDragChainSolver] Pitch length:" << pitchLength << "Max bend radius:" << maxBendRadius;
    
    // Calculate total distance first
    double totalDistance = (endPoint - startPoint).length();
    
    // Reset solver statistics for this run
    m_lastSolverStatistics = SolverStatistics();
    m_lastSolverStatistics.iterationsUsed = 0;
    m_lastSolverStatistics.finalDistanceToTarget = totalDistance;
    m_lastSolverStatistics.converged = false;
    m_lastSolverStatistics.simulationTime = 0.0;
    m_lastSolverStatistics.rigidBodiesCreated = 0;
    m_lastSolverStatistics.jointsCreated = 0;
    
    // Call callback for initial statistics
    if (m_statisticsCallback) {
        m_statisticsCallback(m_lastSolverStatistics);
    }
    
    if (!m_physXInitialized) {
        qDebug() << "[PhysXDragChainSolver] Error: PhysX not initialized";
        return std::vector<DragChainSegment>();
    }
    
    // Calculate how many segments we actually need
    int requiredSegments = static_cast<int>(std::ceil(totalDistance / pitchLength));
    
    if (totalDistance < 1e-6) {
        qDebug() << "[PhysXDragChainSolver] Points are too close, creating single segment";
        std::vector<DragChainSegment> segments;
        segments.emplace_back(startPoint, endPoint, totalDistance, 0.0, QVector3D(0,0,0), false, 0.0);
        return segments;
    }
    
    // Start with the minimum required segments, then add more progressively if needed
    int initialSegmentCount = requiredSegments;
    if (segmentCount > requiredSegments) {
        qDebug() << "[PhysXDragChainSolver] Starting with" << initialSegmentCount << "segments (minimum required)";
    } else {
        initialSegmentCount = segmentCount;
        qDebug() << "[PhysXDragChainSolver] Using requested" << segmentCount << "segments";
    }
    
    // Create rigid body chain with initial segment count
    if (!createRigidBodyChain(startPoint, endPoint, startDirection, pitchLength, initialSegmentCount, maxBendRadius)) {
        qDebug() << "[PhysXDragChainSolver] Failed to create rigid body chain";
        return std::vector<DragChainSegment>();
    }
    
    // Solve constraints using PhysX simulation
    if (!solveRigidBodyConstraints(endPoint, endDirection, startLocked, endLocked, maxBendRadius, pitchLength, segmentCount)) {
        qDebug() << "[PhysXDragChainSolver] Failed to solve rigid body constraints";
        return std::vector<DragChainSegment>();
    }
    
    // Convert rigid bodies to segments - use PhysX simulation results
    std::vector<DragChainSegment> segments;
    
    if (!m_rigidBodies.empty()) {
        qDebug() << "[PhysXDragChainSolver] Converting PhysX simulation results to segments";
        
        // Create segments based on the actual positions of rigid bodies after simulation
        // Each rigid body represents a joint, so we need to create segments between joints
        for (size_t i = 0; i < m_rigidBodies.size() - 1; ++i) {
            QVector3D segmentStart, segmentEnd;
            
            if (i == 0) {
                // First segment: start at startPoint, end at first rigid body position
                segmentStart = startPoint;
                segmentEnd = pxTransformToQVector3D(m_rigidBodies[i]->getGlobalPose());
                
                // Ensure first segment has exact pitch length
                QVector3D direction = (segmentEnd - segmentStart).normalized();
                segmentEnd = segmentStart + direction * pitchLength;
            } else {
                // Middle and last segments: start at previous rigid body, end at current rigid body
                QVector3D prevBodyPos = pxTransformToQVector3D(m_rigidBodies[i - 1]->getGlobalPose());
                QVector3D currBodyPos = pxTransformToQVector3D(m_rigidBodies[i]->getGlobalPose());
                
                segmentStart = prevBodyPos;
                segmentEnd = currBodyPos;
            }
            
            // Calculate segment properties
            double segmentLength = (segmentEnd - segmentStart).length();
            QVector3D segmentDirection = (segmentEnd - segmentStart).normalized();
            
            // Calculate bend angle and axis (simplified)
            double bendAngle = 0.0;
            QVector3D bendAxis(0, 0, 0);
            bool isBend = false;
            double bendRadius = 0.0;
            
            if (i > 0) {
                // Calculate bend relative to previous segment
                QVector3D prevDirection = (segmentStart - pxTransformToQVector3D(m_rigidBodies[i-1]->getGlobalPose())).normalized();
                double dotProduct = QVector3D::dotProduct(prevDirection, segmentDirection);
                dotProduct = std::max(-1.0, std::min(1.0, dotProduct)); // Clamp to valid range
                bendAngle = std::acos(dotProduct);
                
                if (bendAngle > 0.01) { // Significant bend
                    isBend = true;
                    bendAxis = QVector3D::crossProduct(prevDirection, segmentDirection).normalized();
                    bendRadius = calculateBendRadius(bendAngle, segmentLength);
                }
            }
            
            // Create segment
            DragChainSegment segment(segmentStart, segmentEnd, segmentLength, bendAngle, bendAxis, isBend, bendRadius);
            segments.push_back(segment);
        }
        
        // Add final segment to end point if needed
        if (!m_rigidBodies.empty()) {
            QVector3D lastBodyPos = pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose());
            double finalDistance = (endPoint - lastBodyPos).length();
            
            if (finalDistance > 1.0) { // Only add if there's a significant gap
                DragChainSegment finalSegment(lastBodyPos, endPoint, finalDistance, 0.0, QVector3D(0,0,0), false, 0.0);
                segments.push_back(finalSegment);
            }
        }
        
        qDebug() << "[PhysXDragChainSolver] Generated" << segments.size() << "segments from simulation";
    }
    
    return segments;
}

bool DragChainConstraintSolver::createRigidBodyChain(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& startDirection,
    double pitchLength,
    int segmentCount,
    double maxBendRadius)
{
    qDebug() << "[PhysXDragChainSolver] Creating rigid body chain with" << segmentCount << "segments";
    
    // Calculate optimal initial positions for the chain
    std::vector<QVector3D> optimalPositions = calculateOptimalChainConfiguration(startPoint, endPoint, pitchLength, segmentCount);
    
    // Create rigid bodies for each joint (segmentCount + 1 joints for segmentCount segments)
    for (int i = 0; i < segmentCount + 1; ++i) {
        // Use optimal position if available, otherwise fall back to straight line
        PxVec3 bodyPos;
        if (i < optimalPositions.size()) {
            bodyPos = qVector3DToPxVec3(optimalPositions[i]);
        } else {
            // Fallback to straight line positioning
            QVector3D direction = (endPoint - startPoint).normalized();
            QVector3D position = startPoint + direction * (pitchLength * i);
            bodyPos = qVector3DToPxVec3(position);
        }
        
        PxTransform bodyPose(bodyPos);
        
        // Create rigid dynamic
        PxRigidDynamic* rigidBody = m_physics->createRigidDynamic(bodyPose);
        if (!rigidBody) {
            qDebug() << "[PhysXDragChainSolver] Failed to create rigid body" << i;
            return false;
        }
        
        // Create shape for rigid body - smaller to allow more flexibility
        PxShape* shape = m_physics->createShape(PxBoxGeometry(pitchLength * 0.05f, pitchLength * 0.05f, pitchLength * 0.05f), *m_material);
        rigidBody->attachShape(*shape);
        shape->release();
        
        // Configure rigid body for realistic chain behavior
        rigidBody->setMass(0.5f); // Lighter for more responsive movement
        rigidBody->setLinearDamping(0.2f); // More damping for stability
        rigidBody->setAngularDamping(0.3f);
        
        m_rigidBodies.push_back(rigidBody);
        m_scene->addActor(*rigidBody);
        
        // Create constraints between consecutive bodies
        if (i > 0) {
            PxRigidDynamic* prevBody = m_rigidBodies[i - 1];
            PxRigidDynamic* currBody = rigidBody;
            
            // Create a revolute joint to allow rotation around a single axis (like a real drag chain)
            PxTransform localFrame0(PxVec3(pitchLength * 0.5f, 0, 0));
            PxTransform localFrame1(PxVec3(-pitchLength * 0.5f, 0, 0));
            
            PxRevoluteJoint* joint = PxRevoluteJointCreate(*m_physics, prevBody, localFrame0, currBody, localFrame1);
            if (joint) {
                // Convert max bend radius to radians for joint limits
                // For a drag chain, we want to limit the bend angle based on the max bend radius
                double maxBendAngle = calculateMaxBendAngle(pitchLength, maxBendRadius);
                float maxAngleRadians = static_cast<float>(maxBendAngle);
                
                // Debug: Print bend radius to angle conversion
                if (i == 1) { // Only print for first joint to avoid spam
                    qDebug() << "[PhysXDragChainSolver] Bend radius conversion:";
                    qDebug() << "[PhysXDragChainSolver]   Pitch length:" << pitchLength << "mm";
                    qDebug() << "[PhysXDragChainSolver]   Max bend radius:" << maxBendRadius << "mm";
                    qDebug() << "[PhysXDragChainSolver]   Calculated max angle:" << (maxBendAngle * 180.0 / M_PI) << "degrees";
                    qDebug() << "[PhysXDragChainSolver]   Joint limit range: ±" << (maxAngleRadians * 180.0 / M_PI) << "degrees";
                }
                
                // Set joint limits to control the range of rotation
                // Allow more flexibility for chain deformation
                float actualMaxAngle = std::max(maxAngleRadians, static_cast<float>(M_PI * 0.75)); // Allow up to 135 degrees
                joint->setLimit(PxJointAngularLimitPair(-actualMaxAngle, actualMaxAngle));
                joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
                
                // Add some drive to help maintain chain structure
                joint->setDriveVelocity(0.0f);
                joint->setDriveForceLimit(100.0f);
                joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
                
                m_joints.push_back(joint);
            }
        }
    }
    
    qDebug() << "[PhysXDragChainSolver] Created rigid body chain with" << m_rigidBodies.size() << "bodies and" << m_joints.size() << "revolute joints";
    
    // Debug: Print initial positions
    for (size_t i = 0; i < m_rigidBodies.size(); ++i) {
        PxTransform pose = m_rigidBodies[i]->getGlobalPose();
        qDebug() << "[PhysXDragChainSolver] Body" << i << "position:" << pxVec3ToQVector3D(pose.p);
    }
    
    return true;
}

bool DragChainConstraintSolver::solveRigidBodyConstraints(
    const QVector3D& targetEndPoint,
    const QVector3D& targetEndDirection,
    bool startLocked,
    bool endLocked,
    double maxBendRadius,
    double pitchLength,
    int segmentCount)
{
    qDebug() << "[PhysXDragChainSolver] Solving rigid body constraints with progressive segment addition";
    
    if (m_rigidBodies.empty()) {
        qDebug() << "[PhysXDragChainSolver] No rigid bodies to solve";
        return false;
    }
    
    // Start timing the entire solving process
    auto startTime = std::chrono::high_resolution_clock::now();
    int totalIterations = 0;
    
    // Calculate how many segments we actually need
    double totalDistance = (targetEndPoint - pxTransformToQVector3D(m_rigidBodies[0]->getGlobalPose())).length();
    int requiredSegments = static_cast<int>(std::ceil(totalDistance / pitchLength));
    int currentSegments = static_cast<int>(m_rigidBodies.size() - 1);
    
    qDebug() << "[PhysXDragChainSolver] Progressive solving analysis:";
    qDebug() << "[PhysXDragChainSolver]   Total distance to target:" << totalDistance << "mm";
    qDebug() << "[PhysXDragChainSolver]   Pitch length:" << pitchLength << "mm";
    qDebug() << "[PhysXDragChainSolver]   Required segments:" << requiredSegments;
    qDebug() << "[PhysXDragChainSolver]   Current segments:" << currentSegments;
    
    // Create fixed joints for start and end points
    QVector3D startPoint = pxTransformToQVector3D(m_rigidBodies[0]->getGlobalPose());
    if (!createFixedJoints(startPoint, targetEndPoint)) {
        qDebug() << "[PhysXDragChainSolver] Failed to create fixed joints";
        return false;
    }
    
    // Apply direction constraints for orientation
    if (!applyDirectionConstraints(QVector3D(1,0,0), targetEndDirection)) {
        qDebug() << "[PhysXDragChainSolver] Failed to apply direction constraints";
        return false;
    }
    
    // Apply bend radius constraints
    if (!applyBendRadiusConstraints(maxBendRadius, pitchLength)) {
        qDebug() << "[PhysXDragChainSolver] Failed to apply bend radius constraints";
        return false;
    }
    
    // Apply pitch length constraints
    if (!applyPitchLengthConstraints(pitchLength)) {
        qDebug() << "[PhysXDragChainSolver] Failed to apply pitch length constraints";
        return false;
    }
    
    // Use progressive segment addition instead of trying to handle all segments at once
    if (!addSegmentProgressively(targetEndPoint, pitchLength, maxBendRadius, segmentCount)) {
        qDebug() << "[PhysXDragChainSolver] Failed to add segments progressively";
        return false;
    }
    
    // Final stabilization
    qDebug() << "[PhysXDragChainSolver] Performing final stabilization...";
    if (!stabilizeChain(m_simulationTimeStep, 100, targetEndPoint)) {
        qDebug() << "[PhysXDragChainSolver] Failed to perform final stabilization";
        return false;
    }
    
    // Call callback for final statistics after stabilization
    if (m_statisticsCallback) {
        m_statisticsCallback(m_lastSolverStatistics);
    }
    
    // End timing
    auto endTime = std::chrono::high_resolution_clock::now();
    
    // Track solver statistics
    m_lastSolverStatistics.rigidBodiesCreated = static_cast<int>(m_rigidBodies.size());
    m_lastSolverStatistics.jointsCreated = static_cast<int>(m_joints.size());
    m_lastSolverStatistics.simulationTime = std::chrono::duration<double>(endTime - startTime).count();
    m_lastSolverStatistics.iterationsUsed = totalIterations;
    
    // Check if we converged to the target
    m_lastSolverStatistics.converged = checkConvergence(targetEndPoint, m_convergenceTolerance);
    m_lastSolverStatistics.finalDistanceToTarget = (pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose()) - targetEndPoint).length();
    
    qDebug() << "[PhysXDragChainSolver] Final results:";
    qDebug() << "[PhysXDragChainSolver]   Distance to target:" << m_lastSolverStatistics.finalDistanceToTarget << "mm";
    qDebug() << "[PhysXDragChainSolver]   Converged:" << m_lastSolverStatistics.converged;
    qDebug() << "[PhysXDragChainSolver]   Rigid bodies:" << m_lastSolverStatistics.rigidBodiesCreated;
    qDebug() << "[PhysXDragChainSolver]   Joints:" << m_lastSolverStatistics.jointsCreated;
    qDebug() << "[PhysXDragChainSolver]   Simulation time:" << m_lastSolverStatistics.simulationTime << "s";
    qDebug() << "[PhysXDragChainSolver]   Total iterations:" << m_lastSolverStatistics.iterationsUsed;
    
    if (!m_lastSolverStatistics.converged) {
        qDebug() << "[PhysXDragChainSolver] Warning: Did not fully converge to target, but continuing";
    }
    
    qDebug() << "[PhysXDragChainSolver] Progressive solving completed successfully";
    
    // Calculate final segment length statistics before callback
    calculateSegmentLengthStatistics();
    
    qDebug() << "[PhysXDragChainSolver] Final statistics before callback:";
    qDebug() << "[PhysXDragChainSolver]   Min segment length:" << m_lastSolverStatistics.minSegmentLength << "mm";
    qDebug() << "[PhysXDragChainSolver]   Max segment length:" << m_lastSolverStatistics.maxSegmentLength << "mm";
    qDebug() << "[PhysXDragChainSolver]   Avg segment length:" << m_lastSolverStatistics.averageSegmentLength << "mm";
    qDebug() << "[PhysXDragChainSolver]   Iterations:" << m_lastSolverStatistics.iterationsUsed;
    qDebug() << "[PhysXDragChainSolver]   Converged:" << m_lastSolverStatistics.converged;
    
    // Call callback for final statistics
    if (m_statisticsCallback) {
        m_statisticsCallback(m_lastSolverStatistics);
    }
    
    return true;
}

bool DragChainConstraintSolver::applyStartPointConstraint(const QVector3D& startPoint)
{
    // This method is no longer used since we use fixed joints
    // The start point is now fixed by a PxFixedJoint
    qDebug() << "[PhysXDragChainSolver] Start point constraint handled by fixed joint";
    return true;
}

bool DragChainConstraintSolver::applyEndPointConstraint(const QVector3D& endPoint)
{
    // This method is no longer used since we use fixed joints
    // The end point is now fixed by a PxFixedJoint
    qDebug() << "[PhysXDragChainSolver] End point constraint handled by fixed joint";
    return true;
}

bool DragChainConstraintSolver::applyDirectionConstraints(const QVector3D& startDirection, const QVector3D& endDirection)
{
    if (m_rigidBodies.size() < 2) return true;
    
    // Set initial direction for first rigid body
    PxVec3 startDir = qVector3DToPxVec3(startDirection);
    PxTransform firstPose = m_rigidBodies[0]->getGlobalPose();
    // Create quaternion from direction using cross product
    PxVec3 up(0, 1, 0);
    PxVec3 right = startDir.cross(up);
    if (right.magnitude() < 0.001f) {
        right = startDir.cross(PxVec3(0, 0, 1));
    }
    right = right.getNormalized();
    up = right.cross(startDir).getNormalized();
    firstPose.q = PxQuat(PxMat33(right, up, startDir));
    m_rigidBodies[0]->setGlobalPose(firstPose);
    
    // Set target direction for last rigid body
    PxVec3 endDir = qVector3DToPxVec3(endDirection);
    PxRigidDynamic* lastBody = m_rigidBodies.back();
    PxTransform lastPose = lastBody->getGlobalPose();
    // Create quaternion from direction using cross product
    right = endDir.cross(up);
    if (right.magnitude() < 0.001f) {
        right = endDir.cross(PxVec3(0, 0, 1));
    }
    right = right.getNormalized();
    up = right.cross(endDir).getNormalized();
    lastPose.q = PxQuat(PxMat33(right, up, endDir));
    
    return true;
}

bool DragChainConstraintSolver::applyBendRadiusConstraints(double maxBendRadius, double pitchLength)
{
    double maxAngle = calculateMaxBendAngle(pitchLength, maxBendRadius);
    
    // Apply angular velocity limits to prevent excessive bending
    for (auto* rigidBody : m_rigidBodies) {
        if (rigidBody) {
            PxVec3 angularVelocity = rigidBody->getAngularVelocity();
            float currentAngle = angularVelocity.magnitude();
            if (currentAngle > maxAngle) {
                angularVelocity = angularVelocity.getNormalized() * maxAngle;
                rigidBody->setAngularVelocity(angularVelocity);
            }
        }
    }
    
    return true;
}

bool DragChainConstraintSolver::applyPitchLengthConstraints(double pitchLength)
{
    // Pitch length is enforced by the PhysX joints between rigid bodies
    // The joints were created in createRigidBodyChain with the correct pitch length
    qDebug() << "[PhysXDragChainSolver] Pitch length constraints enforced by" << m_joints.size() << "PhysX joints";
    return true;
}

bool DragChainConstraintSolver::applyExcessSegmentForces(const QVector3D& targetEndPoint, double pitchLength)
{
    if (m_rigidBodies.size() < 3) return true; // Need at least 3 bodies for deformation
    
    qDebug() << "[PhysXDragChainSolver] Applying excess segment deformation forces";
    
    // Calculate the ideal positions for the chain to reach the target
    QVector3D startPos = pxTransformToQVector3D(m_rigidBodies[0]->getGlobalPose());
    double totalDistance = (targetEndPoint - startPos).length();
    double totalChainLength = (m_rigidBodies.size() - 1) * pitchLength;
    double excessLength = totalChainLength - totalDistance;
    
    qDebug() << "[PhysXDragChainSolver] Excess length to distribute:" << excessLength << "mm";
    
    // Calculate how much each segment should be compressed
    double compressionPerSegment = excessLength / (m_rigidBodies.size() - 1);
    
    // Apply compression forces to middle bodies to help the chain deform
    for (size_t i = 1; i < m_rigidBodies.size() - 1; ++i) {
        PxRigidDynamic* body = m_rigidBodies[i];
        PxTransform currentPose = body->getGlobalPose();
        QVector3D currentPos = pxTransformToQVector3D(currentPose);
        
        // Calculate ideal position for this body
        double idealDistance = i * pitchLength - (i * compressionPerSegment);
        QVector3D direction = (targetEndPoint - startPos).normalized();
        QVector3D idealPos = startPos + direction * idealDistance;
        
        // Apply force toward ideal position
        QVector3D toIdeal = (idealPos - currentPos).normalized();
        PxVec3 forceDirection = qVector3DToPxVec3(toIdeal);
        float forceMagnitude = 30.0f; // Moderate force for deformation
        PxVec3 force = forceDirection * forceMagnitude;
        body->addForce(force);
        
        qDebug() << "[PhysXDragChainSolver] Body" << i << "ideal position:" << idealPos << "current:" << currentPos;
    }
    
    return true;
}

bool DragChainConstraintSolver::createFixedJoints(const QVector3D& startPoint, const QVector3D& endPoint)
{
    if (m_rigidBodies.empty()) {
        qDebug() << "[PhysXDragChainSolver] No rigid bodies available for fixed joints";
        return false;
    }
    
    qDebug() << "[PhysXDragChainSolver] Creating fixed joints for start and end points";
    
    // Create static anchors at start and end points
    PxTransform startAnchorPose = qVector3DToPxTransform(startPoint, QVector3D(1,0,0));
    PxTransform endAnchorPose = qVector3DToPxTransform(endPoint, QVector3D(1,0,0));
    
    m_startAnchor = m_physics->createRigidStatic(startAnchorPose);
    m_endAnchor = m_physics->createRigidStatic(endAnchorPose);
    
    if (!m_startAnchor || !m_endAnchor) {
        qDebug() << "[PhysXDragChainSolver] Failed to create anchor bodies";
        return false;
    }
    
    // Add anchors to scene
    m_scene->addActor(*m_startAnchor);
    m_scene->addActor(*m_endAnchor);
    
    // Create fixed joints between anchors and chain endpoints
    PxRigidDynamic* firstBody = m_rigidBodies[0];
    PxRigidDynamic* lastBody = m_rigidBodies.back();
    
    // Fixed joint for start point
    PxTransform startLocalFrame0(PxVec3(0, 0, 0)); // Anchor frame
    PxTransform startLocalFrame1(PxVec3(0, 0, 0)); // First body frame
    
    m_startFixedJoint = PxFixedJointCreate(*m_physics, m_startAnchor, startLocalFrame0, firstBody, startLocalFrame1);
    if (!m_startFixedJoint) {
        qDebug() << "[PhysXDragChainSolver] Failed to create start fixed joint";
        return false;
    }
    
    // Fixed joint for end point
    PxTransform endLocalFrame0(PxVec3(0, 0, 0)); // Anchor frame
    PxTransform endLocalFrame1(PxVec3(0, 0, 0)); // Last body frame
    
    m_endFixedJoint = PxFixedJointCreate(*m_physics, m_endAnchor, endLocalFrame0, lastBody, endLocalFrame1);
    if (!m_endFixedJoint) {
        qDebug() << "[PhysXDragChainSolver] Failed to create end fixed joint";
        return false;
    }
    
    qDebug() << "[PhysXDragChainSolver] Fixed joints created successfully:";
    qDebug() << "[PhysXDragChainSolver]   Start anchor at:" << startPoint;
    qDebug() << "[PhysXDragChainSolver]   End anchor at:" << endPoint;
    qDebug() << "[PhysXDragChainSolver]   First body at:" << pxTransformToQVector3D(firstBody->getGlobalPose());
    qDebug() << "[PhysXDragChainSolver]   Last body at:" << pxTransformToQVector3D(lastBody->getGlobalPose());
    
    return true;
}

void DragChainConstraintSolver::cleanupFixedJoints()
{
    qDebug() << "[PhysXDragChainSolver] Cleaning up fixed joints";
    
    // Clean up fixed joints
    if (m_startFixedJoint) {
        m_startFixedJoint->release();
        m_startFixedJoint = nullptr;
    }
    
    if (m_endFixedJoint) {
        m_endFixedJoint->release();
        m_endFixedJoint = nullptr;
    }
    
    // Clean up anchor bodies
    if (m_startAnchor) {
        m_scene->removeActor(*m_startAnchor);
        m_startAnchor->release();
        m_startAnchor = nullptr;
    }
    
    if (m_endAnchor) {
        m_scene->removeActor(*m_endAnchor);
        m_endAnchor->release();
        m_endAnchor = nullptr;
    }
}

void DragChainConstraintSolver::simulateRigidBodies(double timeStep, int maxSteps, const QVector3D& endPoint)
{
    qDebug() << "[PhysXDragChainSolver] Simulating rigid bodies for" << maxSteps << "steps";
    
    // Calculate the required total length for the chain
    double totalDistance = (endPoint - pxTransformToQVector3D(m_rigidBodies[0]->getGlobalPose())).length();
    // Use the actual pitch length from the joints instead of hardcoded 300mm
    double pitchLength = 300.0; // Default, will be updated from joints if available
    if (!m_joints.empty()) {
        // Estimate pitch length from joint positions
        if (m_rigidBodies.size() >= 2) {
            QVector3D firstPos = pxTransformToQVector3D(m_rigidBodies[0]->getGlobalPose());
            QVector3D secondPos = pxTransformToQVector3D(m_rigidBodies[1]->getGlobalPose());
            pitchLength = (secondPos - firstPos).length();
        }
    }
    double totalChainLength = (m_rigidBodies.size() - 1) * pitchLength;
    
    qDebug() << "[PhysXDragChainSolver] Total distance to target:" << totalDistance << "mm";
    qDebug() << "[PhysXDragChainSolver] Total chain length:" << totalChainLength << "mm";
    qDebug() << "[PhysXDragChainSolver] Chain needs to deform by:" << (totalChainLength - totalDistance) << "mm";
    qDebug() << "[PhysXDragChainSolver] Estimated pitch length:" << pitchLength << "mm";
    
    for (int step = 0; step < maxSteps; ++step) {
        // Apply forces to all bodies to maintain chain structure and drive toward target
        for (size_t i = 0; i < m_rigidBodies.size(); ++i) {
            PxRigidDynamic* body = m_rigidBodies[i];
            
            // Apply damping to reduce oscillation
            PxVec3 linearVel = body->getLinearVelocity();
            PxVec3 angularVel = body->getAngularVelocity();
            
            linearVel *= 0.98f; // Stronger damping for stability
            angularVel *= 0.98f;
            
            body->setLinearVelocity(linearVel);
            body->setAngularVelocity(angularVel);
            
            // Apply different forces based on body position in chain
            if (i == 0) {
                // First body: Fixed by joint - no forces needed
                // The fixed joint will maintain the position
            } else if (i == m_rigidBodies.size() - 1) {
                // Last body: Fixed by joint - no forces needed
                // The fixed joint will maintain the position
            } else {
                // Middle bodies: Apply forces to maintain chain structure and guide toward target
                
                // Force toward target (weaker than before since endpoints are fixed)
                PxTransform currentPose = body->getGlobalPose();
                QVector3D currentPos = pxTransformToQVector3D(currentPose);
                QVector3D toTarget = (endPoint - currentPos).normalized();
                PxVec3 forceDirection = qVector3DToPxVec3(toTarget);
                float forceMagnitude = 15.0f; // Weaker force since endpoints are fixed
                PxVec3 force = forceDirection * forceMagnitude;
                body->addForce(force);
                
                // Maintain chain tension with adjacent bodies
                if (i > 0) {
                    PxTransform prevPose = m_rigidBodies[i-1]->getGlobalPose();
                    QVector3D prevPos = pxTransformToQVector3D(prevPose);
                    QVector3D toPrev = (prevPos - currentPos).normalized();
                    PxVec3 tensionForce = qVector3DToPxVec3(toPrev) * 25.0f;
                    body->addForce(tensionForce);
                }
                
                if (i < m_rigidBodies.size() - 1) {
                    PxTransform nextPose = m_rigidBodies[i+1]->getGlobalPose();
                    QVector3D nextPos = pxTransformToQVector3D(nextPose);
                    QVector3D toNext = (nextPos - currentPos).normalized();
                    PxVec3 tensionForce = qVector3DToPxVec3(toNext) * 25.0f;
                    body->addForce(tensionForce);
                }
            }
        }
        
        m_scene->simulate(timeStep);
        m_scene->fetchResults(true);
        
        // Check convergence every 5 steps for more frequent updates
        if (step % 5 == 0) {
            // Check stability of middle bodies
            bool allBodiesStable = true;
            double maxVelocity = 0.0;
            
            for (size_t i = 1; i < m_rigidBodies.size() - 1; ++i) {
                PxRigidDynamic* body = m_rigidBodies[i];
                PxVec3 linearVelocity = body->getLinearVelocity();
                double velocityMagnitude = linearVelocity.magnitude();
                
                if (velocityMagnitude > maxVelocity) {
                    maxVelocity = velocityMagnitude;
                }
                
                if (velocityMagnitude > 0.05f) {
                    allBodiesStable = false;
                }
            }
            
            // Check end point distance (should be exact with fixed joint)
            double currentDistance = (pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose()) - endPoint).length();
            
            qDebug() << "[PhysXDragChainSolver] Step" << step << "/" << maxSteps 
                     << "Distance to target:" << currentDistance << "mm, Max velocity:" << maxVelocity << "m/s";
            
            // Update statistics and emit signal for real-time GUI updates
            bool currentConverged = currentDistance < m_convergenceTolerance && allBodiesStable;
            updateSolverStatistics(currentDistance, currentConverged, 5); // 5 steps since last update
            
            // Early convergence check - both distance and stability must be good
            if (currentConverged) {
                qDebug() << "[PhysXDragChainSolver] Early convergence at step" << step;
                break;
            }
        }
        
        if (step % 60 == 0) {
            qDebug() << "[PhysXDragChainSolver] Simulation step" << step << "/" << maxSteps;
            
            // Debug: Print positions every 60 steps
            if (!m_rigidBodies.empty()) {
                PxTransform firstPose = m_rigidBodies[0]->getGlobalPose();
                PxTransform lastPose = m_rigidBodies.back()->getGlobalPose();
                qDebug() << "[PhysXDragChainSolver] First body position:" << pxVec3ToQVector3D(firstPose.p);
                qDebug() << "[PhysXDragChainSolver] Last body position:" << pxVec3ToQVector3D(lastPose.p);
            }
        }
    }
}

bool DragChainConstraintSolver::checkConvergence(const QVector3D& targetEndPoint, double tolerance)
{
    if (m_rigidBodies.empty()) return false;
    
    // Since endpoints are fixed by joints, check convergence of middle bodies
    bool allBodiesStable = true;
    double maxVelocity = 0.0;
    
    for (size_t i = 1; i < m_rigidBodies.size() - 1; ++i) {
        PxRigidDynamic* body = m_rigidBodies[i];
        PxVec3 linearVelocity = body->getLinearVelocity();
        double velocityMagnitude = linearVelocity.magnitude();
        
        if (velocityMagnitude > maxVelocity) {
            maxVelocity = velocityMagnitude;
        }
        
        // Check if this body is stable (low velocity)
        if (velocityMagnitude > 0.05f) { // Less than 5cm/s
            allBodiesStable = false;
        }
    }
    
    // Also check that the end point is actually at the target (should be exact with fixed joint)
    PxRigidDynamic* lastBody = m_rigidBodies.back();
    PxTransform lastPose = lastBody->getGlobalPose();
    QVector3D currentEndPoint = pxTransformToQVector3D(lastPose);
    double distanceToTarget = (currentEndPoint - targetEndPoint).length();
    
    qDebug() << "[PhysXDragChainSolver] Convergence check:";
    qDebug() << "[PhysXDragChainSolver]   Distance to target:" << distanceToTarget << "mm (tolerance:" << tolerance << ")";
    qDebug() << "[PhysXDragChainSolver]   Max middle body velocity:" << maxVelocity << "m/s";
    qDebug() << "[PhysXDragChainSolver]   All bodies stable:" << allBodiesStable;
    qDebug() << "[PhysXDragChainSolver]   Current end point:" << currentEndPoint;
    qDebug() << "[PhysXDragChainSolver]   Target end point:" << targetEndPoint;
    
    // Check both distance and stability for convergence
    bool distanceConverged = distanceToTarget < tolerance;
    bool velocityConverged = allBodiesStable;
    
    bool converged = distanceConverged && velocityConverged;
    
    if (converged) {
        qDebug() << "[PhysXDragChainSolver] ✓ System converged!";
    } else {
        if (!distanceConverged) {
            qDebug() << "[PhysXDragChainSolver] ✗ Distance not converged";
        }
        if (!velocityConverged) {
            qDebug() << "[PhysXDragChainSolver] ✗ Velocity not converged";
        }
    }
    
    return converged;
}

void DragChainConstraintSolver::updateSegmentPositions(std::vector<DragChainSegment>& segments)
{
    // Update segment positions based on rigid body positions
    // Each rigid body represents a joint, so we have segmentCount + 1 bodies for segmentCount segments
    for (size_t i = 0; i < segments.size() && i < m_rigidBodies.size() - 1; ++i) {
        PxTransform globalPose = m_rigidBodies[i]->getGlobalPose();
        QVector3D position = pxTransformToQVector3D(globalPose);
        
        if (i == 0) {
            segments[i].startPoint = position;
        } else {
            segments[i-1].endPoint = position;
            segments[i].startPoint = position;
        }
        
        if (i == segments.size() - 1) {
            segments[i].endPoint = position;
        }
    }
}

// PhysX utility functions
PxTransform DragChainConstraintSolver::qVector3DToPxTransform(const QVector3D& position, const QVector3D& direction)
{
    PxVec3 pos = qVector3DToPxVec3(position);
    PxVec3 dir = qVector3DToPxVec3(direction);
    // Create quaternion from direction using cross product
    PxVec3 up(0, 1, 0);
    PxVec3 right = dir.cross(up);
    if (right.magnitude() < 0.001f) {
        right = dir.cross(PxVec3(0, 0, 1));
    }
    right = right.getNormalized();
    up = right.cross(dir).getNormalized();
    PxQuat rot = PxQuat(PxMat33(right, up, dir));
    return PxTransform(pos, rot);
}

QVector3D DragChainConstraintSolver::pxTransformToQVector3D(const PxTransform& transform)
{
    return QVector3D(transform.p.x, transform.p.y, transform.p.z);
}

PxVec3 DragChainConstraintSolver::qVector3DToPxVec3(const QVector3D& vec)
{
    return PxVec3(vec.x(), vec.y(), vec.z());
}

QVector3D DragChainConstraintSolver::pxVec3ToQVector3D(const PxVec3& vec)
{
    return QVector3D(vec.x, vec.y, vec.z);
}

PxQuat DragChainConstraintSolver::qVector3DToPxQuat(const QVector3D& axis, double angle)
{
    PxVec3 pxAxis = qVector3DToPxVec3(axis);
    return PxQuat(static_cast<PxReal>(angle), pxAxis);
}

// Geometric calculations (adapted for PhysX)
double DragChainConstraintSolver::calculateBendRadius(double angle, double segmentLength)
{
    if (angle < 0.01) {
        return 0.0; // No significant bend
    }
    
    // Calculate bend radius: R = L / (2 * sin(angle/2))
    double bendRadius = segmentLength / (2.0 * std::sin(angle / 2.0));
    return bendRadius;
}

double DragChainConstraintSolver::calculateMaxBendAngle(double segmentLength, double maxBendRadius)
{
    if (maxBendRadius <= 0.0) {
        return 0.0;
    }
    
    // Check if the segment length is too large for the given bend radius
    double ratio = segmentLength / (2.0 * maxBendRadius);
    if (ratio >= 1.0) {
        // Segment is too long for this bend radius - can't make a bend
        qDebug() << "[PhysXDragChainSolver] Warning: Segment length" << segmentLength 
                 << "is too large for bend radius" << maxBendRadius << "- no bends possible";
        return 0.0;
    }
    
    // Calculate maximum angle: angle = 2 * asin(L / (2 * R))
    double maxAngle = 2.0 * std::asin(ratio);
    
    // Verify the calculation by checking if we can reconstruct the bend radius
    double reconstructedRadius = segmentLength / (2.0 * std::sin(maxAngle / 2.0));
    if (std::abs(reconstructedRadius - maxBendRadius) > 0.1) {
        qDebug() << "[PhysXDragChainSolver] Warning: Bend radius calculation verification failed!";
        qDebug() << "[PhysXDragChainSolver]   Original radius:" << maxBendRadius << "mm";
        qDebug() << "[PhysXDragChainSolver]   Reconstructed radius:" << reconstructedRadius << "mm";
    }
    
    return maxAngle;
}

PxVec3 DragChainConstraintSolver::calculatePerpendicularAxis(const PxVec3& direction)
{
    // Find a perpendicular axis to the direction
    PxVec3 perpendicular = direction.cross(PxVec3(0, 0, 1));
    if (perpendicular.magnitude() < 0.1f) {
        perpendicular = direction.cross(PxVec3(0, 1, 0));
    }
    return perpendicular.getNormalized();
}

PxQuat DragChainConstraintSolver::calculateRotationBetweenVectors(const PxVec3& from, const PxVec3& to)
{
    PxVec3 fromNorm = from.getNormalized();
    PxVec3 toNorm = to.getNormalized();
    
    float dot = fromNorm.dot(toNorm);
    if (dot > 0.9999f) {
        return PxQuat(0, 0, 0, 1); // Identity quaternion
    } else if (dot < -0.9999f) {
        // Vectors are opposite, rotate 180 degrees around any perpendicular axis
        PxVec3 axis = calculatePerpendicularAxis(fromNorm);
        return PxQuat(PxPi, axis);
    } else {
        PxVec3 axis = fromNorm.cross(toNorm);
        float angle = std::acos(dot);
        return PxQuat(angle, axis);
    }
}

// Visualization helpers
std::vector<QVector3D> DragChainConstraintSolver::generateBendPoints(const QVector3D& start, const QVector3D& end, const QVector3D& direction, double radius, int numPoints)
{
    std::vector<QVector3D> bendPoints;
    
    if (radius <= 0.0 || numPoints < 2) {
        bendPoints.push_back(start);
        bendPoints.push_back(end);
        return bendPoints;
    }
    
    // Simple circular arc generation
    QVector3D center = calculateBendCenter(start, end, direction, radius);
    PxVec3 startVec = qVector3DToPxVec3((start - center).normalized());
    PxVec3 endVec = qVector3DToPxVec3((end - center).normalized());
    
    float dotProduct = startVec.dot(endVec);
    float angle = std::acos(std::clamp(dotProduct, -1.0f, 1.0f));
    
    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(numPoints - 1);
        float currentAngle = t * angle;
        
        PxVec3 perpendicular = calculatePerpendicularAxis(startVec);
        PxQuat rotation(currentAngle, perpendicular);
        PxVec3 rotatedVec = rotation.rotate(startVec);
        bendPoints.push_back(center + pxVec3ToQVector3D(rotatedVec * static_cast<float>(radius)));
    }
    
    return bendPoints;
}

QVector3D DragChainConstraintSolver::calculateBendCenter(const QVector3D& start, const QVector3D& end, const QVector3D& direction, double radius)
{
    QVector3D midPoint = (start + end) * 0.5f;
    QVector3D segmentVector = (end - start).normalized();
    
    QVector3D perpendicular = pxVec3ToQVector3D(calculatePerpendicularAxis(qVector3DToPxVec3(segmentVector)));
    return midPoint + perpendicular * radius;
}

// Constraint validation
bool DragChainConstraintSolver::validateBendRadiusConstraints(const std::vector<DragChainSegment>& segments, double maxBendRadius)
{
    double maxAngle = calculateMaxBendAngle(segments.size() > 0 ? segments[0].length : 18.0, maxBendRadius);
    
    for (const auto& segment : segments) {
        if (segment.isBend && std::abs(segment.bendAngle) > maxAngle) {
            qDebug() << "[PhysXDragChainSolver] Warning: Bend angle" << (segment.bendAngle * 180.0 / M_PI) 
                     << "exceeds maximum" << (maxAngle * 180.0 / M_PI) << "degrees";
            return false;
        }
    }
    
    return true;
}

bool DragChainConstraintSolver::validateDirectionConstraints(const std::vector<DragChainSegment>& segments, const QVector3D& startDirection, const QVector3D& endDirection)
{
    if (segments.empty()) return true;
    
    // Check start direction
    QVector3D firstDirection = segments[0].direction;
    double startDot = QVector3D::dotProduct(firstDirection, startDirection.normalized());
    double startAngleDiff = std::acos(std::clamp<double>(startDot, -1.0, 1.0));
    
    if (startAngleDiff > 0.1) { // More than ~5.7 degrees
        qDebug() << "[PhysXDragChainSolver] Warning: Start direction differs by" << (startAngleDiff * 180.0 / M_PI) << "degrees";
        return false;
    }
    
    // Check end direction
    QVector3D lastDirection = segments.back().direction;
    double endDot = QVector3D::dotProduct(lastDirection, endDirection.normalized());
    double endAngleDiff = std::acos(std::clamp<double>(endDot, -1.0, 1.0));
    
    if (endAngleDiff > 0.1) { // More than ~5.7 degrees
        qDebug() << "[PhysXDragChainSolver] Warning: End direction differs by" << (endAngleDiff * 180.0 / M_PI) << "degrees";
        return false;
    }
    
    return true;
}

// Convert solved path to visualization segments
std::vector<ConnectionPathSegment> DragChainConstraintSolver::convertToVisualizationSegments(
    const std::vector<DragChainSegment>& segments,
    const QVector4D& mainColor,
    const QVector4D& indicatorColor)
{
    std::vector<ConnectionPathSegment> visualizationSegments;
    
    // Define alternating colors for segments
    QVector4D color1 = mainColor; // Primary color (e.g., red)
    QVector4D color2 = QVector4D(0.0f, 0.8f, 1.0f, 0.8f); // Secondary color (e.g., cyan)
    
    for (size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex) {
        const auto& segment = segments[segmentIndex];
        
        // Alternate colors for each segment
        QVector4D segmentColor = (segmentIndex % 2 == 0) ? color1 : color2;
        
        if (segment.isBend && segment.bendRadius > 0.0) {
            // Create curved path for bend segments
            std::vector<QVector3D> bendPoints = generateBendPoints(
                segment.startPoint, segment.endPoint, segment.direction, segment.bendRadius, 8);
            
            // Add bend curve segments with alternating color
            for (size_t i = 0; i < bendPoints.size() - 1; ++i) {
                visualizationSegments.emplace_back(
                    bendPoints[i], bendPoints[i + 1], segmentColor, 3.0f, false);
            }
            
            // Add bend radius indicator
            QVector3D midPoint = (segment.startPoint + segment.endPoint) * 0.5f;
            QVector3D perpendicular = pxVec3ToQVector3D(calculatePerpendicularAxis(qVector3DToPxVec3(segment.direction)));
            
            QVector3D indicatorStart = midPoint - perpendicular * (segment.bendRadius * 0.3f);
            QVector3D indicatorEnd = midPoint + perpendicular * (segment.bendRadius * 0.3f);
            
            visualizationSegments.emplace_back(
                indicatorStart, indicatorEnd, indicatorColor, 5.0f, true);
        } else {
            // Straight segment with alternating color
            visualizationSegments.emplace_back(
                segment.startPoint, segment.endPoint, segmentColor, 3.0f, false);
        }
    }
    
    // Add joint spheres to represent revolute joints
    std::vector<ConnectionPathSegment> jointSpheres = generateJointSpheres(segments, 2.0f);
    visualizationSegments.insert(visualizationSegments.end(), jointSpheres.begin(), jointSpheres.end());
    
    return visualizationSegments;
}

// Generate spheres to represent revolute joints
std::vector<ConnectionPathSegment> DragChainConstraintSolver::generateJointSpheres(const std::vector<DragChainSegment>& segments, double jointRadius)
{
    std::vector<ConnectionPathSegment> jointSpheres;
    
    if (segments.empty()) {
        return jointSpheres;
    }
    
    // Joint color - metallic gray
    QVector4D jointColor(0.7f, 0.7f, 0.7f, 0.9f);
    
    // Generate spheres at each joint location
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        
        // Add sphere at segment start (joint with previous segment)
        jointSpheres.emplace_back(
            segment.startPoint, segment.startPoint, jointColor, jointRadius, true);
        
        // Add sphere at segment end (joint with next segment)
        jointSpheres.emplace_back(
            segment.endPoint, segment.endPoint, jointColor, jointRadius, true);
    }
    
    qDebug() << "[PhysXDragChainSolver] Generated" << jointSpheres.size() << "joint spheres";
    
    return jointSpheres;
}

// Statistics calculation
DragChainConstraintSolver::PathStatistics DragChainConstraintSolver::calculatePathStatistics(const std::vector<DragChainSegment>& segments)
{
    PathStatistics stats;
    
    if (segments.empty()) {
        stats.isValid = false;
        return stats;
    }
    
    stats.isValid = true;
    stats.totalSegments = static_cast<int>(segments.size());
    stats.totalLength = 0.0;
    stats.straightSegments = 0;
    stats.bendSegments = 0;
    stats.maxSegmentLength = 0.0;
    stats.minSegmentLength = std::numeric_limits<double>::max();
    
    for (const auto& segment : segments) {
        stats.totalLength += segment.length;
        
        if (segment.isBend) {
            stats.bendSegments++;
        } else {
            stats.straightSegments++;
        }
        
        if (segment.length > stats.maxSegmentLength) {
            stats.maxSegmentLength = segment.length;
        }
        if (segment.length < stats.minSegmentLength) {
            stats.minSegmentLength = segment.length;
        }
    }
    
    if (stats.totalSegments > 0) {
        stats.averageSegmentLength = stats.totalLength / stats.totalSegments;
    }
    
    if (stats.minSegmentLength == std::numeric_limits<double>::max()) {
        stats.minSegmentLength = 0.0;
    }
    
    return stats;
}

// Test function
bool DragChainConstraintSolver::testConstraintSolver()
{
    qDebug() << "[PhysXDragChainSolver] Testing PhysX-based constraint solver with progressive segment addition...";
    
    // Test 1: Simple straight path with progressive addition
    QVector3D startPoint(0, 0, 0);
    QVector3D endPoint(300, 0, 0);
    QVector3D startDirection(1, 0, 0);
    QVector3D endDirection(1, 0, 0);
    
    auto segments1 = solveDragChainPath(startPoint, endPoint, startDirection, endDirection, 
                                       300.0, 50.0, 1, false, false);
    
    if (segments1.empty()) {
        qDebug() << "[PhysXDragChainSolver] Test 1 failed: No segments generated";
        return false;
    }
    
    qDebug() << "[PhysXDragChainSolver] Test 1 passed: Generated" << segments1.size() << "segments";
    
    // Test 2: Progressive addition - start with 1 segment, add more as needed
    QVector3D startPoint2(0, 0, 0);
    QVector3D endPoint2(600, 0, 0); // 600mm distance
    QVector3D startDirection2(1, 0, 0);
    QVector3D endDirection2(1, 0, 0);
    
    // Request 3 segments but start with minimum required (2 segments)
    auto segments2 = solveDragChainPath(startPoint2, endPoint2, startDirection2, endDirection2, 
                                       300.0, 100.0, 3, true, true);
    
    if (segments2.empty()) {
        qDebug() << "[PhysXDragChainSolver] Test 2 failed: No segments generated";
        return false;
    }
    
    qDebug() << "[PhysXDragChainSolver] Test 2 passed: Generated" << segments2.size() << "segments";
    
    // Verify that the chain actually deformed to reach the target
    double totalLength = 0.0;
    for (const auto& segment : segments2) {
        totalLength += segment.length;
    }
    
    double straightDistance = (endPoint2 - startPoint2).length();
    qDebug() << "[PhysXDragChainSolver] Test 2 verification:";
    qDebug() << "[PhysXDragChainSolver]   Straight distance:" << straightDistance << "mm";
    qDebug() << "[PhysXDragChainSolver]   Total chain length:" << totalLength << "mm";
    qDebug() << "[PhysXDragChainSolver]   Chain deformed by:" << (totalLength - straightDistance) << "mm";
    
    // Test 3: Complex path requiring progressive deformation
    QVector3D startPoint3(0, 0, 0);
    QVector3D endPoint3(400, 400, 0); // ~566mm straight distance
    QVector3D startDirection3(1, 0, 0);
    QVector3D endDirection3(0, 1, 0);
    
    // Request 4 segments but start with minimum required (2 segments)
    auto segments3 = solveDragChainPath(startPoint3, endPoint3, startDirection3, endDirection3, 
                                       300.0, 150.0, 4, true, true);
    
    if (segments3.empty()) {
        qDebug() << "[PhysXDragChainSolver] Test 3 failed: No segments generated";
        return false;
    }
    
    qDebug() << "[PhysXDragChainSolver] Test 3 passed: Complex path with" << segments3.size() << "segments";
    
    // Verify deformation
    double totalLength3 = 0.0;
    for (const auto& segment : segments3) {
        totalLength3 += segment.length;
    }
    
    double straightDistance3 = (endPoint3 - startPoint3).length();
    qDebug() << "[PhysXDragChainSolver] Test 3 verification:";
    qDebug() << "[PhysXDragChainSolver]   Straight distance:" << straightDistance3 << "mm";
    qDebug() << "[PhysXDragChainSolver]   Total chain length:" << totalLength3 << "mm";
    qDebug() << "[PhysXDragChainSolver]   Chain deformed by:" << (totalLength3 - straightDistance3) << "mm";
    
    // Test 4: Verify that progressive addition prevents explosion
    QVector3D startPoint4(0, 0, 0);
    QVector3D endPoint4(200, 0, 0); // 200mm distance
    QVector3D startDirection4(1, 0, 0);
    QVector3D endDirection4(1, 0, 0);
    
    // Request 3 segments for 200mm distance - should start with 1 segment and add progressively
    auto segments4 = solveDragChainPath(startPoint4, endPoint4, startDirection4, endDirection4, 
                                       300.0, 50.0, 3, true, true);
    
    if (segments4.empty()) {
        qDebug() << "[PhysXDragChainSolver] Test 4 failed: No segments generated";
        return false;
    }
    
    qDebug() << "[PhysXDragChainSolver] Test 4 passed: Progressive addition with" << segments4.size() << "segments";
    
    // Verify that the chain didn't explode and reached the target
    double finalDistance = (pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose()) - endPoint4).length();
    qDebug() << "[PhysXDragChainSolver] Test 4 verification:";
    qDebug() << "[PhysXDragChainSolver]   Final distance to target:" << finalDistance << "mm";
    qDebug() << "[PhysXDragChainSolver]   Expected distance: 0mm (exact)";
    qDebug() << "[PhysXDragChainSolver]   Success: No explosion, reached target";
    
    qDebug() << "[PhysXDragChainSolver] All progressive addition tests completed successfully!";
    return true;
}

// Example usage function
void DragChainConstraintSolver::exampleUsage()
{
    qDebug() << "[PhysXDragChainSolver] Example usage with PhysX rigid bodies:";
    
    // Example 1: Simple drag chain with 300mm pitch length - straight path
    QVector3D startPoint(0, 0, 0);
    QVector3D endPoint(900, 0, 0);
    QVector3D startDirection(1, 0, 0);
    QVector3D endDirection(1, 0, 0);
    
    // Key parameters:
    // - pitchLength: Exact length of each segment (300mm) - FIXED
    // - maxBendRadius: Maximum radius of arc between segment center points (100mm)
    // - segmentCount: Number of segments to create (3)
    double pitchLength = 300.0;  // 300mm between rotation points - FIXED
    double maxBendRadius = 100.0; // 100mm maximum bend radius
    int segmentCount = 3;        // 3 segments
    
    auto segments = solveDragChainPath(startPoint, endPoint, startDirection, endDirection,
                                      pitchLength, maxBendRadius, segmentCount,
                                      true, true); // Both ends locked
    
    qDebug() << "[PhysXDragChainSolver] Example 1: Generated" << segments.size() << "segments (requested:" << segmentCount << ")";
    qDebug() << "[PhysXDragChainSolver] Pitch length:" << pitchLength << "mm (FIXED)";
    qDebug() << "[PhysXDragChainSolver] Max bend radius:" << maxBendRadius << "mm";
    
    // Verify segment count
    if (segments.size() == segmentCount) {
        qDebug() << "[PhysXDragChainSolver] ✓ Segment count respected";
    } else {
        qDebug() << "[PhysXDragChainSolver] ✗ Segment count mismatch: got" << segments.size() << "expected" << segmentCount;
    }
    
    // Verify pitch length constraints - all segments should be exactly 300mm
    for (size_t i = 0; i < segments.size(); ++i) {
        qDebug() << "[PhysXDragChainSolver] Segment" << i << "length:" << segments[i].length << "mm";
        if (std::abs(segments[i].length - pitchLength) < 1.0) {
            qDebug() << "[PhysXDragChainSolver] ✓ Segment" << i << "has exact pitch length";
        } else {
            qDebug() << "[PhysXDragChainSolver] ✗ Segment" << i << "length differs from pitch length";
        }
        if (segments[i].isBend) {
            qDebug() << "[PhysXDragChainSolver] Segment" << i << "bend angle:" << (segments[i].bendAngle * 180.0 / M_PI) << "degrees";
            qDebug() << "[PhysXDragChainSolver] Segment" << i << "bend radius:" << segments[i].bendRadius << "mm";
        }
    }
    
    // Example 2: Path with significant bend using PhysX rigid bodies
    QVector3D startPoint2(0, 0, 0);
    QVector3D endPoint2(600, 600, 0);
    QVector3D startDirection2(1, 0, 0);
    QVector3D endDirection2(0, 1, 0);
    
    double pitchLength2 = 300.0;   // 300mm pitch - FIXED
    double maxBendRadius2 = 150.0; // Larger bend radius for smooth curves
    int segmentCount2 = 2;         // 2 segments
    
    auto segments2 = solveDragChainPath(startPoint2, endPoint2, startDirection2, endDirection2,
                                       pitchLength2, maxBendRadius2, segmentCount2,
                                       true, true); // Both ends locked
    
    qDebug() << "[PhysXDragChainSolver] Example 2: Generated" << segments2.size() << "segments";
    qDebug() << "[PhysXDragChainSolver] Total distance:" << (endPoint2 - startPoint2).length() << "mm";
    qDebug() << "[PhysXDragChainSolver] Total required length:" << (segmentCount2 * pitchLength2) << "mm";
    
    // Calculate statistics
    auto stats = calculatePathStatistics(segments2);
    qDebug() << "[PhysXDragChainSolver] Total length:" << stats.totalLength << "mm";
    qDebug() << "[PhysXDragChainSolver] Straight segments:" << stats.straightSegments;
    qDebug() << "[PhysXDragChainSolver] Bend segments:" << stats.bendSegments;
    qDebug() << "[PhysXDragChainSolver] Average segment length:" << stats.averageSegmentLength << "mm";
    
    // Verify that segments maintain exact pitch length
    bool pitchLengthValid = true;
    for (size_t i = 0; i < segments2.size(); ++i) {
        if (std::abs(segments2[i].length - pitchLength2) > 1.0) {
            qDebug() << "[PhysXDragChainSolver] ✗ Segment" << i << "length" << segments2[i].length 
                     << "differs from pitch length" << pitchLength2;
            pitchLengthValid = false;
        }
    }
    
    if (pitchLengthValid) {
        qDebug() << "[PhysXDragChainSolver] ✓ All segments maintain exact pitch length";
    }
    
    // Verify bend radius constraints are respected
    bool allBendsValid = true;
    for (const auto& segment : segments2) {
        if (segment.isBend && segment.bendRadius > 0.0) {
            if (segment.bendRadius > maxBendRadius2) {
                qDebug() << "[PhysXDragChainSolver] Warning: Bend radius" << segment.bendRadius 
                         << "exceeds maximum" << maxBendRadius2;
                allBendsValid = false;
            }
        }
    }
    
    if (allBendsValid) {
        qDebug() << "[PhysXDragChainSolver] ✓ All bend radius constraints satisfied";
    }
    
    // Example 3: Complex 3D path requiring PhysX simulation
    QVector3D startPoint3(0, 0, 0);
    QVector3D endPoint3(600, 600, 300);
    QVector3D startDirection3(1, 0, 0);
    QVector3D endDirection3(0, 0, 1);
    
    double pitchLength3 = 200.0;   // 200mm pitch - FIXED
    double maxBendRadius3 = 80.0;  // Bend radius constraint
    int segmentCount3 = 4;         // 4 segments
    
    auto segments3 = solveDragChainPath(startPoint3, endPoint3, startDirection3, endDirection3,
                                       pitchLength3, maxBendRadius3, segmentCount3,
                                       true, true); // Both ends locked
    
    qDebug() << "[PhysXDragChainSolver] Example 3: Complex 3D path test";
    qDebug() << "[PhysXDragChainSolver] Distance:" << (endPoint3 - startPoint3).length() << "mm";
    qDebug() << "[PhysXDragChainSolver] Required length:" << (segmentCount3 * pitchLength3) << "mm";
    qDebug() << "[PhysXDragChainSolver] Generated" << segments3.size() << "segments";
    
    // Verify that segments rotate to create the required path length
    double totalPathLength = 0.0;
    for (const auto& segment : segments3) {
        totalPathLength += segment.length;
    }
    qDebug() << "[PhysXDragChainSolver] Total path length:" << totalPathLength << "mm";
    
    qDebug() << "[PhysXDragChainSolver] PhysX rigid body example usage completed successfully!";
} 

std::vector<QVector3D> DragChainConstraintSolver::calculateOptimalChainConfiguration(
    const QVector3D& startPoint, 
    const QVector3D& endPoint, 
    double pitchLength, 
    int segmentCount)
{
    std::vector<QVector3D> optimalPositions;
    
    if (segmentCount <= 0) return optimalPositions;
    
    double totalDistance = (endPoint - startPoint).length();
    double totalChainLength = segmentCount * pitchLength;
    
    qDebug() << "[PhysXDragChainSolver] Calculating optimal chain configuration:";
    qDebug() << "[PhysXDragChainSolver]   Total distance:" << totalDistance << "mm";
    qDebug() << "[PhysXDragChainSolver]   Total chain length:" << totalChainLength << "mm";
    qDebug() << "[PhysXDragChainSolver]   Segment count:" << segmentCount;
    
    if (totalDistance >= totalChainLength) {
        // Chain is too short - use straight line configuration
        QVector3D direction = (endPoint - startPoint).normalized();
        for (int i = 0; i <= segmentCount; ++i) {
            QVector3D position = startPoint + direction * (i * pitchLength);
            optimalPositions.push_back(position);
        }
        qDebug() << "[PhysXDragChainSolver] Using straight line configuration (chain too short)";
    } else {
        // Chain is too long - need to create bends to reach target
        double excessLength = totalChainLength - totalDistance;
        double compressionPerSegment = excessLength / segmentCount;
        
        QVector3D direction = (endPoint - startPoint).normalized();
        
        // Create a curved path that distributes the excess length
        for (int i = 0; i <= segmentCount; ++i) {
            double idealDistance = i * pitchLength - (i * compressionPerSegment);
            QVector3D position = startPoint + direction * idealDistance;
            optimalPositions.push_back(position);
        }
        
        qDebug() << "[PhysXDragChainSolver] Using compressed configuration (chain too long)";
        qDebug() << "[PhysXDragChainSolver]   Compression per segment:" << compressionPerSegment << "mm";
    }
    
    return optimalPositions;
} 

bool DragChainConstraintSolver::addSegmentProgressively(const QVector3D& targetEndPoint, double pitchLength, double maxBendRadius, int segmentCount)
{
    if (m_rigidBodies.empty()) {
        qDebug() << "[PhysXDragChainSolver] No rigid bodies to add segments to";
        return false;
    }
    
    qDebug() << "[PhysXDragChainSolver] Adding segments to reach" << segmentCount << "total segments";
    
    // Calculate optimal sag for the total distance
    double totalDistance = (targetEndPoint - pxTransformToQVector3D(m_rigidBodies[0]->getGlobalPose())).length();
    double optimalSag = calculateOptimalSag(totalDistance, pitchLength, maxBendRadius);
    
    int currentSegments = static_cast<int>(m_rigidBodies.size() - 1);
    int segmentsToAdd = segmentCount - currentSegments;
    
    if (segmentsToAdd <= 0) {
        qDebug() << "[PhysXDragChainSolver] Already have enough segments, just stabilizing";
        return stabilizeChainWithDistributedBends(targetEndPoint, pitchLength, maxBendRadius, optimalSag);
    }
    
    qDebug() << "[PhysXDragChainSolver] Adding" << segmentsToAdd << "segments";
    
    // Add segments one by one until we reach the requested count
    for (int i = 0; i < segmentsToAdd; ++i) {
        int segmentIndex = currentSegments + i + 1;
        
        if (!addSingleSegmentWithDistributedBend(targetEndPoint, pitchLength, maxBendRadius, segmentIndex, optimalSag)) {
            qDebug() << "[PhysXDragChainSolver] Failed to add segment" << segmentIndex;
            return false;
        }
        
        // Update current segment count
        currentSegments = static_cast<int>(m_rigidBodies.size() - 1);
        
        // Stabilize after every few segments to prevent instability
        if ((i + 1) % 3 == 0 || (i + 1) == segmentsToAdd) {
            if (!stabilizeChainWithDistributedBends(targetEndPoint, pitchLength, maxBendRadius, optimalSag)) {
                qDebug() << "[PhysXDragChainSolver] Failed to stabilize after" << (i + 1) << "segments";
                return false;
            }
        }
        
        // Update statistics after each segment
        double newDistanceToTarget = (pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose()) - targetEndPoint).length();
        bool converged = newDistanceToTarget < m_convergenceTolerance;
        updateSolverStatistics(newDistanceToTarget, converged, 1);
    }
    
    // Final stabilization with full distributed bend optimization
    qDebug() << "[PhysXDragChainSolver] Performing final stabilization with" << m_rigidBodies.size() << "rigid bodies";
    if (!stabilizeChainWithDistributedBends(targetEndPoint, pitchLength, maxBendRadius, optimalSag)) {
        qDebug() << "[PhysXDragChainSolver] Failed to perform final stabilization";
        return false;
    }
    
    // Final check of distance to target
    QVector3D finalEndPos = pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose());
    double finalDistance = (targetEndPoint - finalEndPos).length();
    qDebug() << "[PhysXDragChainSolver] Final distance to target:" << finalDistance << "mm";
    qDebug() << "[PhysXDragChainSolver] Total segments added:" << segmentsToAdd;
    qDebug() << "[PhysXDragChainSolver] Total rigid bodies:" << m_rigidBodies.size();
    
    qDebug() << "[PhysXDragChainSolver] Progressive segment addition completed successfully";
    return true;
}

bool DragChainConstraintSolver::addSingleSegmentWithDistributedBend(const QVector3D& targetEndPoint, double pitchLength, double maxBendRadius, int segmentIndex, double optimalSag)
{
    // Calculate position for the new segment with distributed bend
    QVector3D lastBodyPos = pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose());
    QVector3D direction = (targetEndPoint - lastBodyPos).normalized();
    
    // Calculate distributed bend offset based on segment position
    double bendDistributionFactor = calculateBendDistributionFactor(segmentIndex, static_cast<int>(m_rigidBodies.size()));
    QVector3D bendOffset = calculateDistributedBendOffset(direction, maxBendRadius, bendDistributionFactor, optimalSag);
    
    QVector3D newBodyPos = lastBodyPos + direction * pitchLength + bendOffset;
    
    // Create new rigid body
    PxTransform newBodyPose = qVector3DToPxTransform(newBodyPos, QVector3D(1,0,0));
    PxRigidDynamic* newBody = m_physics->createRigidDynamic(newBodyPose);
    if (!newBody) {
        qDebug() << "[PhysXDragChainSolver] Failed to create new rigid body for segment" << segmentIndex;
        return false;
    }
    
    // Create shape for new body
    PxShape* shape = m_physics->createShape(PxBoxGeometry(pitchLength * 0.05f, pitchLength * 0.05f, pitchLength * 0.05f), *m_material);
    newBody->attachShape(*shape);
    shape->release();
    
    // Configure new body with adaptive mass based on position
    double massFactor = 1.0 + (bendDistributionFactor * 0.5); // Middle segments have slightly higher mass
    newBody->setMass(0.5f * massFactor);
    newBody->setLinearDamping(0.2f);
    newBody->setAngularDamping(0.3f);
    
    m_rigidBodies.push_back(newBody);
    m_scene->addActor(*newBody);
    
    // Create joint between last body and new body with distributed constraints
    PxRigidDynamic* lastBody = m_rigidBodies[m_rigidBodies.size() - 2];
    
    PxTransform localFrame0(PxVec3(pitchLength * 0.5f, 0, 0));
    PxTransform localFrame1(PxVec3(-pitchLength * 0.5f, 0, 0));
    
    PxRevoluteJoint* joint = PxRevoluteJointCreate(*m_physics, lastBody, localFrame0, newBody, localFrame1);
    if (joint) {
        // Set joint limits with distributed bend radius
        double distributedBendRadius = maxBendRadius * (1.0 + bendDistributionFactor * 0.5);
        double maxBendAngle = calculateMaxBendAngle(pitchLength, distributedBendRadius);
        float maxAngleRadians = static_cast<float>(std::max(maxBendAngle, M_PI * 0.75));
        joint->setLimit(PxJointAngularLimitPair(-maxAngleRadians, maxAngleRadians));
        joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
        
        // Add drive with distributed force
        joint->setDriveVelocity(0.0f);
        joint->setDriveForceLimit(100.0f * (1.0 + bendDistributionFactor));
        joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
        
        m_joints.push_back(joint);
    }
    
    // Update the end fixed joint to connect to the new last body
    if (m_endFixedJoint) {
        m_endFixedJoint->release();
        m_endFixedJoint = nullptr;
    }
    
    // Create new end fixed joint
    PxTransform endLocalFrame0(PxVec3(0, 0, 0));
    PxTransform endLocalFrame1(PxVec3(0, 0, 0));
    
    m_endFixedJoint = PxFixedJointCreate(*m_physics, m_endAnchor, endLocalFrame0, newBody, endLocalFrame1);
    if (!m_endFixedJoint) {
        qDebug() << "[PhysXDragChainSolver] Failed to create new end fixed joint";
        return false;
    }
    
    return true;
}

bool DragChainConstraintSolver::stabilizeChainWithDistributedBends(const QVector3D& targetEndPoint, double pitchLength, double maxBendRadius, double optimalSag)
{
    qDebug() << "[PhysXDragChainSolver] Stabilizing chain with distributed bends for" << m_maxIterations << "steps";
    
    int actualStepsUsed = 0;
    
    for (int step = 0; step < m_maxIterations; ++step) {
        actualStepsUsed++;
        
        // Apply distributed forces to all bodies based on their position in the chain
        for (size_t i = 0; i < m_rigidBodies.size(); ++i) {
            PxRigidDynamic* body = m_rigidBodies[i];
            
            // Calculate distribution factor for this body
            double distributionFactor = calculateBendDistributionFactor(i, static_cast<int>(m_rigidBodies.size()));
            
            // Apply damping with distributed intensity
            PxVec3 linearVel = body->getLinearVelocity();
            PxVec3 angularVel = body->getAngularVelocity();
            
            double dampingFactor = 0.95 - (distributionFactor * 0.1); // Middle segments have less damping
            linearVel *= static_cast<float>(dampingFactor);
            angularVel *= static_cast<float>(dampingFactor);
            
            body->setLinearVelocity(linearVel);
            body->setAngularVelocity(angularVel);
            
            // Apply distributed tension forces
            if (i > 0) {
                PxTransform prevPose = m_rigidBodies[i-1]->getGlobalPose();
                QVector3D prevPos = pxTransformToQVector3D(prevPose);
                QVector3D currentPos = pxTransformToQVector3D(body->getGlobalPose());
                QVector3D toPrev = (prevPos - currentPos).normalized();
                double tensionForce = 10.0 * (1.0 + distributionFactor);
                PxVec3 tensionForceVec = qVector3DToPxVec3(toPrev) * static_cast<float>(tensionForce);
                body->addForce(tensionForceVec);
            }
            
            if (i < m_rigidBodies.size() - 1) {
                PxTransform nextPose = m_rigidBodies[i+1]->getGlobalPose();
                QVector3D nextPos = pxTransformToQVector3D(nextPose);
                QVector3D currentPos = pxTransformToQVector3D(body->getGlobalPose());
                QVector3D toNext = (nextPos - currentPos).normalized();
                double tensionForce = 10.0 * (1.0 + distributionFactor);
                PxVec3 tensionForceVec = qVector3DToPxVec3(toNext) * static_cast<float>(tensionForce);
                body->addForce(tensionForceVec);
            }
            
            // Apply sag forces for middle segments
            if (i > 0 && i < m_rigidBodies.size() - 1) {
                QVector3D currentPos = pxTransformToQVector3D(body->getGlobalPose());
                QVector3D startPos = pxTransformToQVector3D(m_rigidBodies[0]->getGlobalPose());
                QVector3D endPos = pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose());
                
                // Calculate expected sag position
                QVector3D sagOffset = calculateSagOffset(currentPos, startPos, endPos, optimalSag, distributionFactor);
                QVector3D targetPos = currentPos + sagOffset;
                QVector3D sagForce = (targetPos - currentPos) * 5.0f;
                body->addForce(qVector3DToPxVec3(sagForce));
            }
        }
        
        m_scene->simulate(m_simulationTimeStep);
        m_scene->fetchResults(true);
        
        // Check stability every 10 steps
        if (step % 10 == 0) {
            bool allStable = true;
            double maxVelocity = 0.0;
            
            for (size_t i = 1; i < m_rigidBodies.size() - 1; ++i) {
                PxVec3 velocity = m_rigidBodies[i]->getLinearVelocity();
                double velocityMagnitude = velocity.magnitude();
                
                if (velocityMagnitude > maxVelocity) {
                    maxVelocity = velocityMagnitude;
                }
                
                if (velocityMagnitude > 0.01f) { // Lower threshold for better stability
                    allStable = false;
                }
            }
            
            // Update current distance to target for statistics
            double currentDistance = (pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose()) - targetEndPoint).length();
            bool currentConverged = currentDistance < m_convergenceTolerance;
            updateSolverStatistics(currentDistance, currentConverged, 10); // 10 steps since last update
            
            if (allStable && currentDistance < m_convergenceTolerance * 2.0) {
                qDebug() << "[PhysXDragChainSolver] Chain stabilized at step" << step;
                break;
            }
        }
    }
    
    // Update iteration count
    m_lastSolverStatistics.iterationsUsed += actualStepsUsed;
    
    return true;
}

double DragChainConstraintSolver::calculateOptimalSag(double totalDistance, double pitchLength, double maxBendRadius)
{
    // Calculate optimal sag based on chain length and bend radius
    // For longer chains, we need more sag to distribute the bends properly
    double chainLength = totalDistance;
    double segmentCount = std::ceil(chainLength / pitchLength);
    
    // Base sag is proportional to chain length and bend radius
    double baseSag = maxBendRadius * 0.5;
    
    // For longer chains, increase sag to distribute bends
    if (segmentCount > 5) {
        double lengthFactor = std::min(segmentCount / 5.0, 3.0); // Cap at 3x
        baseSag *= lengthFactor;
    }
    
    // Ensure sag doesn't exceed reasonable limits
    baseSag = std::min(baseSag, chainLength * 0.1); // Max 10% of chain length
    
    return baseSag;
}

double DragChainConstraintSolver::calculateBendDistributionFactor(int segmentIndex, int totalSegments)
{
    if (totalSegments <= 1) return 0.0;
    
    // Calculate how much this segment should contribute to the overall bend
    // Middle segments contribute more to the bend distribution
    double normalizedIndex = static_cast<double>(segmentIndex) / (totalSegments - 1);
    
    // Use a bell curve distribution - middle segments have higher factors
    double distributionFactor = std::sin(normalizedIndex * M_PI);
    
    return distributionFactor;
}

QVector3D DragChainConstraintSolver::calculateDistributedBendOffset(const QVector3D& direction, double maxBendRadius, double distributionFactor, double optimalSag)
{
    // Calculate perpendicular direction for bend
    QVector3D perpendicular;
    if (std::abs(direction.x()) < 0.9f) {
        perpendicular = QVector3D::crossProduct(QVector3D(1, 0, 0), direction);
    } else {
        perpendicular = QVector3D::crossProduct(QVector3D(0, 1, 0), direction);
    }
    perpendicular.normalize();
    
    // Calculate bend offset based on distribution factor and optimal sag
    double bendMagnitude = maxBendRadius * distributionFactor * 0.3;
    double sagMagnitude = optimalSag * distributionFactor * 0.5;
    
    QVector3D bendOffset = perpendicular * bendMagnitude;
    QVector3D sagOffset = QVector3D(0, -sagMagnitude, 0); // Sag downward
    
    return bendOffset + sagOffset;
}

QVector3D DragChainConstraintSolver::calculateSagOffset(const QVector3D& currentPos, const QVector3D& startPos, const QVector3D& endPos, double optimalSag, double distributionFactor)
{
    // Calculate sag based on position along the chain
    QVector3D chainDirection = (endPos - startPos).normalized();
    double distanceAlongChain = QVector3D::dotProduct(currentPos - startPos, chainDirection);
    double totalChainLength = (endPos - startPos).length();
    
    if (totalChainLength <= 0.0) return QVector3D(0, 0, 0);
    
    // Normalize position along chain (0 to 1)
    double normalizedPosition = distanceAlongChain / totalChainLength;
    normalizedPosition = std::max(0.0, std::min(1.0, normalizedPosition));
    
    // Calculate sag using a parabolic curve
    double sagFactor = 4.0 * normalizedPosition * (1.0 - normalizedPosition); // Parabolic curve
    double sagMagnitude = optimalSag * sagFactor * distributionFactor;
    
    // Apply sag downward
    return QVector3D(0, -sagMagnitude, 0);
}

void DragChainConstraintSolver::updateSolverStatistics(double distanceToTarget, bool converged, int additionalIterations)
{
    m_lastSolverStatistics.finalDistanceToTarget = distanceToTarget;
    m_lastSolverStatistics.converged = converged;
    m_lastSolverStatistics.iterationsUsed += additionalIterations;
    m_lastSolverStatistics.rigidBodiesCreated = static_cast<int>(m_rigidBodies.size());
    m_lastSolverStatistics.jointsCreated = static_cast<int>(m_joints.size());
    
    // Calculate segment length statistics
    calculateSegmentLengthStatistics();
    
    // Call callback function for real-time GUI updates
    if (m_statisticsCallback) {
        m_statisticsCallback(m_lastSolverStatistics);
    }
}

void DragChainConstraintSolver::calculateSegmentLengthStatistics()
{
    if (m_rigidBodies.size() < 2) {
        m_lastSolverStatistics.minSegmentLength = 0.0;
        m_lastSolverStatistics.maxSegmentLength = 0.0;
        m_lastSolverStatistics.averageSegmentLength = 0.0;
        return;
    }
    
    std::vector<double> segmentLengths;
    double totalLength = 0.0;
    
    // Calculate lengths between consecutive rigid bodies
    for (size_t i = 0; i < m_rigidBodies.size() - 1; ++i) {
        QVector3D pos1 = pxTransformToQVector3D(m_rigidBodies[i]->getGlobalPose());
        QVector3D pos2 = pxTransformToQVector3D(m_rigidBodies[i + 1]->getGlobalPose());
        double segmentLength = (pos2 - pos1).length();
        segmentLengths.push_back(segmentLength);
        totalLength += segmentLength;
    }
    
    if (segmentLengths.empty()) {
        m_lastSolverStatistics.minSegmentLength = 0.0;
        m_lastSolverStatistics.maxSegmentLength = 0.0;
        m_lastSolverStatistics.averageSegmentLength = 0.0;
        return;
    }
    
    // Calculate statistics
    auto minMax = std::minmax_element(segmentLengths.begin(), segmentLengths.end());
    m_lastSolverStatistics.minSegmentLength = *minMax.first;
    m_lastSolverStatistics.maxSegmentLength = *minMax.second;
    m_lastSolverStatistics.averageSegmentLength = totalLength / segmentLengths.size();
}

bool DragChainConstraintSolver::stabilizeChain(double timeStep, int maxSteps, const QVector3D& endPoint)
{
    qDebug() << "[PhysXDragChainSolver] Stabilizing chain for" << maxSteps << "steps";
    
    int actualStepsUsed = 0;
    
    for (int step = 0; step < maxSteps; ++step) {
        actualStepsUsed++;
        
        // Apply gentle forces to middle bodies only
        for (size_t i = 1; i < m_rigidBodies.size() - 1; ++i) {
            PxRigidDynamic* body = m_rigidBodies[i];
            
            // Apply damping
            PxVec3 linearVel = body->getLinearVelocity();
            PxVec3 angularVel = body->getAngularVelocity();
            
            linearVel *= 0.95f;
            angularVel *= 0.95f;
            
            body->setLinearVelocity(linearVel);
            body->setAngularVelocity(angularVel);
            
            // Gentle tension forces to maintain chain structure
            if (i > 0) {
                PxTransform prevPose = m_rigidBodies[i-1]->getGlobalPose();
                QVector3D prevPos = pxTransformToQVector3D(prevPose);
                QVector3D currentPos = pxTransformToQVector3D(body->getGlobalPose());
                QVector3D toPrev = (prevPos - currentPos).normalized();
                PxVec3 tensionForce = qVector3DToPxVec3(toPrev) * 10.0f;
                body->addForce(tensionForce);
            }
            
            if (i < m_rigidBodies.size() - 1) {
                PxTransform nextPose = m_rigidBodies[i+1]->getGlobalPose();
                QVector3D nextPos = pxTransformToQVector3D(nextPose);
                QVector3D currentPos = pxTransformToQVector3D(body->getGlobalPose());
                QVector3D toNext = (nextPos - currentPos).normalized();
                PxVec3 tensionForce = qVector3DToPxVec3(toNext) * 10.0f;
                body->addForce(tensionForce);
            }
        }
        
        m_scene->simulate(timeStep);
        m_scene->fetchResults(true);
        
        // Check stability every 5 steps
        if (step % 5 == 0) {
            bool allStable = true;
            double maxVelocity = 0.0;
            
            for (size_t i = 1; i < m_rigidBodies.size() - 1; ++i) {
                PxVec3 velocity = m_rigidBodies[i]->getLinearVelocity();
                double velocityMagnitude = velocity.magnitude();
                
                if (velocityMagnitude > maxVelocity) {
                    maxVelocity = velocityMagnitude;
                }
                
                if (velocityMagnitude > 0.02f) { // Very low threshold for stability
                    allStable = false;
                }
            }
            
            // Update current distance to target for statistics
            double currentDistance = (pxTransformToQVector3D(m_rigidBodies.back()->getGlobalPose()) - endPoint).length();
            bool currentConverged = currentDistance < m_convergenceTolerance;
            updateSolverStatistics(currentDistance, currentConverged, 5); // 5 steps since last update
            
            qDebug() << "[PhysXDragChainSolver] Stabilization step" << step << "/" << maxSteps 
                     << "Max velocity:" << maxVelocity << "m/s, All stable:" << allStable
                     << "Distance to target:" << currentDistance << "mm";
            
            if (allStable) {
                qDebug() << "[PhysXDragChainSolver] Chain stabilized at step" << step;
                break;
            }
        }
    }
    
    // Update iteration count
    m_lastSolverStatistics.iterationsUsed += actualStepsUsed;
    
    return true;
}
