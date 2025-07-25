#ifndef IPHYSICSENGINE_H
#define IPHYSICSENGINE_H

#include <QObject>
#include <memory>
#include <map>
#include "PxPhysicsAPI.h"

#include "CadNode.h"

using namespace physx;

/**
 * @brief Configuration for soft body objects (cables, ropes, etc.)
 */
struct SoftBodyConfig {
    // Geometry parameters (simulation units)
    float radius = 5.0f;           // Cable radius in mm
    float length = 1000.0f;        // Cable length in mm
    int numSegments = 20;          // Number of segments for discretization
    
    // Geometry source
    bool useExistingGeometry = false;  // Use existing object geometry instead of creating default shape
    
    // Material properties - Rubber defaults (simulation units)
    float density = 0.0012f;       // Density in g/mm³ (0.0012 g/mm³ = 1,200 kg/m³ - typical rubber)
    float youngsModulus = 20.0f;    // Young's modulus in MPa (5 MPa - typical for soft rubber)
    float poissonsRatio = 0.49f;   // Poisson's ratio (0.49 for nearly incompressible rubber)
    
    // Simulation parameters - Moderate damping for rubber
    float damping = 0.005f;          // Damping coefficient (0-1, moderate for rubber)
    float friction = 0.01f;         // Friction coefficient (0-1, high for rubber)
    float restitution = 0.7f;      // Restitution coefficient (0-1, high bounce for rubber)
    
    // Constraint parameters
    bool fixStartPoint = true;      // Fix the start point of the cable
    bool fixEndPoint = false;       // Fix the end point of the cable
    float maxStretch = 1.1f;       // Maximum stretch factor (1.0 = no stretch)
    
    // Collision parameters
    bool enableSelfCollision = true;    // Enable self-collision
    bool enableCollisionWithRigid = true; // Enable collision with rigid bodies
    
    SoftBodyConfig() = default;
};

/**
 * @brief Simplified abstract interface for physics engines
 * 
 * This interface allows switching between different physics engines
 * (PhysX, Bullet, etc.) without changing the rest of the application.
 */
class IPhysicsEngine : public QObject
{
    Q_OBJECT

public:
    explicit IPhysicsEngine(QObject* parent = nullptr) : QObject(parent) {}
    virtual ~IPhysicsEngine() = default;

    // Core lifecycle
    virtual bool initialize() = 0;
    virtual void cleanup() = 0;
    
    // Object management
    /*
    virtual bool addObject(Item item) = 0;
    virtual bool removeObject(Item item) = 0;
    virtual bool isObjectInSimulation(Item item) const = 0;
    
    // Robot management
    virtual bool addRobot(Item robot) = 0;
    virtual bool removeRobot(Item robot) = 0;
    virtual bool isRobotInSimulation(Item robot) const = 0;
    virtual void updateRobotJoints(Item robot) = 0;
    
    // Robot interaction
    virtual bool convertToKinematic(Item item) = 0;
    virtual bool convertToDynamic(Item item) = 0;
    virtual bool isObjectGrabbedByRobot(Item item) const = 0;
    
    // Soft body management
    virtual bool addSoftBody(Item item, const SoftBodyConfig& config) = 0;
    virtual bool removeSoftBody(Item item) = 0;
    virtual bool isSoftBodyInSimulation(Item item) const = 0;
    virtual void updateSoftBodyFromRoboDK(Item item) = 0;
    virtual void updateRoboDKFromSoftBody(Item item) = 0;
    
    // Attachment-specific collision control
    virtual bool disableSoftBodyCollisionWithSurface(Item softBodyItem, Item attachmentSurfaceItem) = 0;
    virtual bool enableSoftBodyCollisionWithSurface(Item softBodyItem, Item attachmentSurfaceItem) = 0;
   
    // Pose synchronization
    virtual void updatePhysicsFromRoboDK(Item item) = 0;
    virtual void updateRoboDKFromPhysics(Item item) = 0;
    */
    
    // Simulation step (called every frame)
    virtual void stepSimulation(float deltaTime) = 0;
    
    // Scene configuration
    virtual void setGravity(const PxVec3& gravity) = 0;
    virtual PxVec3 getGravity() const = 0;
    
    // Material management
    virtual bool setObjectMaterial(CadNode* item, float staticFriction, float dynamicFriction, float restitution) = 0;
    
    // Default configuration methods for SceneConfigurationDialog
    virtual void setSolverIterations(int positionIterations, int velocityIterations) = 0;
    virtual void getSolverIterations(int& positionIterations, int& velocityIterations) const = 0;
    virtual void setGlobalMaterialProperties(float staticFriction, float dynamicFriction, float restitution) = 0;
    virtual void getGlobalMaterialProperties(float& staticFriction, float& dynamicFriction, float& restitution) const = 0;
    virtual void setPCMEnabled(bool enabled) = 0;
    virtual bool isPCMEnabled() const = 0;
    virtual void setStabilizationEnabled(bool enabled) = 0;
    virtual bool isStabilizationEnabled() const = 0;
    virtual void setContactOffset(float offset) = 0;
    virtual float getContactOffset() const = 0;
    virtual void setRestOffset(float offset) = 0;
    virtual float getRestOffset() const = 0;
    virtual void setSleepThreshold(float threshold) = 0;
    virtual float getSleepThreshold() const = 0;
    virtual void setStabilizationThreshold(float threshold) = 0;
    virtual float getStabilizationThreshold() const = 0;
    virtual void setCCDEnabled(bool enabled) = 0;
    virtual bool isCCDEnabled() const = 0;
    virtual void setWakeDistance(float distance) = 0;
    virtual float getWakeDistance() const = 0;
    
    // Debug and monitoring
    virtual int getActorCount() const = 0;
    virtual void enableDebugVisualization(bool enable) = 0;
    virtual bool isDebugVisualizationEnabled() const = 0;
    
    // Engine information
    virtual QString getEngineName() const = 0;
    virtual QString getEngineVersion() const = 0;
    
    // Object validation
    virtual void validateObjects() = 0;

signals:
    void objectAdded(CadNode* item);
    void objectRemoved(CadNode* item);
    void robotAdded(CadNode* robot);
    void robotRemoved(CadNode* robot);
};

#endif // IPHYSICSENGINE_H 
