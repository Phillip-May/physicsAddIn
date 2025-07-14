#ifndef ROBODKPHYSICSBRIDGE_H
#define ROBODKPHYSICSBRIDGE_H

#include <memory>
#include <map>
#include <QObject>
#include <QTimer>
#include "PxPhysicsAPI.h"

class RoboDK;
class Item;
class PhysicsEngine;
class CoordinateConverter;

/**
 * @brief Bridge between RoboDK and the physics engine
 * 
 * This class manages the synchronization between RoboDK objects
 * and their physics representations, handling coordinate transformations,
 * pose updates, and robot interactions.
 */
class RoboDKPhysicsBridge : public QObject
{
    Q_OBJECT

public:
    explicit RoboDKPhysicsBridge(RoboDK* rdk, std::shared_ptr<PhysicsEngine> physicsEngine, QObject* parent = nullptr);
    ~RoboDKPhysicsBridge();

    // Initialization
    bool initialize();
    void cleanup();

    // Object management
    bool addObjectToPhysics(Item item);
    bool removeObjectFromPhysics(Item item);
    bool isObjectInPhysics(Item item) const;
    
    // Robot interaction
    bool convertToKinematic(Item item);
    bool convertToDynamic(Item item);
    bool isObjectGrabbedByRobot(Item item) const;
    
    // Pose synchronization
    void updatePhysicsFromRoboDK();
    void updateRoboDKFromPhysics();
    void syncKinematicObjects();
    
    // Configuration
    void setUpdateInterval(int milliseconds);
    int getUpdateInterval() const;
    
    void enableAutomaticSync(bool enable);
    bool isAutomaticSyncEnabled() const;
    
    // Debug and monitoring
    int getTrackedObjectCount() const;
    void enableDebugOutput(bool enable);
    bool isDebugOutputEnabled() const;

signals:
    void objectAdded(Item item);
    void objectRemoved(Item item);
    void objectConvertedToKinematic(Item item);
    void objectConvertedToDynamic(Item item);
    void syncError(const QString& error);

private slots:
    void onSimulationStep();
    void onUpdateTimer();

private:
    // Core components
    RoboDK* m_robodk;
    std::shared_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<CoordinateConverter> m_coordinateConverter;
    
    // Object tracking
    struct TrackedObject {
        Item robodkItem;
        PxRigidActor* physicsActor;
        bool isKinematic;
        bool isGrabbedByRobot;
        PxTransform lastPhysicsPose;
        Mat lastRoboDKPose;
        
        TrackedObject() : physicsActor(nullptr), isKinematic(false), isGrabbedByRobot(false) {}
    };
    
    std::map<Item, TrackedObject> m_trackedObjects;
    
    // Configuration
    int m_updateInterval;
    bool m_automaticSyncEnabled;
    bool m_debugOutputEnabled;
    
    // Timers
    QTimer* m_updateTimer;
    
    // Helper methods
    bool createPhysicsActor(Item item, TrackedObject& trackedObj);
    void destroyPhysicsActor(TrackedObject& trackedObj);
    
    PxTransform convertRoboDKPoseToPhysX(const Mat& robodkPose);
    Mat convertPhysXPoseToRoboDK(const PxTransform& physxPose);
    
    void updateKinematicActorPose(TrackedObject& trackedObj);
    void updateDynamicActorPose(TrackedObject& trackedObj);
    
    bool checkRobotInteraction(Item item);
    void wakeNearbyDynamicActors(PxRigidDynamic* kinematicActor);
    
    // Debug helpers
    void logObjectState(const TrackedObject& trackedObj, const QString& context);
    void validateObjectConsistency(const TrackedObject& trackedObj);
    
    // Constants
    static constexpr int DEFAULT_UPDATE_INTERVAL = 16; // 60 FPS
    static constexpr float NEARBY_ACTOR_DISTANCE = 200.0f; // mm
    static constexpr float POSE_UPDATE_THRESHOLD = 0.1f; // mm
};

#endif // ROBODKPHYSICSBRIDGE_H 