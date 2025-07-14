#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include <memory>
#include <map>
#include <vector>
#include <QObject>
#include "PxPhysicsAPI.h"

class RoboDK;
class Item;

// Forward declarations
class PhysicsScene;
class PhysicsActor;
class PhysicsMaterial;

/**
 * @brief Core physics engine that manages PhysX simulation
 * 
 * This class encapsulates all PhysX functionality and provides
 * a clean interface for the rest of the application.
 */
class PhysicsEngine : public QObject
{
    Q_OBJECT

public:
    explicit PhysicsEngine(QObject* parent = nullptr);
    ~PhysicsEngine();

    // Initialization and cleanup
    bool initialize();
    void cleanup();
    bool isInitialized() const;

    // Scene management
    std::shared_ptr<PhysicsScene> getScene() const;
    bool createScene();
    void destroyScene();

    // Actor management
    bool addObject(Item item);
    bool removeObject(Item item);
    bool isObjectInSimulation(Item item) const;
    
    // Simulation control
    void startSimulation();
    void stopSimulation();
    void pauseSimulation();
    bool isSimulationRunning() const;
    
    // Physics step
    void stepSimulation(float deltaTime);

    // Configuration
    void setGravity(const PxVec3& gravity);
    PxVec3 getGravity() const;
    
    void setMaterialProperties(float staticFriction, float dynamicFriction, float restitution);
    void getMaterialProperties(float& staticFriction, float& dynamicFriction, float& restitution) const;

    // Debug and monitoring
    int getActorCount() const;
    void enableDebugVisualization(bool enable);
    bool isDebugVisualizationEnabled() const;

signals:
    void simulationStarted();
    void simulationStopped();
    void objectAdded(Item item);
    void objectRemoved(Item item);
    void simulationError(const QString& error);

private slots:
    void onSimulationStep();

private:
    // PhysX core components
    PxDefaultAllocator m_allocator;
    PxDefaultErrorCallback m_errorCallback;
    PxFoundation* m_foundation;
    PxPhysics* m_physics;
    PxDefaultCpuDispatcher* m_dispatcher;
    PxPvd* m_pvd;
    
    // Scene and material
    std::shared_ptr<PhysicsScene> m_scene;
    PxMaterial* m_material;
    
    // State tracking
    bool m_initialized;
    bool m_simulationRunning;
    bool m_debugVisualizationEnabled;
    
    // Actor tracking
    std::map<Item, std::shared_ptr<PhysicsActor>> m_actors;
    
    // Configuration
    PxVec3 m_gravity;
    float m_staticFriction;
    float m_dynamicFriction;
    float m_restitution;
    
    // Timer for simulation steps
    QTimer* m_simulationTimer;
    
    // Helper methods
    bool initializePhysX();
    void cleanupPhysX();
    bool createMaterial();
    void setupContactFilter();
    
    // Custom contact filter
    static PxFilterFlags simulationFilterShader(
        PxFilterObjectAttributes attributes0, PxFilterData filterData0,
        PxFilterObjectAttributes attributes1, PxFilterData filterData1,
        PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize);
};

#endif // PHYSICSENGINE_H 