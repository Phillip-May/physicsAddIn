#ifndef SIMULATIONMANAGER_H
#define SIMULATIONMANAGER_H

#include <QObject>
#include <memory>
#include <QMainWindow>
#include <QToolBar>
#include <QComboBox>
#include "MaterialManager.h"
#include <QTreeView>
#include <thread>
#include <mutex>
#include <queue>

class PhysXEngine {
    friend class SimulationManager;
public:
    PhysXEngine()
        : m_globalCookingParams(PxTolerancesScale())
    {
    }
    ~PhysXEngine();
    bool initializePhysX(const std::shared_ptr<CadNode>& rootNode);
private:
    // PhysX components (moved from global variables)
    PxDefaultAllocator m_allocator;
    PxDefaultErrorCallback m_errorCallback;
    PxFoundation* m_foundation;
    PxPhysics* m_physics;
    PxCpuDispatcher* m_dispatcher;
    PxScene* m_scene;
    PxMaterial* m_material;
    PxMaterial* m_kinematicMaterial;  // Low-friction material for kinematic actors (robots)
    PxPvd* m_pvd;
    PxPvdTransport* m_pvdTransport;  // Keep transport alive for PhysX 5.x
    PxRigidStatic* m_groundPlane;
    PxCudaContextManager* m_cudaContextManager;  // CUDA context for deformable volumes
    // Global cooking parameters for all mesh creation
    PxCookingParams m_globalCookingParams;
    // Global variables for physics simulation
    PxVec3 m_gravity;

    // Scene configuration settings
    int m_solverPositionIterations;
    int m_solverVelocityIterations;
    float m_globalStaticFriction;
    float m_globalDynamicFriction;
    float m_globalRestitution;
    bool m_pcmEnabled;
    bool m_stabilizationEnabled;
    float m_contactOffset;
    float m_restOffset;
    float m_sleepThreshold;
    float m_stabilizationThreshold;
    bool m_ccdEnabled;
    float m_wakeDistance;
    
    // Reference to root CadNode for ground plane configuration
    std::shared_ptr<CadNode> m_rootNode;
private:
    static PxFilterFlags simulationFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0, PxFilterObjectAttributes attributes1, PxFilterData filterData1, PxPairFlags &pairFlags, const void *constantBlock, PxU32 constantBlockSize);

    bool createKinematicMaterial();
    bool createGroundPlane();
    void configureDynamicActor(PxRigidDynamic *actor);
    void stepSimulationExtended(float deltaTime);
    void stepSimulation(float deltaTime);
    void buildSceneFromNodes(const std::vector<CadNode*>& physicsNodes, 
                           std::unordered_map<CadNode*, PxRigidDynamic*>& nodeToActor,
                           std::unordered_map<PxRigidDynamic*, CadNode*>& actorToNode);
};

class SimulationManager
{
    friend class PhysXEngine;
public:
    SimulationManager(const std::shared_ptr<CadNode>& m_CadNodeRootIn);
    void addGuiElements(QMainWindow* mainWindow);
    void registerPhysicsNodeContextMenu(QMenu* menu, CadNode* node);
    bool isSceneBuilding() const { return m_sceneBuilding; }
    bool hasNodeUpdates() const { return m_buffersSwapped; }
    
    // Get the latest node locations (thread-safe for GUI thread)
    const std::unordered_map<CadNode*, TopLoc_Location>& getLatestNodeLocations() const;
    
    // Mark that GUI has processed the latest updates
    void markUpdatesProcessed();
    
    // Check PVD connection status
    bool isPvdConnected() const;
    
    // Manually trigger PVD connection
    void connectPvd();
private:
    enum class SimulationCommand {
        START,
        PAUSE,
        RESUME,
        RESET,
        STEP,
        ADD_OBJECT,
        REMOVE_OBJECT,
        UPDATE_PARAMETERS,
        QUIT
    };

    // Data structure for commands with parameters
    struct SimulationCommandData {
        SimulationCommand command;
        std::vector<float> parameters;
        std::string objectId;  // For add/remove object commands
    };

    // Simulation state that can be shared between threads
    struct SimulationState {
        float time;
        bool isPaused;
        int stepCount;
    };

    // Double-buffered node location data
    struct NodeLocationData {
        std::unordered_map<CadNode*, TopLoc_Location> nodeLocations;
        std::atomic<bool> isDirty{false};
        std::atomic<int> version{0};
    };

    //Gui/main thread side code
    using SimulationUpdateCallback = std::function<void(const SimulationState&)>;
    void startSimulation();
    void pauseSimulation();
    void resumeSimulation();
    void stopSimulation();

    /*
    void addObject(const PhysicsObject& object);
    void removeObject(const std::string& objectId);
    void updateObject(const PhysicsObject& object);
    void updateSimulationParameters(float gravity, float timeStep, float damping);
    // State retrieval
    SimulationState getCurrentState();
    bool isRunning() const;
    bool isPaused() const;
    // Callback registration
    void setUpdateCallback(SimulationUpdateCallback callback);
    // Wait for simulation to complete current step
    void waitForStepComplete();
    */
private:
    // Thread management
    void simulationLoop();
    void processCommands();
    void updateSimulation();

    // Helper methods
    void sendCommand(SimulationCommand command);
    void notifyGUIUpdate();

    // Thread and synchronization
    std::thread m_simulationThread;
    std::mutex m_stateMutex;
    std::mutex m_commandMutex;
    std::condition_variable m_commandCV;
    std::condition_variable m_stepCompleteCV;

    // State and control
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_paused{false};
    std::atomic<bool> m_stepRequested{false};
    std::atomic<bool> m_stepComplete{false};
    std::atomic<bool> m_sceneBuilding{false};  // Flag to indicate scene building is in progress

    // Double-buffered node locations
    NodeLocationData m_nodeBufferA;
    NodeLocationData m_nodeBufferB;
    std::atomic<NodeLocationData*> m_readBuffer{&m_nodeBufferA};
    std::atomic<NodeLocationData*> m_writeBuffer{&m_nodeBufferB};
    std::atomic<bool> m_buffersSwapped{false};

    // Data structures
    SimulationState m_currentState;
    std::queue<SimulationCommandData> m_commandQueue;
    SimulationUpdateCallback m_updateCallback;

    //Target timestep
    float m_timeStepMS = 16; //Milliseconds

    // PhysX engine (will be created in simulation thread)
    std::unique_ptr<PhysXEngine> m_physXEngine;
    //Underlying tree
    const std::shared_ptr<CadNode> m_CadNodeRoot;
    // Mappings
    std::unordered_map<CadNode*, physx::PxRigidDynamic*> m_nodeToActor;
    std::unordered_map<physx::PxRigidDynamic*, CadNode*> m_actorToNode;
    // Timing
    std::chrono::steady_clock::time_point m_lastStepTime;
private:
    MaterialManager *const materialManager;
    static void collectPhysicsNodes(const std::shared_ptr<CadNode>& root, std::vector<CadNode*>& outNodes);
    void buildPhysXSceneFromNodes();
};

#endif // SIMULATIONMANAGER_H 
