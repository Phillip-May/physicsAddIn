#ifndef SIMULATIONMANAGER_H
#define SIMULATIONMANAGER_H

#include <QObject>
#include <memory>
#include <QMainWindow>
#include <QToolBar>
#include <QComboBox>
#include "MaterialManager.h"
#include "PxPhysicsAPI.h"
#include <QTreeView>
#include <thread>
#include <mutex>
#include <queue>

class PhysXEngine {
    friend class SimulationManager;
public:
    PhysXEngine()
        : m_foundation(nullptr),
          m_physics(nullptr),
          m_dispatcher(nullptr),
          m_scene(nullptr),
          m_material(nullptr),
          m_kinematicMaterial(nullptr),
          m_groundPlane(nullptr),
          m_cudaContextManager(nullptr),
          m_globalCookingParams(physx::PxTolerancesScale())
    {
    }
    ~PhysXEngine();
    bool initializePhysX(const std::shared_ptr<CadNode>& rootNode);
private:
    physx::PxDefaultAllocator m_allocator;
    physx::PxDefaultErrorCallback m_errorCallback;
    physx::PxFoundation* m_foundation;
    physx::PxPhysics* m_physics;
    physx::PxCpuDispatcher* m_dispatcher;
    physx::PxScene* m_scene;
    physx::PxMaterial* m_material;
    physx::PxMaterial* m_kinematicMaterial;
    physx::PxRigidStatic* m_groundPlane;
    physx::PxCudaContextManager* m_cudaContextManager;
    physx::PxCookingParams m_globalCookingParams;
    physx::PxVec3 m_gravity;

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
    static physx::PxFilterFlags simulationFilterShader(
        physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
        physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
        physx::PxPairFlags& pairFlags, const void* constantBlock,
        physx::PxU32 constantBlockSize);

    bool createKinematicMaterial();
    bool createGroundPlane();
    void configureDynamicActor(physx::PxRigidDynamic* actor);
    void stepSimulationExtended();
    void stepSimulation();
    void buildSceneFromNodes(const std::vector<CadNode*>& physicsNodes, 
                           std::unordered_map<CadNode*, physx::PxRigidDynamic*>& nodeToActor,
                           std::unordered_map<physx::PxRigidDynamic*, CadNode*>& actorToNode);
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
    
    void startSimulation();
    void pauseSimulation();
    void resumeSimulation();
    void stopSimulation();
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

    using SimulationUpdateCallback = std::function<void(const SimulationState&)>;
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

    float m_timeStepMS = 16;

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
    std::unique_ptr<MaterialManager> materialManager;
    static void collectPhysicsNodes(const std::shared_ptr<CadNode>& root, std::vector<CadNode*>& outNodes);
    void buildPhysXSceneFromNodes();
};

#endif // SIMULATIONMANAGER_H 
