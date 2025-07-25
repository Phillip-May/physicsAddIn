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
public:
    PhysXEngine()
        : m_globalCookingParams(PxTolerancesScale())
    {
    }
    bool initializePhysX();
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
private:
    static PxFilterFlags simulationFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0, PxFilterObjectAttributes attributes1, PxFilterData filterData1, PxPairFlags &pairFlags, const void *constantBlock, PxU32 constantBlockSize);

    bool createKinematicMaterial();
    bool createGroundPlane();
    void configureDynamicActor(PxRigidDynamic *actor);
    void stepSimulationExtended(float deltaTime);
    void stepSimulation(float deltaTime);
};

class SimulationManager
{
public:
    SimulationManager();
    void addGuiElements(QMainWindow* mainWindow);
    void registerPhysicsNodeContextMenu(QTreeView* treeView);
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

    // Data structures
    SimulationState m_currentState;
    std::queue<SimulationCommandData> m_commandQueue;
    SimulationUpdateCallback m_updateCallback;

    //Target timestep
    float m_timeStep = 16; //Milliseconds

    // PhysX engine (will be created in simulation thread)
    std::unique_ptr<PhysXEngine> m_physXEngine;
    // Timing
    std::chrono::steady_clock::time_point m_lastStepTime;
private:
    MaterialManager *const materialManager;
};

#endif // SIMULATIONMANAGER_H 
