#include "SimulationManager.h"
#include <QPointer>
#include "MaterialEditorDialog.h"
#include <QToolBar>
#include <QComboBox>
#include <QAction>
#include <QMainWindow>
#include <QToolButton>
#include <QMenu>
#include "ObjectPropertiesDialog.h"
#include <QTreeView>
#include <QMenu>
#include <QMessageBox>
#include <QModelIndex>
#include <QMetaMethod>
#include "CadNode.h"
#include "CustomModelTreeModel.h"
#include "CadTreeModel.h"

#define PVD_HOST "127.0.0.1"
#include "PxPhysicsAPI.h"
#include "extensions/PxDeformableVolumeExt.h"
#include "PxDeformableVolume.h"
#include "extensions/PxTetMakerExt.h"
#include "foundation/PxArray.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include "extensions/PxTetMakerExt.h"
#include "extensions/PxRemeshingExt.h"
#include "extensions/PxCudaHelpersExt.h"
#include "extensions/PxTetrahedronMeshExt.h"


using namespace physx;

PxFilterFlags PhysXEngine::simulationFilterShader (
    PxFilterObjectAttributes attributes0, PxFilterData filterData0,
    PxFilterObjectAttributes attributes1, PxFilterData filterData1,
    PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
    // Extract filter data components
    // word0: object type (0=regular, 1=robot, 2=soft body)
    // word1: attachment group (0=none, 1=group1, 2=group2, etc.)
    // word2: collision group (0=default, 1=group1, 2=group2, etc.)
    // word3: attachment surface ID (0=none, 1=surface1, 2=surface2, etc.)

    PxU32 objectType0 = filterData0.word0;
    PxU32 attachmentGroup0 = filterData0.word1;
    PxU32 collisionGroup0 = filterData0.word2;
    PxU32 attachmentSurface0 = filterData0.word3;

    PxU32 objectType1 = filterData1.word0;
    PxU32 attachmentGroup1 = filterData1.word1;
    PxU32 collisionGroup1 = filterData1.word2;
    PxU32 attachmentSurface1 = filterData1.word3;

    // Default behavior: allow all contacts
    pairFlags = PxPairFlag::eCONTACT_DEFAULT;

    // Special case: If both objects are in the same attachment group but different surfaces,
    // disable collision between them (soft body and its attachment surface)
    if (attachmentGroup0 != 0 && attachmentGroup0 == attachmentGroup1) {
        if (attachmentSurface0 != attachmentSurface1) {
            // Same attachment group but different surfaces - disable collision
            return PxFilterFlag::eKILL;
        }
    }

    // Special case: Soft body self-collision (already handled by flag, but double-check)
    if (objectType0 == 2 && objectType1 == 2) {
        // Soft body to soft body - use default behavior (self-collision disabled by flag)
        return PxFilterFlag::eDEFAULT;
    }

    // Special case: Soft body with its attachment surface
    if ((objectType0 == 2 && attachmentSurface1 != 0) ||
        (objectType1 == 2 && attachmentSurface0 != 0)) {
        // Check if this is the soft body's own attachment surface
        if (attachmentGroup0 != 0 && attachmentGroup0 == attachmentGroup1) {
            // This is the soft body's attachment surface - disable collision
            return PxFilterFlag::eKILL;
        }
    }

    return PxFilterFlag::eDEFAULT;
}

bool PhysXEngine::initializePhysX()
{
    m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_allocator, m_errorCallback);
    if (!m_foundation) {
        qDebug() << "Failed to create PhysX foundation";
        return false;
    }

    m_pvd = PxCreatePvd(*m_foundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
    m_pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

    PxTolerancesScale scale;
    m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, scale, true, m_pvd);
    if (!m_physics) {
        qDebug() << "Failed to create PhysX physics";
        return false;
    }

    try {
        if (!PxInitExtensions(*m_physics, m_pvd)) {
            qDebug() << "Failed to initialize PhysX extensions";
            return false;
        }
    } catch (const std::exception& e) {
        qDebug() << "Exception during PhysX extensions initialization:" << e.what();
        return false;
    }

    PxCudaContextManagerDesc cudaContextManagerDesc;
    m_cudaContextManager = PxCreateCudaContextManager(*m_foundation, cudaContextManagerDesc, PxGetProfilerCallback());
    if (m_cudaContextManager && !m_cudaContextManager->contextIsValid()) {
        PX_RELEASE(m_cudaContextManager);
        qDebug() << "Failed to initialize cuda context.";
        return false;
    }

    m_globalCookingParams = PxCookingParams(m_physics->getTolerancesScale());
    m_globalCookingParams.meshWeldTolerance = 0.001f;
    m_globalCookingParams.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
    m_globalCookingParams.buildTriangleAdjacencies = false;
    m_globalCookingParams.buildGPUData = true;

    PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
    sceneDesc.gravity = m_gravity;

    if (!sceneDesc.cudaContextManager)
        sceneDesc.cudaContextManager = m_cudaContextManager;

    sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
    sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

    if (m_stabilizationEnabled) {
        sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
    }

    sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
    sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
    sceneDesc.gpuMaxNumPartitions = 8;
    sceneDesc.solverType = PxSolverType::eTGS;

    PxU32 numCores = 2;
    m_dispatcher = PxDefaultCpuDispatcherCreate(numCores);
    sceneDesc.cpuDispatcher = m_dispatcher;
    sceneDesc.filterShader = simulationFilterShader;

    m_scene = m_physics->createScene(sceneDesc);
    if (!m_scene) {
        qDebug() << "Failed to create PhysX scene";
        return false;
    }

    PxPvdSceneClient* pvdClient = m_scene->getScenePvdClient();
    if (pvdClient) {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    m_material = m_physics->createMaterial(m_globalStaticFriction, m_globalDynamicFriction, m_globalRestitution);
    if (!m_material) {
        qDebug() << "Failed to create PhysX material";
        return false;
    }

    // Create low-friction material for kinematic actors (robots)
    if (!createKinematicMaterial()) {
        qDebug() << "Failed to create kinematic material";
        return false;
    }

    if (!createGroundPlane()) {
        qDebug() << "Failed to create ground plane";
        return false;
    }

    return true;
}

bool PhysXEngine::createKinematicMaterial()
{
    // Create low-friction material for kinematic actors (robots)
    // Using sheet metal-like properties: very low friction for smooth surfaces
    m_kinematicMaterial = m_physics->createMaterial(0.15f, 0.12f, 0.05f);
    if (!m_kinematicMaterial) {
        qDebug() << "Failed to create kinematic material";
        return false;
    }

    qDebug() << "Created kinematic material with low friction (static: 0.15, dynamic: 0.12, restitution: 0.05)";
    return true;
}


bool PhysXEngine::createGroundPlane()
{
    m_groundPlane = m_physics->createRigidStatic(PxTransform(PxVec3(0, -50.0f, 0)));
    PxShape* groundShape = m_physics->createShape(PxBoxGeometry(10000.0f, 0.1f, 10000.0f), *m_material);

    float enhancedContactOffset = m_contactOffset * 2.0f;
    float enhancedRestOffset = m_restOffset * 1.5f;
    groundShape->setContactOffset(enhancedContactOffset);
    groundShape->setRestOffset(enhancedRestOffset);
    groundShape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));

    m_groundPlane->attachShape(*groundShape);
    m_scene->addActor(*m_groundPlane);
    groundShape->release();

    return true;
}

void PhysXEngine::configureDynamicActor(PxRigidDynamic* actor)
{
    PxRigidBodyExt::setMassAndUpdateInertia(*actor, 10000.0f);
    actor->setAngularDamping(0.5f);
    actor->setLinearDamping(0.2f);
    actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
    actor->setSleepThreshold(m_sleepThreshold);
    actor->setStabilizationThreshold(m_stabilizationThreshold);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, m_ccdEnabled);
    actor->setSolverIterationCounts(m_solverPositionIterations, m_solverVelocityIterations);
}


void PhysXEngine::stepSimulationExtended(float deltaTime)
{
    if (!m_scene) {
        return;
    }

    const float fixedTimeStep = 1.0f / 60.0f;
    stepSimulation(fixedTimeStep);

    PxActorTypeFlags actorTypes = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
    PxU32 nbActors = m_scene->getNbActors(actorTypes);
    std::vector<PxActor*> actors(nbActors);
    m_scene->getActors(actorTypes, actors.data(), nbActors);

    for (PxActor* physxActor : actors) {
        PxRigidDynamic* dynamic = physxActor->is<PxRigidDynamic>();
        if (dynamic) {
            bool isKinematic = dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC;

            if (isKinematic) {
                dynamic->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
            } else {
                PxVec3 linearVel = dynamic->getLinearVelocity();
                PxVec3 angularVel = dynamic->getAngularVelocity();

                float linearThreshold = 0.1f;
                float angularThreshold = 0.01f;

                if (linearVel.magnitudeSquared() < linearThreshold * linearThreshold &&
                    angularVel.magnitudeSquared() < angularThreshold * angularThreshold) {

                    if (!dynamic->isSleeping()) {
                        dynamic->putToSleep();
                    }
                } else {
                    if (dynamic->isSleeping()) {
                        dynamic->wakeUp();
                    }
                }
            }
        }
    }

    bool actorsChanged = false;

    for (PxActor* physxActor : actors) {
        PxRigidDynamic* dynamic = physxActor->is<PxRigidDynamic>();
        if (dynamic && (dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
            dynamic->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
        }
    }

    //Insert code to update objects in tree

}

void PhysXEngine::stepSimulation(float deltaTime)
{
    if (!m_scene) return;

    const float fixedTimeStep = 1.0f / 60.0f;
    const float numberOfSubIterations = 1.0f;

    for (int i = 0; i < numberOfSubIterations; i++) {
        bool isLastSubstep = (i == numberOfSubIterations - 1);

        if (!isLastSubstep) {
            m_scene->setSceneQueryUpdateMode(PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_DISABLED);
        } else {
            m_scene->setSceneQueryUpdateMode(PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED);
        }

        m_scene->simulate(fixedTimeStep/numberOfSubIterations);
        m_scene->fetchResults(true);
    }
}

SimulationManager::SimulationManager() :
    materialManager(nullptr) {
    // Initialize state
    m_currentState.time = 0.0f;
    m_currentState.isPaused = true;
    m_currentState.stepCount = 0;
}

void SimulationManager::startSimulation() {
    if (m_running) {
        std::cout << "Simulation is already running" << std::endl;
        return;
    }

    m_running = true;
    m_paused = false;
    m_currentState.isPaused = false;

    // Start simulation thread
    m_simulationThread = std::thread(&SimulationManager::simulationLoop, this);

    std::cout << "Simulation started" << std::endl;
}

void SimulationManager::pauseSimulation() {
    if (!m_running) {
        std::cout << "Simulation is not running" << std::endl;
        return;
    }

    m_paused = true;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.isPaused = true;
    }

    std::cout << "Simulation paused" << std::endl;
}

void SimulationManager::resumeSimulation() {
    if (!m_running) {
        std::cout << "Simulation is not running" << std::endl;
        return;
    }

    m_paused = false;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.isPaused = false;
    }

    std::cout << "Simulation resumed" << std::endl;
}

void SimulationManager::stopSimulation() {
    if (!m_running) {
        return;
    }

    // Send quit command
    sendCommand(SimulationCommand::QUIT);

    // Wait for thread to finish
    if (m_simulationThread.joinable()) {
        m_simulationThread.join();
    }

    m_running = false;
    m_paused = false;
    m_currentState.isPaused = false;
}


void SimulationManager::addGuiElements(QMainWindow* mainWindow)
{
    if (!mainWindow) return;
    // Create a toolbar
    QToolBar* toolbar = new QToolBar("Simulation Toolbar", mainWindow);
    mainWindow->addToolBar(toolbar);

    // Create a QToolButton with a dropdown menu
    QToolButton* toolButton = new QToolButton(toolbar);
    toolButton->setText("Test");
    toolButton->setPopupMode(QToolButton::InstantPopup);

    // Create the menu
    QMenu* menu = new QMenu(toolButton);
    QAction* showMaterialManagerAction = menu->addAction("Show Material Manager");
    toolButton->setMenu(menu);

    toolbar->addWidget(toolButton);

    QObject::connect(showMaterialManagerAction, &QAction::triggered, mainWindow, [mainWindow, this]() {
        if (materialManager) {
            materialManager->showMaterialManagerDialog(mainWindow);
        }
    });
}

void SimulationManager::registerPhysicsNodeContextMenu(QTreeView* treeView)
{
    if (!treeView) return;
    treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(treeView, &QTreeView::customContextMenuRequested, treeView, [this, treeView](const QPoint& pos) {
        QModelIndex idx = treeView->indexAt(pos);
        if (!idx.isValid()) return;
        // Try to get CadNode pointer from the model
        auto model = treeView->model();
        CadNode* node = nullptr;
        // Try to use getNode(QModelIndex) if available
        auto metaObj = model->metaObject();
        int methodIndex = metaObj->indexOfMethod("getNode(QModelIndex) const");
        if (methodIndex != -1) {
            QMetaMethod method = metaObj->method(methodIndex);
            void* args[2] = { (void*)&idx, (void*)&node };
            method.invoke(const_cast<QAbstractItemModel*>(model), Qt::DirectConnection, Q_RETURN_ARG(CadNode*, node), Q_ARG(QModelIndex, idx));
        } else {
            // Fallback: try static_cast for known model types
            if (auto customModel = dynamic_cast<CustomModelTreeModel*>(model)) {
                node = const_cast<CadNode*>(customModel->getNode(idx));
            } else if (auto cadTreeModel = dynamic_cast<CadTreeModel*>(model)) {
                node = const_cast<CadNode*>(cadTreeModel->getNode(idx));
            }
        }
        if (!node) return;
        if (node->type != CadNodeType::Physics || !node->asPhysics()) return;
        QMenu menu;
        QAction* editPropsAction = menu.addAction("Edit Object Properties...");
        QObject::connect(editPropsAction, &QAction::triggered, treeView, [this, node, treeView]() {
            ObjectPropertiesDialog dlg(node, materialManager, treeView);
            if (dlg.exec() == QDialog::Accepted) {
                // Optionally, update the view/model if needed
                treeView->model()->dataChanged(treeView->currentIndex(), treeView->currentIndex());
            }
        });
        menu.exec(treeView->viewport()->mapToGlobal(pos));
    });
}


void SimulationManager::simulationLoop() {
    std::cout << "Simulation thread started" << std::endl;

    // Initialize PhysX engine in this thread
    m_physXEngine = std::make_unique<PhysXEngine>();
    if (!m_physXEngine->initializePhysX()) {
        std::cout << "Failed to initialize PhysX engine" << std::endl;
        return;
    }

    m_lastStepTime = std::chrono::steady_clock::now();

    while (m_running) {
        // Process any pending commands
        processCommands();

        // Check if we should quit
        if (!m_running) {
            break;
        }

        // If paused, wait for resume or step command
        if (m_paused && !m_stepRequested) {
            std::unique_lock<std::mutex> lock(m_commandMutex);
            m_commandCV.wait(lock, [this] {
                return !m_running || !m_paused || m_stepRequested;
            });
            continue;
        }

        // Perform simulation step
        updateSimulation();

        // If this was a single step request, mark it complete
        if (m_stepRequested) {
            m_stepRequested = false;
            m_stepComplete = true;
            m_stepCompleteCV.notify_one();
        }

        // Maintain fixed timestep
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastStepTime);
        auto targetDuration = std::chrono::milliseconds(static_cast<int>(m_timeStep));

        if (elapsed < targetDuration) {
            std::this_thread::sleep_for(targetDuration - elapsed);
        }

        m_lastStepTime = std::chrono::steady_clock::now();
    }

    std::cout << "Simulation thread finished" << std::endl;
}

void SimulationManager::processCommands() {
    std::lock_guard<std::mutex> lock(m_commandMutex);

    while (!m_commandQueue.empty()) {
        SimulationCommandData cmd = m_commandQueue.front();
        m_commandQueue.pop();

        switch (cmd.command) {
            case SimulationCommand::START:
                // Already handled by startSimulation()
                break;

            case SimulationCommand::PAUSE:
                m_paused = true;
                {
                    std::lock_guard<std::mutex> stateLock(m_stateMutex);
                    m_currentState.isPaused = true;
                }
                break;

            case SimulationCommand::RESUME:
                m_paused = false;
                {
                    std::lock_guard<std::mutex> stateLock(m_stateMutex);
                    m_currentState.isPaused = false;
                }
                break;

            case SimulationCommand::QUIT:
                m_running = false;
                break;
        }
    }
}

void SimulationManager::updateSimulation() {
    if (!m_physXEngine) {
        return;
    }

    // Perform PhysX simulation step
    // m_physXEngine->stepSimulation(m_timeStep);

    // Update our state from PhysX
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.time += m_timeStep;
        m_currentState.stepCount++;

        // Update object positions/velocities from PhysX
        // This is where you would sync your PhysicsObject data with PhysX actors
    }

    // Notify GUI of update
    notifyGUIUpdate();
}

void SimulationManager::sendCommand(SimulationCommand command) {
    std::lock_guard<std::mutex> lock(m_commandMutex);

    SimulationCommandData cmd;
    cmd.command = command;

    m_commandQueue.push(cmd);
    m_commandCV.notify_one();
}


void SimulationManager::notifyGUIUpdate() {
    if (m_updateCallback) {
        SimulationState stateCopy;
        {
            std::lock_guard<std::mutex> lock(m_stateMutex);
            stateCopy = m_currentState;
        }

        // Note: In a real Qt application, you might want to use QMetaObject::invokeMethod
        // to ensure the callback runs on the main thread
        m_updateCallback(stateCopy);
    }
}
