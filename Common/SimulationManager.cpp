#include "SimulationManager.h"
#include <QDebug>
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
#include <QMessageBox>
#include <QModelIndex>
#include <QMetaMethod>
#include "CadNode.h"
#include "CustomModelTreeModel.h"

#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Mat.hxx>
#include <TopLoc_Location.hxx>

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

PhysXEngine::~PhysXEngine() {
    if (m_scene) {
        m_scene->release();
        m_scene = nullptr;
    }
    if (m_dispatcher) {
        m_dispatcher = nullptr;
    }
    if (m_physics) {
        m_physics->release();
        m_physics = nullptr;
    }
    if (m_foundation) {
        m_foundation->release();
        m_foundation = nullptr;
    }
}

PxFilterFlags PhysXEngine::simulationFilterShader (
    PxFilterObjectAttributes, PxFilterData filterData0,
    PxFilterObjectAttributes, PxFilterData filterData1,
    PxPairFlags& pairFlags, const void*, PxU32)
{
    // word0: object type (0=regular, 1=robot, 2=soft body)
    // word1: attachment group (0=none, 1=group1, 2=group2, etc.)
    // word2: collision group (0=default, 1=group1, 2=group2, etc.)
    // word3: attachment surface ID (0=none, 1=surface1, 2=surface2, etc.)

    PxU32 objectType0 = filterData0.word0;
    PxU32 attachmentGroup0 = filterData0.word1;
    PxU32 attachmentSurface0 = filterData0.word3;

    PxU32 objectType1 = filterData1.word0;
    PxU32 attachmentGroup1 = filterData1.word1;
    PxU32 attachmentSurface1 = filterData1.word3;

    pairFlags = PxPairFlag::eCONTACT_DEFAULT;

    // A soft body and its attachment surface share an attachment group (word1) but differ in
    // surface id (word3): kill that contact so the body does not collide with its anchor.
    if (attachmentGroup0 != 0 && attachmentGroup0 == attachmentGroup1) {
        if (attachmentSurface0 != attachmentSurface1) {
            return PxFilterFlag::eKILL;
        }
    }

    if (objectType0 == 2 && objectType1 == 2) {
        return PxFilterFlag::eDEFAULT;
    }

    if ((objectType0 == 2 && attachmentSurface1 != 0) ||
        (objectType1 == 2 && attachmentSurface0 != 0)) {
        if (attachmentGroup0 != 0 && attachmentGroup0 == attachmentGroup1) {
            return PxFilterFlag::eKILL;
        }
    }

    return PxFilterFlag::eDEFAULT;
}

bool PhysXEngine::initializePhysX(const std::shared_ptr<CadNode>& rootNode)
{
    m_rootNode = rootNode;
    
    m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_allocator, m_errorCallback);
    if (!m_foundation) {
        qDebug() << "Failed to create PhysX foundation";
        return false;
    }

    PxTolerancesScale scale;
    
    m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, scale, true, nullptr);
    if (!m_physics) {
        qDebug() << "Failed to create PhysX physics";
        return false;
    }

    m_gravity = PxVec3(0.0f, 0.0f, -9806.65f);  // mm/s², Z-up
    
    m_solverPositionIterations = 32;
    m_solverVelocityIterations = 32;
    m_globalStaticFriction = 0.95f;
    m_globalDynamicFriction = 0.90f;
    m_globalRestitution = 0.1f;
    m_pcmEnabled = true;
    m_stabilizationEnabled = true;
    m_contactOffset = 5.0f;
    m_restOffset = 1.0f;
    m_sleepThreshold = 10.0f;
    m_stabilizationThreshold = 10.0f;
    m_ccdEnabled = true;              // CCD keeps fast bodies from tunneling through thin shapes
    m_wakeDistance = 1000.0f;

    try {
        if (!PxInitExtensions(*m_physics, nullptr)) {
            qDebug() << "Failed to initialize PhysX extensions";
            return false;
        }
    } catch (const std::exception& e) {
        qDebug() << "Exception during PhysX extensions initialization:" << e.what();
        return false;
    }

#ifdef PHYSX_ENABLE_GPU
    PxCudaContextManagerDesc cudaContextManagerDesc;
    m_cudaContextManager = PxCreateCudaContextManager(*m_foundation, cudaContextManagerDesc, PxGetProfilerCallback());
    if (m_cudaContextManager && !m_cudaContextManager->contextIsValid()) {
        PX_RELEASE(m_cudaContextManager);
        qDebug() << "Failed to initialize cuda context; falling back to CPU PhysX.";
    }
#else
    m_cudaContextManager = nullptr;
#endif

    m_globalCookingParams = PxCookingParams(m_physics->getTolerancesScale());
    m_globalCookingParams.meshWeldTolerance = 0.001f;
    m_globalCookingParams.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
    m_globalCookingParams.buildTriangleAdjacencies = false;
    m_globalCookingParams.buildGPUData = (m_cudaContextManager != nullptr);

    PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
    sceneDesc.gravity = m_gravity;

    if (m_cudaContextManager) {
        sceneDesc.cudaContextManager = m_cudaContextManager;
        sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
        sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
        sceneDesc.gpuMaxNumPartitions = 8;
    } else {
        sceneDesc.broadPhaseType = PxBroadPhaseType::eSAP;
    }

    sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

    if (m_stabilizationEnabled) {
        sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
    }

    // Scene-level CCD stays on; each actor opts in via eENABLE_CCD.
    sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
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

    m_material = m_physics->createMaterial(m_globalStaticFriction, m_globalDynamicFriction, m_globalRestitution);
    if (!m_material) {
        qDebug() << "Failed to create PhysX material";
        return false;
    }

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
    m_kinematicMaterial = m_physics->createMaterial(0.15f, 0.12f, 0.05f);
    if (!m_kinematicMaterial) {
        qDebug() << "Failed to create kinematic material";
        return false;
    }

    return true;
}


bool PhysXEngine::createGroundPlane()
{
    float groundPlaneY = -50.0f;
    float groundPlaneSize = 10000.0f;
    float groundPlaneThickness = 0.1f;
    
    if (m_rootNode && m_rootNode->type == CadNodeType::MutexRoot) {
        MutexRootNodeData* mutexData = m_rootNode->asMutexRoot();
        if (mutexData) {
            groundPlaneY = mutexData->groundPlaneY;
            groundPlaneSize = mutexData->groundPlaneSize;
            groundPlaneThickness = mutexData->groundPlaneThickness;
        }
    }
    
    PxVec3 groundPos(0, 0, groundPlaneY);
    
    m_groundPlane = m_physics->createRigidStatic(PxTransform(groundPos));
    PxShape* groundShape = m_physics->createShape(PxBoxGeometry(groundPlaneSize, groundPlaneSize, groundPlaneThickness), *m_material);

    // Contact offsets must exceed the mesh weld tolerance or thin-shell collisions are missed.
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


void PhysXEngine::stepSimulationExtended()
{
    if (!m_scene) {
        return;
    }

    stepSimulation();

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

    for (PxActor* physxActor : actors) {
        PxRigidDynamic* dynamic = physxActor->is<PxRigidDynamic>();
        if (dynamic && (dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
            dynamic->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
        }
    }


}

void PhysXEngine::stepSimulation()
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

void PhysXEngine::buildSceneFromNodes(const std::vector<CadNode*>& physicsNodes,
                                     std::unordered_map<CadNode*, PxRigidDynamic*>& nodeToActor,
                                     std::unordered_map<PxRigidDynamic*, CadNode*>& actorToNode)
{
    for (CadNode* node : physicsNodes) {
        PhysicsNodeData* physData = node->asPhysics();
        if (!physData || physData->hulls.empty()) continue;

        std::vector<PxShape*> shapes;
        for (size_t hullIdx = 0; hullIdx < physData->hulls.size(); ++hullIdx) {
            const auto& hull = physData->hulls[hullIdx];
            double minX=1e30,maxX=-1e30,minY=1e30,maxY=-1e30,minZ=1e30,maxZ=-1e30;
            for (const auto& v : hull.vertices) {
                minX = std::min(minX, (double)v[0]); maxX = std::max(maxX, (double)v[0]);
                minY = std::min(minY, (double)v[1]); maxY = std::max(maxY, (double)v[1]);
                minZ = std::min(minZ, (double)v[2]); maxZ = std::max(maxZ, (double)v[2]);
            }
            double extentX = maxX - minX;
            double extentY = maxY - minY;
            double extentZ = maxZ - minZ;
            double diag = std::sqrt(extentX*extentX + extentY*extentY + extentZ*extentZ);
            double hullVolume = 0.0;
            for (const auto& tri : hull.indices) {
                const auto& v1 = hull.vertices[tri[0]];
                const auto& v2 = hull.vertices[tri[1]];
                const auto& v3 = hull.vertices[tri[2]];
                hullVolume += ((v1[0] * v2[1] * v3[2]) + (v1[1] * v2[2] * v3[0]) + (v1[2] * v2[0] * v3[1]) -
                              (v3[0] * v2[1] * v1[2]) - (v3[1] * v2[2] * v1[0]) - (v3[2] * v2[0] * v1[1])) / 6.0;
            }
            hullVolume = std::abs(hullVolume);
            if (hullVolume < 1e-6) {
                qDebug() << "[PhysX] Skipping hull" << hullIdx << "due to very small volume:" << hullVolume;
                continue;
            }
            // Prepare vertices - PhysX and CAD use the same coordinate system
            std::vector<PxVec3> pxVertices(hull.vertices.size());
            for (size_t i = 0; i < hull.vertices.size(); ++i) {
                pxVertices[i] = PxVec3((float)hull.vertices[i][0], (float)hull.vertices[i][1], (float)hull.vertices[i][2]);
            }
            PxConvexMeshDesc convexDesc;
            convexDesc.points.count = static_cast<uint32_t>(pxVertices.size());
            convexDesc.points.stride = sizeof(PxVec3);
            convexDesc.points.data = pxVertices.data();
            convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
            if (pxVertices.size() < 4) {
                qDebug() << "[PhysX] Hull" << hullIdx << "has fewer than 4 vertices, skipping";
                continue;
            }
            PxDefaultMemoryOutputStream buf;
            if (!PxCookConvexMesh(m_globalCookingParams, convexDesc, buf)) {
                qDebug() << "[PhysX] Failed to cook convex mesh for hull" << hullIdx
                         << "verts:" << hull.vertices.size()
                         << "tris:" << hull.indices.size()
                         << "extents:" << extentX << extentY << extentZ
                         << "diag:" << diag
                         << "volume:" << hullVolume;
                continue;
            }
            PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
            PxConvexMesh* convexMesh = m_physics->createConvexMesh(input);
            PxShape* shape = m_physics->createShape(
                PxConvexMeshGeometry(convexMesh),
                *m_material
            );
            shape->setMaterials(&m_material, 1);
            shape->setContactOffset(m_contactOffset);
            shape->setRestOffset(m_restOffset);
            shapes.push_back(shape);
        }

        PxTransform pose;
        
        if (node->loc.IsIdentity()) {
            pose = PxTransform(PxVec3(0,0,0), PxQuat(0,0,0,1));
        } else {
            gp_Trsf trsf = node->loc.Transformation();
            
            gp_Pnt origin = trsf.TranslationPart();
            PxVec3 pos((float)origin.X(), (float)origin.Y(), (float)origin.Z());
            
            gp_Mat rotMat = trsf.VectorialPart();
            PxMat33 rotMat33(
                PxVec3((float)rotMat.Value(1,1), (float)rotMat.Value(1,2), (float)rotMat.Value(1,3)),
                PxVec3((float)rotMat.Value(2,1), (float)rotMat.Value(2,2), (float)rotMat.Value(2,3)),
                PxVec3((float)rotMat.Value(3,1), (float)rotMat.Value(3,2), (float)rotMat.Value(3,3))
            );
            
            pose = PxTransform(pos, PxQuat(rotMat33));
        }
        
        PxRigidDynamic* actor = m_physics->createRigidDynamic(pose);
        
        for (PxShape* shape : shapes) {
            actor->attachShape(*shape);
            shape->release(); // actor now owns the shape
        }

        actor->setMass(physData->mass);
        configureDynamicActor(actor);

        m_scene->addActor(*actor);
        node->asPhysics()->isPhysicsActive = true;

        nodeToActor[node] = actor;
        actorToNode[actor] = node;
    }
}

SimulationManager::SimulationManager(const std::shared_ptr<CadNode>& m_CadNodeRootIn) :
    materialManager(std::make_unique<MaterialManager>()),
    m_CadNodeRoot(m_CadNodeRootIn) {
    m_currentState.time = 0.0f;
    m_currentState.isPaused = true;
    m_currentState.stepCount = 0;
}

void SimulationManager::buildPhysXSceneFromNodes() {
    // Set scene building flag to prevent GUI rendering during this process
    m_sceneBuilding = true;
    
    m_nodeToActor.clear();
    m_actorToNode.clear();
    
    if (!m_physXEngine) {
        m_physXEngine = std::make_unique<PhysXEngine>();
        if (!m_physXEngine->initializePhysX(m_CadNodeRoot)) {
            m_sceneBuilding = false;
            return;
        }
    }

    if (!m_physXEngine || !m_CadNodeRoot) {
        m_sceneBuilding = false;
        return;
    }

    std::vector<CadNode*> physicsNodes;
    collectPhysicsNodes(m_CadNodeRoot, physicsNodes);

    m_physXEngine->buildSceneFromNodes(physicsNodes, m_nodeToActor, m_actorToNode);
    
    // The flag stays true until updateSimulation() completes the first step.
}

void SimulationManager::startSimulation() {
    if (m_running) {
        return;
    }

    m_running = true;
    m_paused = false;
    m_currentState.isPaused = false;

    // Start simulation thread - scene building will happen on the simulation thread
    m_simulationThread = std::thread(&SimulationManager::simulationLoop, this);
}

void SimulationManager::pauseSimulation() {
    if (!m_running) {
        return;
    }

    m_paused = true;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.isPaused = true;
    }

}

void SimulationManager::resumeSimulation() {
    if (!m_running) {
        return;
    }

    m_paused = false;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.isPaused = false;
    }

}

void SimulationManager::stopSimulation() {
    if (!m_running) {
        return;
    }

    sendCommand(SimulationCommand::QUIT);

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
    QToolBar* toolbar = new QToolBar("Simulation Toolbar", mainWindow);
    mainWindow->addToolBar(toolbar);

    QToolButton* toolButton = new QToolButton(toolbar);
    toolButton->setText("Test");
    toolButton->setPopupMode(QToolButton::InstantPopup);

    QMenu* menu = new QMenu(toolButton);
    QAction* showMaterialManagerAction = menu->addAction("Show Material Manager");
    QAction* startSimulationAction = menu->addAction("Start Simulation");
    QAction* stopSimulationAction = menu->addAction("Stop Simulation");
    toolButton->setMenu(menu);

    toolbar->addWidget(toolButton);

    QObject::connect(showMaterialManagerAction, &QAction::triggered, mainWindow, [mainWindow, this]() {
        if (materialManager) {
            materialManager->showMaterialManagerDialog(mainWindow);
        }
    });
    QObject::connect(startSimulationAction, &QAction::triggered, mainWindow, [this]() {
        this->startSimulation();
    });
    QObject::connect(stopSimulationAction, &QAction::triggered, mainWindow, [this]() {
        this->stopSimulation();
    });
}

void SimulationManager::registerPhysicsNodeContextMenu(QMenu* menu, CadNode* node)
{
    if (!menu || !node) return;
    if (node->type != CadNodeType::Physics || !node->asPhysics()) return;
    QAction* editPropsAction = menu->addAction("Edit Object Properties...");
    QObject::connect(editPropsAction, &QAction::triggered, nullptr, [this, node]() {
        ObjectPropertiesDialog dlg(node, materialManager.get(), nullptr);
        dlg.exec();
    });
}


void SimulationManager::simulationLoop() {
    // Build PhysX scene on the simulation thread
    buildPhysXSceneFromNodes();

    while (m_running) {
        auto frameStart = std::chrono::steady_clock::now();

        processCommands();

        if (!m_running) {
            break;
        }

        if (m_paused && !m_stepRequested) {
            std::unique_lock<std::mutex> lock(m_commandMutex);
            m_commandCV.wait(lock, [this] {
                return !m_running || !m_paused || m_stepRequested;
            });
            continue;
        }

        updateSimulation();

        if (m_stepRequested) {
            m_stepRequested = false;
            m_stepComplete = true;
            m_stepCompleteCV.notify_one();
        }

        auto frameEnd = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(frameEnd - frameStart);
        auto targetDuration = std::chrono::milliseconds(static_cast<int>(m_timeStepMS));

        if (elapsed < targetDuration) {
            std::this_thread::sleep_for(targetDuration - elapsed);
        }
    }

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

    NodeLocationData* writeBuffer = m_writeBuffer.load();
    writeBuffer->nodeLocations.clear();
    writeBuffer->isDirty = true;

     m_physXEngine->stepSimulationExtended();

    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.time += m_timeStepMS / 1000.0f;
        m_currentState.stepCount++;

        for (const auto& pair : m_actorToNode) {
            PxRigidActor* actor = pair.first;
            CadNode* node = pair.second;
            if (!actor || !node) continue;

            PxTransform physxPose = actor->getGlobalPose();
            
            PxVec3 pos = physxPose.p;
            PxQuat quat = physxPose.q;

            gp_Trsf trsf;
            double qx = quat.x, qy = quat.y, qz = quat.z, qw = quat.w;
            double xx = qx * qx, yy = qy * qy, zz = qz * qz;
            double xy = qx * qy, xz = qx * qz, yz = qy * qz;
            double wx = qw * qx, wy = qw * qy, wz = qw * qz;

            double m[3][3];
            m[0][0] = 1.0 - 2.0 * (yy + zz);
            m[0][1] = 2.0 * (xy - wz);
            m[0][2] = 2.0 * (xz + wy);
            m[1][0] = 2.0 * (xy + wz);
            m[1][1] = 1.0 - 2.0 * (xx + zz);
            m[1][2] = 2.0 * (yz - wx);
            m[2][0] = 2.0 * (xz - wy);
            m[2][1] = 2.0 * (yz + wx);
            m[2][2] = 1.0 - 2.0 * (xx + yy);

            trsf.SetValues(
                m[0][0], m[0][1], m[0][2], pos.x,
                m[1][0], m[1][1], m[1][2], pos.y,
                m[2][0], m[2][1], m[2][2], pos.z
            );

            // Written to the buffer, never to the node: the GUI thread owns node state.
            TopLoc_Location newLocation(trsf);
            writeBuffer->nodeLocations[node] = newLocation;
            
        }
    }

    if (m_sceneBuilding && m_currentState.stepCount == 1) {
        m_sceneBuilding = false;
    }

    writeBuffer->isDirty = false;
    writeBuffer->version++;
    
    NodeLocationData* oldReadBuffer = m_readBuffer.exchange(writeBuffer);
    m_writeBuffer.store(oldReadBuffer);
    m_buffersSwapped = true;

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

        m_updateCallback(stateCopy);
    }
}


const std::unordered_map<CadNode*, TopLoc_Location>& SimulationManager::getLatestNodeLocations() const {
    return m_readBuffer.load()->nodeLocations;
}

void SimulationManager::markUpdatesProcessed() {
    m_buffersSwapped = false;
}

void SimulationManager::collectPhysicsNodes(const std::shared_ptr<CadNode>& root, std::vector<CadNode*>& outNodes) {
    if (!root) return;
    
    if (root->type == CadNodeType::Physics) {
        outNodes.push_back(root.get());
    }
    
    for (const auto& child : root->children) {
        collectPhysicsNodes(child, outNodes);
    }
}
