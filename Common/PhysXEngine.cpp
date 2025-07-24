#include "PhysXEngine.h"
#include "MaterialManager.h"
#include "ObjectPropertiesManager.h"
#include "robodktools.h"
#include "irobodk.h"
#include "iitem.h"

#include <QDebug>
#include <QTimer>
#include <QElapsedTimer>
#include <QDir>
#include <fstream>
#include <iostream>
#include <vector>
#include <numeric>

#define PVD_HOST "127.0.0.1"

#include "extensions/PxDeformableVolumeExt.h"
#include "PxDeformableVolume.h"
#include "extensions/PxTetMakerExt.h"
#include "foundation/PxArray.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include "extensions/PxTetMakerExt.h"
#include "extensions/PxRemeshingExt.h"
#include "extensions/PxCudaHelpersExt.h"
#include "extensions/PxTetrahedronMeshExt.h"

// VHACD Library - must be included after defining ENABLE_VHACD_IMPLEMENTATION
#define ENABLE_VHACD_IMPLEMENTATION 1
#include "VHACD.h"

using namespace physx;

PxReal PhysXEngine::m_stackZ = 10.0f;
RoboDK* PhysXEngine::m_rdk = nullptr;
bool PhysXEngine::m_isLoadingDone = false;

PxFilterFlags PhysXEngine::simulationFilterShader(
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

PhysXEngine::PhysXEngine(RoboDK* rdk, QObject* parent)
    : IPhysicsEngine(parent)
    , m_foundation(nullptr)
    , m_physics(nullptr)
    , m_dispatcher(nullptr)
    , m_scene(nullptr)
    , m_material(nullptr)
    , m_kinematicMaterial(nullptr)
    , m_pvd(nullptr)
    , m_groundPlane(nullptr)
    , m_cudaContextManager(nullptr)
    , m_ownsFoundation(true)
    , m_ownsPhysics(true)
    , m_ownsScene(true)
    , m_ownsMaterial(true)
    , m_ownsKinematicMaterial(true)
    , m_ownsGroundPlane(true)
    , m_objectPropertiesManager(nullptr)
    , m_globalCookingParams(PxTolerancesScale())
{
    initializeDefaults();
    if (!initializePhysX()) {
        qDebug() << "Failed to initialize PhysX engine";
    }
}

PhysXEngine::PhysXEngine(RoboDK* rdk,
                         PxFoundation* foundation,
                         PxPhysics* physics,
                         PxScene* scene,
                         PxMaterial* material,
                         PxRigidStatic* groundPlane,
                         QObject* parent)
    : IPhysicsEngine(parent)
    , m_foundation(foundation)
    , m_physics(physics)
    , m_dispatcher(nullptr)
    , m_scene(scene)
    , m_material(material)
    , m_kinematicMaterial(nullptr)
    , m_pvd(nullptr)
    , m_groundPlane(groundPlane)
    , m_cudaContextManager(nullptr)
    , m_ownsFoundation(false)
    , m_ownsPhysics(false)
    , m_ownsScene(false)
    , m_ownsMaterial(false)
    , m_ownsKinematicMaterial(true)
    , m_ownsGroundPlane(false)
    , m_objectPropertiesManager(nullptr)
    , m_globalCookingParams(physics ? physics->getTolerancesScale() : PxTolerancesScale())
{
    initializeDefaults();

    if (m_scene) {
        m_dispatcher = m_scene->getCpuDispatcher();
    }
}

PhysXEngine::~PhysXEngine()
{
    cleanup();
}

bool PhysXEngine::initialize()
{
    if (m_physics) {
        qDebug() << "PhysX engine already initialized";
        return true;
    }

    // If we have existing components, we're already initialized
    if (m_foundation && m_physics && m_scene && m_material) {
        qDebug() << "PhysX engine initialized with existing components";
        return true;
    }

    return initializePhysX();
}

void PhysXEngine::cleanup()
{
    for (auto& pair : m_objects) {
        PhysXObjectData& data = pair.second;
        if (data.actor && m_scene) {
            m_scene->removeActor(*data.actor);
            data.actor->release();
        }
    }
    m_objects.clear();

    for (auto& pair : m_robots) {
        RobotData& robotData = pair.second;
        for (auto& jointPair : robotData.jointActors) {
            PxRigidDynamic* actor = jointPair.second;
            if (actor && m_scene) {
                m_scene->removeActor(*actor);
                actor->release();
            }
        }
    }
    m_robots.clear();

    for (auto& pair : m_softBodies) {
        SoftBodyData& softBodyData = pair.second;
        if (softBodyData.softBody && m_scene) {
            m_scene->removeActor(*softBodyData.softBody);
            softBodyData.softBody->release();
        }

        if (softBodyData.simPositionInvMassPinned) {
            PX_FREE(softBodyData.simPositionInvMassPinned);
        }
        if (softBodyData.simVelocityPinned) {
            PX_FREE(softBodyData.simVelocityPinned);
        }
        if (softBodyData.collPositionInvMassPinned) {
            PX_FREE(softBodyData.collPositionInvMassPinned);
        }
        if (softBodyData.restPositionPinned) {
            PX_FREE(softBodyData.restPositionPinned);
        }

        if (softBodyData.auxData) {
            softBodyData.auxData->release();
        }
    }
    m_softBodies.clear();

    if (m_groundPlane && m_scene && m_ownsGroundPlane) {
        m_scene->removeActor(*m_groundPlane);
        m_groundPlane->release();
        m_groundPlane = nullptr;
    }

    cleanupPhysX();
}

bool PhysXEngine::addObject(Item item)
{
    if (!item || !m_physics || !m_material || !m_scene) {
        qDebug() << "Cannot add object - physics system not initialized";
        return false;
    }

    if (m_objects.find(item) != m_objects.end()) {
        return true;
    }

    Mat itemPose = item->PoseAbs();
    PxTransform physxPose = convertRoboDKPoseToPhysX_STL(itemPose);

    QString tempFilePath = QDir::tempPath() + QDir::separator() + "temp_object.stl";
    item->Save(tempFilePath);

    std::vector<PxVec3> vertices;
    std::vector<PxU32> indices;

    if (LoadBinarySTL(tempFilePath.toStdString(), vertices, indices)) {
        std::vector<PxVec3> physxVertices;
        physxVertices.reserve(vertices.size());
        for (const PxVec3& vertex : vertices) {
            physxVertices.push_back(convertSTLToPhysX(vertex));
        }

        // Use VHACD for convex decomposition instead of simple convex hull
        std::vector<PxConvexMesh*> convexMeshes = CreateVHACDConvexMeshes(m_physics, physxVertices, indices, m_globalCookingParams, false); // Regular object
        
        if (!convexMeshes.empty()) {
            qDebug() << "VHACD: Created" << convexMeshes.size() << "convex meshes for object" << item->Name();
            
            // Create a compound shape with all convex meshes
            PxRigidDynamic* actor = m_physics->createRigidDynamic(physxPose);
            
            if (actor) {
                // Attach all convex meshes as separate shapes
                for (PxConvexMesh* convexMesh : convexMeshes) {
                    if (convexMesh) {
                        PxConvexMeshGeometry geometry(convexMesh, PxMeshScale(), PxConvexMeshGeometryFlag::eTIGHT_BOUNDS);
                        
                        PxShape* shape = m_physics->createShape(geometry, *m_material);
                        if (shape) {
                            shape->setContactOffset(m_contactOffset);
                            shape->setRestOffset(m_restOffset);
                            shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));
                            
                            actor->attachShape(*shape);
                        }
                    }
                }
                
                // Configure the actor
                configureDynamicActor(actor);
                m_scene->addActor(*actor);

                PhysXObjectData data;
                data.actor = actor;
                data.isDynamic = true;
                data.isKinematic = false;
                data.isGrabbedByRobot = false;
                m_objects[item] = data;

                actor->userData = item;
                emit objectAdded(item);
                
                qDebug() << "VHACD: Successfully added object" << item->Name() << "with" << convexMeshes.size() << "convex parts";

                QFile::remove(tempFilePath);
                
                // Clean up convex meshes (they are now owned by the shapes)
                for (PxConvexMesh* mesh : convexMeshes) {
                    if (mesh) mesh->release();
                }
                
                return true;
            } else {
                qDebug() << "VHACD: Failed to create actor for object" << item->Name();
            }
            
            // Clean up convex meshes if actor creation failed
            for (PxConvexMesh* mesh : convexMeshes) {
                if (mesh) mesh->release();
            }
        } else {
            qDebug() << "VHACD: Failed to create convex meshes for object" << item->Name() << "- falling back to simple convex hull";
            
            // Fallback to simple convex hull if VHACD fails
            PxConvexMesh* convexMesh = CreateConvexMesh(m_physics, physxVertices, m_globalCookingParams);
            if (convexMesh) {
                PxConvexMeshGeometry geometry(convexMesh, PxMeshScale(), PxConvexMeshGeometryFlag::eTIGHT_BOUNDS);

                PxShape* shape = m_physics->createShape(geometry, *m_material);
                shape->setContactOffset(m_contactOffset);
                shape->setRestOffset(m_restOffset);
                shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));

                PxRigidDynamic* actor = m_physics->createRigidDynamic(physxPose);

                if (actor && shape) {
                    actor->attachShape(*shape);
                    configureDynamicActor(actor);
                    m_scene->addActor(*actor);

                    PhysXObjectData data;
                    data.actor = actor;
                    data.isDynamic = true;
                    data.isKinematic = false;
                    data.isGrabbedByRobot = false;
                    m_objects[item] = data;

                    actor->userData = item;
                    emit objectAdded(item);

                    QFile::remove(tempFilePath);
                    return true;
                } else {
                    if (shape) shape->release();
                }
            }
        }
        QFile::remove(tempFilePath);
    } else {
        QFile::remove(tempFilePath);
    }

    return false;
}

// Robot management methods
bool PhysXEngine::addRobot(Item robot)
{
    if (!robot || !getRoboDK()->Valid(robot) || !m_physics || !m_material || !m_scene) {
        qDebug() << "Cannot add robot - invalid robot or physics system not initialized";
        return false;
    }

    if (m_robots.find(robot) != m_robots.end()) {
        return true;
    }

    if (robot->Type() != IItem::ITEM_TYPE_ROBOT) {
        qDebug() << "Item is not a robot:" << robot->Name();
        return false;
    }

    RobotData robotData;

    int nJoints = robot->Joints().Length() + 1;
    std::vector<Item> jointsOriginal;
    std::vector<Item> jointsCopy;

    for (int i = 0; i < nJoints; ++i) {
        Item jointObjectOriginal = robot->ObjectLink(i);
        jointsOriginal.push_back(jointObjectOriginal);
    }

    Item parentFrame = getRoboDK()->ItemUserPick("Click modify robot, convert all the links to objects, then choose the parent frame here, make sure the robot and the models pose match exactly", IItem::ITEM_TYPE_FRAME);
    if (parentFrame) {
        auto children = parentFrame->Childs();
        for (int i = 0; i < children.size(); i++) {
            Item child = children[i];
            jointsCopy.push_back(child);
        }
    } else {
        return false;
    }

    Mat robotPoseAbs = robot->PoseAbs();
    for (int jointIndex = 0; jointIndex < nJoints; ++jointIndex) {
        Item link = robot->ObjectLink(jointIndex);
        Item geometryObject = jointsCopy.at(jointIndex);

        Mat poseJoint = link->PoseAbs();

        Mat poseLinkOffset = robotPoseAbs.inv() * poseJoint;
        poseLinkOffset = poseLinkOffset.inv();
        jointsCopy.at(jointIndex)->setGeometryPose(poseLinkOffset,true);
        jointsCopy.at(jointIndex)->setPoseAbs(poseJoint);

        PxTransform physxPose = convertRoboDKPoseToPhysX_STL(poseJoint);

        QString tempFilePath = QDir::tempPath() + QDir::separator() + QString("temp_joint_%1.stl").arg(jointIndex);
        geometryObject->Save(tempFilePath);

        std::vector<PxVec3> vertices;
        std::vector<PxU32> indices;

        if (LoadBinarySTL(tempFilePath.toStdString(), vertices, indices)) {
            std::vector<PxVec3> physxVertices;
            physxVertices.reserve(vertices.size());
            for (const PxVec3& vertex : vertices) {
                physxVertices.push_back(convertSTLToPhysX(vertex));
            }

            // Use VHACD for convex decomposition instead of simple convex hull
            std::vector<PxConvexMesh*> convexMeshes = CreateVHACDConvexMeshes(m_physics, physxVertices, indices, m_globalCookingParams, true); // Robot mesh
            
            if (!convexMeshes.empty()) {
                qDebug() << "VHACD: Created" << convexMeshes.size() << "convex meshes for robot joint" << jointIndex;
                
                // Create a compound shape with all convex meshes
                PxRigidDynamic* actor = m_physics->createRigidDynamic(physxPose);
                
                if (actor) {
                    // Attach all convex meshes as separate shapes
                    for (PxConvexMesh* convexMesh : convexMeshes) {
                        if (convexMesh) {
                            PxConvexMeshGeometry geometry(convexMesh, PxMeshScale(), PxConvexMeshGeometryFlag::eTIGHT_BOUNDS);
                            
                            PxShape* shape = m_physics->createShape(geometry, *m_kinematicMaterial);
                            if (shape) {
                                shape->setContactOffset(m_contactOffset);
                                shape->setRestOffset(m_restOffset);
                                shape->setSimulationFilterData(PxFilterData(0, 0, 1, 1));
                                
                                actor->attachShape(*shape);
                            }
                        }
                    }
                    
                    // Configure the actor
                    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
                    actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
                    actor->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
                    actor->setSleepThreshold(0.0f);
                    actor->setStabilizationThreshold(0.0f);

                    m_scene->addActor(*actor);
                    robotData.jointActors[jointIndex] = actor;
                    actor->userData = geometryObject;
                    
                    qDebug() << "VHACD: Successfully added robot joint" << jointIndex << "with" << convexMeshes.size() << "convex parts";
                } else {
                    qDebug() << "VHACD: Failed to create actor for robot joint" << jointIndex;
                }
                
                // Clean up convex meshes (they are now owned by the shapes)
                for (PxConvexMesh* mesh : convexMeshes) {
                    if (mesh) mesh->release();
                }
            } else {
                qDebug() << "VHACD: Failed to create convex meshes for robot joint" << jointIndex << "- falling back to simple convex hull";
                
                // Fallback to simple convex hull if VHACD fails
                PxConvexMesh* convexMesh = CreateConvexMesh(m_physics, physxVertices, m_globalCookingParams);
                if (convexMesh) {
                    PxConvexMeshGeometry geometry(convexMesh, PxMeshScale(), PxConvexMeshGeometryFlag::eTIGHT_BOUNDS);

                    PxShape* shape = m_physics->createShape(geometry, *m_kinematicMaterial);
                    shape->setContactOffset(m_contactOffset);
                    shape->setRestOffset(m_restOffset);
                    shape->setSimulationFilterData(PxFilterData(0, 0, 1, 1));

                    PxRigidDynamic* actor = m_physics->createRigidDynamic(physxPose);

                    if (actor && shape) {
                        actor->attachShape(*shape);
                        actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
                        actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
                        actor->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
                        actor->setSleepThreshold(0.0f);
                        actor->setStabilizationThreshold(0.0f);

                        m_scene->addActor(*actor);
                        robotData.jointActors[jointIndex] = actor;
                        actor->userData = geometryObject;
                    } else {
                        if (shape) shape->release();
                    }
                }
            }
        }
        QFile::remove(tempFilePath);
    }

    robotData.isInSimulation = true;
    m_robots[robot] = robotData;
    emit robotAdded(robot);

    return true;
}

bool PhysXEngine::removeRobot(Item robot)
{
    auto it = m_robots.find(robot);
    if (it == m_robots.end()) {
        return false;
    }

    RobotData& robotData = it->second;

    for (auto& jointPair : robotData.jointActors) {
        PxRigidDynamic* actor = jointPair.second;
        if (actor && m_scene) {
            m_scene->removeActor(*actor);
            actor->release();
        }
    }

    robotData.jointActors.clear();
    robotData.isInSimulation = false;
    m_robots.erase(it);
    emit robotRemoved(robot);

    return true;
}

bool PhysXEngine::isRobotInSimulation(Item robot) const
{
    auto it = m_robots.find(robot);
    return it != m_robots.end() && it->second.isInSimulation;
}

void PhysXEngine::updateRobotJoints(Item robot)
{
    auto it = m_robots.find(robot);
    if (it == m_robots.end()) {
        return;
    }

    RobotData& robotData = it->second;
    int nJoints = robot->Joints().Length() + 1;

    if (nJoints == 0) {
        return;
    }

    for (int jointIndex = 0; jointIndex < nJoints; ++jointIndex) {
        auto actorIt = robotData.jointActors.find(jointIndex);
        if (actorIt == robotData.jointActors.end()) {
            continue;
        }

        PxRigidDynamic* actor = actorIt->second;
        if (!actor) continue;

        Item jointObject = robot->ObjectLink(jointIndex);
        Mat jointPose = jointObject->PoseAbs();
        
        if (actor->userData) {
            Item robodkObj = static_cast<Item>(actor->userData);
            if (robodkObj) {
                robodkObj->setPoseAbs(jointPose);
            }
        }

        PxTransform physxPose = convertRoboDKPoseToPhysX_STL(jointPose);
        actor->setGlobalPose(physxPose);
        actor->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
    }
}

bool PhysXEngine::removeObject(Item item)
{
    auto it = m_objects.find(item);
    if (it == m_objects.end()) {
        return false;
    }

    PhysXObjectData& data = it->second;
    if (data.actor) {
        m_scene->removeActor(*data.actor);
        data.actor->release();
    }

    m_objects.erase(it);
    emit objectRemoved(item);

    return true;
}

bool PhysXEngine::isObjectInSimulation(Item item) const
{
    return m_objects.find(item) != m_objects.end();
}

bool PhysXEngine::convertToKinematic(Item item)
{
    auto it = m_objects.find(item);
    if (it == m_objects.end() || it->second.isKinematic) return false;

    PhysXObjectData& data = it->second;
    PxRigidDynamic* dynamicActor = static_cast<PxRigidDynamic*>(data.actor);

    dynamicActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    dynamicActor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
    dynamicActor->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
    dynamicActor->setSleepThreshold(0.0f);
    dynamicActor->setStabilizationThreshold(0.0f);

    data.isDynamic = false;
    data.isKinematic = true;
    data.isGrabbedByRobot = true;

    return true;
}

bool PhysXEngine::convertToDynamic(Item item)
{
    auto it = m_objects.find(item);
    if (it == m_objects.end() || !it->second.isKinematic) return false;

    PhysXObjectData& data = it->second;
    PxRigidDynamic* kinematicActor = static_cast<PxRigidDynamic*>(data.actor);

    kinematicActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
    kinematicActor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
    configureDynamicActor(kinematicActor);
    kinematicActor->wakeUp();

    data.isDynamic = true;
    data.isKinematic = false;
    data.isGrabbedByRobot = false;

    return true;
}

bool PhysXEngine::isObjectGrabbedByRobot(Item item) const
{
    auto it = m_objects.find(item);
    return it != m_objects.end() && it->second.isGrabbedByRobot;
}

bool PhysXEngine::detectRobotGrab(Item item) const
{
    if (!item || !getRoboDK()->Valid(item)) {
        return false;
    }

    QList<Item> robots = getRoboDK()->getItemList(IItem::ITEM_TYPE_ROBOT_ARM);
    for (Item robot : robots) {
        if (!robot || !getRoboDK()->Valid(robot)) continue;

        Item parent = item->Parent();
        while (parent && getRoboDK()->Valid(parent)) {
            if (parent == robot) {
                return true;
            }
            parent = parent->Parent();
        }
    }

    return false;
}

void PhysXEngine::updatePhysicsFromRoboDK(Item item)
{
    auto it = m_objects.find(item);
    if (it == m_objects.end() || !it->second.isKinematic) return;

    PhysXObjectData& data = it->second;
    PxRigidDynamic* kinematicActor = static_cast<PxRigidDynamic*>(data.actor);

    Mat rdkPose = item->PoseAbs();
    PxTransform physxPose = convertRoboDKPoseToPhysX_STL(rdkPose);

    kinematicActor->setKinematicTarget(physxPose);
    wakeUpNearbyDynamicObjects(kinematicActor, physxPose);
    kinematicActor->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
}

void PhysXEngine::updateRoboDKFromPhysics(Item item)
{
    auto it = m_objects.find(item);
    if (it == m_objects.end() || it->second.isKinematic) return;

    PhysXObjectData& data = it->second;
    PxRigidActor* rigidActor = data.actor;

    if (rigidActor) {
        PxTransform pose = rigidActor->getGlobalPose();
        Mat rdkPose = convertPhysXPoseToRoboDK_STL(pose);
        item->setPoseAbs(rdkPose);
    }
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

void PhysXEngine::stepSimulationExtended(float deltaTime)
{
    if (!m_scene) {
        return;
    }

    const float fixedTimeStep = 1.0f / 60.0f;
    stepSimulation(fixedTimeStep);
    validateAndRemoveInvalidObjects();

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

    QList<Item> allObjects = getRoboDK()->getItemList(IItem::ITEM_TYPE_OBJECT);
    for (Item item : allObjects) {
        if (!item || !getRoboDK()->Valid(item)) continue;

        bool currentlyGrabbed = detectRobotGrab(item);

        if (currentlyGrabbed) {
            if (!isObjectInSimulation(item)) {
                if (addObject(item)) {
                    actorsChanged = true;
                }
            } else {
                if (!isObjectGrabbedByRobot(item)) {
                    convertToKinematic(item);
                    actorsChanged = true;
                }
            }
        } else {
            if (isObjectInSimulation(item)) {
                if (isObjectGrabbedByRobot(item)) {
                    convertToDynamic(item);
                    actorsChanged = true;
                }
            }
        }
    }

    QList<Item> simulationObjects = getRoboDK()->getItemList(IItem::ITEM_TYPE_OBJECT);
    for (Item item : simulationObjects) {
        if (!item || !getRoboDK()->Valid(item)) continue;

        if (isObjectInSimulation(item)) {
            if (isObjectGrabbedByRobot(item)) {
                updatePhysicsFromRoboDK(item);
            } else {
                updateRoboDKFromPhysics(item);
            }
        }
    }

    QList<Item> robots = getRoboDK()->getItemList(IItem::ITEM_TYPE_ROBOT);
    for (Item robot : robots) {
        if (!robot || !getRoboDK()->Valid(robot)) continue;

        if (isRobotInSimulation(robot)) {
            updateRobotJoints(robot);
        }
    }

    for (auto& pair : m_softBodies) {
        Item item = pair.first;
        SoftBodyData& softBodyData = pair.second;

        if (!item || !getRoboDK()->Valid(item)) continue;

        if (softBodyData.isInSimulation && softBodyData.softBody && softBodyData.simPositionInvMassPinned) {
            PxTetrahedronMesh* tetMesh = softBodyData.softBody->getCollisionMesh();
            if (tetMesh) {
                PxScopedCudaLock _lock(*m_cudaContextManager);
                m_cudaContextManager->getCudaContext()->memcpyDtoH(
                    softBodyData.simPositionInvMassPinned,
                    reinterpret_cast<CUdeviceptr>(softBodyData.softBody->getPositionInvMassBufferD()),
                    tetMesh->getNbVertices() * sizeof(PxVec4)
                );
            }
        }
    }
}

void PhysXEngine::setGravity(const PxVec3& gravity)
{
    m_gravity = gravity;
    if (m_scene) {
        m_scene->setGravity(gravity);

        for (auto& pair : m_objects) {
            PhysXObjectData& data = pair.second;
            if (data.actor && data.isDynamic) {
                PxRigidDynamic* dynamic = static_cast<PxRigidDynamic*>(data.actor);
                if (dynamic->isSleeping()) {
                    dynamic->wakeUp();
                }
            }
        }
    }
}

PxVec3 PhysXEngine::getGravity() const
{
    return m_gravity;
}

int PhysXEngine::getActorCount() const
{
    if (!m_scene) return 0;

    PxActorTypeFlags actorTypes = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
    return m_scene->getNbActors(actorTypes);
}

void PhysXEngine::enableDebugVisualization(bool enable)
{
    m_debugVisualizationEnabled = enable;
    // Implementation would depend on your debug visualization system
}

bool PhysXEngine::isDebugVisualizationEnabled() const
{
    return m_debugVisualizationEnabled;
}

QString PhysXEngine::getEngineName() const
{
    return "PhysX";
}

QString PhysXEngine::getEngineVersion() const
{
    return "5.6.0";
}

void PhysXEngine::validateObjects()
{
    validateAndRemoveInvalidObjects();
}

// Private helper methods

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

void PhysXEngine::cleanupPhysX()
{
    // Only release components that we own
    if (m_ownsScene) PX_RELEASE(m_scene);
    if (m_ownsPhysics) PX_RELEASE(m_physics);
    if (m_ownsMaterial) PX_RELEASE(m_material);
    if (m_ownsKinematicMaterial) PX_RELEASE(m_kinematicMaterial);
    if (m_ownsFoundation) PX_RELEASE(m_foundation);

    // Release CUDA context manager
    PX_RELEASE(m_cudaContextManager);

    // Close PhysX extensions (required for proper cleanup)
    PxCloseExtensions();

    // Note: We don't own dispatcher or PVD when using existing components
    // They are managed by the existing system
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

// Note: Material creation moved to initializePhysX() to match NVIDIA example

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

PxTransform PhysXEngine::convertRoboDKPoseToPhysX(const Mat& robodkPose)
{
    PxTransform physxPose;
    // RoboDK: Z-up, PhysX: Y-up
    // Position: RoboDK (X,Y,Z) -> PhysX (X,Z,Y)
    physxPose.p = PxVec3(robodkPose(0, 3), robodkPose(2, 3), robodkPose(1, 3));

    // Convert RoboDK rotation to PhysX quaternion
    // Apply coordinate system transformation: RoboDK (X,Y,Z) -> PhysX (X,Z,Y)
    PxMat33 robodkRotMat(
        PxVec3(robodkPose(0, 0), robodkPose(1, 0), robodkPose(2, 0)), // X axis
        PxVec3(robodkPose(0, 1), robodkPose(1, 1), robodkPose(2, 1)), // Y axis
        PxVec3(robodkPose(0, 2), robodkPose(1, 2), robodkPose(2, 2))  // Z axis
    );

    // Create coordinate system transformation matrix: RoboDK (X,Y,Z) -> PhysX (X,Z,Y)
    PxMat33 coordTransform(
        PxVec3(1, 0, 0),  // X axis stays the same
        PxVec3(0, 0, 1),  // Y axis becomes Z axis
        PxVec3(0, 1, 0)   // Z axis becomes Y axis
    );

    // Apply transformation: physxRot = coordTransform * robodkRot * coordTransform^T
    PxMat33 physxRotMat = coordTransform * robodkRotMat * coordTransform.getTranspose();

    // Convert to quaternion
    physxPose.q = PxQuat(physxRotMat);

    return physxPose;
}

Mat PhysXEngine::convertPhysXPoseToRoboDK(const PxTransform& physxPose)
{
    PxMat33 R(physxPose.q);
    PxVec3 p = physxPose.p;

    // Create a QMatrix4x4-compatible matrix (row-major)
    Mat m;

    // First row (X axis)
    m(0, 0) = R.column0.x;
    m(0, 1) = R.column1.x;
    m(0, 2) = R.column2.x;
    m(0, 3) = p.x;  // position X

    // Second row (Y axis) ← PhysX Z axis
    m(1, 0) = R.column0.z;
    m(1, 1) = R.column1.z;
    m(1, 2) = R.column2.z;
    m(1, 3) = p.z;  // position Z

    // Third row (Z axis) ← PhysX Y axis
    m(2, 0) = R.column0.y;
    m(2, 1) = R.column1.y;
    m(2, 2) = R.column2.y;
    m(2, 3) = p.y;  // position Y

    // Last row
    m(3, 0) = 0.0;
    m(3, 1) = 0.0;
    m(3, 2) = 0.0;
    m(3, 3) = 1.0;

    return m;
}

// STL coordinate system conversion functions
// These handle the case where meshes are loaded from STL files and need
// coordinate system transformation: STL (X,Y,Z) -> RoboDK (X,Z,Y)
PxTransform PhysXEngine::convertRoboDKPoseToPhysX_STL(const Mat& robodkPose)
{
    PxTransform physxPose;

    // Position: RoboDK (X,Y,Z) -> PhysX (X,Z,Y)
    physxPose.p = PxVec3(robodkPose(0, 3), robodkPose(2, 3), robodkPose(1, 3));

    // Convert RoboDK rotation to PhysX quaternion with STL coordinate system handling
    // Apply coordinate system transformation: RoboDK (X,Y,Z) -> PhysX (X,Z,Y)
    PxMat33 robodkRotMat(
        PxVec3(robodkPose(0, 0), robodkPose(1, 0), robodkPose(2, 0)), // X axis
        PxVec3(robodkPose(0, 1), robodkPose(1, 1), robodkPose(2, 1)), // Y axis
        PxVec3(robodkPose(0, 2), robodkPose(1, 2), robodkPose(2, 2))  // Z axis
    );

    // Create coordinate system transformation matrix: RoboDK (X,Y,Z) -> PhysX (X,Z,Y)
    PxMat33 coordTransform(
        PxVec3(1, 0, 0),  // X axis stays the same
        PxVec3(0, 0, 1),  // Y axis becomes Z axis
        PxVec3(0, 1, 0)   // Z axis becomes Y axis
    );

    // Apply transformation: physxRot = coordTransform * robodkRot * coordTransform^T
    PxMat33 physxRotMat = coordTransform * robodkRotMat * coordTransform.getTranspose();

    // Convert to quaternion
    physxPose.q = PxQuat(physxRotMat);

    return physxPose;
}

Mat PhysXEngine::convertPhysXPoseToRoboDK_STL(const PxTransform& physxPose)
{
    PxMat33 R(physxPose.q);
    PxVec3 p = physxPose.p;

    // For STL coordinates, we need to apply a coordinate system transformation
    // STL: (X,Y,Z) -> RoboDK: (X,Z,Y)
    // The transformation matrix is: [1 0 0; 0 0 1; 0 1 0]
    // We need to apply: RoboDK_R = Transform * PhysX_R * Transform^T

    // Create the coordinate system transformation matrix
    PxMat33 transform(
        PxVec3(1, 0, 0),  // X axis stays the same
        PxVec3(0, 0, 1),  // Y becomes Z
        PxVec3(0, 1, 0)   // Z becomes Y
    );

    // Apply the transformation: RoboDK_R = Transform * PhysX_R * Transform^T
    PxMat33 transformedR = transform * R * transform.getTranspose();

    Mat m;

    // First row (X axis)
    m(0, 0) = transformedR.column0.x;
    m(0, 1) = transformedR.column1.x;
    m(0, 2) = transformedR.column2.x;
    m(0, 3) = p.x;  // position X

    // Second row (Y axis in RoboDK)
    m(1, 0) = transformedR.column0.y;
    m(1, 1) = transformedR.column1.y;
    m(1, 2) = transformedR.column2.y;
    m(1, 3) = p.z;  // position Z becomes Y

    // Third row (Z axis in RoboDK)
    m(2, 0) = transformedR.column0.z;
    m(2, 1) = transformedR.column1.z;
    m(2, 2) = transformedR.column2.z;
    m(2, 3) = p.y;  // position Y becomes Z

    // Last row
    m(3, 0) = 0.0;
    m(3, 1) = 0.0;
    m(3, 2) = 0.0;
    m(3, 3) = 1.0;

    return m;
}

// Helper conversion functions
PxVec3 PhysXEngine::convertSTLToPhysX(const PxVec3& stlVertex)
{
    // Convert STL coordinate system to PhysX coordinate system
    // STL: (X,Y,Z) -> PhysX: (X,Z,Y)
    PxVec3 physx = PxVec3(stlVertex.x, stlVertex.z, stlVertex.y);
    if (!isValidVector(physx)) {
        qDebug() << "PhysXEngine: Invalid conversion STL->PhysX:" << stlVertex.x << stlVertex.y << stlVertex.z << "->" << physx.x << physx.y << physx.z;
    }
    return physx;
}

PxVec3 PhysXEngine::convertPhysXToSTL(const PxVec3& physxVertex)
{
    // Convert PhysX coordinate system to STL coordinate system
    // PhysX: (X,Z,Y) -> STL: (X,Y,Z)
    PxVec3 stl = PxVec3(physxVertex.x, physxVertex.z, physxVertex.y);
    if (!isValidVector(stl)) {
        qDebug() << "PhysXEngine: Invalid conversion PhysX->STL:" << physxVertex.x << physxVertex.y << physxVertex.z << "->" << stl.x << stl.y << stl.z;
    }
    return stl;
}

Mat PhysXEngine::quaternionToMatrix(const PxQuat& q)
{
    PxMat33 R(q);

    // Reorder axes: PhysX Y-up → RoboDK Z-up
    // X axis stays the same, Y and Z are swapped with proper sign handling
    return Mat(
        R.column0.x, R.column1.x, R.column2.x, 0.0,
        R.column0.z, R.column1.z, R.column2.z, 0.0,
        R.column0.y, R.column1.y, R.column2.y, 0.0
    );
}

// Mesh extraction helper functions
tMatrix2D* PhysXEngine::createBoxMesh(PxVec3 halfExtents)
{
    // 8 corners
    PxVec3 v[8] = {
        {-halfExtents.x, -halfExtents.y, -halfExtents.z},
        { halfExtents.x, -halfExtents.y, -halfExtents.z},
        { halfExtents.x,  halfExtents.y, -halfExtents.z},
        {-halfExtents.x,  halfExtents.y, -halfExtents.z},
        {-halfExtents.x, -halfExtents.y,  halfExtents.z},
        { halfExtents.x, -halfExtents.y,  halfExtents.z},
        { halfExtents.x,  halfExtents.y,  halfExtents.z},
        {-halfExtents.x,  halfExtents.y,  halfExtents.z}
    };

    // 12 triangles (3 vertices each) = 36 vertices
    int tris[12][3] = {
        {0,1,2}, {0,2,3}, // Bottom
        {4,6,5}, {4,7,6}, // Top
        {0,4,5}, {0,5,1}, // Front
        {1,5,6}, {1,6,2}, // Right
        {2,6,7}, {2,7,3}, // Back
        {3,7,4}, {3,4,0}  // Left
    };

    tMatrix2D* mat = Matrix2D_Create();
    Matrix2D_Set_Size(mat, 3, 36);  // 12 triangles * 3 vertices

    for (int t = 0; t < 12; ++t) {
        for (int j = 0; j < 3; ++j) {
            int idx = tris[t][j];
            int col = t * 3 + j;
            // Convert from PhysX to STL coordinate system for RoboDK
            PxVec3 stlVertex = convertPhysXToSTL(v[idx]);
            Matrix2D_Set_ij(mat, 0, col, stlVertex.x);
            Matrix2D_Set_ij(mat, 1, col, stlVertex.y);
            Matrix2D_Set_ij(mat, 2, col, stlVertex.z);
        }
    }

    return mat;
}

tMatrix2D* PhysXEngine::extractFromTriangleMesh(const PxTriangleMesh* mesh)
{
    const PxVec3* vertices = mesh->getVertices();
    PxU32 numTris = mesh->getNbTriangles();

    const void* triData = mesh->getTriangles();
    bool has16bit = mesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;

    tMatrix2D* mat = Matrix2D_Create();
    Matrix2D_Set_Size(mat, 3, numTris * 3);

    for (PxU32 i = 0; i < numTris; ++i) {
        PxU32 idx[3];

        if (has16bit) {
            const PxU16* tris = reinterpret_cast<const PxU16*>(triData);
            idx[0] = tris[i * 3 + 0];
            idx[1] = tris[i * 3 + 1];
            idx[2] = tris[i * 3 + 2];
        } else {
            const PxU32* tris = reinterpret_cast<const PxU32*>(triData);
            idx[0] = tris[i * 3 + 0];
            idx[1] = tris[i * 3 + 1];
            idx[2] = tris[i * 3 + 2];
        }

        for (int j = 0; j < 3; ++j) {
            const PxVec3& v = vertices[idx[j]];
            int col = i * 3 + j;
            // Convert from PhysX to STL coordinate system for RoboDK
            PxVec3 stlVertex = convertPhysXToSTL(v);
            Matrix2D_Set_ij(mat, 0, col, stlVertex.x);
            Matrix2D_Set_ij(mat, 1, col, stlVertex.y);
            Matrix2D_Set_ij(mat, 2, col, stlVertex.z);
        }
    }

    return mat;
}

tMatrix2D* PhysXEngine::extractFromConvexMesh(const PxConvexMesh* mesh)
{
    const PxVec3* vertices = mesh->getVertices();
    PxU32 numVertices = mesh->getNbVertices();

    // Get the polygon data
    const PxU8* indexBuffer = mesh->getIndexBuffer();
    PxU32 numPolygons = mesh->getNbPolygons();

    // Count total triangles
    PxU32 totalTriangles = 0;
    for (PxU32 i = 0; i < numPolygons; ++i) {
        PxHullPolygon polygon;
        mesh->getPolygonData(i, polygon);
        totalTriangles += polygon.mNbVerts - 2; // Each polygon with n vertices creates n-2 triangles
    }

    tMatrix2D* mat = Matrix2D_Create();
    Matrix2D_Set_Size(mat, 3, totalTriangles * 3);

    PxU32 triangleIndex = 0;
    for (PxU32 i = 0; i < numPolygons; ++i) {
        PxHullPolygon polygon;
        mesh->getPolygonData(i, polygon);

        // Triangulate the polygon (simple fan triangulation)
        for (PxU32 j = 1; j < polygon.mNbVerts - 1; ++j) {
            PxU32 idx0 = polygon.mIndexBase;
            PxU32 idx1 = polygon.mIndexBase + j;
            PxU32 idx2 = polygon.mIndexBase + j + 1;

            // Get vertex indices - assume 16-bit indices for convex meshes
            const PxU16* indices = reinterpret_cast<const PxU16*>(indexBuffer);
            PxU32 v0 = indices[idx0];
            PxU32 v1 = indices[idx1];
            PxU32 v2 = indices[idx2];

            // Add triangle vertices - convert from PhysX to STL coordinate system for RoboDK
            int col = triangleIndex * 3;
            PxVec3 stlVertex0 = convertPhysXToSTL(vertices[v0]);
            PxVec3 stlVertex1 = convertPhysXToSTL(vertices[v1]);
            PxVec3 stlVertex2 = convertPhysXToSTL(vertices[v2]);

            Matrix2D_Set_ij(mat, 0, col, stlVertex0.x);
            Matrix2D_Set_ij(mat, 1, col, stlVertex0.y);
            Matrix2D_Set_ij(mat, 2, col, stlVertex0.z);

            Matrix2D_Set_ij(mat, 0, col + 1, stlVertex1.x);
            Matrix2D_Set_ij(mat, 1, col + 1, stlVertex1.y);
            Matrix2D_Set_ij(mat, 2, col + 1, stlVertex1.z);

            Matrix2D_Set_ij(mat, 0, col + 2, stlVertex2.x);
            Matrix2D_Set_ij(mat, 1, col + 2, stlVertex2.y);
            Matrix2D_Set_ij(mat, 2, col + 2, stlVertex2.z);

            triangleIndex++;
        }
    }

    return mat;
}

tMatrix2D* PhysXEngine::extractShapeMesh(const PxShape* shape)
{
    PxGeometryHolder geom = shape->getGeometry();
    switch (geom.getType()) {
        case PxGeometryType::eTRIANGLEMESH:
            return extractFromTriangleMesh(geom.triangleMesh().triangleMesh);

        case PxGeometryType::eBOX: {
            const PxBoxGeometry& box = geom.box();
            return createBoxMesh(box.halfExtents);
        }

        case PxGeometryType::eCONVEXMESH: {
            const PxConvexMeshGeometry& convex = geom.convexMesh();
            return extractFromConvexMesh(convex.convexMesh);
        }

        default:
            printf("extractShapeMesh(): Unsupported geometry type %d", (int)geom.getType());
            return nullptr;
    }
}

bool LoadBinarySTL(const std::string& filename, std::vector<PxVec3>& outVertices, std::vector<PxU32>& outIndices)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file) return false;

    file.seekg(80);
    uint32_t numTriangles;
    file.read(reinterpret_cast<char*>(&numTriangles), 4);

    outVertices.clear();
    outIndices.clear();
    outVertices.reserve(numTriangles * 3);
    outIndices.reserve(numTriangles * 3);

    for (uint32_t i = 0; i < numTriangles; ++i) {
        float normal[3], v[9];
        uint16_t attr;

        file.read(reinterpret_cast<char*>(normal), 12);
        file.read(reinterpret_cast<char*>(v), 36);
        file.read(reinterpret_cast<char*>(&attr), 2);

        for (int j = 0; j < 3; ++j) {
            PxVec3 vertex(v[j * 3], v[j * 3 + 1], v[j * 3 + 2]);
            outVertices.push_back(vertex);
            outIndices.push_back(static_cast<PxU32>(outVertices.size() - 1));
        }
    }
    return true;
}

PxConvexMesh* CreateConvexMesh(PxPhysics* physics, const std::vector<PxVec3>& vertices, const PxCookingParams& cookingParams)
{
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = static_cast<PxU32>(vertices.size());
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = vertices.data();
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxDefaultMemoryOutputStream buf;

    if (!PxCookConvexMesh(cookingParams, convexDesc, buf)) {
        return nullptr;
    }

    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    return physics->createConvexMesh(input);
}

PxTriangleMesh* CreateTriangleMesh(PxPhysics* physics, const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices, const PxCookingParams& cookingParams) {
    PxTriangleMeshDesc meshDesc;
    meshDesc.points.count     = static_cast<PxU32>(vertices.size());
    meshDesc.points.stride    = sizeof(PxVec3);
    meshDesc.points.data      = vertices.data();

    meshDesc.triangles.count  = static_cast<PxU32>(indices.size() / 3);
    meshDesc.triangles.stride = 3 * sizeof(PxU32);
    meshDesc.triangles.data   = indices.data();

    PxDefaultMemoryOutputStream writeBuffer;

    if (!PxCookTriangleMesh(cookingParams, meshDesc, writeBuffer)) {
        return nullptr;
    }

    PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
    return physics->createTriangleMesh(readBuffer);
}

std::vector<PxConvexMesh*> CreateVHACDConvexMeshes(PxPhysics* physics, const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices, const PxCookingParams& cookingParams, bool isRobotMesh)
{
    std::vector<PxConvexMesh*> convexMeshes;
    
    if (vertices.empty() || indices.empty()) {
        qDebug() << "VHACD: Empty mesh data";
        return convexMeshes;
    }

    try {
        // Convert PhysX vertices to VHACD format
        std::vector<VHACD::Vertex> vhacdVertices;
        vhacdVertices.reserve(vertices.size());
        
        for (const PxVec3& vertex : vertices) {
            // Convert from PhysX coordinate system to VHACD coordinate system
            // PhysX: (X,Z,Y) -> VHACD: (X,Y,Z)
            vhacdVertices.push_back(VHACD::Vertex(vertex.x, vertex.z, vertex.y));
        }

        // Convert PhysX indices to VHACD format
        std::vector<VHACD::Triangle> vhacdTriangles;
        vhacdTriangles.reserve(indices.size() / 3);
        
        for (size_t i = 0; i < indices.size(); i += 3) {
            if (i + 2 < indices.size()) {
                vhacdTriangles.push_back(VHACD::Triangle(indices[i], indices[i + 1], indices[i + 2]));
            }
        }

        // Create VHACD instance
        VHACD::IVHACD* vhacd = VHACD::CreateVHACD();
        if (!vhacd) {
            qDebug() << "VHACD: Failed to create VHACD instance";
            return convexMeshes;
        }

        // Set VHACD parameters - use more detailed settings for robot meshes
        VHACD::IVHACD::Parameters params;
        if (isRobotMesh) {
            // Balanced settings for robot meshes - good detail without excessive computation
            params.m_maxConvexHulls = 8192*8;  // Good detail without overwhelming computation
            params.m_resolution = 500000; // High but reasonable voxel resolution
            params.m_minimumVolumePercentErrorAllowed = 0.000001; // Good fit without excessive precision
            params.m_maxRecursionDepth = 12; // Deep enough for complex shapes
            params.m_maxNumVerticesPerCH = 256; // Good vertex count for detailed hulls
            params.m_minEdgeLength = 0.0005; // Preserve important details
            qDebug() << "VHACD: Using balanced settings for robot mesh";
        } else {
            // Standard detailed settings for regular objects
            params.m_maxConvexHulls = 16;  // Increased from 8 to 16 for more detailed decomposition
            params.m_resolution = 200000; // Increased from 100000 to 200000 for higher voxel resolution
            params.m_minimumVolumePercentErrorAllowed = 0.5; // Reduced from 1.0 to 0.5 for tighter fit
            params.m_maxRecursionDepth = 15; // Increased from 10 to 15 for deeper decomposition
            params.m_maxNumVerticesPerCH = 128; // Increased from 64 to 128 for more detailed hulls
            params.m_minEdgeLength = 1; // Reduced from 2 to 1 for finer detail preservation
            qDebug() << "VHACD: Using detailed settings for regular object";
        }
        
        params.m_shrinkWrap = true; // Shrinkwrap voxel positions to source mesh
        params.m_fillMode = VHACD::FillMode::FLOOD_FILL; // How to fill interior
        params.m_asyncACD = false; // Run synchronously for better control
        params.m_findBestPlane = false; // Experimental feature, keep false
        params.m_callback = nullptr;
        params.m_logger = nullptr;
        params.m_taskRunner = nullptr;

        // Compute convex decomposition
        bool success = vhacd->Compute(&vhacdVertices[0].mX, 
                                     vhacdVertices.size(), 
                                     &vhacdTriangles[0].mI0, 
                                     vhacdTriangles.size(), 
                                     params);

        if (success) {
            uint32_t numHulls = vhacd->GetNConvexHulls();
            qDebug() << "VHACD: Generated" << numHulls << "convex hulls";

            // Process each convex hull
            for (uint32_t hullIndex = 0; hullIndex < numHulls; ++hullIndex) {
                VHACD::IVHACD::ConvexHull hull;
                if (vhacd->GetConvexHull(hullIndex, hull)) {
                    if (hull.m_points.size() >= 4) { // Need at least 4 points for a convex hull
                        // Convert VHACD vertices back to PhysX format
                        std::vector<PxVec3> hullVertices;
                        hullVertices.reserve(hull.m_points.size());
                        
                        for (const VHACD::Vertex& vertex : hull.m_points) {
                            // Convert from VHACD coordinate system to PhysX coordinate system
                            // VHACD: (X,Y,Z) -> PhysX: (X,Z,Y)
                            hullVertices.push_back(PxVec3(vertex.mX, vertex.mZ, vertex.mY));
                        }

                        // Create PhysX convex mesh from the hull vertices
                        PxConvexMesh* convexMesh = CreateConvexMesh(physics, hullVertices, cookingParams);
                        if (convexMesh) {
                            convexMeshes.push_back(convexMesh);
                            qDebug() << "VHACD: Created convex mesh" << hullIndex << "with" << hullVertices.size() << "vertices";
                        } else {
                            qDebug() << "VHACD: Failed to create convex mesh for hull" << hullIndex;
                        }
                    } else {
                        qDebug() << "VHACD: Hull" << hullIndex << "has insufficient vertices:" << hull.m_points.size();
                    }
                }
            }
        } else {
            qDebug() << "VHACD: Decomposition failed";
        }

        // Clean up VHACD
        vhacd->Clean();
        vhacd->Release();

    } catch (const std::exception& e) {
        qDebug() << "VHACD: Exception during decomposition:" << e.what();
    } catch (...) {
        qDebug() << "VHACD: Unknown exception during decomposition";
    }

    return convexMeshes;
}

void PhysXEngine::setRoboDK(RoboDK* rdk) {
    m_rdk = rdk;
}

RoboDK* PhysXEngine::getRoboDK() {
    return m_rdk;
}

void PhysXEngine::setLoadingDone(bool done) {
    m_isLoadingDone = done;
}

bool PhysXEngine::isLoadingDone() {
    return m_isLoadingDone;
}

void PhysXEngine::setStackZ(PxReal z) {
    m_stackZ = z;
}

PxReal PhysXEngine::getStackZ() {
    return m_stackZ;
}

// Configuration methods for SceneConfigurationDialog
void PhysXEngine::setSolverIterations(int positionIterations, int velocityIterations)
{
    m_solverPositionIterations = positionIterations;
    m_solverVelocityIterations = velocityIterations;
}

void PhysXEngine::getSolverIterations(int& positionIterations, int& velocityIterations) const
{
    positionIterations = m_solverPositionIterations;
    velocityIterations = m_solverVelocityIterations;
}

void PhysXEngine::setGlobalMaterialProperties(float staticFriction, float dynamicFriction, float restitution)
{
    m_globalStaticFriction = staticFriction;
    m_globalDynamicFriction = dynamicFriction;
    m_globalRestitution = restitution;

    if (m_material) {
        m_material->setStaticFriction(staticFriction);
        m_material->setDynamicFriction(dynamicFriction);
        m_material->setRestitution(restitution);
    }
}

void PhysXEngine::getGlobalMaterialProperties(float& staticFriction, float& dynamicFriction, float& restitution) const
{
    staticFriction = m_globalStaticFriction;
    dynamicFriction = m_globalDynamicFriction;
    restitution = m_globalRestitution;
}

bool PhysXEngine::setObjectMaterial(Item item, float staticFriction, float dynamicFriction, float restitution)
{
    if (isSoftBodyInSimulation(item)) {
        return false;
    }

    auto it = m_objects.find(item);
    if (it == m_objects.end()) {
        return false;
    }

    PhysXObjectData& data = it->second;
    if (!data.actor) {
        return false;
    }

    PxShape* shape = nullptr;
    data.actor->getShapes(&shape, 1);

    if (!shape || !shape->isExclusive()) {
        return false;
    }

    PxMaterial* newMaterial = m_physics->createMaterial(staticFriction, dynamicFriction, restitution);
    if (!newMaterial) {
        return false;
    }

    PxGeometryHolder geom = shape->getGeometry();
    PxShape* newShape = m_physics->createShape(geom.any(), *newMaterial);
    if (!newShape) {
        newMaterial->release();
        return false;
    }

    newShape->setFlags(shape->getFlags());
    newShape->setContactOffset(shape->getContactOffset());
    newShape->setRestOffset(shape->getRestOffset());

    data.actor->detachShape(*shape);
    data.actor->attachShape(*newShape);

    shape->release();
    newMaterial->release();

    return true;
}

void PhysXEngine::setPCMEnabled(bool enabled)
{
    m_pcmEnabled = enabled;
}

bool PhysXEngine::isPCMEnabled() const
{
    return m_pcmEnabled;
}

void PhysXEngine::setStabilizationEnabled(bool enabled)
{
    m_stabilizationEnabled = enabled;
}

bool PhysXEngine::isStabilizationEnabled() const
{
    return m_stabilizationEnabled;
}

void PhysXEngine::setContactOffset(float offset)
{
    m_contactOffset = offset;
}

float PhysXEngine::getContactOffset() const
{
    return m_contactOffset;
}

void PhysXEngine::setRestOffset(float offset)
{
    m_restOffset = offset;
}

float PhysXEngine::getRestOffset() const
{
    return m_restOffset;
}

void PhysXEngine::setSleepThreshold(float threshold)
{
    m_sleepThreshold = threshold;
}

float PhysXEngine::getSleepThreshold() const
{
    return m_sleepThreshold;
}

void PhysXEngine::setStabilizationThreshold(float threshold)
{
    m_stabilizationThreshold = threshold;
}

float PhysXEngine::getStabilizationThreshold() const
{
    return m_stabilizationThreshold;
}

void PhysXEngine::setCCDEnabled(bool enabled)
{
    m_ccdEnabled = enabled;
}

bool PhysXEngine::isCCDEnabled() const
{
    return m_ccdEnabled;
}

void PhysXEngine::setWakeDistance(float distance)
{
    m_wakeDistance = distance;
}

float PhysXEngine::getWakeDistance() const
{
    return m_wakeDistance;
}

void PhysXEngine::initializeDefaults()
{
    m_gravity = PxVec3(0.0f, -9806.65f, 0.0f);
    m_debugVisualizationEnabled = false;
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
    m_ccdEnabled = false;
    m_wakeDistance = 1000.0f;
}

void PhysXEngine::wakeUpNearbyDynamicObjects(PxRigidDynamic* kinematicActor, const PxTransform& newPose)
{
    if (!m_scene || !kinematicActor) return;

    PxActorTypeFlags actorTypes = PxActorTypeFlag::eRIGID_DYNAMIC;
    PxU32 nbActors = m_scene->getNbActors(actorTypes);
    std::vector<PxActor*> actors(nbActors);
    m_scene->getActors(actorTypes, actors.data(), nbActors);

    int wokenCount = 0;
    PxVec3 kinematicPos = newPose.p;
    float wakeDistanceSq = m_wakeDistance * m_wakeDistance;

    for (PxActor* otherActor : actors) {
        PxRigidDynamic* otherDynamic = otherActor->is<PxRigidDynamic>();
        if (otherDynamic && !(otherDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
            if (otherDynamic->isSleeping()) {
                PxTransform otherPose = otherDynamic->getGlobalPose();
                PxVec3 otherPos = otherPose.p;
                PxVec3 distanceVec = kinematicPos - otherPos;
                float distanceSq = distanceVec.magnitudeSquared();

                if (distanceSq <= wakeDistanceSq) {
                otherDynamic->wakeUp();
                wokenCount++;
                }
            }
        }
    }
}

void PhysXEngine::validateAndRemoveInvalidObjects()
{
    if (!getRoboDK()) return;

    std::vector<Item> objectsToRemove;

    for (const auto& pair : m_objects) {
        Item item = pair.first;
        if (!item || !getRoboDK()->Valid(item)) {
            objectsToRemove.push_back(item);
        }
    }

    for (Item item : objectsToRemove) {
        removeObject(item);
    }

    std::vector<Item> robotsToRemove;

    for (const auto& pair : m_robots) {
        Item robot = pair.first;
        if (!robot || !getRoboDK()->Valid(robot)) {
            robotsToRemove.push_back(robot);
        }
    }

    for (Item robot : robotsToRemove) {
        removeRobot(robot);
    }

    std::vector<Item> softBodiesToRemove;

    for (const auto& pair : m_softBodies) {
        Item item = pair.first;
        if (!item || !getRoboDK()->Valid(item)) {
            softBodiesToRemove.push_back(item);
        }
    }

    for (Item item : softBodiesToRemove) {
        removeSoftBody(item);
    }
}

// Debug visualization methods
void PhysXEngine::renderDebugGeometry()
{
    if (!getRoboDK()) {
        return;
    }

    // Always draw deformable mesh wireframes (regardless of debug visualization setting)
    drawDeformableMeshes();

    if (!m_debugVisualizationEnabled) {
        //SKip the rest if it's not enabled
        return;
    }

    // Draw collision geometry for all objects in simulation
    drawCollisionGeometry();


    // Draw center of mass for objects with custom properties
    drawCenterOfMass();

    // Draw ground plane if it exists
    if (m_groundPlane) {
        drawGroundPlane();
    }

    // Draw collision bounds for debugging collision detection
    drawCollisionBounds();
    
    // Draw attachment points for soft bodies
    drawSoftBodyAttachmentPoints();
}

void PhysXEngine::drawCollisionGeometry()
{
    if (!m_scene || !getRoboDK()) return;

    // Get all actors in the scene
    PxActorTypeFlags actorTypes = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
    PxU32 nbActors = m_scene->getNbActors(actorTypes);
    std::vector<PxActor*> actors(nbActors);
    m_scene->getActors(actorTypes, actors.data(), nbActors);

    // Draw geometry for each actor
    for (PxActor* actor : actors) {
        PxRigidActor* rigidActor = actor->is<PxRigidActor>();
        if (rigidActor) {
            PxTransform pose = rigidActor->getGlobalPose();
            drawActorGeometry(rigidActor, pose);
        }
    }

    // Draw robot joint geometry with special color
    for (auto& pair : m_robots) {
        const RobotData& robotData = pair.second;
        for (auto& jointPair : robotData.jointActors) {
            PxRigidDynamic* actor = jointPair.second;
            if (actor) {
                PxTransform pose = actor->getGlobalPose();
                drawActorGeometry(actor, pose, false, true); // isSleeping=false, isRobot=true
            }
        }
    }

    // Draw soft body geometry
    for (auto& pair : m_softBodies) {
        const SoftBodyData& softBodyData = pair.second;
        if (softBodyData.softBody) {
                            drawDeformableGeometry(softBodyData.softBody, softBodyData.initialTransform, false, false); // isSleeping=false, drawCollisionMesh=false
        }
    }
}

// Always draw deformable mesh wireframes (regardless of debug visualization setting)
void PhysXEngine::drawDeformableMeshes()
{
    if (!getRoboDK()) return;

    // Draw deformable mesh wireframes for all soft bodies in simulation
    for (auto& pair : m_softBodies) {
        const SoftBodyData& softBodyData = pair.second;
        if (softBodyData.softBody && softBodyData.isInSimulation) {
            drawDeformableGeometry(softBodyData.softBody, softBodyData.initialTransform, false, false);
        }
    }
}

void PhysXEngine::drawActorGeometry(PxRigidActor* actor, const PxTransform& pose, bool isSleeping, bool isRobot)
{
    if (!actor || !getRoboDK()) return;

    // Check if this is a dynamic actor and if it's sleeping (only for non-robot actors)
    if (!isRobot) {
        PxRigidDynamic* dynamicActor = actor->is<PxRigidDynamic>();
        if (dynamicActor) {
            isSleeping = dynamicActor->isSleeping();
        }
    }

    // Get all shapes from the actor
    PxU32 nbShapes = actor->getNbShapes();
    std::vector<PxShape*> shapes(nbShapes);
    actor->getShapes(shapes.data(), nbShapes);

    // Draw each shape
    for (PxShape* shape : shapes) {
        drawShapeGeometry(shape, pose, isSleeping, isRobot);
    }
}

void PhysXEngine::drawShapeGeometry(const PxShape* shape, const PxTransform& pose, bool isSleeping, bool isRobot)
{
    if (!shape || !getRoboDK()) return;

    PxGeometryHolder geom = shape->getGeometry();

    switch (geom.getType()) {
        case PxGeometryType::eTRIANGLEMESH:
            drawTriangleMesh(geom.triangleMesh().triangleMesh, pose, isSleeping, isRobot);
            break;

        case PxGeometryType::eCONVEXMESH:
            drawConvexMesh(geom.convexMesh().convexMesh, pose, isSleeping, isRobot);
            break;

        case PxGeometryType::eBOX:
            drawBoxGeometry(geom.box(), pose, isSleeping, isRobot);
            break;

        default:
            // Unsupported geometry type
            break;
    }
}

void PhysXEngine::drawTriangleMesh(const PxTriangleMesh* mesh, const PxTransform& pose, bool isSleeping, bool isRobot)
{
    if (!mesh || !getRoboDK()) return;

    if (!isValidTransform(pose)) {
        qDebug() << "PhysXEngine: Skipping triangle mesh visualization - invalid pose";
        return;
    }

    const PxVec3* vertices = mesh->getVertices();
    PxU32 numTris = mesh->getNbTriangles();
    const void* triData = mesh->getTriangles();
    bool has16bit = mesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;

    std::vector<float> vertexArray;
    vertexArray.reserve(numTris * 9);

    // Choose color based on sleep state and robot status
    float color[4];
    if (isRobot) {
        // Purple for robot joints
        color[0] = 0.8f; color[1] = 0.0f; color[2] = 0.8f; color[3] = 0.7f;
    } else if (isSleeping) {
        // Yellow for sleeping objects
        color[0] = 1.0f; color[1] = 1.0f; color[2] = 0.0f; color[3] = 0.5f;
    } else {
        // Blue for awake objects
        color[0] = 0.0f; color[1] = 0.0f; color[2] = 1.0f; color[3] = 0.5f;
    }

    for (PxU32 i = 0; i < numTris; ++i) {
        PxU32 idx[3];
        if (has16bit) {
            const PxU16* tris = reinterpret_cast<const PxU16*>(triData);
            idx[0] = tris[i * 3 + 0];
            idx[1] = tris[i * 3 + 1];
            idx[2] = tris[i * 3 + 2];
        } else {
            const PxU32* tris = reinterpret_cast<const PxU32*>(triData);
            idx[0] = tris[i * 3 + 0];
            idx[1] = tris[i * 3 + 1];
            idx[2] = tris[i * 3 + 2];
        }
        for (int j = 0; j < 3; ++j) {
            const PxVec3& physxVertex = vertices[idx[j]];
            if (!isValidVector(physxVertex)) {
                qDebug() << "PhysXEngine: Skipping invalid vertex in triangle mesh, index:" << idx[j];
                continue;
            }
            PxVec3 transformedVertex;
            try {
                transformedVertex = pose.transform(physxVertex);
                if (!isValidVector(transformedVertex)) {
                    qDebug() << "PhysXEngine: Skipping invalid transformed vertex. Orig:" << physxVertex.x << physxVertex.y << physxVertex.z << "Transformed:" << transformedVertex.x << transformedVertex.y << transformedVertex.z;
                    continue;
                }
            } catch (...) {
                qDebug() << "PhysXEngine: Exception during vertex transformation, skipping. Orig:" << physxVertex.x << physxVertex.y << physxVertex.z;
                continue;
            }
            PxVec3 transformedStlVertex = convertPhysXToSTL(transformedVertex);
            if (!isValidVector(transformedStlVertex)) {
                qDebug() << "PhysXEngine: Skipping invalid STL vertex. Transformed:" << transformedVertex.x << transformedVertex.y << transformedVertex.z << "STL:" << transformedStlVertex.x << transformedStlVertex.y << transformedStlVertex.z;
                continue;
            }
            vertexArray.push_back(transformedStlVertex.x);
            vertexArray.push_back(transformedStlVertex.y);
            vertexArray.push_back(transformedStlVertex.z);
        }
    }
    if (!vertexArray.empty()) {
        getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, vertexArray.data(), numTris, color);
    }
}

void PhysXEngine::drawConvexMesh(const PxConvexMesh* mesh, const PxTransform& pose, bool isSleeping, bool isRobot)
{
    if (!mesh || !getRoboDK()) return;
    if (!isValidTransform(pose)) {
        qDebug() << "PhysXEngine: Skipping convex mesh visualization - invalid pose";
        return;
    }
    const PxVec3* vertices = mesh->getVertices();
    PxU32 numPolygons = mesh->getNbPolygons();
    const PxU8* indexBuffer = mesh->getIndexBuffer();
    std::vector<float> vertexArray;

    // Choose color based on sleep state and robot status
    float color[4];
    if (isRobot) {
        // Purple for robot joints
        color[0] = 0.8f; color[1] = 0.0f; color[2] = 0.8f; color[3] = 0.7f;
    } else if (isSleeping) {
        // Yellow for sleeping objects
        color[0] = 1.0f; color[1] = 1.0f; color[2] = 0.0f; color[3] = 0.7f;
    } else {
        // Green for awake objects
        color[0] = 0.0f; color[1] = 1.0f; color[2] = 0.0f; color[3] = 0.7f;
    }

    PxU32 totalTriangles = 0;
    for (PxU32 i = 0; i < numPolygons; ++i) {
        PxHullPolygon polygon;
        mesh->getPolygonData(i, polygon);
        totalTriangles += polygon.mNbVerts - 2;
    }
    vertexArray.reserve(totalTriangles * 9);
    for (PxU32 i = 0; i < numPolygons; ++i) {
        PxHullPolygon polygon;
        mesh->getPolygonData(i, polygon);
        // PhysX convex mesh index buffer can be 8 or 16 bit. Use mesh->getIndexBufferSize() to check.
        bool use16bit = mesh->getNbVertices() > 255;
        for (PxU32 j = 1; j < polygon.mNbVerts - 1; ++j) {
            PxU32 idx0 = polygon.mIndexBase;
            PxU32 idx1 = polygon.mIndexBase + j;
            PxU32 idx2 = polygon.mIndexBase + j + 1;
            PxU32 v0, v1, v2;
            if (use16bit) {
                const PxU16* indices = reinterpret_cast<const PxU16*>(indexBuffer);
                v0 = indices[idx0];
                v1 = indices[idx1];
                v2 = indices[idx2];
            } else {
                const PxU8* indices = reinterpret_cast<const PxU8*>(indexBuffer);
                v0 = indices[idx0];
                v1 = indices[idx1];
                v2 = indices[idx2];
            }
            for (int k = 0; k < 3; ++k) {
                PxU32 vertexIndex = (k == 0) ? v0 : ((k == 1) ? v1 : v2);
                if (vertexIndex >= mesh->getNbVertices()) {
                    qDebug() << "PhysXEngine: Convex mesh vertex index out of bounds:" << vertexIndex << "(max:" << mesh->getNbVertices() << ")";
                    continue;
                }
                const PxVec3& physxVertex = vertices[vertexIndex];
                if (!isValidVector(physxVertex)) {
                    qDebug() << "PhysXEngine: Skipping invalid vertex in convex mesh, index:" << vertexIndex;
                    continue;
                }
                PxVec3 transformedVertex;
                try {
                    transformedVertex = pose.transform(physxVertex);
                    if (!isValidVector(transformedVertex)) {
                        qDebug() << "PhysXEngine: Skipping invalid transformed vertex in convex mesh. Orig:" << physxVertex.x << physxVertex.y << physxVertex.z << "Transformed:" << transformedVertex.x << transformedVertex.y << transformedVertex.z;
                        continue;
                    }
                } catch (...) {
                    qDebug() << "PhysXEngine: Exception during vertex transformation in convex mesh, skipping. Orig:" << physxVertex.x << physxVertex.y << physxVertex.z;
                    continue;
                }
                PxVec3 transformedStlVertex = convertPhysXToSTL(transformedVertex);
                if (!isValidVector(transformedStlVertex)) {
                    qDebug() << "PhysXEngine: Skipping invalid STL vertex in convex mesh. Transformed:" << transformedVertex.x << transformedVertex.y << transformedVertex.z << "STL:" << transformedStlVertex.x << transformedStlVertex.y << transformedStlVertex.z;
                    continue;
                }
                vertexArray.push_back(transformedStlVertex.x);
                vertexArray.push_back(transformedStlVertex.y);
                vertexArray.push_back(transformedStlVertex.z);
            }
        }
    }
    if (!vertexArray.empty()) {
        getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, vertexArray.data(), totalTriangles, color);
    }
}

void PhysXEngine::drawBoxGeometry(const PxBoxGeometry& box, const PxTransform& pose, bool isSleeping, bool isRobot)
{
    if (!getRoboDK()) return;

    // Validate pose to prevent crashes
    if (!isValidTransform(pose)) {
        qDebug() << "PhysXEngine: Skipping box geometry visualization - invalid pose";
        return;
    }

    // Validate box geometry
    if (!isValidVector(box.halfExtents)) {
        qDebug() << "PhysXEngine: Skipping box geometry visualization - invalid half extents";
        return;
    }

    // Create box vertices
    PxVec3 halfExtents = box.halfExtents;
    PxVec3 vertices[8] = {
        {-halfExtents.x, -halfExtents.y, -halfExtents.z},
        { halfExtents.x, -halfExtents.y, -halfExtents.z},
        { halfExtents.x,  halfExtents.y, -halfExtents.z},
        {-halfExtents.x,  halfExtents.y, -halfExtents.z},
        {-halfExtents.x, -halfExtents.y,  halfExtents.z},
        { halfExtents.x, -halfExtents.y,  halfExtents.z},
        { halfExtents.x,  halfExtents.y,  halfExtents.z},
        {-halfExtents.x,  halfExtents.y,  halfExtents.z}
    };

    // Define box faces (12 triangles)
    int faces[12][3] = {
        {0,1,2}, {0,2,3}, // Bottom
        {4,6,5}, {4,7,6}, // Top
        {0,4,5}, {0,5,1}, // Front
        {1,5,6}, {1,6,2}, // Right
        {2,6,7}, {2,7,3}, // Back
        {3,7,4}, {3,4,0}  // Left
    };

    // Prepare vertex array for RoboDK (12 triangles * 3 vertices * 3 floats)
    std::vector<float> vertexArray;
    vertexArray.reserve(12 * 9); // 12 triangles * 3 vertices * 3 floats per vertex

    // Choose color based on sleep state and robot status
    float color[4];
    if (isRobot) {
        // Purple for robot joints
        color[0] = 0.8f; color[1] = 0.0f; color[2] = 0.8f; color[3] = 0.8f;
    } else if (isSleeping) {
        // Yellow for sleeping objects
        color[0] = 1.0f; color[1] = 1.0f; color[2] = 0.0f; color[3] = 0.8f;
    } else {
        // Red for awake objects
        color[0] = 1.0f; color[1] = 0.0f; color[2] = 0.0f; color[3] = 0.8f;
    }

    // Convert triangles to vertex array
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 3; ++j) {
            PxVec3 physxVertex = vertices[faces[i][j]];

            // Validate vertex before transformation
            if (!isValidVector(physxVertex)) {
                qDebug() << "PhysXEngine: Skipping invalid vertex in box geometry";
                continue;
            }

            // Transform vertex by pose with error checking
            PxVec3 transformedVertex;
            try {
                transformedVertex = pose.transform(physxVertex);
                if (!isValidVector(transformedVertex)) {
                    qDebug() << "PhysXEngine: Skipping invalid transformed vertex";
                    continue;
                }
            } catch (...) {
                qDebug() << "PhysXEngine: Exception during vertex transformation, skipping";
                continue;
            }

            PxVec3 transformedStlVertex = convertPhysXToSTL(transformedVertex);

            // Validate final vertex
            if (!isValidVector(transformedStlVertex)) {
                qDebug() << "PhysXEngine: Skipping invalid STL vertex";
                continue;
            }

            // Add vertex coordinates to array
            vertexArray.push_back(transformedStlVertex.x);
            vertexArray.push_back(transformedStlVertex.y);
            vertexArray.push_back(transformedStlVertex.z);
        }
    }

    // Draw triangles using RoboDK's DrawGeometry API
    if (!vertexArray.empty()) {
        getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, vertexArray.data(), 12, color);
    }
}

// Validation helper functions
bool PhysXEngine::isValidVector(const PxVec3& vec)
{
    bool valid = !std::isnan(vec.x) && !std::isnan(vec.y) && !std::isnan(vec.z) &&
                 !std::isinf(vec.x) && !std::isinf(vec.y) && !std::isinf(vec.z);
    if (!valid) {
        qDebug() << "PhysXEngine: Invalid PxVec3:" << vec.x << vec.y << vec.z;
    }
    return valid;
}

bool PhysXEngine::isValidTransform(const PxTransform& transform)
{
    // Check position
    if (!isValidVector(transform.p)) {
        return false;
    }

    // Check quaternion (should be unit length)
    const PxQuat& q = transform.q;
    if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w) ||
        std::isinf(q.x) || std::isinf(q.y) || std::isinf(q.z) || std::isinf(q.w)) {
        return false;
    }

    // Check if quaternion is approximately unit length (allowing for small numerical errors)
    float lengthSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (std::abs(lengthSq - 1.0f) > 0.1f) {
        return false;
    }

    return true;
}
void PhysXEngine::drawGroundPlane()
{
    if (!m_groundPlane || !getRoboDK()) return;

    // Get the ground plane pose
    PxTransform groundPose = m_groundPlane->getGlobalPose();

    // Validate ground pose to prevent crashes
    if (!isValidTransform(groundPose)) {
        qDebug() << "PhysXEngine: Skipping ground plane visualization - invalid pose";
        return;
    }

    // Create a large flat plane for visualization (not a box)
    // The ground plane is at Y=0 in PhysX coordinates, so we create a large flat surface
    const float size = 10000.0f; // 10 meters in each direction
    const float thickness = 1.0f; // 1mm thickness for visibility

    // Create a simple flat plane with 2 triangles (4 vertices)
    PxVec3 vertices[4] = {
        {-size, -thickness, -size},  // Bottom-left
        { size, -thickness, -size},  // Bottom-right
        { size, -thickness,  size},  // Top-right
        {-size, -thickness,  size}   // Top-left
    };

    // Define 2 triangles to form a flat plane
    int faces[2][3] = {
        {0, 1, 2}, // First triangle
        {0, 2, 3}  // Second triangle
    };

    // Prepare vertex array for RoboDK
    std::vector<float> vertexArray;
    vertexArray.reserve(6 * 3); // 2 triangles * 3 vertices * 3 floats per vertex

    // Prepare color array [R, G, B, A] for gray solid
    float color[4] = {0.7f, 0.7f, 0.7f, 0.8f}; // Light gray with 80% opacity

    // Convert triangles to vertex array
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j) {
            PxVec3 physxVertex = vertices[faces[i][j]];

            // Validate vertex before transformation
            if (!isValidVector(physxVertex)) {
                qDebug() << "PhysXEngine: Skipping invalid vertex in ground plane";
                continue;
            }

            // Transform vertex by pose with error checking
            PxVec3 transformedVertex;
            try {
                transformedVertex = groundPose.transform(physxVertex);
                if (!isValidVector(transformedVertex)) {
                    qDebug() << "PhysXEngine: Skipping invalid transformed vertex";
                    continue;
                }
            } catch (...) {
                qDebug() << "PhysXEngine: Exception during vertex transformation, skipping";
                continue;
            }

            PxVec3 transformedStlVertex = convertPhysXToSTL(transformedVertex);

            // Validate final vertex
            if (!isValidVector(transformedStlVertex)) {
                qDebug() << "PhysXEngine: Skipping invalid STL vertex";
                continue;
            }

            // Add vertex coordinates to array
            vertexArray.push_back(transformedStlVertex.x);
            vertexArray.push_back(transformedStlVertex.y);
            vertexArray.push_back(transformedStlVertex.z);
        }
    }

    // Draw triangles using RoboDK's DrawGeometry API
    if (!vertexArray.empty()) {
        getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, vertexArray.data(), 2, color);
    }
}

void PhysXEngine::drawCenterOfMass()
{
    if (!getRoboDK()) return;

    // Get all objects in simulation
    for (auto& pair : m_objects) {
        Item item = pair.first;
        PxRigidDynamic* actor = pair.second.actor ? pair.second.actor->is<PxRigidDynamic>() : nullptr;

        if (!item || !actor) continue;

        // Check if this object has custom center of mass enabled
        if (m_objectPropertiesManager && m_objectPropertiesManager->hasObjectProperties(item)) {
            if (m_objectPropertiesManager->getUseCustomCenterOfMass(item)) {
                PxVec3 centerOfMass = m_objectPropertiesManager->getObjectCenterOfMass(item);

                // Convert center of mass from RoboDK space to PhysX space
                PxVec3 centerOfMassPhysX = convertSTLToPhysX(centerOfMass);

                // Get the actor's global pose
                PxTransform actorPose = actor->getGlobalPose();

                // Transform the center of mass to world coordinates in PhysX space
                PxVec3 worldCenterOfMass = actorPose.transform(centerOfMassPhysX);

                // Convert from PhysX space to RoboDK space for visualization
                PxVec3 worldStlCenterOfMass = convertPhysXToSTL(worldCenterOfMass);

                // Calculate object size to make sphere proportional
                float objectSize = 0.0f;

                // Get the actor's bounds
                PxBounds3 bounds = actor->getWorldBounds();
                if (bounds.isValid()) {
                    PxVec3 extents = bounds.getExtents();
                    // Use the largest dimension as the object size
                    objectSize = std::max({extents.x, extents.y, extents.z});
                }

                // Calculate proportional sphere radius (5-20% of object size, minimum 2mm)
                float sphereRadius = 0.0f;
                if (objectSize > 0.0f) {
                    sphereRadius = std::max(2.0f, objectSize * 0.1f); // 10% of object size, minimum 2mm
                } else {
                    sphereRadius = 5.0f; // Fallback to 5mm if we can't determine object size
                }
                qDebug() << "Calculated sphere radius:" << sphereRadius << "mm";

                float color[4] = {1.0f, 0.0f, 0.0f, 0.8f}; // Red color

                // Create sphere vertices (simplified - just a few points for visualization)
                std::vector<float> sphereVertices;
                const int numPoints = 8;
                const float pi = 3.14159265359f;

                for (int i = 0; i < numPoints; ++i) {
                    float phi = (2.0f * pi * i) / numPoints;
                    for (int j = 0; j < numPoints; ++j) {
                        float theta = (pi * j) / numPoints;

                        float x = worldStlCenterOfMass.x + sphereRadius * sin(theta) * cos(phi);
                        float y = worldStlCenterOfMass.y + sphereRadius * sin(theta) * sin(phi);
                        float z = worldStlCenterOfMass.z + sphereRadius * cos(theta);

                        sphereVertices.push_back(x);
                        sphereVertices.push_back(y);
                        sphereVertices.push_back(z);
                    }
                }

                if (!sphereVertices.empty()) {
                    getRoboDK()->DrawGeometry(RoboDK::DrawSpheres, sphereVertices.data(), sphereVertices.size() / 3, color, sphereRadius);
                }
            }
        }
    }
}

void PhysXEngine::setObjectPropertiesManager(ObjectPropertiesManager* manager)
{
    m_objectPropertiesManager = manager;
}

// Soft body management methods
bool PhysXEngine::addSoftBody(Item item, const SoftBodyConfig& config)
{
    // Basic validation
    if (!item || !getRoboDK()->Valid(item) || !m_physics || !m_material || !m_scene) {
        qDebug() << "Cannot add soft body - invalid item or physics system not initialized";
        return false;
    }

    // Check if already in simulation
    if (m_softBodies.find(item) != m_softBodies.end()) {
        qDebug() << "Soft body already in PhysX simulation:" << item->Name();
        return true;
    }

    // Validate CUDA context exactly as in NVIDIA example
    if (!m_cudaContextManager || !m_cudaContextManager->contextIsValid()) {
        qDebug() << "Cannot create soft body - CUDA context manager not available";
        qDebug() << "The Deformable Volume feature is currently only supported on GPU";
        return false;
    }

    qDebug() << "Adding soft body to PhysX simulation:" << item->Name();

    // Get the current pose of the RoboDK item (world coordinates)
    Mat itemPose = item->PoseAbs();
    qDebug() << "Creating soft body for" << item->Name() << " (using exact NVIDIA example)";

    // --- Use exact NVIDIA example mesh generation ---
    PxArray<PxVec3> triVerts;
    PxArray<PxU32> triIndices;

    // Create a local copy of config that can be modified
    SoftBodyConfig localConfig = config;
    bool useExistingGeometry = localConfig.useExistingGeometry;

    if (useExistingGeometry) {
        // Extract geometry from the existing RoboDK item
        std::vector<PxVec3> vertices;
        std::vector<PxU32> indices;
        
        if (extractGeometryFromItem(item, vertices, indices)) {
            // Validate the extracted geometry
            if (vertices.size() < 4 || indices.size() < 12) {
                qDebug() << "Extracted geometry is too small for soft body simulation, falling back to default cube";
                qDebug() << "  Vertices:" << vertices.size() << "(minimum 4)";
                qDebug() << "  Triangles:" << indices.size() / 3 << "(minimum 4)";
                useExistingGeometry = false;
            } else {
                // Convert std::vector to PxArray for PhysX compatibility
                triVerts.clear();
                triIndices.clear();
                
                for (const PxVec3& vertex : vertices) {
                    triVerts.pushBack(vertex);
                }
                
                for (PxU32 index : indices) {
                    triIndices.pushBack(index);
                }
                
                qDebug() << "Using existing geometry from" << item->Name() << ":";
                qDebug() << "  Vertices:" << triVerts.size();
                qDebug() << "  Triangles:" << triIndices.size() / 3;
                qDebug() << "  Note: Using actual object geometry for soft body";
            }
        } else {
            qDebug() << "Failed to extract geometry from" << item->Name() << ", falling back to default cube";
            // Fall back to default cube geometry
            useExistingGeometry = false;
        }
    }
    
    // Define maxEdgeLength at function scope so it can be used for remeshing
    PxReal maxEdgeLength = 200.0f;  // Increased for reduced collision mesh complexity
    
    if (!useExistingGeometry) {
        // Use a reasonable cube shape for rubber simulation, with center at the bottom middle
        PxReal size = 500.0f;     // 100mm cube (reasonable size for testing)
        PxReal width = size;
        PxReal height = size;
        PxReal depth = size;
        // Center is at the bottom middle: (0, 0, -height*0.5)
        PxVec3 center(0.0f, 0.0f, -height * 0.5f);

        // Create a reasonable cube for rubber simulation
        triVerts.clear();
        triIndices.clear();
        
        // 8 vertices of a rectangular prism with bottom at Y=0
        triVerts.pushBack(PxVec3(-width * 0.5f, 0.0f, -depth * 0.5f));           // 0: bottom-back-left
        triVerts.pushBack(PxVec3( width * 0.5f, 0.0f, -depth * 0.5f));           // 1: bottom-back-right
        triVerts.pushBack(PxVec3( width * 0.5f, height, -depth * 0.5f));         // 2: top-back-right
        triVerts.pushBack(PxVec3(-width * 0.5f, height, -depth * 0.5f));         // 3: top-back-left
        triVerts.pushBack(PxVec3(-width * 0.5f, 0.0f,  depth * 0.5f));           // 4: bottom-front-left
        triVerts.pushBack(PxVec3( width * 0.5f, 0.0f,  depth * 0.5f));           // 5: bottom-front-right
        triVerts.pushBack(PxVec3( width * 0.5f, height,  depth * 0.5f));         // 6: top-front-right
        triVerts.pushBack(PxVec3(-width * 0.5f, height,  depth * 0.5f));         // 7: top-front-left

        // 12 triangles (2 per face, 6 faces) with proper winding order
        // Back face (facing -Z)
        triIndices.pushBack(0); triIndices.pushBack(1); triIndices.pushBack(2);
        triIndices.pushBack(0); triIndices.pushBack(2); triIndices.pushBack(3);
        // Front face (facing +Z)
        triIndices.pushBack(4); triIndices.pushBack(7); triIndices.pushBack(6);
        triIndices.pushBack(4); triIndices.pushBack(6); triIndices.pushBack(5);
        // Left face (facing -X)
        triIndices.pushBack(0); triIndices.pushBack(3); triIndices.pushBack(7);
        triIndices.pushBack(0); triIndices.pushBack(7); triIndices.pushBack(4);
        // Right face (facing +X)
        triIndices.pushBack(1); triIndices.pushBack(5); triIndices.pushBack(6);
        triIndices.pushBack(1); triIndices.pushBack(6); triIndices.pushBack(2);
        // Bottom face (facing -Y)
        triIndices.pushBack(0); triIndices.pushBack(4); triIndices.pushBack(5);
        triIndices.pushBack(0); triIndices.pushBack(5); triIndices.pushBack(1);
        // Top face (facing +Y)
        triIndices.pushBack(3); triIndices.pushBack(2); triIndices.pushBack(6);
        triIndices.pushBack(3); triIndices.pushBack(6); triIndices.pushBack(7);

        // Print initial mesh statistics
        qDebug() << "Initial cube mesh created:";
        qDebug() << "  Vertices:" << triVerts.size();
        qDebug() << "  Triangles:" << triIndices.size() / 3;
        qDebug() << "  Dimensions: " << size << "mm cube";
        qDebug() << "  Note: Reasonable cube shape for rubber simulation";
    }

    // Apply remeshing exactly as in NVIDIA example (after mesh creation)
    PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);

    // Print mesh statistics after remeshing
    qDebug() << "After remeshing (max edge length" << maxEdgeLength << "mm):";
    qDebug() << "  Vertices:" << triVerts.size();
    qDebug() << "  Triangles:" << triIndices.size() / 3;
    qDebug() << "  Note: Balanced mesh density for performance and collision detection";

    // --- Store debug mesh data immediately ---
    SoftBodyData softBodyData;
    softBodyData.config = localConfig;
    // Convert PxArray to std::vector for debug storage
    softBodyData.debugMeshVertices.assign(triVerts.begin(), triVerts.end());
    softBodyData.debugMeshIndices.assign(triIndices.begin(), triIndices.end());
    softBodyData.debugMeshPose = itemPose;
    softBodyData.hasDebugMesh = true;
    softBodyData.isInSimulation = false;
    m_softBodies[item] = softBodyData;

    // --- Use global cooking parameters for deformable volume mesh creation ---
    // --- Create the surface mesh struct exactly as in NVIDIA example ---
    PxSimpleTriangleMesh surfaceMesh;
    surfaceMesh.points.count = triVerts.size();
    surfaceMesh.points.data = triVerts.begin();
    surfaceMesh.triangles.count = triIndices.size() / 3;
    surfaceMesh.triangles.data = triIndices.begin();

    PxU32 numVoxelsAlongLongestAABBAxis = 6; 

    // --- Call createDeformableVolumeMesh exactly as in NVIDIA example ---
    PxDeformableVolumeMesh* deformableVolumeMesh = PxDeformableVolumeExt::createDeformableVolumeMesh(
        m_globalCookingParams,
        surfaceMesh,
        numVoxelsAlongLongestAABBAxis,
        m_physics->getPhysicsInsertionCallback()
    );
    PX_ASSERT(deformableVolumeMesh);
    if (!deformableVolumeMesh) {
        qDebug() << "Failed to create deformable volume mesh (cone)";
        return false;
    }

    // --- Create deformable volume exactly as in NVIDIA example ---
    if (!m_cudaContextManager) {
        qDebug() << "Failed to create deformable volume - CUDA context manager not available";
        deformableVolumeMesh->release();
        return false;
    }
    PxDeformableVolume* deformableVolume = m_physics->createDeformableVolume(*m_cudaContextManager);
    if (!deformableVolume) {
        qDebug() << "Failed to create deformable volume";
        qDebug() << "This could be due to GPU memory issues or invalid CUDA context";
        deformableVolumeMesh->release();
        return false;
    }

    // --- Create shape exactly as in NVIDIA example ---
    PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;
    
    // Ensure the shape is configured for proper collision detection
    // The key is to make sure the shape can generate contacts with rigid bodies
    
    // Convert from MPa to Pa for PhysX API - Use stiffer material for better collision response
    float youngsModulus = localConfig.youngsModulus * 1e6f; // Convert MPa to Pa
    float poissonsRatio = localConfig.poissonsRatio;
    float damping = localConfig.damping;

    
    // Debug collision mesh information
    PxTetrahedronMesh* collisionMesh = deformableVolumeMesh->getCollisionMesh();
    if (collisionMesh) {
        qDebug() << "Collision mesh created successfully:";
        qDebug() << "  Vertices:" << collisionMesh->getNbVertices();
        qDebug() << "  Tetrahedra:" << collisionMesh->getNbTetrahedrons();
        qDebug() << "  Note: Tetrahedron mesh for collision detection with rigid bodies";
    } else {
        qDebug() << "ERROR: Failed to get collision mesh from deformable volume mesh";
    }

    qDebug() << "=== Soft Body Material Properties ===";
    qDebug() << "Young's modulus (UI):" << config.youngsModulus << "MPa";
    qDebug() << "Young's modulus (PhysX):" << youngsModulus << "Pa";
    qDebug() << "Poisson's ratio:" << poissonsRatio;
    qDebug() << "Damping:" << damping;
    qDebug() << "Density:" << config.density << "g/mm³";
    qDebug() << "=== End Material Properties ===";

    // Create material with dynamic friction as third parameter
    PxDeformableVolumeMaterial* materialPtr = PxGetPhysics().createDeformableVolumeMaterial(youngsModulus, poissonsRatio, localConfig.friction);
    
    // Set damping separately
    if (materialPtr) {
        materialPtr->setDamping(damping);
        qDebug() << "Deformable volume material created with dynamic friction:" << localConfig.friction << "and damping:" << damping;
    }
    
    // Create geometry with proper collision configuration
    PxTetrahedronMeshGeometry geometry(deformableVolumeMesh->getCollisionMesh());
    
    // Ensure the geometry is properly configured for collision detection
    // This is critical for contact generation between deformable and rigid bodies
    
    // Create shape with enhanced collision detection flags
    PxShape* shape = m_physics->createShape(geometry, &materialPtr, 1, true, shapeFlags);
    
    // Ensure the shape is properly configured for collision detection with rigid bodies
    // This is critical for contact generation between deformable and rigid bodies
    if (shape) {
        deformableVolume->attachShape(*shape);
        // Enhanced filter data for soft bodies:
        // word0: object type (2 = soft body)
        // word1: attachment group (0 = none initially, will be set when attached)
        // word2: collision group (2 = soft body group)
        // word3: attachment surface ID (0 = none initially)
        shape->setSimulationFilterData(PxFilterData(2, 0, 2, 0));                
    } else {
        qDebug() << "Failed to create shape for deformable volume";
        deformableVolume->release();
        materialPtr->release();
        deformableVolumeMesh->release();
        return false;
    }

    // --- Attach simulation mesh exactly as in NVIDIA example ---
    deformableVolume->attachSimulationMesh(*deformableVolumeMesh->getSimulationMesh(), *deformableVolumeMesh->getDeformableVolumeAuxData());

    // --- Add to scene exactly as in NVIDIA example ---
    m_scene->addActor(*deformableVolume);
    
    // Debug: Verify the deformable volume was added to the scene
    qDebug() << "Deformable volume added to scene successfully";

    // --- Set up deformable volume exactly as in NVIDIA example addDeformableVolume function ---
    qDebug() << "Setting up deformable volume with proper GPU memory management...";

    // Use parameters optimized for better collision response with kinematic actors
    // Convert RoboDK pose to PhysX pose for proper positioning
    PxTransform transform = convertRoboDKPoseToPhysX_STL(itemPose);
    PxReal scaleF = 1.0f;
    PxReal density = localConfig.density;   // Use config density (should be reasonable for rubber)
    PxReal maxInvMassRatio = 1.25f;   // Increase max inverse mass ratio for more responsive mass distribution

    // Store the initial transform (for reference - not used for drawing since positioning is handled by PxDeformableVolumeExt::transform)
    softBodyData.initialTransform = transform;

    // Allocate and initialize pinned host buffers exactly as in NVIDIA example
    PxVec4* simPositionInvMassPinned = nullptr;
    PxVec4* simVelocityPinned = nullptr;
    PxVec4* collPositionInvMassPinned = nullptr;
    PxVec4* restPositionPinned = nullptr;

    PxDeformableVolumeExt::allocateAndInitializeHostMirror(*deformableVolume, m_cudaContextManager,
        simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);

    // Transform, update mass, and upload to GPU exactly as in NVIDIA example
    // This applies the transform to the vertex data directly, not to the actor's pose
    PxDeformableVolumeExt::transform(*deformableVolume, transform, scaleF,
        simPositionInvMassPinned, simVelocityPinned,
        collPositionInvMassPinned, restPositionPinned);
    
    qDebug() << "Soft body positioned at:" << transform.p.x << transform.p.y << transform.p.z;

    PxDeformableVolumeExt::updateMass(*deformableVolume, density, maxInvMassRatio, simPositionInvMassPinned);

    PxDeformableVolumeExt::copyToDevice(*deformableVolume, PxDeformableVolumeDataFlag::eALL,
        simPositionInvMassPinned, simVelocityPinned,
        collPositionInvMassPinned, restPositionPinned);

    // --- Store transformed collision mesh vertices for runtime access ---
    // Keep the collision mesh vertices available for visualization
    softBodyData.collPositionInvMassPinned = collPositionInvMassPinned;
    
    // --- Free setup pinned memory for simulation data (keep collision data) ---
    PX_EXT_PINNED_MEMORY_FREE(*m_cudaContextManager, simPositionInvMassPinned);
    PX_EXT_PINNED_MEMORY_FREE(*m_cudaContextManager, simVelocityPinned);
    PX_EXT_PINNED_MEMORY_FREE(*m_cudaContextManager, restPositionPinned);

    // Set flags exactly as in the NVIDIA example (AFTER pinned memory operations)
    deformableVolume->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, true);
    deformableVolume->setSolverIterationCounts(m_solverPositionIterations, m_solverVelocityIterations);  // Set both position and velocity iterations for better collision response
    
    // Additional solver configuration for better collision response
    qDebug() << "Soft body solver configuration:";
    qDebug() << "  Position iterations:" << m_solverPositionIterations;
    qDebug() << "  Velocity iterations:" << m_solverVelocityIterations;
    qDebug() << "  Self collision disabled:" << true;
        
    // Log soft body creation parameters
    qDebug() << "Soft body created with parameters:";
    qDebug() << "  Density:" << density << "(g/mm³, typical rubber = 0.0012)";
    qDebug() << "  Max inverse mass ratio:" << maxInvMassRatio << "(increased for more responsive mass distribution)";
    qDebug() << "  Young's modulus:" << youngsModulus << "(increased for better collision response)";
    qDebug() << "  Poisson's ratio:" << poissonsRatio << "(0.49 = nearly incompressible rubber)";
    qDebug() << "  Damping:" << damping << "(0-1, moderate for rubber)";
    qDebug() << "  Shape: Short rectangular prism (50mm height for stability)";
    qDebug() << "  Solver iterations:" << m_solverPositionIterations << "position," << m_solverVelocityIterations << "velocity";
    qDebug() << "  Note: Enhanced parameters for meaningful interaction with rigid bodies";

    // --- Update the soft body data (allocate pinned memory immediately like NVIDIA example) ---
    softBodyData.softBody = deformableVolume;
    softBodyData.isInSimulation = true;

    // Allocate pinned memory immediately like the DeformableVolume constructor in NVIDIA example
    PxTetrahedronMesh* tetMesh = deformableVolume->getCollisionMesh();
    if (tetMesh) {
        softBodyData.simPositionInvMassPinned = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *m_cudaContextManager, tetMesh->getNbVertices());
    } else {
        softBodyData.simPositionInvMassPinned = nullptr;
    }

    softBodyData.simVelocityPinned = nullptr; // Not needed for runtime
    softBodyData.collPositionInvMassPinned = collPositionInvMassPinned; // Keep for collision mesh visualization
    softBodyData.restPositionPinned = nullptr; // Not needed for runtime
    softBodyData.auxData = deformableVolumeMesh->getDeformableVolumeAuxData();

    // Store the objects that must remain valid for the lifetime of the soft body
    softBodyData.deformableVolumeMesh = deformableVolumeMesh;
    softBodyData.shape = shape;
    softBodyData.material = materialPtr;

    // --- Automatically attach soft body to overlapping rigid bodies ---
    qDebug() << "Attempting to attach soft body to overlapping rigid bodies...";
    attachSoftBodyToOverlappingRigidBodies(deformableVolume, transform, softBodyData);

    m_softBodies[item] = softBodyData;

    // Do NOT release these objects here - they must remain valid for the lifetime of the soft body
    // They will be released in removeSoftBody

    emit objectAdded(item);
    return true;
}

bool PhysXEngine::removeSoftBody(Item item)
{
    auto it = m_softBodies.find(item);
    if (it == m_softBodies.end()) {
        qDebug() << "Soft body not found in PhysX simulation";
        return false;
    }

    SoftBodyData& softBodyData = it->second;

    // Only remove from scene if it was actually created and added
    if (softBodyData.softBody && m_scene && softBodyData.isInSimulation) {
        m_scene->removeActor(*softBodyData.softBody);
        softBodyData.softBody->release();
    }

    // Free the pinned host buffers
    if (softBodyData.simPositionInvMassPinned) {
        PX_FREE(softBodyData.simPositionInvMassPinned);
    }
    if (softBodyData.simVelocityPinned) {
        PX_FREE(softBodyData.simVelocityPinned);
    }
    if (softBodyData.collPositionInvMassPinned) {
        PX_FREE(softBodyData.collPositionInvMassPinned);
        softBodyData.collPositionInvMassPinned = nullptr;
    }
    if (softBodyData.restPositionPinned) {
        PX_FREE(softBodyData.restPositionPinned);
    }

    // Release auxiliary data
    if (softBodyData.auxData) {
        qDebug() << "Releasing auxiliary data for soft body:" << item->Name();
        softBodyData.auxData->release();
        softBodyData.auxData = nullptr;
    }

    // Release the mesh objects that were kept alive for the soft body
    if (softBodyData.deformableVolumeMesh) {
        qDebug() << "Releasing deformable volume mesh for soft body:" << item->Name();
        softBodyData.deformableVolumeMesh->release();
        softBodyData.deformableVolumeMesh = nullptr;
    }

    if (softBodyData.shape) {
        qDebug() << "Releasing shape for soft body:" << item->Name();
        softBodyData.shape->release();
        softBodyData.shape = nullptr;
    }

    if (softBodyData.material) {
        qDebug() << "Releasing material for soft body:" << item->Name();
        softBodyData.material->release();
        softBodyData.material = nullptr;
    }

    // Clear debug mesh data
    softBodyData.debugMeshVertices.clear();
    softBodyData.debugMeshIndices.clear();
    softBodyData.hasDebugMesh = false;

    softBodyData.isInSimulation = false;
    m_softBodies.erase(it);

    qDebug() << "Successfully removed soft body from PhysX simulation";
    emit objectRemoved(item);
    return true;
}

bool PhysXEngine::isSoftBodyInSimulation(Item item) const
{
    auto it = m_softBodies.find(item);
    return it != m_softBodies.end() && it->second.isInSimulation;
}

bool PhysXEngine::hasSoftBodyDebugData(Item item) const
{
    auto it = m_softBodies.find(item);
    return it != m_softBodies.end() && it->second.hasDebugMesh;
}

void PhysXEngine::updateSoftBodyFromRoboDK(Item item)
{
    auto it = m_softBodies.find(item);
    if (it == m_softBodies.end()) return;

    SoftBodyData& data = it->second;
    if (!data.softBody) return;

    // For deformable volumes, we can't easily update the pose after creation
    // since the pinned memory is freed immediately after creation (as in the NVIDIA example)
    // The deformable volume will maintain its initial pose and deform based on physics
    qDebug() << "updateSoftBodyFromRoboDK: Deformable volume pose updates not supported after creation for" << item->Name();
    qDebug() << "  (Pinned memory is freed immediately after creation as per NVIDIA example)";
}

void PhysXEngine::updateRoboDKFromSoftBody(Item item)
{
    auto it = m_softBodies.find(item);
    if (it == m_softBodies.end()) return;

    SoftBodyData& data = it->second;
    if (!data.softBody) return;

    // For deformable volumes, we can't easily get the current pose
    // since they don't have a getGlobalPose method. For now, we'll
    // just log that this operation is not fully implemented.
    qDebug() << "updateRoboDKFromSoftBody not fully implemented for deformable volumes for" << item->Name();

    // TODO: Implement proper pose extraction from deformable volume data
    // This would require reading back the simulation mesh vertices and computing
    // the center of mass or bounding box to determine the current pose.
}

void PhysXEngine::updateDeformablePose(Item item, const PxTransform& pose)
{
    auto it = m_softBodies.find(item);
    if (it == m_softBodies.end()) return;

    SoftBodyData& data = it->second;
    if (!data.softBody) return;

    // For deformable volumes, we can't easily update the pose after creation
    // since the pinned memory is freed immediately after creation (as in the NVIDIA example)
    // The deformable volume will maintain its initial pose and deform based on physics
    qDebug() << "updateDeformablePose: Deformable volume pose updates not supported after creation for" << item->Name();
    qDebug() << "  (Pinned memory is freed immediately after creation as per NVIDIA example)";
    qDebug() << "  Requested pose:" << pose.p.x << pose.p.y << pose.p.z;
}

void PhysXEngine::drawDeformableGeometry(PxDeformableVolume* softBody, const PxTransform& initialTransform, bool isSleeping, bool drawCollisionMesh)
{
    // Draw deformable volume geometry with stress visualization
    // - deformed mesh: shows the current state after physics simulation with stress-based coloring
    // - stress is calculated based on deformation from rest position
    if (!softBody || !getRoboDK()) return;

    // Get the collision mesh (tetrahedron mesh) from the deformable volume
    PxTetrahedronMesh* tetMesh = softBody->getCollisionMesh();
    if (!tetMesh) {
        qDebug() << "PhysXEngine: No collision mesh found for deformable volume";
        return;
    }

    // Get the number of vertices in the collision mesh
    PxU32 numVertices = tetMesh->getNbVertices();
    if (numVertices == 0) {
        qDebug() << "PhysXEngine: No vertices found in deformable volume collision mesh";
        return;
    }

    // Collision mesh visualization disabled - it doesn't move and clutters the display
    // The deformed mesh (stress-colored wireframe) shows the current state after physics simulation
    if (drawCollisionMesh) {
        qDebug() << "PhysXEngine: Collision mesh visualization disabled - use deformed mesh for current state";
    }

    // Allocate pinned memory for reading back deformed vertices (like SnippetDeformableVolume)
    PxVec4* positionsInvMass = nullptr;
    if (m_cudaContextManager) {
        positionsInvMass = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *m_cudaContextManager, numVertices);
        
        // Copy deformed vertices from GPU to host memory (like SnippetDeformableVolume)
        PxScopedCudaLock _lock(*m_cudaContextManager);
        m_cudaContextManager->getCudaContext()->memcpyDtoH(
            positionsInvMass,
            reinterpret_cast<CUdeviceptr>(softBody->getPositionInvMassBufferD()),
            numVertices * sizeof(PxVec4)
        );
    } else {
        qDebug() << "PhysXEngine: No CUDA context manager available for deformable volume visualization";
        return;
    }

    // Extract vertex positions from the deformed mesh (already in global space)
    // The vertices are already positioned correctly because PxDeformableVolumeExt::transform was applied during setup
    std::vector<PxVec3> deformedVertices;
    deformedVertices.reserve(numVertices);
    
    for (PxU32 i = 0; i < numVertices; ++i) {
        // Extract position from PxVec4 (x,y,z,w where w is inverse mass)
        // These vertices are already in global space after physics simulation and initial transform
        PxVec3 position(positionsInvMass[i].x, positionsInvMass[i].y, positionsInvMass[i].z);
        
        // Validate vertex position
        if (!isValidVector(position)) {
            qDebug() << "PhysXEngine: Skipping invalid deformed vertex" << i << ":" << position.x << position.y << position.z;
            continue;
        }
        
        deformedVertices.push_back(position);
    }

    // Get rest positions for stress calculation
    std::vector<PxVec3> restVertices;
    restVertices.reserve(numVertices);
    
    // Get rest positions from the collision mesh (these are the original undeformed positions)
    const PxVec3* restPositions = tetMesh->getVertices();
    if (restPositions) {
        for (PxU32 i = 0; i < numVertices; ++i) {
            // Transform rest position to global space using initial transform
            PxVec3 restPos = initialTransform.transform(restPositions[i]);
            restVertices.push_back(restPos);
        }
    } else {
        qDebug() << "PhysXEngine: No rest positions available for stress calculation";
        // Free the pinned memory
        PX_EXT_PINNED_MEMORY_FREE(*m_cudaContextManager, positionsInvMass);
        return;
    }

    // Free the pinned memory
    PX_EXT_PINNED_MEMORY_FREE(*m_cudaContextManager, positionsInvMass);

    if (deformedVertices.empty() || restVertices.empty()) {
        qDebug() << "PhysXEngine: No valid deformed or rest vertices found for visualization";
        return;
    }

    // Draw the deformed tetrahedron mesh using the same connectivity as the collision mesh
    // Get tetrahedron connectivity data from the collision mesh
    PxU32 numTets = tetMesh->getNbTetrahedrons();
    const void* tetData = tetMesh->getTetrahedrons();
    bool has16bit = tetMesh->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

    if (!tetData) {
        qDebug() << "PhysXEngine: No tetrahedron connectivity data found";
        return;
    }

    // Calculate stress for each tetrahedron and draw with stress-based coloring
    float maxStress = 0.0f;
    std::vector<float> tetrahedronStresses;
    tetrahedronStresses.reserve(numTets);

    // First pass: calculate stress for each tetrahedron
    for (PxU32 i = 0; i < numTets; ++i) {
        PxU32 idx[4];
        if (has16bit) {
            const PxU16* tets = reinterpret_cast<const PxU16*>(tetData);
            idx[0] = tets[i * 4 + 0];
            idx[1] = tets[i * 4 + 1];
            idx[2] = tets[i * 4 + 2];
            idx[3] = tets[i * 4 + 3];
        } else {
            const PxU32* tets = reinterpret_cast<const PxU32*>(tetData);
            idx[0] = tets[i * 4 + 0];
            idx[1] = tets[i * 4 + 1];
            idx[2] = tets[i * 4 + 2];
            idx[3] = tets[i * 4 + 3];
        }

        // Validate indices
        if (idx[0] >= numVertices || idx[1] >= numVertices || idx[2] >= numVertices || idx[3] >= numVertices) {
            qDebug() << "PhysXEngine: Invalid tetrahedron indices:" << idx[0] << idx[1] << idx[2] << idx[3] << "max:" << numVertices;
            tetrahedronStresses.push_back(0.0f);
            continue;
        }

        // Calculate stress based on deformation
        float stress = calculateTetrahedronStress(
            restVertices[idx[0]], restVertices[idx[1]], restVertices[idx[2]], restVertices[idx[3]],
            deformedVertices[idx[0]], deformedVertices[idx[1]], deformedVertices[idx[2]], deformedVertices[idx[3]]
        );
        
        tetrahedronStresses.push_back(stress);
        maxStress = std::max(maxStress, stress);
    }



    // Draw the deformed tetrahedron mesh wireframe with stress-bucket-based coloring for performance
    // Group tetrahedra into stress buckets and draw each bucket with a single DrawGeometry call
    if (numTets > 0) {
        // Define stress buckets (12 buckets for finer visual distinction, green to red)
        const int NUM_STRESS_BUCKETS = 12;
        std::vector<std::vector<float>> stressBuckets[NUM_STRESS_BUCKETS];
        float bucketColors[NUM_STRESS_BUCKETS][4];

        // Initialize bucket colors (green -> yellow -> orange -> red)
        // We'll interpolate from green (low) to red (high)
        for (int i = 0; i < NUM_STRESS_BUCKETS; ++i) {
            float t = static_cast<float>(i) / (NUM_STRESS_BUCKETS - 1); // 0.0 (green) to 1.0 (red)
            // Interpolate: green (0,1,0) -> yellow (1,1,0) -> red (1,0,0)
            float r, g, b;
            if (t < 0.5f) {
                // Green to yellow
                float localT = t / 0.5f;
                r = localT;
                g = 1.0f;
                b = 0.0f;
            } else {
                // Yellow to red
                float localT = (t - 0.5f) / 0.5f;
                r = 1.0f;
                g = 1.0f - localT;
                b = 0.0f;
            }
            bucketColors[i][0] = r;
            bucketColors[i][1] = g;
            bucketColors[i][2] = b;
            bucketColors[i][3] = 1.0f; // Alpha
        }
        
        // Group tetrahedra into stress buckets
        for (PxU32 i = 0; i < numTets; ++i) {
            PxU32 idx[4];
            if (has16bit) {
                const PxU16* tets = reinterpret_cast<const PxU16*>(tetData);
                idx[0] = tets[i * 4 + 0];
                idx[1] = tets[i * 4 + 1];
                idx[2] = tets[i * 4 + 2];
                idx[3] = tets[i * 4 + 3];
            } else {
                const PxU32* tets = reinterpret_cast<const PxU32*>(tetData);
                idx[0] = tets[i * 4 + 0];
                idx[1] = tets[i * 4 + 1];
                idx[2] = tets[i * 4 + 2];
                idx[3] = tets[i * 4 + 3];
            }

            // Get stress for this tetrahedron
            float stress = tetrahedronStresses[i];
            float normalizedStress = maxStress > 0.0f ? stress / maxStress : 0.0f;

            // Determine stress bucket (0 to NUM_STRESS_BUCKETS-1)
            int bucketIndex = static_cast<int>(normalizedStress * (NUM_STRESS_BUCKETS - 1));
            bucketIndex = std::max(0, std::min(bucketIndex, NUM_STRESS_BUCKETS - 1));

            // Define the 6 edges of the tetrahedron
            PxU32 edgeIndices[6][2] = {
                {idx[0], idx[1]}, // Edge 0-1
                {idx[0], idx[2]}, // Edge 0-2
                {idx[0], idx[3]}, // Edge 0-3
                {idx[1], idx[2]}, // Edge 1-2
                {idx[1], idx[3]}, // Edge 1-3
                {idx[2], idx[3]}  // Edge 2-3
            };

            // Create triangle vertices for this tetrahedron (4 triangular faces)
            std::vector<float> tetTriangles;
            
            // Define the 4 triangular faces of the tetrahedron
            PxU32 faceIndices[4][3] = {
                {idx[0], idx[1], idx[2]}, // Face 1: vertices 0, 1, 2
                {idx[0], idx[2], idx[3]}, // Face 2: vertices 0, 2, 3
                {idx[0], idx[3], idx[1]}, // Face 3: vertices 0, 3, 1
                {idx[1], idx[3], idx[2]}  // Face 4: vertices 1, 3, 2
            };
            
            for (int face = 0; face < 4; ++face) {
                for (int j = 0; j < 3; ++j) {
                    PxU32 vIdx = faceIndices[face][j];
                    
                    if (vIdx < deformedVertices.size()) {
                        const PxVec3& deformedV = deformedVertices[vIdx];
                        
                        // Convert to STL coordinate system
                        PxVec3 stlVertex = convertPhysXToSTL(deformedV);
                        
                        if (isValidVector(stlVertex)) {
                            // Add triangle vertex (3 coordinates per vertex)
                            tetTriangles.push_back(stlVertex.x);
                            tetTriangles.push_back(stlVertex.y);
                            tetTriangles.push_back(stlVertex.z);
                        }
                    }
                }
            }
            
            if (!tetTriangles.empty()) {
                stressBuckets[bucketIndex].push_back(tetTriangles);
            }
        }

        // Draw each stress bucket with a single DrawGeometry call
        int totalDrawCalls = 0;
        for (int bucket = 0; bucket < NUM_STRESS_BUCKETS; ++bucket) {
            if (!stressBuckets[bucket].empty()) {
                // Combine all triangles from this bucket into a single array
                std::vector<float> combinedTriangles;
                for (const auto& tetTriangles : stressBuckets[bucket]) {
                    combinedTriangles.insert(combinedTriangles.end(), tetTriangles.begin(), tetTriangles.end());
                }
                
                if (!combinedTriangles.empty()) {
                    int numTriangles = static_cast<int>(combinedTriangles.size() / 9); // 3 vertices per triangle, 3 coordinates per vertex
                    getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, const_cast<float*>(combinedTriangles.data()), numTriangles, bucketColors[bucket]);
                    totalDrawCalls++;
                }
            }
        }
        
        qDebug() << "PhysXEngine: Drew deformed tetrahedron mesh with stress-bucket triangle visualization:";
        qDebug() << "  Tetrahedra:" << numTets << "Vertices:" << deformedVertices.size();
        qDebug() << "  Max stress:" << maxStress << "Average stress:" << (maxStress > 0.0f ? std::accumulate(tetrahedronStresses.begin(), tetrahedronStresses.end(), 0.0f) / tetrahedronStresses.size() : 0.0f);
        qDebug() << "  Draw calls reduced from" << numTets << "to" << totalDrawCalls << "(" << NUM_STRESS_BUCKETS << "stress buckets)";
        qDebug() << "  Color coding: Green (low stress) -> Yellow -> Red (high stress)";
        qDebug() << "  Rendering: Solid triangles instead of wireframe for better stress visualization";
    } else {
        qDebug() << "PhysXEngine: No triangles drawn for deformed tetrahedron mesh";
    }
}

// Helper function to calculate stress for a tetrahedron based on deformation
float PhysXEngine::calculateTetrahedronStress(const PxVec3& restV0, const PxVec3& restV1, const PxVec3& restV2, const PxVec3& restV3,
                                             const PxVec3& deformedV0, const PxVec3& deformedV1, const PxVec3& deformedV2, const PxVec3& deformedV3)
{
    // Calculate deformation gradient by comparing rest and deformed positions
    // This is a simplified stress calculation based on strain energy
    
    // Calculate edge vectors in rest configuration
    PxVec3 restEdge1 = restV1 - restV0;
    PxVec3 restEdge2 = restV2 - restV0;
    PxVec3 restEdge3 = restV3 - restV0;
    
    // Calculate edge vectors in deformed configuration
    PxVec3 deformedEdge1 = deformedV1 - deformedV0;
    PxVec3 deformedEdge2 = deformedV2 - deformedV0;
    PxVec3 deformedEdge3 = deformedV3 - deformedV0;
    
    // Calculate strain as the change in edge lengths
    float strain1 = (deformedEdge1.magnitude() - restEdge1.magnitude()) / restEdge1.magnitude();
    float strain2 = (deformedEdge2.magnitude() - restEdge2.magnitude()) / restEdge2.magnitude();
    float strain3 = (deformedEdge3.magnitude() - restEdge3.magnitude()) / restEdge3.magnitude();
    
    // Calculate average strain magnitude (simplified stress measure)
    float avgStrain = (std::abs(strain1) + std::abs(strain2) + std::abs(strain3)) / 3.0f;
    
    // Convert strain to stress (simplified linear relationship)
    // In real materials, this would involve the material's Young's modulus
    float stress = avgStrain * 1e6f; // Simplified stress calculation
    
    return stress;
}

// Helper function to draw tetrahedron mesh (collision mesh for deformable volumes)
void PhysXEngine::drawTetrahedronMesh(PxTetrahedronMesh* tetMesh, const PxTransform& pose, bool isSleeping)
{
    if (!tetMesh || !getRoboDK()) return;

    if (!isValidTransform(pose)) {
        qDebug() << "PhysXEngine: Skipping tetrahedron mesh visualization - invalid pose";
        return;
    }

    // Get tetrahedron mesh data
    const PxVec3* vertices = tetMesh->getVertices();
    PxU32 numTets = tetMesh->getNbTetrahedrons();
    const void* tetData = tetMesh->getTetrahedrons();
    bool has16bit = tetMesh->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

    if (!vertices || !tetData) {
        qDebug() << "PhysXEngine: Invalid tetrahedron mesh data";
        return;
    }

    std::vector<float> vertexArray;
    vertexArray.reserve(numTets * 12); // 4 vertices per tetrahedron, 3 coordinates per vertex

    // Choose color for collision mesh (red to distinguish from deformed mesh)
    float color[4] = {1.0f, 0.0f, 0.0f, 0.6f}; // Red for collision mesh

    // Extract tetrahedron faces (each tetrahedron has 4 triangular faces)
    for (PxU32 i = 0; i < numTets; ++i) {
        PxU32 idx[4];
        if (has16bit) {
            const PxU16* tets = reinterpret_cast<const PxU16*>(tetData);
            idx[0] = tets[i * 4 + 0];
            idx[1] = tets[i * 4 + 1];
            idx[2] = tets[i * 4 + 2];
            idx[3] = tets[i * 4 + 3];
        } else {
            const PxU32* tets = reinterpret_cast<const PxU32*>(tetData);
            idx[0] = tets[i * 4 + 0];
            idx[1] = tets[i * 4 + 1];
            idx[2] = tets[i * 4 + 2];
            idx[3] = tets[i * 4 + 3];
        }

        // Define the 4 triangular faces of the tetrahedron
        // Face 1: vertices 0, 1, 2
        // Face 2: vertices 0, 2, 3
        // Face 3: vertices 0, 3, 1
        // Face 4: vertices 1, 3, 2
        PxU32 faceIndices[4][3] = {
            {idx[0], idx[1], idx[2]},
            {idx[0], idx[2], idx[3]},
            {idx[0], idx[3], idx[1]},
            {idx[1], idx[3], idx[2]}
        };

        // Add each face as a triangle
        for (int face = 0; face < 4; ++face) {
            for (int j = 0; j < 3; ++j) {
                const PxVec3& physxVertex = vertices[faceIndices[face][j]];
                if (!isValidVector(physxVertex)) {
                    qDebug() << "PhysXEngine: Skipping invalid vertex in tetrahedron mesh, index:" << faceIndices[face][j];
                    continue;
                }
                
                PxVec3 transformedVertex;
                try {
                    transformedVertex = pose.transform(physxVertex);
                    if (!isValidVector(transformedVertex)) {
                        qDebug() << "PhysXEngine: Skipping invalid transformed vertex. Orig:" << physxVertex.x << physxVertex.y << physxVertex.z << "Transformed:" << transformedVertex.x << transformedVertex.y << transformedVertex.z;
                        continue;
                    }
                } catch (...) {
                    qDebug() << "PhysXEngine: Exception during vertex transformation, skipping. Orig:" << physxVertex.x << physxVertex.y << physxVertex.z;
                    continue;
                }
                
                PxVec3 transformedStlVertex = convertPhysXToSTL(transformedVertex);
                if (!isValidVector(transformedStlVertex)) {
                    qDebug() << "PhysXEngine: Skipping invalid STL vertex. Transformed:" << transformedVertex.x << transformedVertex.y << transformedVertex.z << "STL:" << transformedStlVertex.x << transformedStlVertex.y << transformedStlVertex.z;
                    continue;
                }
                
                vertexArray.push_back(transformedStlVertex.x);
                vertexArray.push_back(transformedStlVertex.y);
                vertexArray.push_back(transformedStlVertex.z);
            }
        }
    }

    // Draw the tetrahedron mesh using RoboDK's DrawGeometry API
    if (!vertexArray.empty()) {
        int numTriangles = static_cast<int>(vertexArray.size() / 9); // 3 vertices per triangle, 3 coordinates per vertex
        getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, vertexArray.data(), numTriangles, color);
        
        qDebug() << "PhysXEngine: Drew tetrahedron collision mesh with" << numTriangles << "triangles";
    } else {
        qDebug() << "PhysXEngine: No triangles drawn for tetrahedron collision mesh";
    }
}

// Helper function to draw tetrahedron mesh using transformed collision vertices (already in global space)
void PhysXEngine::drawTransformedTetrahedronMesh(PxTetrahedronMesh* tetMesh, const PxVec4* transformedVertices, PxU32 numVertices, bool isSleeping)
{
    if (!tetMesh || !getRoboDK() || !transformedVertices) return;

    // Get tetrahedron mesh connectivity data
    PxU32 numTets = tetMesh->getNbTetrahedrons();
    const void* tetData = tetMesh->getTetrahedrons();
    bool has16bit = tetMesh->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

    if (!tetData) {
        qDebug() << "PhysXEngine: No tetrahedron connectivity data found";
        return;
    }

    std::vector<float> vertexArray;
    vertexArray.reserve(numTets * 12); // 4 vertices per tetrahedron, 3 coordinates per vertex

    // Choose color for collision mesh (red to distinguish from deformed mesh)
    float color[4] = {1.0f, 0.0f, 0.0f, 0.6f}; // Red for collision mesh

    // Extract tetrahedron faces (each tetrahedron has 4 triangular faces)
    for (PxU32 i = 0; i < numTets; ++i) {
        PxU32 idx[4];
        if (has16bit) {
            const PxU16* tets = reinterpret_cast<const PxU16*>(tetData);
            idx[0] = tets[i * 4 + 0];
            idx[1] = tets[i * 4 + 1];
            idx[2] = tets[i * 4 + 2];
            idx[3] = tets[i * 4 + 3];
        } else {
            const PxU32* tets = reinterpret_cast<const PxU32*>(tetData);
            idx[0] = tets[i * 4 + 0];
            idx[1] = tets[i * 4 + 1];
            idx[2] = tets[i * 4 + 2];
            idx[3] = tets[i * 4 + 3];
        }

        // Validate indices
        if (idx[0] >= numVertices || idx[1] >= numVertices || idx[2] >= numVertices || idx[3] >= numVertices) {
            qDebug() << "PhysXEngine: Invalid tetrahedron indices:" << idx[0] << idx[1] << idx[2] << idx[3] << "max:" << numVertices;
            continue;
        }

        // Define the 4 triangular faces of the tetrahedron
        // Face 1: vertices 0, 1, 2
        // Face 2: vertices 0, 2, 3
        // Face 3: vertices 0, 3, 1
        // Face 4: vertices 1, 3, 2
        PxU32 faceIndices[4][3] = {
            {idx[0], idx[1], idx[2]},
            {idx[0], idx[2], idx[3]},
            {idx[0], idx[3], idx[1]},
            {idx[1], idx[3], idx[2]}
        };

        // Add each face as a triangle using transformed vertices (already in global space)
        for (int face = 0; face < 4; ++face) {
            for (int j = 0; j < 3; ++j) {
                const PxVec4& transformedVertex = transformedVertices[faceIndices[face][j]];
                // Extract position from PxVec4 (x,y,z,w where w is inverse mass)
                PxVec3 position(transformedVertex.x, transformedVertex.y, transformedVertex.z);
                
                if (!isValidVector(position)) {
                    qDebug() << "PhysXEngine: Skipping invalid transformed collision vertex" << faceIndices[face][j] << ":" << position.x << position.y << position.z;
                    continue;
                }
                
                // Convert to STL coordinate system (already in global space)
                PxVec3 transformedStlVertex = convertPhysXToSTL(position);
                if (!isValidVector(transformedStlVertex)) {
                    qDebug() << "PhysXEngine: Skipping invalid STL vertex. Position:" << position.x << position.y << position.z << "STL:" << transformedStlVertex.x << transformedStlVertex.y << transformedStlVertex.z;
                    continue;
                }
                
                vertexArray.push_back(transformedStlVertex.x);
                vertexArray.push_back(transformedStlVertex.y);
                vertexArray.push_back(transformedStlVertex.z);
            }
        }
    }

    // Draw the transformed tetrahedron mesh using RoboDK's DrawGeometry API
    if (!vertexArray.empty()) {
        int numTriangles = static_cast<int>(vertexArray.size() / 9); // 3 vertices per triangle, 3 coordinates per vertex
        getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, vertexArray.data(), numTriangles, color);
        
        qDebug() << "PhysXEngine: Drew transformed tetrahedron collision mesh with" << numTriangles << "triangles";
    } else {
        qDebug() << "PhysXEngine: No triangles drawn for transformed tetrahedron collision mesh";
    }
}

// Helper function to draw only the collision mesh of a deformable volume
// Usage examples:
// - drawDeformableGeometry(softBody, transform, false, true);  // Draw both deformed and collision mesh
// - drawDeformableGeometry(softBody, transform, false, false); // Draw only deformed mesh
// - drawDeformableCollisionMesh(softBody, transform, false);   // Draw only collision mesh
void PhysXEngine::drawDeformableCollisionMesh(PxDeformableVolume* softBody, const PxTransform& initialTransform, bool isSleeping)
{
    if (!softBody || !getRoboDK()) return;

    // Get the collision mesh (tetrahedron mesh) from the deformable volume
    PxTetrahedronMesh* tetMesh = softBody->getCollisionMesh();
    if (!tetMesh) {
        qDebug() << "PhysXEngine: No collision mesh found for deformable volume";
        return;
    }

    // Draw the collision mesh - use the same transform as the deformed mesh
    // The collision mesh should be drawn at the same location as the deformed mesh
    drawTetrahedronMesh(tetMesh, initialTransform, isSleeping);
}

// Helper function to create cylindrical surface mesh
void PhysXEngine::createCylindricalSurfaceMesh(float radius, float length, int numSegments,
                                               std::vector<PxVec3>& vertices, std::vector<PxU32>& indices)
{
    vertices.clear();
    indices.clear();

    const int numRings = std::max(4, numSegments);
    const int numSides = 32; // Higher for better quality

    // Generate vertices for the cylindrical surface (excluding end caps)
    for (int ring = 0; ring <= numRings; ++ring) {
        float z = (ring * length) / numRings;
        for (int side = 0; side < numSides; ++side) {
            float angle = (2.0f * PxPi * side) / numSides;
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            vertices.push_back(PxVec3(x, y, z));
        }
    }

    // Generate triangles for the cylindrical surface with consistent winding (outward normals)
    for (int ring = 0; ring < numRings; ++ring) {
        for (int side = 0; side < numSides; ++side) {
            int current = ring * numSides + side;
            int next = ring * numSides + ((side + 1) % numSides);
            int currentUp = (ring + 1) * numSides + side;
            int nextUp = (ring + 1) * numSides + ((side + 1) % numSides);

            // First triangle (counter-clockwise winding for outward normal)
            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(currentUp);

            // Second triangle (counter-clockwise winding for outward normal)
            indices.push_back(next);
            indices.push_back(nextUp);
            indices.push_back(currentUp);
        }
    }

    // Add end caps with proper winding
    // Bottom cap (facing outward from cylinder)
    int bottomCenter = static_cast<int>(vertices.size());
    vertices.push_back(PxVec3(0, 0, 0)); // Bottom center

    for (int side = 0; side < numSides; ++side) {
        int current = side;
        int next = (side + 1) % numSides;

        // Counter-clockwise winding for outward normal (facing down)
        indices.push_back(bottomCenter);
        indices.push_back(current);
        indices.push_back(next);
    }

    // Top cap (facing outward from cylinder)
    int topCenter = static_cast<int>(vertices.size());
    vertices.push_back(PxVec3(0, 0, length)); // Top center

    for (int side = 0; side < numSides; ++side) {
        int current = numRings * numSides + side;
        int next = numRings * numSides + ((side + 1) % numSides);

        // Counter-clockwise winding for outward normal (facing up)
        indices.push_back(topCenter);
        indices.push_back(next);
        indices.push_back(current);
    }

    qDebug() << "Created cylindrical mesh with" << vertices.size() << "vertices and" << indices.size() / 3 << "triangles";
    qDebug() << "Cylinder dimensions: radius=" << radius << "length=" << length << "rings=" << numRings << "sides=" << numSides;
}

// Alternative simpler cylindrical mesh creation method
void PhysXEngine::createSimpleCylindricalMesh(float radius, float length, int numSegments,
                                               std::vector<PxVec3>& vertices, std::vector<PxU32>& indices)
{
    vertices.clear();
    indices.clear();

    const int numRings = std::max(4, numSegments);
    const int numSides = 16; // Simpler mesh with fewer sides

    // Generate vertices for the cylindrical surface
    for (int ring = 0; ring <= numRings; ++ring) {
        float z = (ring * length) / numRings;
        for (int side = 0; side < numSides; ++side) {
            float angle = (2.0f * PxPi * side) / numSides;
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            vertices.push_back(PxVec3(x, y, z));
        }
    }

    // Generate triangles for the cylindrical surface with proper winding (outward normals)
    for (int ring = 0; ring < numRings; ++ring) {
        for (int side = 0; side < numSides; ++side) {
            int current = ring * numSides + side;
            int next = ring * numSides + ((side + 1) % numSides);
            int currentUp = (ring + 1) * numSides + side;
            int nextUp = (ring + 1) * numSides + ((side + 1) % numSides);

            // First triangle (counter-clockwise winding for outward normal)
            indices.push_back(current);
            indices.push_back(currentUp);
            indices.push_back(next);

            // Second triangle (counter-clockwise winding for outward normal)
            indices.push_back(next);
            indices.push_back(currentUp);
            indices.push_back(nextUp);
        }
    }

    // Add end caps with proper winding
    // Bottom cap (facing outward from cylinder)
    int bottomCenter = static_cast<int>(vertices.size());
    vertices.push_back(PxVec3(0, 0, 0));

    for (int side = 0; side < numSides; ++side) {
        int current = side;
        int next = (side + 1) % numSides;

        // Counter-clockwise winding for outward normal (facing down)
        indices.push_back(bottomCenter);
        indices.push_back(next);
        indices.push_back(current);
    }

    // Top cap (facing outward from cylinder)
    int topCenter = static_cast<int>(vertices.size());
    vertices.push_back(PxVec3(0, 0, length));

    for (int side = 0; side < numSides; ++side) {
        int current = numRings * numSides + side;
        int next = numRings * numSides + ((side + 1) % numSides);

        // Counter-clockwise winding for outward normal (facing up)
        indices.push_back(topCenter);
        indices.push_back(current);
        indices.push_back(next);
    }

    qDebug() << "Created simple cylindrical mesh with" << vertices.size() << "vertices and" << indices.size() / 3 << "triangles";
    qDebug() << "Simple cylinder: radius=" << radius << "length=" << length << "rings=" << numRings << "sides=" << numSides;
}

// Very basic cylindrical mesh creation method
void PhysXEngine::createBasicCylindricalMesh(float radius, float length, int numSegments,
                                               std::vector<PxVec3>& vertices, std::vector<PxU32>& indices)
{
    vertices.clear();
    indices.clear();

    const int numRings = std::max(4, numSegments);
    const int numSides = 12; // Increased sides to reduce acute angles

    // Generate vertices for the cylindrical surface
    for (int ring = 0; ring <= numRings; ++ring) {
        float z = (ring * length) / numRings;
        for (int side = 0; side < numSides; ++side) {
            float angle = (2.0f * PxPi * side) / numSides;
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            vertices.push_back(PxVec3(x, y, z));
        }
    }

    // Generate triangles for the cylindrical surface with consistent winding (outward normals)
    for (int ring = 0; ring < numRings; ++ring) {
        for (int side = 0; side < numSides; ++side) {
            int current = ring * numSides + side;
            int next = ring * numSides + ((side + 1) % numSides);
            int currentUp = (ring + 1) * numSides + side;
            int nextUp = (ring + 1) * numSides + ((side + 1) % numSides);

            // First triangle (counter-clockwise winding for outward normal)
            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(currentUp);

            // Second triangle (counter-clockwise winding for outward normal)
            indices.push_back(next);
            indices.push_back(nextUp);
            indices.push_back(currentUp);
        }
    }

    // Add end caps with consistent winding
    // Bottom cap (facing outward from cylinder)
    int bottomCenter = static_cast<int>(vertices.size());
    vertices.push_back(PxVec3(0, 0, 0));

    for (int side = 0; side < numSides; ++side) {
        int current = side;
        int next = (side + 1) % numSides;

        // Counter-clockwise winding for outward normal (facing down)
        indices.push_back(bottomCenter);
        indices.push_back(current);
        indices.push_back(next);
    }

    // Top cap (facing outward from cylinder)
    int topCenter = static_cast<int>(vertices.size());
    vertices.push_back(PxVec3(0, 0, length));

    for (int side = 0; side < numSides; ++side) {
        int current = numRings * numSides + side;
        int next = numRings * numSides + ((side + 1) % numSides);

        // Counter-clockwise winding for outward normal (facing up)
        indices.push_back(topCenter);
        indices.push_back(next);
        indices.push_back(current);
    }

    qDebug() << "Created basic cylindrical mesh with" << vertices.size() << "vertices and" << indices.size() / 3 << "triangles";
    qDebug() << "Basic cylinder: radius=" << radius << "length=" << length << "rings=" << numRings << "sides=" << numSides;

    // Debug: Check winding order of first few triangles
    if (indices.size() >= 9) {
        qDebug() << "First triangle winding:" << indices[0] << indices[1] << indices[2];
        qDebug() << "Second triangle winding:" << indices[3] << indices[4] << indices[5];
        qDebug() << "Third triangle winding:" << indices[6] << indices[7] << indices[8];
    }
}

// Completely new cylindrical mesh creation method with guaranteed proper connectivity
void PhysXEngine::createSimpleCylindricalMeshV2(float radius, float length, int numSegments,
                                               std::vector<PxVec3>& vertices, std::vector<PxU32>& indices)
{
    vertices.clear();
    indices.clear();

    // Create a very simple, robust mesh with minimal complexity
    const int numSides = 4; // Use 4 sides for a square cylinder (most stable)

    // Generate vertices for a simple cylinder with just 2 rings (start and end)
    for (int ring = 0; ring <= 1; ++ring) {
        float z = ring * length;
        for (int side = 0; side < numSides; ++side) {
            float angle = (2.0f * PxPi * side) / numSides;
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            vertices.push_back(PxVec3(x, y, z));
        }
    }

    // Generate triangles for the cylindrical surface with explicit, guaranteed correct winding
    for (int side = 0; side < numSides; ++side) {
        int v0 = side;                                    // Current side, bottom ring
        int v1 = (side + 1) % numSides;                  // Next side, bottom ring
        int v2 = numSides + side;                         // Current side, top ring
        int v3 = numSides + ((side + 1) % numSides);     // Next side, top ring

        // First triangle: v0 -> v1 -> v2 (counter-clockwise for outward normal)
        indices.push_back(v0);
        indices.push_back(v1);
        indices.push_back(v2);

        // Second triangle: v1 -> v3 -> v2 (counter-clockwise for outward normal)
        indices.push_back(v1);
        indices.push_back(v3);
        indices.push_back(v2);
    }

    // Add end caps with explicit, correct winding
    // Bottom cap (facing outward from cylinder)
    int bottomCenter = static_cast<int>(vertices.size());
    vertices.push_back(PxVec3(0, 0, 0));

    for (int side = 0; side < numSides; ++side) {
        int v0 = side;
        int v1 = (side + 1) % numSides;

        // Counter-clockwise winding for outward normal (facing down)
        indices.push_back(bottomCenter);
        indices.push_back(v0);
        indices.push_back(v1);
    }

    // Top cap (facing outward from cylinder)
    int topCenter = static_cast<int>(vertices.size());
    vertices.push_back(PxVec3(0, 0, length));

    for (int side = 0; side < numSides; ++side) {
        int v0 = numSides + side;
        int v1 = numSides + ((side + 1) % numSides);

        // Counter-clockwise winding for outward normal (facing up)
        indices.push_back(topCenter);
        indices.push_back(v1);
        indices.push_back(v0);
    }

    qDebug() << "Created V2 cylindrical mesh with" << vertices.size() << "vertices and" << indices.size() / 3 << "triangles";
    qDebug() << "V2 cylinder: radius=" << radius << "length=" << length << "sides=" << numSides;
    qDebug() << "V2 cylinder: Simple square cylinder with guaranteed correct winding";

    // Debug: Check winding order of first few triangles
    if (indices.size() >= 9) {
        qDebug() << "V2 First triangle winding:" << indices[0] << indices[1] << indices[2];
        qDebug() << "V2 Second triangle winding:" << indices[3] << indices[4] << indices[5];
        qDebug() << "V2 Third triangle winding:" << indices[6] << indices[7] << indices[8];
    }

    // Additional validation: Check for any potential issues
    qDebug() << "V2 Mesh validation:";
    qDebug() << "  Total vertices:" << vertices.size();
    qDebug() << "  Total triangles:" << indices.size() / 3;
    qDebug() << "  Expected vertices:" << (2 * numSides + 2); // 2 rings + 2 centers
    qDebug() << "  Expected triangles:" << (2 * numSides + 2 * numSides); // 2 per quad + 2 caps
}

// Create a simple box mesh that's guaranteed to work with PhysX
void PhysXEngine::createSimpleBoxMesh(float width, float height,
                                     PxArray<PxVec3>& vertices, PxArray<PxU32>& indices)
{
    vertices.clear();
    indices.clear();

    // Create a simple box with 8 vertices
    float halfWidth = width * 0.5f;
    float halfHeight = height * 0.5f;

    // 8 vertices of a box
    vertices.pushBack(PxVec3(-halfWidth, -halfHeight, -halfWidth));  // 0: bottom-back-left
    vertices.pushBack(PxVec3( halfWidth, -halfHeight, -halfWidth));  // 1: bottom-back-right
    vertices.pushBack(PxVec3( halfWidth,  halfHeight, -halfWidth));  // 2: top-back-right
    vertices.pushBack(PxVec3(-halfWidth,  halfHeight, -halfWidth));  // 3: top-back-left
    vertices.pushBack(PxVec3(-halfWidth, -halfHeight,  halfWidth));  // 4: bottom-front-left
    vertices.pushBack(PxVec3( halfWidth, -halfHeight,  halfWidth));  // 5: bottom-front-right
    vertices.pushBack(PxVec3( halfWidth,  halfHeight,  halfWidth));  // 6: top-front-right
    vertices.pushBack(PxVec3(-halfWidth,  halfHeight,  halfWidth));  // 7: top-front-left

    // 12 triangles (2 per face, 6 faces)
    // Each triangle is counter-clockwise for outward normals
    // Back face (facing -Z)
    indices.pushBack(0); indices.pushBack(1); indices.pushBack(2);
    indices.pushBack(0); indices.pushBack(2); indices.pushBack(3);
    // Front face (facing +Z)
    indices.pushBack(4); indices.pushBack(7); indices.pushBack(6);
    indices.pushBack(4); indices.pushBack(6); indices.pushBack(5);
    // Left face (facing -X)
    indices.pushBack(0); indices.pushBack(3); indices.pushBack(7);
    indices.pushBack(0); indices.pushBack(7); indices.pushBack(4);
    // Right face (facing +X)
    indices.pushBack(1); indices.pushBack(5); indices.pushBack(6);
    indices.pushBack(1); indices.pushBack(6); indices.pushBack(2);
    // Bottom face (facing -Y)
    indices.pushBack(0); indices.pushBack(4); indices.pushBack(5);
    indices.pushBack(0); indices.pushBack(5); indices.pushBack(1);
    // Top face (facing +Y)
    indices.pushBack(3); indices.pushBack(2); indices.pushBack(6);
    indices.pushBack(3); indices.pushBack(6); indices.pushBack(7);

    qDebug() << "Created simple box mesh with" << vertices.size() << "vertices and" << indices.size() / 3 << "triangles";
    qDebug() << "Box dimensions: width=" << width << "height=" << height;
    qDebug() << "Box mesh: Guaranteed valid topology for PhysX";
}

// Create a simple tetrahedron mesh (most basic 3D shape, guaranteed to work with PhysX)
void PhysXEngine::createSimpleTetrahedronMesh(float size,
                                             std::vector<PxVec3>& vertices, std::vector<PxU32>& indices)
{
    vertices.clear();
    indices.clear();

    // Create a regular tetrahedron with 4 vertices
    float halfSize = size * 0.5f;

    // 4 vertices of a regular tetrahedron
    vertices = {
        PxVec3( halfSize,  halfSize,  halfSize),   // 0: top vertex
        PxVec3(-halfSize, -halfSize,  halfSize),   // 1: bottom-front vertex
        PxVec3(-halfSize,  halfSize, -halfSize),   // 2: bottom-back vertex
        PxVec3( halfSize, -halfSize, -halfSize)    // 3: bottom-right vertex
    };

    // 4 triangles (one per face of tetrahedron)
    // Each triangle is counter-clockwise for outward normals
    indices = {
        // Face 1: 0-1-2
        0, 1, 2,
        // Face 2: 0-2-3
        0, 2, 3,
        // Face 3: 0-3-1
        0, 3, 1,
        // Face 4: 1-3-2
        1, 3, 2
    };

    qDebug() << "Created simple tetrahedron mesh with" << vertices.size() << "vertices and" << indices.size() / 3 << "triangles";
    qDebug() << "Tetrahedron size:" << size;
    qDebug() << "Tetrahedron mesh: Most basic 3D shape, guaranteed valid topology for PhysX";
}

// Helper function to visualize mesh in RoboDK for debugging
void PhysXEngine::visualizeMeshInRoboDK(const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices, const Mat& pose)
{
    if (!getRoboDK()) return;

    // Convert vertices to RoboDK coordinate system
    std::vector<float> vertexArray;
    vertexArray.reserve(vertices.size() * 3);

    for (const PxVec3& vertex : vertices) {
        // Convert from PhysX to RoboDK coordinate system
        PxVec3 stlVertex = convertPhysXToSTL(vertex);
        vertexArray.push_back(stlVertex.x);
        vertexArray.push_back(stlVertex.y);
        vertexArray.push_back(stlVertex.z);
    }

    // Create color array (red for debugging)
    float color[4] = {1.0f, 0.0f, 0.0f, 0.8f}; // Red with 80% opacity

    // Draw the mesh using RoboDK's DrawGeometry API
    if (!vertexArray.empty() && !indices.empty()) {
        int numTriangles = static_cast<int>(indices.size() / 3);
        getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, vertexArray.data(), numTriangles, color);

        qDebug() << "Visualized mesh in RoboDK with" << vertices.size() << "vertices and" << numTriangles << "triangles";
    }
}



// Helper function to draw debug meshes during render
void PhysXEngine::drawDebugMeshes()
{
    if (!getRoboDK()) return;

    // Draw debug meshes for all soft bodies
    for (auto& pair : m_softBodies) {
        const SoftBodyData& softBodyData = pair.second;

        if (softBodyData.hasDebugMesh && !softBodyData.debugMeshVertices.empty() && !softBodyData.debugMeshIndices.empty()) {
            // Convert indexed mesh to triangle vertex array for RoboDK visualization
            std::vector<float> triangleVertexArray;
            triangleVertexArray.reserve(softBodyData.debugMeshIndices.size() * 3); // 3 floats per vertex

            // Convert each triangle's vertices to RoboDK coordinate system
            for (size_t i = 0; i < softBodyData.debugMeshIndices.size(); i += 3) {
                for (int j = 0; j < 3; ++j) {
                    PxU32 vertexIndex = softBodyData.debugMeshIndices[i + j];
                    if (vertexIndex < softBodyData.debugMeshVertices.size()) {
                        const PxVec3& vertex = softBodyData.debugMeshVertices[vertexIndex];
                        // Convert from PhysX to RoboDK coordinate system
                        PxVec3 stlVertex = convertPhysXToSTL(vertex);
                        triangleVertexArray.push_back(stlVertex.x);
                        triangleVertexArray.push_back(stlVertex.y);
                        triangleVertexArray.push_back(stlVertex.z);
                    }
                }
            }

            // Create color array (orange for successful debug meshes, red for failed ones)
            float color[4];
            if (softBodyData.isInSimulation) {
                color[0] = 1.0f; color[1] = 0.5f; color[2] = 0.0f; color[3] = 0.6f; // Orange for successful
            } else {
                color[0] = 1.0f; color[1] = 0.0f; color[2] = 0.0f; color[3] = 0.8f; // Red for failed
            }

            // Draw the mesh using RoboDK's DrawGeometry API
            if (!triangleVertexArray.empty()) {
                int numTriangles = static_cast<int>(triangleVertexArray.size() / 9); // 9 floats per triangle (3 vertices * 3 floats)
                getRoboDK()->DrawGeometry(RoboDK::DrawTriangles, triangleVertexArray.data(), numTriangles, color);
            }
        }
    }
}

// Helper function to clean up failed soft body data
void PhysXEngine::cleanupFailedSoftBody(Item item)
{
    auto it = m_softBodies.find(item);
    if (it != m_softBodies.end()) {
        SoftBodyData& softBodyData = it->second;

        // Clear all PhysX-related data but keep debug mesh data for visualization
        if (softBodyData.softBody) {
            if (m_scene) {
                m_scene->removeActor(*softBodyData.softBody);
            }
            softBodyData.softBody->release();
            softBodyData.softBody = nullptr;
        }

        // Free the runtime pinned memory (only simPositionInvMassPinned is used for runtime)
        if (softBodyData.simPositionInvMassPinned) {
            PX_EXT_PINNED_MEMORY_FREE(*m_cudaContextManager, softBodyData.simPositionInvMassPinned);
            softBodyData.simPositionInvMassPinned = nullptr;
        }

        // Release auxiliary data
        if (softBodyData.auxData) {
            softBodyData.auxData->release();
            softBodyData.auxData = nullptr;
        }

        // Release the mesh objects that were kept alive for the soft body
        if (softBodyData.deformableVolumeMesh) {
            softBodyData.deformableVolumeMesh->release();
            softBodyData.deformableVolumeMesh = nullptr;
        }

        if (softBodyData.shape) {
            softBodyData.shape->release();
            softBodyData.shape = nullptr;
        }

        if (softBodyData.material) {
            softBodyData.material->release();
            softBodyData.material = nullptr;
        }

        // Mark as not in simulation but keep debug mesh data
        softBodyData.isInSimulation = false;

        qDebug() << "Cleaned up failed soft body data for" << item->Name() << "but kept debug mesh for visualization";
    }
}

// Helper function to validate mesh quality before tetrahedralization
bool PhysXEngine::validateMeshQuality(const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices)
{
    if (vertices.empty() || indices.empty()) {
        qDebug() << "Mesh validation failed: Empty vertices or indices";
        return false;
    }

    if (indices.size() % 3 != 0) {
        qDebug() << "Mesh validation failed: Indices count is not divisible by 3";
        return false;
    }

    // Check for valid vertex indices
    for (size_t i = 0; i < indices.size(); ++i) {
        if (indices[i] >= vertices.size()) {
            qDebug() << "Mesh validation failed: Invalid vertex index" << indices[i] << ">==" << vertices.size();
            return false;
        }
    }

    // Check for degenerate triangles (zero area)
    for (size_t i = 0; i < indices.size(); i += 3) {
        const PxVec3& v0 = vertices[indices[i]];
        const PxVec3& v1 = vertices[indices[i + 1]];
        const PxVec3& v2 = vertices[indices[i + 2]];

        // Check if vertices are the same (degenerate triangle)
        if (v0 == v1 || v1 == v2 || v0 == v2) {
            qDebug() << "Mesh validation failed: Degenerate triangle found at index" << i / 3;
            return false;
        }

        // Check triangle area (should be positive for proper winding)
        PxVec3 edge1 = v1 - v0;
        PxVec3 edge2 = v2 - v0;
        PxVec3 normal = edge1.cross(edge2);
        float area = normal.magnitude() * 0.5f;

        if (area < 1e-6f) { // Very small area threshold
            qDebug() << "Mesh validation failed: Zero or near-zero area triangle at index" << i / 3 << "area=" << area;
            return false;
        }
    }

    // Check mesh bounds
    PxVec3 minBounds(FLT_MAX, FLT_MAX, FLT_MAX);
    PxVec3 maxBounds(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const PxVec3& vertex : vertices) {
        minBounds.x = std::min(minBounds.x, vertex.x);
        minBounds.y = std::min(minBounds.y, vertex.y);
        minBounds.z = std::min(minBounds.z, vertex.z);
        maxBounds.x = std::max(maxBounds.x, vertex.x);
        maxBounds.y = std::max(maxBounds.y, vertex.y);
        maxBounds.z = std::max(maxBounds.z, vertex.z);
    }

    PxVec3 extents = maxBounds - minBounds;
    if (extents.x < 1e-3f || extents.y < 1e-3f || extents.z < 1e-3f) {
        qDebug() << "Mesh validation failed: Mesh is too small or flat";
        qDebug() << "Bounds:" << minBounds.x << minBounds.y << minBounds.z << "to" << maxBounds.x << maxBounds.y << maxBounds.z;
        return false;
    }

    qDebug() << "Mesh validation passed:";
    qDebug() << "  Vertices:" << vertices.size();
    qDebug() << "  Triangles:" << indices.size() / 3;
    qDebug() << "  Bounds:" << minBounds.x << minBounds.y << minBounds.z << "to" << maxBounds.x << maxBounds.y << maxBounds.z;
    qDebug() << "  Extents:" << extents.x << extents.y << extents.z;

    return true;
}









// Draw collision bounds for all objects in simulation
void PhysXEngine::drawCollisionBounds()
{
    if (!m_scene || !getRoboDK()) return;
        
    // Get all rigid body actors in the scene
    PxActorTypeFlags actorTypes = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
    PxU32 nbActors = m_scene->getNbActors(actorTypes);
    std::vector<PxActor*> actors(nbActors);
    m_scene->getActors(actorTypes, actors.data(), nbActors);
    
    // Draw bounding boxes for rigid bodies
    for (PxActor* actor : actors) {
        PxRigidActor* rigidActor = actor->is<PxRigidActor>();
        if (!rigidActor) continue;
        
        PxBounds3 bounds = rigidActor->getWorldBounds();
        if (!bounds.isValid()) continue;
        
        // Convert bounds to RoboDK coordinate system
        PxVec3 minBounds = convertPhysXToSTL(bounds.minimum);
        PxVec3 maxBounds = convertPhysXToSTL(bounds.maximum);
        
        if (!isValidVector(minBounds) || !isValidVector(maxBounds)) continue;
        
        // Create wireframe box vertices for the bounds
        std::vector<float> lineVertices;
        
        // 8 corners of the bounding box
        PxVec3 corners[8] = {
            PxVec3(minBounds.x, minBounds.y, minBounds.z), // 0: bottom-back-left
            PxVec3(maxBounds.x, minBounds.y, minBounds.z), // 1: bottom-back-right
            PxVec3(maxBounds.x, maxBounds.y, minBounds.z), // 2: top-back-right
            PxVec3(minBounds.x, maxBounds.y, minBounds.z), // 3: top-back-left
            PxVec3(minBounds.x, minBounds.y, maxBounds.z), // 4: bottom-front-left
            PxVec3(maxBounds.x, minBounds.y, maxBounds.z), // 5: bottom-front-right
            PxVec3(maxBounds.x, maxBounds.y, maxBounds.z), // 6: top-front-right
            PxVec3(minBounds.x, maxBounds.y, maxBounds.z)  // 7: top-front-left
        };
        
        // 12 edges of the box (24 vertices for 12 lines)
        int edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Bottom face
            {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Top face
            {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Vertical edges
        };
        
        for (int i = 0; i < 12; ++i) {
            // Add start point
            lineVertices.push_back(corners[edges[i][0]].x);
            lineVertices.push_back(corners[edges[i][0]].y);
            lineVertices.push_back(corners[edges[i][0]].z);
            // Add end point
            lineVertices.push_back(corners[edges[i][1]].x);
            lineVertices.push_back(corners[edges[i][1]].y);
            lineVertices.push_back(corners[edges[i][1]].z);
        }
        
        // Choose color based on actor type
        float color[4];
        if (rigidActor->is<PxRigidDynamic>()) {
            PxRigidDynamic* dynamic = rigidActor->is<PxRigidDynamic>();
            if (dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) {
                // Purple for kinematic actors (robot joints)
                color[0] = 0.8f; color[1] = 0.0f; color[2] = 0.8f; color[3] = 0.8f;
            } else {
                // Blue for dynamic actors
                color[0] = 0.0f; color[1] = 0.0f; color[2] = 1.0f; color[3] = 0.8f;
            }
        } else {
            // Green for static actors (ground, etc.)
            color[0] = 0.0f; color[1] = 1.0f; color[2] = 0.0f; color[3] = 0.8f;
        }
        
        // Draw the wireframe box
        if (!lineVertices.empty()) {
            int numLines = static_cast<int>(lineVertices.size() / 3);
            getRoboDK()->DrawGeometry(RoboDK::DrawLines, lineVertices.data(), numLines, color);
        }
    }
    
    // Draw bounding boxes for soft bodies
    for (auto& softBodyPair : m_softBodies) {
        const SoftBodyData& softBodyData = softBodyPair.second;
        if (!softBodyData.isInSimulation || !softBodyData.softBody) continue;
        
        // For deformable volumes, we need to calculate bounds from the deformed vertices
        // instead of using getWorldBounds() which might not account for the initial transform
        PxBounds3 bounds;
        bounds.setEmpty();
        
        // Get the collision mesh to calculate bounds from deformed vertices
        PxTetrahedronMesh* tetMesh = softBodyData.softBody->getCollisionMesh();
        if (tetMesh && m_cudaContextManager) {
            PxU32 numVertices = tetMesh->getNbVertices();
            if (numVertices > 0) {
                // Allocate pinned memory for reading back deformed vertices
                PxVec4* positionsInvMass = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *m_cudaContextManager, numVertices);
                
                // Copy deformed vertices from GPU to host memory
                PxScopedCudaLock _lock(*m_cudaContextManager);
                m_cudaContextManager->getCudaContext()->memcpyDtoH(
                    positionsInvMass,
                    reinterpret_cast<CUdeviceptr>(softBodyData.softBody->getPositionInvMassBufferD()),
                    numVertices * sizeof(PxVec4)
                );
                
                // Calculate bounds from deformed vertices (already in global space)
                for (PxU32 i = 0; i < numVertices; ++i) {
                    PxVec3 position(positionsInvMass[i].x, positionsInvMass[i].y, positionsInvMass[i].z);
                    if (isValidVector(position)) {
                        bounds.include(position);
                        
                        // Debug: Print first few vertices used for bounding box calculation
                        if (i < 3) {
                            qDebug() << "PhysXEngine: Bounding box vertex" << i << "PhysX:" << position.x << position.y << position.z;
                        }
                    }
                }
                
                // Free the pinned memory
                PX_EXT_PINNED_MEMORY_FREE(*m_cudaContextManager, positionsInvMass);
            }
        }
        
        // If we couldn't calculate bounds from deformed vertices, fall back to getWorldBounds()
        if (!bounds.isValid()) {
            bounds = softBodyData.softBody->getWorldBounds();
        }
        
        if (!bounds.isValid()) continue;
        
        // Convert bounds to RoboDK coordinate system
        PxVec3 minBounds = convertPhysXToSTL(bounds.minimum);
        PxVec3 maxBounds = convertPhysXToSTL(bounds.maximum);
        
        if (!isValidVector(minBounds) || !isValidVector(maxBounds)) continue;
        
        // Create wireframe box vertices for the bounds
        std::vector<float> lineVertices;
        
        // 8 corners of the bounding box
        PxVec3 corners[8] = {
            PxVec3(minBounds.x, minBounds.y, minBounds.z), // 0: bottom-back-left
            PxVec3(maxBounds.x, minBounds.y, minBounds.z), // 1: bottom-back-right
            PxVec3(maxBounds.x, maxBounds.y, minBounds.z), // 2: top-back-right
            PxVec3(minBounds.x, maxBounds.y, minBounds.z), // 3: top-back-left
            PxVec3(minBounds.x, minBounds.y, maxBounds.z), // 4: bottom-front-left
            PxVec3(maxBounds.x, minBounds.y, maxBounds.z), // 5: bottom-front-right
            PxVec3(maxBounds.x, maxBounds.y, maxBounds.z), // 6: top-front-right
            PxVec3(minBounds.x, maxBounds.y, maxBounds.z)  // 7: top-front-left
        };
        
        // 12 edges of the box (24 vertices for 12 lines)
        int edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Bottom face
            {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Top face
            {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Vertical edges
        };
        
        for (int i = 0; i < 12; ++i) {
            // Add start point
            lineVertices.push_back(corners[edges[i][0]].x);
            lineVertices.push_back(corners[edges[i][0]].y);
            lineVertices.push_back(corners[edges[i][0]].z);
            // Add end point
            lineVertices.push_back(corners[edges[i][1]].x);
            lineVertices.push_back(corners[edges[i][1]].y);
            lineVertices.push_back(corners[edges[i][1]].z);
        }
        
        // Red color for soft body bounds
        float color[4] = {1.0f, 0.0f, 0.0f, 0.8f}; // Red for soft bodies
        
        // Draw the wireframe box
        if (!lineVertices.empty()) {
            int numLines = static_cast<int>(lineVertices.size() / 3); // 3 floats per point
            getRoboDK()->DrawGeometry(RoboDK::DrawLines, lineVertices.data(), numLines, color);
        }
    }
}

// Helper function to extract geometry from a RoboDK item
bool PhysXEngine::extractGeometryFromItem(Item item, std::vector<PxVec3>& vertices, std::vector<PxU32>& indices)
{
    if (!item || !getRoboDK()->Valid(item)) {
        qDebug() << "Cannot extract geometry - invalid item";
        return false;
    }

    QString tempFilePath = QDir::tempPath() + QDir::separator() + QString("temp_softbody_%1.stl").arg(item->Name());
    item->Save(tempFilePath);

    if (LoadBinarySTL(tempFilePath.toStdString(), vertices, indices)) {
        // Convert vertices from STL coordinate system to PhysX coordinate system
        std::vector<PxVec3> physxVertices;
        physxVertices.reserve(vertices.size());
        for (const PxVec3& vertex : vertices) {
            physxVertices.push_back(convertSTLToPhysX(vertex));
        }
        vertices = physxVertices; // Replace with converted vertices
        
        QFile::remove(tempFilePath);
        qDebug() << "Successfully extracted geometry from" << item->Name() << "with" << vertices.size() << "vertices and" << indices.size() / 3 << "triangles";
        return true;
    } else {
        QFile::remove(tempFilePath);
        qDebug() << "Failed to extract geometry from" << item->Name();
        return false;
    }
}

// Attachment-specific collision control methods
// 
// These methods allow fine-grained control over collision between soft bodies and specific surfaces.
// Instead of disabling collision with entire objects, you can disable collision only with the
// specific surface that the soft body is attached to.
//
// Usage example:
// 1. Create a soft body and add it to simulation
// 2. Create a rigid body (attachment surface) and add it to simulation  
// 3. Call disableSoftBodyCollisionWithSurface(softBodyItem, attachmentSurfaceItem)
// 4. The soft body will still collide with other parts of the attachment surface's object,
//    but not with the specific surface it's attached to
//
bool PhysXEngine::disableSoftBodyCollisionWithSurface(Item softBodyItem, Item attachmentSurfaceItem)
{
    if (!softBodyItem || !attachmentSurfaceItem || !getRoboDK()->Valid(softBodyItem) || !getRoboDK()->Valid(attachmentSurfaceItem)) {
        qDebug() << "Cannot disable collision - invalid items";
        return false;
    }
    
    // Find the soft body
    auto softBodyIt = m_softBodies.find(softBodyItem);
    if (softBodyIt == m_softBodies.end()) {
        qDebug() << "Soft body not found in simulation";
        return false;
    }
    
    // Find the attachment surface (rigid body)
    auto objectIt = m_objects.find(attachmentSurfaceItem);
    if (objectIt == m_objects.end()) {
        qDebug() << "Attachment surface not found in simulation";
        return false;
    }
    
    PxDeformableVolume* softBody = softBodyIt->second.softBody;
    PxRigidActor* rigidActor = objectIt->second.actor;
    
    if (!softBody || !rigidActor) {
        qDebug() << "Invalid actors found";
        return false;
    }
    
    // Generate a unique attachment group ID
    static PxU32 nextAttachmentGroupId = 1000;
    PxU32 attachmentGroupId = ++nextAttachmentGroupId;
    
    // Update soft body filter data
    PxShape* softBodyShape = softBody->getShape();
    if (softBodyShape) {
        PxFilterData currentFilter = softBodyShape->getSimulationFilterData();
        currentFilter.word1 = attachmentGroupId; // Set attachment group
        currentFilter.word3 = 1; // Mark as attached
        softBodyShape->setSimulationFilterData(currentFilter);
        qDebug() << "Updated soft body filter data - attachment group:" << attachmentGroupId;
    }
    
    // Update rigid body filter data
    PxU32 nbShapes = rigidActor->getNbShapes();
    std::vector<PxShape*> shapes(nbShapes);
    rigidActor->getShapes(shapes.data(), nbShapes);
    
    for (PxShape* rigidShape : shapes) {
        PxFilterData currentFilter = rigidShape->getSimulationFilterData();
        currentFilter.word1 = attachmentGroupId; // Same attachment group
        currentFilter.word3 = 2; // Mark as attachment surface
        rigidShape->setSimulationFilterData(currentFilter);
    }
    
    qDebug() << "Disabled collision between soft body and attachment surface";
    qDebug() << "  Attachment group ID:" << attachmentGroupId;
    qDebug() << "  Soft body:" << softBodyItem->Name();
    qDebug() << "  Attachment surface:" << attachmentSurfaceItem->Name();
    
    return true;
}

bool PhysXEngine::enableSoftBodyCollisionWithSurface(Item softBodyItem, Item attachmentSurfaceItem)
{
    if (!softBodyItem || !attachmentSurfaceItem || !getRoboDK()->Valid(softBodyItem) || !getRoboDK()->Valid(attachmentSurfaceItem)) {
        qDebug() << "Cannot enable collision - invalid items";
        return false;
    }
    
    // Find the soft body
    auto softBodyIt = m_softBodies.find(softBodyItem);
    if (softBodyIt == m_softBodies.end()) {
        qDebug() << "Soft body not found in simulation";
        return false;
    }
    
    // Find the attachment surface (rigid body)
    auto objectIt = m_objects.find(attachmentSurfaceItem);
    if (objectIt == m_objects.end()) {
        qDebug() << "Attachment surface not found in simulation";
        return false;
    }
    
    PxDeformableVolume* softBody = softBodyIt->second.softBody;
    PxRigidActor* rigidActor = objectIt->second.actor;
    
    if (!softBody || !rigidActor) {
        qDebug() << "Invalid actors found";
        return false;
    }
    
    // Reset soft body filter data to default
    PxShape* softBodyShape = softBody->getShape();
    if (softBodyShape) {
        PxFilterData currentFilter = softBodyShape->getSimulationFilterData();
        currentFilter.word1 = 0; // Reset attachment group
        currentFilter.word3 = 0; // Reset attachment status
        softBodyShape->setSimulationFilterData(currentFilter);
        qDebug() << "Reset soft body filter data to default";
    }
    
    // Reset rigid body filter data to default
    PxU32 nbShapes = rigidActor->getNbShapes();
    std::vector<PxShape*> shapes(nbShapes);
    rigidActor->getShapes(shapes.data(), nbShapes);
    
    for (PxShape* rigidShape : shapes) {
        PxFilterData currentFilter = rigidShape->getSimulationFilterData();
        currentFilter.word1 = 0; // Reset attachment group
        currentFilter.word3 = 0; // Reset attachment surface status
        rigidShape->setSimulationFilterData(currentFilter);
    }
    
    qDebug() << "Enabled collision between soft body and attachment surface";
    qDebug() << "  Soft body:" << softBodyItem->Name();
    qDebug() << "  Attachment surface:" << attachmentSurfaceItem->Name();
    
    return true;
}

// Helper function to convert collision mesh indices to simulation mesh indices
void PhysXEngine::convertCollisionToSim(PxDeformableVolume* deformableVolume, PxU32* tetId, PxVec4* barycentric, PxU32 size)
{
    for (PxU32 i = 0; i < size; i++) {
        PxDeformableVolumeExt::convertCollisionToSimulationTet(*deformableVolume, tetId[i], barycentric[i], tetId[i], barycentric[i]);
    }
}

// Function to automatically attach soft body to overlapping rigid bodies
void PhysXEngine::attachSoftBodyToOverlappingRigidBodies(PxDeformableVolume* softBody, const PxTransform& softBodyTransform, SoftBodyData& softBodyData)
{
    if (!softBody || !m_scene) {
        qDebug() << "Cannot attach soft body - invalid soft body or scene";
        return;
    }

    // Get all rigid body actors in the scene
    PxActorTypeFlags actorTypes = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
    PxU32 nbActors = m_scene->getNbActors(actorTypes);
    std::vector<PxActor*> actors(nbActors);
    m_scene->getActors(actorTypes, actors.data(), nbActors);

    // Get the collision mesh of the soft body
    PxTetrahedronMesh* tetMesh = softBody->getCollisionMesh();
    if (!tetMesh) {
        qDebug() << "Cannot attach soft body - no collision mesh found";
        return;
    }

    int attachmentCount = 0;

    // Check each rigid body for overlap
    for (PxActor* actor : actors) {
        PxRigidActor* rigidActor = actor->is<PxRigidActor>();
        if (!rigidActor) continue;

        // Skip the ground plane (static actor at Y=-50)
        if (rigidActor->is<PxRigidStatic>()) {
            PxTransform pose = rigidActor->getGlobalPose();
            if (pose.p.y < -40.0f) { // Ground plane check
                continue;
            }
        }

        // Get the rigid body's bounds
        PxBounds3 rigidBounds = rigidActor->getWorldBounds();
        if (!rigidBounds.isValid()) continue;

        // Get the soft body's bounds
        PxBounds3 softBodyBounds = softBody->getWorldBounds();
        if (!softBodyBounds.isValid()) continue;

        // Check if bounds overlap
        if (!rigidBounds.intersects(softBodyBounds)) {
            continue;
        }

        qDebug() << "Found potential overlap between soft body and rigid body";
        qDebug() << "  Rigid body bounds:" << rigidBounds.minimum.x << rigidBounds.minimum.y << rigidBounds.minimum.z << "to" << rigidBounds.maximum.x << rigidBounds.maximum.y << rigidBounds.maximum.z;
        qDebug() << "  Soft body bounds:" << softBodyBounds.minimum.x << softBodyBounds.minimum.y << softBodyBounds.minimum.z << "to" << softBodyBounds.maximum.x << softBodyBounds.maximum.y << softBodyBounds.maximum.z;

        // Create attachment data arrays
        PxArray<PxU32> tetArray;
        PxArray<PxVec4> baryArray;
        PxArray<PxVec4> posArray;

        // Get the rigid body's transform
        PxTransform rigidTransform = rigidActor->getGlobalPose();
        
        // Get the tetrahedron mesh vertices directly
        const PxVec3* tetVertices = tetMesh->getVertices();
        PxU32 numTetVertices = tetMesh->getNbVertices();
        
        qDebug() << "  Checking" << numTetVertices << "tetrahedron mesh vertices for overlap";
        
        int overlapPointsFound = 0;

        // Geometric overlap detection - check if points actually overlap with rigid body geometry
        const int MIN_OVERLAP_POINTS = 5;  // Require at least 5 overlapping points
        
        qDebug() << "Geometric overlap detection:";
        qDebug() << "  Rigid body bounds:" << rigidBounds.minimum.x << rigidBounds.minimum.y << rigidBounds.minimum.z << "to" << rigidBounds.maximum.x << rigidBounds.maximum.y << rigidBounds.maximum.z;
        
        // Check vertices for actual geometric overlap
        for (PxU32 vertexIndex = 0; vertexIndex < numTetVertices; ++vertexIndex) {
            const PxVec3& localVertex = tetVertices[vertexIndex];
            PxVec3 worldVertex = softBodyTransform.transform(localVertex);
            
            // First check if vertex is inside the bounding box (quick rejection)
            if (rigidBounds.contains(worldVertex)) {
                // Now check actual geometric overlap with rigid body shapes
                bool hasGeometricOverlap = false;
                
                // Get all shapes of the rigid body
                PxU32 nbShapes = rigidActor->getNbShapes();
                std::vector<PxShape*> shapes(nbShapes);
                rigidActor->getShapes(shapes.data(), nbShapes);
                
                // Check each shape for geometric overlap
                for (PxShape* shape : shapes) {
                    if (!shape) continue;
                    
                    // Get the shape's geometry
                    PxGeometryHolder geometry = shape->getGeometry();
                    PxTransform shapePose = shape->getLocalPose();
                    PxTransform worldPose = rigidTransform * shapePose;
                    
                    // Create a small sphere at the vertex position for overlap testing
                    PxSphereGeometry sphereGeom(1.0f); // 1mm radius sphere
                    PxTransform spherePose(worldVertex);
                    
                    // Test for geometric overlap
                    if (PxGeometryQuery::overlap(sphereGeom, spherePose, geometry.any(), worldPose)) {
                        hasGeometricOverlap = true;
                        break;
                    }
                }
                
                // Only proceed if we have actual geometric overlap
                if (hasGeometricOverlap) {
                    // Find the tetrahedron containing this vertex
                    PxVec4 bary;
                    PxI32 tet = PxTetrahedronMeshExt::findTetrahedronContainingPoint(tetMesh, localVertex, bary);
                    
                    if (tet >= 0) {
                        tetArray.pushBack(tet);
                        baryArray.pushBack(bary);
                        
                        // For the rigid body side, we need to ensure the coordinates are in the correct coordinate system
                        // The rigid body expects coordinates in its local coordinate system
                        PxVec3 rigidLocalPoint = rigidTransform.getInverse().transform(worldVertex);
                        posArray.pushBack(PxVec4(rigidLocalPoint, 0.0f));
                        
                        overlapPointsFound++;
                        
                        // Store attachment points for debug visualization
                        // Store in world coordinates for proper visualization
                        softBodyData.attachmentPoints.push_back(worldVertex);
                        softBodyData.rigidBodyPoints.push_back(worldVertex);
                        
                        // Debug output for first few points
                        if (overlapPointsFound <= 5) {
                            qDebug() << "    Found geometric overlap vertex" << overlapPointsFound << ":";
                            qDebug() << "      Local vertex:" << localVertex.x << localVertex.y << localVertex.z;
                            qDebug() << "      World vertex:" << worldVertex.x << worldVertex.y << worldVertex.z;
                            qDebug() << "      Rigid local point:" << rigidLocalPoint.x << rigidLocalPoint.y << rigidLocalPoint.z;
                            qDebug() << "      Tetrahedron:" << tet << "Barycentric:" << bary.x << bary.y << bary.z << bary.w;
                        }
                    }
                }
            }
        }
        
        // Simple overlap check
        bool sufficientOverlap = (overlapPointsFound >= MIN_OVERLAP_POINTS);
        
        qDebug() << "=== GEOMETRIC OVERLAP ANALYSIS ===";
        qDebug() << "Total vertices checked:" << numTetVertices;
        qDebug() << "Total geometrically overlapping vertices:" << overlapPointsFound;
        qDebug() << "Minimum required overlap points:" << MIN_OVERLAP_POINTS;
        
        qDebug() << "--- Final Result ---";
        qDebug() << "  Sufficient overlap for attachment:" << (sufficientOverlap ? "YES" : "NO");
        qDebug() << "=== END OVERLAP ANALYSIS ===";
        
        // Only proceed if we have sufficient geometric overlap points
        if (!sufficientOverlap) {
            qDebug() << "Insufficient geometric overlap points - skipping attachment creation";
            qDebug() << "  Reason: Need at least" << MIN_OVERLAP_POINTS << "geometrically overlapping points";
            overlapPointsFound = 0;  // Reset to prevent attachment creation
        }

        // If we found overlapping points, create the attachment
        if (overlapPointsFound > 0) {
            qDebug() << "Found" << overlapPointsFound << "overlapping points, creating attachment";
            // Convert collision mesh indices to simulation mesh indices
            convertCollisionToSim(softBody, tetArray.begin(), baryArray.begin(), tetArray.size());

            // Create attachment data
            PxDeformableAttachmentData desc;
            
            // Soft body side
            desc.actor[0] = softBody;
            desc.type[0] = PxDeformableAttachmentTargetType::eTETRAHEDRON;
            desc.indices[0].data = tetArray.begin();
            desc.indices[0].count = tetArray.size();
            desc.coords[0].data = baryArray.begin();
            desc.coords[0].count = baryArray.size();

            // Rigid body side
            desc.actor[1] = rigidActor;
            desc.type[1] = PxDeformableAttachmentTargetType::eRIGID;
            desc.coords[1].data = posArray.begin();
            desc.coords[1].count = posArray.size();

            // Create the attachment
            PxDeformableAttachment* attachment = m_physics->createDeformableAttachment(desc);
            if (attachment) {
                attachmentCount++;
                qDebug() << "Successfully created attachment" << attachmentCount << "between soft body and rigid body";
                qDebug() << "  Rigid body type:" << (rigidActor->is<PxRigidDynamic>() ? "Dynamic" : "Static");
                qDebug() << "  Overlap points:" << overlapPointsFound;
                
                // Update filter data to enable attachment-specific collision filtering
                // Generate a unique attachment group ID
                PxU32 attachmentGroupId = attachmentCount + 1000; // Start from 1000 to avoid conflicts
                
                // Update soft body filter data
                PxShape* softBodyShape = softBody->getShape();
                if (softBodyShape) {
                    PxFilterData currentFilter = softBodyShape->getSimulationFilterData();
                    currentFilter.word1 = attachmentGroupId; // Set attachment group
                    currentFilter.word3 = 1; // Mark as attached
                    softBodyShape->setSimulationFilterData(currentFilter);
                }
                
                // Update rigid body filter data to mark it as an attachment surface
                PxU32 nbShapes = rigidActor->getNbShapes();
                std::vector<PxShape*> shapes(nbShapes);
                rigidActor->getShapes(shapes.data(), nbShapes);
                
                for (PxShape* rigidShape : shapes) {
                    PxFilterData currentFilter = rigidShape->getSimulationFilterData();
                    currentFilter.word1 = attachmentGroupId; // Same attachment group
                    currentFilter.word3 = 2; // Mark as attachment surface
                    rigidShape->setSimulationFilterData(currentFilter);
                }
                
                qDebug() << "  Updated filter data - attachment group:" << attachmentGroupId;
                qDebug() << "  Collision disabled between soft body and attachment surface";
                
                // Store the attachment for cleanup later
                // Note: You might want to add a member variable to store attachments
                // For now, we'll just log the creation
            } else {
                qDebug() << "Failed to create deformable attachment";
            }
        } else {
            qDebug() << "No overlapping points found between soft body and rigid body";
        }
    }

    if (attachmentCount > 0) {
        qDebug() << "Created" << attachmentCount << "attachments for soft body";
        qDebug() << "Note: Soft body is now attached to overlapping rigid bodies at contact points";
        
        // Mark that we have attachment points for debug visualization
        softBodyData.hasAttachmentPoints = true;
        qDebug() << "attachSoftBodyToOverlappingRigidBodies: Stored" << softBodyData.attachmentPoints.size() << "attachment points for debug visualization";
        qDebug() << "attachSoftBodyToOverlappingRigidBodies: hasAttachmentPoints set to:" << softBodyData.hasAttachmentPoints;
    } else {
        qDebug() << "No attachments created for soft body";
        qDebug() << "Note: No overlapping rigid bodies found for attachment";
    }
}



    // Function to draw attachment points for all soft bodies in the scene
    void PhysXEngine::drawSoftBodyAttachmentPoints()
    {
        qDebug() << "drawSoftBodyAttachmentPoints called - robodk valid:" << (getRoboDK() != nullptr);
        
        if (!getRoboDK()) {
            qDebug() << "drawSoftBodyAttachmentPoints: Early return - RoboDK invalid";
            return;
        }

    // Draw attachment points for all soft bodies in simulation
    static int debugCount = 0; // Reset debug counter for each visualization frame
    static int rigidDebugCount = 0; // Reset rigid debug counter for each visualization frame
    
    qDebug() << "drawSoftBodyAttachmentPoints: Processing" << m_softBodies.size() << "soft bodies";
    
    for (auto& pair : m_softBodies) {
        const SoftBodyData& softBodyData = pair.second;
        
        qDebug() << "drawSoftBodyAttachmentPoints: Checking soft body - in simulation:" << softBodyData.isInSimulation << "has attachment points:" << softBodyData.hasAttachmentPoints << "attachment points count:" << softBodyData.attachmentPoints.size();
        
        if (softBodyData.isInSimulation && softBodyData.hasAttachmentPoints) {
            // Draw stored attachment points (green spheres)
            if (!softBodyData.attachmentPoints.empty()) {
                std::vector<float> sphereVertices;
                sphereVertices.reserve(softBodyData.attachmentPoints.size() * 3);

                for (const PxVec3& worldPoint : softBodyData.attachmentPoints) {
                    // Convert from PhysX to RoboDK coordinate system
                    PxVec3 stlPoint = convertPhysXToSTL(worldPoint);
                    sphereVertices.push_back(stlPoint.x);
                    sphereVertices.push_back(stlPoint.y);
                    sphereVertices.push_back(stlPoint.z);
                    
                    // Debug: Print coordinate conversion for first few points
                    static int debugCount = 0;
                    if (debugCount < 5) {
                        qDebug() << "Debug visualization - Attachment point" << debugCount << ":";
                        qDebug() << "  World point:" << worldPoint.x << worldPoint.y << worldPoint.z;
                        qDebug() << "  STL coordinates for visualization:" << stlPoint.x << stlPoint.y << stlPoint.z;
                        debugCount++;
                    }
                }

                // Draw spheres for attachment points (green color)
                float attachmentColor[4] = {0.0f, 1.0f, 0.0f, 0.8f}; // Green for attachment points
                float sphereRadius = 3.0f; // 3mm radius for visibility

                if (!sphereVertices.empty()) {
                    getRoboDK()->DrawGeometry(RoboDK::DrawSpheres, sphereVertices.data(), sphereVertices.size() / 3, attachmentColor, sphereRadius);
                }
            }

            // Draw stored rigid body points (red spheres)
            if (!softBodyData.rigidBodyPoints.empty()) {
                std::vector<float> sphereVertices;
                sphereVertices.reserve(softBodyData.rigidBodyPoints.size() * 3);

                for (const PxVec3& worldPoint : softBodyData.rigidBodyPoints) {
                    // Convert from PhysX to RoboDK coordinate system
                    PxVec3 stlPoint = convertPhysXToSTL(worldPoint);
                    sphereVertices.push_back(stlPoint.x);
                    sphereVertices.push_back(stlPoint.y);
                    sphereVertices.push_back(stlPoint.z);
                    
                    // Debug: Print coordinate conversion for first few points
                    static int rigidDebugCount = 0;
                    if (rigidDebugCount < 5) {
                        qDebug() << "Debug visualization - Rigid body point" << rigidDebugCount << ":";
                        qDebug() << "  World point:" << worldPoint.x << worldPoint.y << worldPoint.z;
                        qDebug() << "  STL coordinates for visualization:" << stlPoint.x << stlPoint.y << stlPoint.z;
                        rigidDebugCount++;
                    }
                }

                // Draw spheres for rigid body points (red color)
                float rigidColor[4] = {1.0f, 0.0f, 0.0f, 0.8f}; // Red for rigid body points
                float sphereRadius = 2.0f; // 2mm radius for visibility

                if (!sphereVertices.empty()) {
                    getRoboDK()->DrawGeometry(RoboDK::DrawSpheres, sphereVertices.data(), sphereVertices.size() / 3, rigidColor, sphereRadius);
                }
            }

            // Debug output
            if (!softBodyData.attachmentPoints.empty() || !softBodyData.rigidBodyPoints.empty()) {
                qDebug() << "Debug visualization: Drawing" << softBodyData.attachmentPoints.size() << "attachment points (green) and" << softBodyData.rigidBodyPoints.size() << "rigid body points (red)";
                
                // Print detailed information about attachment points
                qDebug() << "=== ATTACHMENT POINTS DETAILS ===";
                qDebug() << "Soft body attachment points (green spheres):";
                for (size_t i = 0; i < softBodyData.attachmentPoints.size() && i < 10; ++i) { // Show first 10 points
                    const PxVec3& point = softBodyData.attachmentPoints[i];
                    PxVec3 stlPoint = convertPhysXToSTL(point);
                    qDebug() << "  Point" << i << "PhysX:" << point.x << point.y << point.z << "STL:" << stlPoint.x << stlPoint.y << stlPoint.z;
                }
                if (softBodyData.attachmentPoints.size() > 10) {
                    qDebug() << "  ... and" << (softBodyData.attachmentPoints.size() - 10) << "more attachment points";
                }
                
                qDebug() << "Rigid body connection points (red spheres):";
                for (size_t i = 0; i < softBodyData.rigidBodyPoints.size() && i < 10; ++i) { // Show first 10 points
                    const PxVec3& point = softBodyData.rigidBodyPoints[i];
                    PxVec3 stlPoint = convertPhysXToSTL(point);
                    qDebug() << "  Point" << i << "PhysX:" << point.x << point.y << point.z << "STL:" << stlPoint.x << stlPoint.y << stlPoint.z;
                }
                if (softBodyData.rigidBodyPoints.size() > 10) {
                    qDebug() << "  ... and" << (softBodyData.rigidBodyPoints.size() - 10) << "more rigid body points";
                }
                qDebug() << "=== END ATTACHMENT POINTS DETAILS ===";
            } else {
                qDebug() << "No attachment points found for visualization";
            }
        }
    }
}




