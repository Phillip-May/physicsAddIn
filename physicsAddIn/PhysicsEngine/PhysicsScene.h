#ifndef PHYSICSSCENE_H
#define PHYSICSSCENE_H

#include <memory>
#include <QObject>
#include "PxPhysicsAPI.h"

/**
 * @brief Manages a PhysX scene with ground plane and simulation settings
 * 
 * This class encapsulates the PhysX scene and provides methods
 * for scene management, ground plane creation, and simulation configuration.
 */
class PhysicsScene : public QObject
{
    Q_OBJECT

public:
    explicit PhysicsScene(PxPhysics* physics, PxMaterial* material, QObject* parent = nullptr);
    ~PhysicsScene();

    // Scene management
    bool create();
    void destroy();
    bool isCreated() const;
    
    // Simulation control
    void simulate(float deltaTime);
    void fetchResults(bool block = true);
    
    // Actor management
    void addActor(PxRigidActor* actor);
    void removeActor(PxRigidActor* actor);
    
    // Scene configuration
    void setGravity(const PxVec3& gravity);
    PxVec3 getGravity() const;
    
    void setSolverType(PxSolverType::Enum solverType);
    PxSolverType::Enum getSolverType() const;
    
    void enablePCM(bool enable);
    bool isPCMEnabled() const;
    
    void enableStabilization(bool enable);
    bool isStabilizationEnabled() const;
    
    // Ground plane management
    bool createGroundPlane();
    void removeGroundPlane();
    bool hasGroundPlane() const;
    PxRigidStatic* getGroundPlane() const;
    
    // Scene queries
    int getActorCount() const;
    std::vector<PxActor*> getAllActors() const;
    std::vector<PxRigidDynamic*> getDynamicActors() const;
    
    // Debug and monitoring
    void enableDebugVisualization(bool enable);
    bool isDebugVisualizationEnabled() const;
    
    // Scene query update control
    void setSceneQueryUpdateMode(PxSceneQueryUpdateMode::Enum mode);
    PxSceneQueryUpdateMode::Enum getSceneQueryUpdateMode() const;

signals:
    void sceneCreated();
    void sceneDestroyed();
    void actorAdded(PxRigidActor* actor);
    void actorRemoved(PxRigidActor* actor);
    void simulationError(const QString& error);

private:
    PxPhysics* m_physics;
    PxMaterial* m_material;
    PxScene* m_scene;
    PxRigidStatic* m_groundPlane;
    
    // Configuration
    PxVec3 m_gravity;
    PxSolverType::Enum m_solverType;
    bool m_pcmEnabled;
    bool m_stabilizationEnabled;
    bool m_debugVisualizationEnabled;
    
    // Scene settings
    PxSceneQueryUpdateMode::Enum m_sceneQueryUpdateMode;
    
    // Helper methods
    bool createSceneDesc(PxSceneDesc& sceneDesc);
    void configureSceneFlags(PxSceneDesc& sceneDesc);
    void setupPVD();
    
    // Ground plane configuration
    static constexpr float GROUND_PLANE_Y = -50.0f;
    static constexpr float GROUND_PLANE_SIZE = 10000.0f;
    static constexpr float GROUND_PLANE_THICKNESS = 0.1f;
    static constexpr float GROUND_CONTACT_OFFSET = 0.02f;
    static constexpr float GROUND_REST_OFFSET = 0.01f;
};

#endif // PHYSICSSCENE_H 