#pragma once

#include "../Common/CadNode.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// A compact PhysX scene dedicated to conveyor products. Static collision is cooked directly from
// the same generated frame/feet/roller meshes the renderer displays; there is deliberately no
// second hidden deck representation that can drift away from the visible conveyor.
class ConveyorPhysics {
public:
    using BodyHandle = uint64_t;
    static constexpr double fixedStepSeconds() { return 1.0 / 60.0; }
    // Capability, not current scene state. Web builds presently omit the native PhysX SDK;
    // callers use this to resolve requested physics behavior to the logical backend instead of
    // accepting a mode whose product creation can never succeed.
    static bool backendAvailable();

    struct CollisionMesh {
        CadTransform pose;
        std::vector<float> verticesMm;
        std::vector<uint32_t> indices;
        // Positive components select a native PxBoxGeometry instead of a cooked triangle mesh.
        CadVec3 boxHalfExtentsMm;
        CadVec3 surfaceLinearVelocityMmS;
        CadVec3 surfaceCenterMm;
        CadVec3 surfaceAngularVelocityRadS;
    };

    struct RuntimeStats {
        uint32_t staticActors = 0;
        uint32_t dynamicBodies = 0;
        uint32_t activeDynamicBodies = 0;
        uint32_t contactPairs = 0;
        uint32_t convexBoxPairs = 0;
        uint32_t convexTrianglePairs = 0;
        uint32_t convexConvexPairs = 0;
        uint32_t dispatcherThreads = 0;
        uint32_t solverBatchSize = 0;
        double averageWakeMs = 0.0;
        double averageSimulateMs = 0.0;
        double averageSnapshotMs = 0.0;
        double minimumBodyYmm = 0.0;
    };

    ConveyorPhysics();
    ~ConveyorPhysics();
    ConveyorPhysics(ConveyorPhysics&&) noexcept;
    ConveyorPhysics& operator=(ConveyorPhysics&&) noexcept;
    ConveyorPhysics(const ConveyorPhysics&) = delete;
    ConveyorPhysics& operator=(const ConveyorPhysics&) = delete;

    // Read when the scene is created, so it must be set before reset() to take effect. Defaults to
    // Y-down, the CAD world the conveyors were built in; a Z-up host sets its own here instead of
    // rotating every pose it hands over.
    void setGravity(const CadVec3& metresPerSecondSquared);

    bool reset(const std::vector<CollisionMesh>& meshes = {},
               std::string* errorMessage = nullptr);
    BodyHandle addCylinder(const CadTransform& worldPose, double radiusMm, double heightMm,
                           std::string* errorMessage = nullptr);
    BodyHandle addBox(const CadTransform& worldPose, const CadVec3& halfExtentsMm,
                      std::string* errorMessage = nullptr);
    BodyHandle addConvex(const CadTransform& worldPose, const std::vector<CadVec3>& verticesMm,
                         double densityKgM3 = 1200.0, std::string* errorMessage = nullptr);
    // Kinematic mechanism shapes share the product scene with conveyors. They are moved by the
    // actuator runtime and hold products through ordinary PhysX contact/friction; they are never
    // converted into attachments or hidden joints.
    BodyHandle addKinematicBox(const CadTransform& worldPose, const CadVec3& halfExtentsMm,
                               std::string* errorMessage = nullptr);
    BodyHandle addKinematicConvex(const CadTransform& worldPose,
                                  const std::vector<CadVec3>& verticesMm,
                                  std::string* errorMessage = nullptr);
    bool setKinematicPose(BodyHandle body, const CadTransform& worldPose);
    // Applies a friction-bounded spring/damper pad force to a dynamic product. normalEffortN is the
    // actuator's clamp force; the shared pad material determines available carrying force. Used only
    // after opposing jaws acquire the body; this does not constrain, reparent, or make it kinematic.
    bool applyGripForce(BodyHandle body, const CadTransform& targetWorldPose,
                        const CadVec3& targetLinearVelocityMmS,
                        double stiffnessNPerM, double dampingNsPerM, double normalEffortN);
    void clearGripForce(BodyHandle body);
    bool bodyPose(BodyHandle body, CadTransform* worldPose) const;
    void removeBody(BodyHandle body);
    // Native interactive mode advances on a dedicated worker. The main thread consumes completed
    // fixed steps and applies the latest cached poses without waiting for PhysX.
    void startAsync(double speedFactor);
    void stopAsync();
    void setAsyncPaused(bool paused);
    void setAsyncSpeedFactor(double speedFactor);
    uint64_t consumeCompletedAsyncSteps();
    bool asyncActive() const;
    bool asyncAchievedRate(double* outRate) const;
    double asyncDroppedSimSeconds() const;
    RuntimeStats runtimeStats() const;
    void step(double elapsedSeconds);
    bool isActive() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
