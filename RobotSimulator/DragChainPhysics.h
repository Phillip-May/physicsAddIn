#pragma once

#include <cstddef>
#include <memory>
#include <string>

struct CadNode;

// Desktop implementation uses PhysX 5.9. The same interface is a no-op on wasm, where the
// deterministic DragChainPoseController remains the renderer's source of link poses.
class DragChainPhysics {
public:
    DragChainPhysics();
    ~DragChainPhysics();
    DragChainPhysics(DragChainPhysics&&) noexcept;
    DragChainPhysics& operator=(DragChainPhysics&&) noexcept;
    DragChainPhysics(const DragChainPhysics&) = delete;
    DragChainPhysics& operator=(const DragChainPhysics&) = delete;

    bool bind(CadNode* chainNode, std::string* errorMessage = nullptr);
    void step(double elapsedSeconds);
    bool isActive() const;
    bool isSimulated() const;
    double maxAbsJointRotationDeg(size_t* jointIndex = nullptr) const;
    size_t baseCollisionHullCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
