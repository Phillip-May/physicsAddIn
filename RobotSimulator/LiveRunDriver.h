#pragma once

#include "RobotProgramSimulator.h"
#include "RollingRateWindow.h"

#include <chrono>
#include <cstddef>
#include <mutex>
#include <vector>

class LiveRunDriver {
public:
    // Copy published by the worker for main-thread reads.
    struct Snapshot {
        RobotProgramSimulator::LiveRunState state;
        std::array<double, 6> joints{};
        bool jointsValid = false;
    };

    struct StepResult {
        bool anyRunning = false;
        bool didWork = false;
    };

    // Arms are non-owning and ordered like the scene. Requires the control lock.
    void setArms(std::vector<RobotProgramSimulator*> arms);
    size_t armCount() const;

    void setSpeedFactor(double factor);

    void setPaused(bool paused);
    bool paused() const;

    StepResult stepOnce();

    // Simulated seconds delivered per wall second.
    bool achievedRate(double* outRate) const;
    double droppedSimSeconds() const;

    void resetClock();

    bool snapshot(size_t armIndex, Snapshot* out) const;
    // Republishes after a main-thread state change. Requires the control lock.
    void publishFrom(size_t armIndex);

    std::unique_lock<std::mutex> lockForControl() { return std::unique_lock<std::mutex>(m_stepMutex); }

private:
    struct Arm {
        RobotProgramSimulator* simulator = nullptr;
        Snapshot published;
    };

    void publishLocked(Arm& arm);

    static constexpr double kSliceSimSeconds = 1.0;
    // Parallel work below this threshold costs more than it saves.
    static constexpr double kParallelSliceSimSeconds = 0.02;
    static constexpr double kMaxCatchUpWallSeconds = 0.5;
    mutable std::mutex m_stepMutex;
    mutable std::mutex m_publishMutex;

    std::vector<Arm> m_arms;
    double m_speedFactor = 1.0;
    bool m_paused = false;

    bool m_clockValid = false;
    std::chrono::steady_clock::time_point m_lastTick;
    double m_pendingSimSeconds = 0.0;
    double m_droppedSimSeconds = 0.0;

    // The slowest running arm determines the displayed rate.
    RollingRateWindow m_rateWindow;
    bool m_rateValid = false;
    double m_rate = 0.0;
};
