#include "LiveRunDriver.h"

#include <algorithm>
#include <thread>
#include <vector>

// Lock order is m_stepMutex then m_publishMutex; snapshot reads take only m_publishMutex.

void LiveRunDriver::setArms(std::vector<RobotProgramSimulator*> arms) {
    // The vector's shape is read by snapshot() under the publish lock, so it is rebuilt under that
    // lock. Exclusion against a slice in flight is the caller's control lock.
    std::lock_guard<std::mutex> publishGuard(m_publishMutex);
    m_arms.clear();
    m_arms.reserve(arms.size());
    for (RobotProgramSimulator* simulator : arms) {
        Arm arm;
        arm.simulator = simulator;
        m_arms.push_back(arm);
    }
    // Replace snapshots that may reference removed instances.
    for (Arm& arm : m_arms) publishLocked(arm);
}

size_t LiveRunDriver::armCount() const {
    std::lock_guard<std::mutex> guard(m_publishMutex);
    return m_arms.size();
}

void LiveRunDriver::setSpeedFactor(double factor) {
    std::lock_guard<std::mutex> guard(m_stepMutex);
    // Discard backlog accumulated at a different speed factor.
    if (factor != m_speedFactor) {
        m_pendingSimSeconds = 0.0;
        m_rateWindow.reset();
        std::lock_guard<std::mutex> publishGuard(m_publishMutex);
        m_rateValid = false;
        m_rate = 0.0;
    }
    m_speedFactor = factor > 0.0 ? factor : 0.0;
}

void LiveRunDriver::setPaused(bool paused) {
    std::lock_guard<std::mutex> guard(m_stepMutex);
    if (m_paused == paused) return;
    m_paused = paused;
    // Neither side of a pause owes the other any wall-clock time.
    m_clockValid = false;
    m_pendingSimSeconds = 0.0;
}

bool LiveRunDriver::paused() const {
    std::lock_guard<std::mutex> guard(m_stepMutex);
    return m_paused;
}

void LiveRunDriver::resetClock() {
    std::lock_guard<std::mutex> guard(m_stepMutex);
    m_clockValid = false;
    m_pendingSimSeconds = 0.0;
    m_rateWindow.reset();
    std::lock_guard<std::mutex> publishGuard(m_publishMutex);
    m_droppedSimSeconds = 0.0;
    m_rateValid = false;
    m_rate = 0.0;
}

void LiveRunDriver::publishLocked(Arm& arm) {
    if (!arm.simulator) {
        arm.published = Snapshot();
        return;
    }
    arm.published.state = arm.simulator->liveRunState();
    arm.published.joints = arm.simulator->liveRunJoints();
    // A run that has begun has joints worth applying even before its first sample: beginLiveRun seeds
    // them from where the arm is standing.
    if (arm.published.state.running) arm.published.jointsValid = true;
}

void LiveRunDriver::publishFrom(size_t armIndex) {
    std::lock_guard<std::mutex> publishGuard(m_publishMutex);
    if (armIndex >= m_arms.size()) return;
    publishLocked(m_arms[armIndex]);
}

bool LiveRunDriver::snapshot(size_t armIndex, Snapshot* out) const {
    if (!out) return false;
    std::lock_guard<std::mutex> guard(m_publishMutex);
    if (armIndex >= m_arms.size()) return false;
    *out = m_arms[armIndex].published;
    return true;
}

bool LiveRunDriver::achievedRate(double* outRate) const {
    std::lock_guard<std::mutex> guard(m_publishMutex);
    if (!m_rateValid) return false;
    if (outRate) *outRate = m_rate;
    return true;
}

double LiveRunDriver::droppedSimSeconds() const {
    std::lock_guard<std::mutex> guard(m_publishMutex);
    return m_droppedSimSeconds;
}

LiveRunDriver::StepResult LiveRunDriver::stepOnce() {
    std::lock_guard<std::mutex> guard(m_stepMutex);
    StepResult result;

    bool anyRunning = false;
    for (const Arm& arm : m_arms) {
        if (arm.simulator && arm.simulator->liveRunState().running) anyRunning = true;
    }
    result.anyRunning = anyRunning;

    if (m_paused) {
        m_clockValid = false;
        m_pendingSimSeconds = 0.0;
        return result;
    }

    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (!m_clockValid) {
        m_clockValid = true;
        m_lastTick = now;
        return result;
    }
    const double wallSeconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - m_lastTick).count();
    m_lastTick = now;

    if (!anyRunning) {
        // No clock debt accrues while the cell is idle, so starting a run does not begin with a backlog
        // the size of however long the window sat there.
        m_pendingSimSeconds = 0.0;
        // The window is history of a run that has ended, so it does not carry into the next one.
        m_rateWindow.reset();
        std::lock_guard<std::mutex> publishGuard(m_publishMutex);
        m_rateValid = false;
        return result;
    }

    m_pendingSimSeconds += std::max(0.0, wallSeconds) * m_speedFactor;
    const double maxPending = kMaxCatchUpWallSeconds * m_speedFactor;
    if (m_pendingSimSeconds > maxPending) {
        const double dropped = m_pendingSimSeconds - maxPending;
        m_pendingSimSeconds = maxPending;
        std::lock_guard<std::mutex> publishGuard(m_publishMutex);
        m_droppedSimSeconds += dropped;
    }

    const double slice = std::min(m_pendingSimSeconds, kSliceSimSeconds);
    if (!(slice > 0.0)) return result;

    // Every arm gets the same slice of simulated time. Which arms, and where each one started, settled
    // before any of them move: the stepping below runs concurrently and must not be reading state
    // another thread is writing, even though the simulators are disjoint.
    struct Stepping {
        Arm* arm = nullptr;
        double before = 0.0;
        RobotProgramSimulator::LiveRunStep step;
    };
    std::vector<Stepping> stepping;
    stepping.reserve(m_arms.size());
    for (Arm& arm : m_arms) {
        if (!arm.simulator || !arm.simulator->liveRunState().running) continue;
        Stepping entry;
        entry.arm = &arm;
        entry.before = arm.simulator->liveRunState().runSeconds;
        stepping.push_back(entry);
    }

    // Concurrent across arms, joined here, so the cell has advanced over one interval by the time this
    // returns. Safe because the simulators are wholly disjoint - each owns its live run, its planner
    // scratch and its base segment buffer - and the shared header has no mutable statics left.
    bool fannedOut = false;
#ifndef __EMSCRIPTEN__
    if (stepping.size() > 1 && slice >= kParallelSliceSimSeconds) {
        std::vector<std::thread> workers;
        workers.reserve(stepping.size() - 1);
        for (size_t i = 1; i < stepping.size(); ++i) {
            Stepping* entry = &stepping[i];
            workers.emplace_back([entry, slice]() {
                entry->step = entry->arm->simulator->stepLiveRun(slice);
            });
        }
        stepping[0].step = stepping[0].arm->simulator->stepLiveRun(slice);
        for (std::thread& worker : workers) worker.join();
        fannedOut = true;
    }
#endif
    if (!fannedOut) {
        for (Stepping& entry : stepping) {
            entry.step = entry.arm->simulator->stepLiveRun(slice);
        }
    }

    double slowestDelivered = -1.0;
    bool advancedSomething = false;
    for (Stepping& entry : stepping) {
        Arm& arm = *entry.arm;
        {
            std::lock_guard<std::mutex> publishGuard(m_publishMutex);
            arm.published.state = arm.simulator->liveRunState();
            if (entry.step.jointsChanged) {
                arm.published.joints = arm.simulator->liveRunJoints();
                arm.published.jointsValid = true;
            }
        }
        // An arm that has just finished delivered less than was asked for a reason that has nothing to
        // do with the CPU, and counting it would report a bottleneck that is not there.
        if (!entry.step.running) continue;
        const double delivered = arm.simulator->liveRunState().runSeconds - entry.before;
        if (slowestDelivered < 0.0 || delivered < slowestDelivered) slowestDelivered = delivered;
        if (entry.step.jointsChanged) advancedSomething = true;
    }

    m_pendingSimSeconds = std::max(0.0, m_pendingSimSeconds - slice);
    // Sub-period slices remain banked until a trajectory sample advances.
    result.didWork = advancedSomething;

    if (slowestDelivered >= 0.0) {
        double rollingRate = 0.0;
        if (m_rateWindow.addSample(wallSeconds, slowestDelivered, &rollingRate)) {
            std::lock_guard<std::mutex> publishGuard(m_publishMutex);
            m_rate = rollingRate;
            m_rateValid = true;
        }
    }
    return result;
}
