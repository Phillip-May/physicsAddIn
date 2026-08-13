#pragma once
#include "StringUtil.h"

#include "RobotMotionCore.h"
#include "RobotProgramSimulator.h"
#include "SerialFraming.h"
#include "SerialPort.h"

#include "JsonCompat.h"

#include <array>
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

// Firmware link for the Hardware IO panel: transport, the JSON protocol and the state the UI
// reads back. Deliberately free of any UI so the protocol can be reasoned about, and tested,
// without a window. Transport is Common/SerialPort and reception is polled once per frame,
// which suits an immediate-mode panel rebuilt from current state every frame.
class HardwareIo {
public:
    struct Status {
        bool valid = false;
        // Where the robot actually is. Not read from a "joints_deg" field - robot_status has no
        // such key. The firmware reports position inside its mastering object, as
        // position_deg_x10000000 (and a coarser position_deg_x100), because it has no floating
        // point formatting; decoding those is the only way to know the pose.
        std::array<double, 6> jointsDeg{};
        std::array<bool, 6> jointMastered{};
        std::array<int, 6> currentSteps{};
        std::array<int, 6> zeroSteps{};
        std::array<int, 6> masterLimitSteps{};
        std::array<double, 6> reportedStepsPerDegree{};
        std::array<double, 6> reportedOffsetDeg{};
        std::array<int, 6> reportedMasterDirection{{1, -1, 1, -1, -1, 1}};
        std::array<bool, 6> reportedJogDirInvert{{false, true, true, false, true, true}};
        bool configStored = false;
        bool allMastered = false;
        bool jogArmed = false;
        bool limitsActive = false;
        // The firmware is running a mastering sweep; jog and mastering controls stay disabled.
        bool masteringActive = false;
        std::array<bool, 6> limitPressed{};
        uint32_t statusCount = 0;
        double firmwareSeconds = 0.0;
        int limitPollMs = 0;
        bool streaming = false;
    };

    void refreshPorts();
    const std::vector<SerialPort::PortInfo>& ports() const { return m_ports; }

    int suggestedPortIndex() const;

    bool connectTo(const std::string& portName);
    void disconnect();
    bool isConnected() const { return m_serial.isOpen(); }
    const std::string& connectedPort() const { return m_serial.portName(); }

    // Drains and dispatches whatever has arrived. Call once per frame.
    void poll();

    using MessageCallback = std::function<void(const std::string& line, const Json& object)>;
    void setMessageCallback(MessageCallback callback) { m_messageCallback = std::move(callback); }

    void sendCommand(const Json& command);

    void requestStatus();
    void setStatusStreaming(bool enabled, int periodMs);
    void setLimitPollPeriod(int periodMs);
    void armJog(bool armed);
    // steps is the firmware's jog magnitude, 1..500. Not microseconds.
    void jogJoint(int jointIndex, int direction, int steps);
    void stopMotion();

    // Mastering. Both require jog to be armed on the firmware side; the panel enforces that
    // before calling.
    void masterJoint(int joint, double stepsPerDegree, double offsetDeg, int backoffSteps,
                     int masterDirection);

    void sendJogDirectionInvert(const std::array<bool, 6>& invert);
    void setJointZero(int joint, double stepsPerDegree, double offsetDeg);

    // Queues a joint or linear move to the pose the simulator is currently showing. The linear
    // form needs the tool transform as well as the joints: the firmware plans a linear move
    // against the tool path and uses the joints only to pick which IK solution to land in, and
    // rejects the command with bad_pose if target_tcp is absent.
    void moveJointsTo(const std::array<double, 6>& jointsRad, double speedDegPerSec);
    void moveLinearTo(const std::array<double, 6>& jointsRad,
                      const RobotMotionCore::Transform& targetTcp,
                      double speedMmPerSec);

    // True while the robot is moving on our behalf, whether from a single move or a program run.
    bool moveInFlight() const { return m_moveActive || m_programActive || m_finalStatusPending; }

    // True when the last robot_status reported all six joints mastered. Running a program, or
    // any pose move, requires it: an unmastered joint has no step-to-degree reference, so the
    // firmware would drive to the wrong place.
    bool allJointsMastered() const { return m_status.allMastered; }

    // Program run. The firmware takes one motion command at a time and acknowledges it twice:
    // movej_queued/movel_queued when it accepts the command into the lookahead queue, and
    // movej_done/movel_done when the motion finishes. So a program is sent as a sequence,
    // advanced by those acks, not written out in one go - the queue holds only
    // kMotionLookaheadQueuedCommands entries and rejects the rest with queue_full.
    bool startProgram(std::vector<Json> commands, std::string* errorMessage);
    void cancelProgram(const std::string& reason, bool sendStop);
    bool programActive() const { return m_programActive; }
    int programCompletedCount() const { return m_programCompleted; }
    int programTotalCount() const { return static_cast<int>(m_programCommands.size()); }
    int programExecutingRow() const;
    int programPendingRow() const;

    // The kinematic model and planner limits the firmware plans against. Without it the firmware
    // rejects every movej and movel with model_not_loaded, so this has to be uploaded before any
    // motion - and re-uploaded on every connect, since the model lives in RAM and does not
    // survive a power cycle.
    void setRobotModelCommand(Json command);
    bool hasRobotModel() const { return !m_robotModelCommand.empty(); }
    // True once the firmware has answered robot_model_load_done, so the panel can tell "sent" from
    // "accepted". Cleared on disconnect, since the next board may not have it.
    bool robotModelAccepted() const { return m_robotModelAccepted; }
    void sendRobotModelIfConnected();
    void sendMotionSettings(const Json& settings);
    void requestMotionSettings();
    void sendWeaveSchedules(const RobotMotionCore::WeaveScheduleTable& schedules);
    void sendWeaveSchedulesIfConnected();
    void requestWeaveSchedules();

    const RobotMotionCore::WeaveScheduleTable& readWeaveSchedules() const { return m_readWeaveSchedules; }
    bool weaveSchedulesReceived() const { return m_weaveSchedulesReceived; }
    void clearWeaveSchedulesReceived() { m_weaveSchedulesReceived = false; }

    const Status& status() const { return m_status; }

    const Json& lastStatusObject() const { return m_lastStatusObject; }
    const std::string& lastError() const { return m_lastError; }
    const Json& lastMotionSettings() const { return m_lastMotionSettings; }
    bool motionSettingsReceived() const { return m_motionSettingsReceived; }

    void clearMotionSettingsReceived() { m_motionSettingsReceived = false; }

    // Most recent protocol lines, newest last, for the panel's log view. Bounded so a long
    // session cannot grow without limit.
    const std::vector<std::string>& log() const { return m_log; }
    void clearLog() { m_log.clear(); }

    struct LinkHealth {
        uint32_t badChecksumLines = 0;
        uint32_t retries = 0;
        uint32_t giveUps = 0;
        bool peerFramed = false;
    };
    const LinkHealth& linkHealth() const { return m_link; }

    void setTriggerMessages(std::map<int, std::string> messages) { m_triggerMessages = std::move(messages); }

    struct TriggerReport {
        int id = -1;
        uint32_t robotMillis = 0;
        bool estop = false;
        std::array<int32_t, 6> steps{};
        std::string message;
    };
    const std::vector<TriggerReport>& triggerReports() const { return m_triggerReports; }
    void clearTriggerReports() { m_triggerReports.clear(); }

private:
    void send(const std::string& json);
    void pumpOutbox();
    void writeFramed(const std::string& payload, int32_t seq);
    // Resends or abandons whatever is in flight. Never blocks; called once per poll.
    void serviceInflight();
    void abandonInflight(const std::string& reason);
    void handleLine(const std::string& line);
    void appendLog(const std::string& entry);
    // Decodes robot_status.mastering into Status: mastered flags, steps, and the joint angles the
    // firmware only expresses as scaled integers.
    void readMasteringStatus(const Json& mastering);
    void sendNextProgramCommand();
    void clearProgramRun();
    // Return true when the message belonged to a running program, so the caller does not also
    // treat it as a single-move reply.
    bool handleProgramQueued(const Json& object);
    bool handleProgramQueueFull(const Json& object);
    bool handleProgramMoveDone(const Json& object, const char* label);
    bool handleProgramMoveFailed(const Json& object, const std::string& message);

    SerialPort m_serial;
    std::vector<SerialPort::PortInfo> m_ports;
    Status m_status;
    std::string m_lastError;
    Json m_lastStatusObject = Json::object();
    Json m_robotModelCommand = Json::object();
    bool m_robotModelAccepted = false;
    Json m_lastMotionSettings;
    RobotMotionCore::WeaveScheduleTable m_weaveSchedules{};
    bool m_weaveSchedulesKnown = false;
    RobotMotionCore::WeaveScheduleTable m_readWeaveSchedules{};
    bool m_weaveSchedulesReceived = false;
    std::map<int, std::string> m_triggerMessages;
    std::vector<TriggerReport> m_triggerReports;
    bool m_motionSettingsReceived = false;
    std::vector<std::string> m_log;
    MessageCallback m_messageCallback;

    std::vector<Json> m_programCommands;
    bool m_programActive = false;
    int m_programNextToSend = 0;
    int m_programCompleted = 0;
    bool m_programAwaitingAck = false;
    int m_nextMotionCommandId = 1;
    std::chrono::steady_clock::time_point m_programRetryAt{};

    // Outbound reliability. One command is in flight at a time and the rest wait their turn, which
    // keeps the retry logic to a single slot and makes a duplicate impossible to confuse with a
    // fresh command. Commands are infrequent enough that serialising them costs nothing: the robot
    // pushes status without being asked, so this direction carries only operator actions.
    struct Inflight {
        std::string payload;   // JSON as sent, sequence number already in it
        int32_t seq = -1;
        int attempts = 0;
        std::chrono::steady_clock::time_point sentAt{};
        bool active = false;
        std::chrono::steady_clock::time_point retryAt{};
    };
    std::deque<std::string> m_outbox;
    Inflight m_inflight;
    int32_t m_nextSeq = 1;
    LinkHealth m_link;

    // One resend, then the command is abandoned. Two attempts covers a single lost or corrupted
    // line, which is what this is for; anything worse is a broken link, and retrying into a broken
    // link is how a UI ends up wedged.
    static constexpr int kMaxAttempts = 2;
    // Comfortably longer than a USB round trip plus the firmware's 2 ms serial task period, so a
    // healthy link never retries, and short enough that giving up stays under a second.
    static constexpr std::chrono::milliseconds kAckTimeout{300};
    static constexpr std::chrono::milliseconds kBackpressureRetryDelay{50};
    bool m_programRetryPending = false;
    bool m_moveActive = false;
    bool m_finalStatusPending = false;
    bool m_finalStatusSeen = false;
};
