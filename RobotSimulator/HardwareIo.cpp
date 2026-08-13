#include "HardwareIo.h"
#include "StringUtil.h"

#include "JsonCompat.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kBaudRate = 115200;
constexpr size_t kMaxLogLines = 400;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

std::string compactJson(const Json& object) {
    // dump() with no indent is the compact form the firmware's line protocol expects.
    return object.dump();
}

Json jointsAsDegrees(const std::array<double, 6>& jointsRad) {
    Json array = Json::array();
    for (double value : jointsRad) array.push_back(value * kRadToDeg);
    return array;
}

// The firmware reads target_tcp as twelve values: the 3x4 row-major rigid transform, exactly as
// RobotMotionCore::Transform already stores it.
Json jsonArrayFromTransform(const RobotMotionCore::Transform& transform) {
    Json array = Json::array();
    for (int i = 0; i < 12; ++i) array.push_back(transform.values[i]);
    return array;
}

int arrayIntAt(const Json& values, size_t index, int fallback) {
    if (index >= values.size()) return fallback;
    return jsoncompat::toInt(values[index], fallback);
}

double arrayDoubleAt(const Json& values, size_t index, double fallback) {
    if (index >= values.size()) return fallback;
    return jsoncompat::toDouble(values[index], fallback);
}

// The firmware has no floating point formatting, so real numbers arrive as integers premultiplied
// by a fixed scale - position_deg_x10000000, steps_per_deg_x1000 and so on. Reading them means
// dividing that scale back out, and falling back when the field is absent.
double arrayScaledAt(const Json& values, size_t index, double scale, double fallback) {
    if (index >= values.size() || scale == 0.0) return fallback;
    return jsoncompat::toDouble(values[index], fallback * scale) / scale;
}

} // namespace

void HardwareIo::refreshPorts() {
    m_ports = SerialPort::availablePorts();
}

int HardwareIo::suggestedPortIndex() const {
    // The controller enumerates as a USB serial device; matching on that saves picking from a
    // list of modem and Bluetooth ports in the common case.
    for (size_t i = 0; i < m_ports.size(); ++i) {
        std::string haystack = m_ports[i].portName + " " + m_ports[i].description;
        std::transform(haystack.begin(), haystack.end(), haystack.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (haystack.find("teensy") != std::string::npos ||
            haystack.find("usb serial") != std::string::npos ||
            haystack.find("usbser") != std::string::npos) {
            return static_cast<int>(i);
        }
    }
    return m_ports.empty() ? -1 : 0;
}

bool HardwareIo::connectTo(const std::string& portName) {
    std::string errorMessage;
    if (!m_serial.open(portName, kBaudRate, &errorMessage)) {
        m_lastError = errorMessage;
        appendLog("connect failed: " + m_lastError);
        return false;
    }
    m_lastError.clear();
    m_status = Status();
    appendLog(strutil::format("connected to %1").arg(portName));
    send("{\"cmd\":\"hello\"}");
    // The firmware keeps the kinematic model in RAM, so a reconnect - or a power cycle - loses it
    // and every move would come back model_not_loaded until it is sent again.
    sendRobotModelIfConnected();
    // The welding schedules live in RAM for the same reason, and a program that names one would be
    // refused with weave_schedule_missing until they are back.
    sendWeaveSchedulesIfConnected();
    requestStatus();
    setStatusStreaming(true, 50);
    return true;
}

void HardwareIo::disconnect() {
    if (m_serial.isOpen()) appendLog("disconnected");
    m_serial.close();
    m_status = Status();
    // The command is kept so a reconnect can resend it, but whether the firmware holds it is no
    // longer known: the next connect may reach a board that was power cycled in between.
    m_robotModelAccepted = false;
    m_finalStatusPending = false;
    m_finalStatusSeen = false;
    // Commands and framing state are scoped to one connection.
    m_outbox.clear();
    m_inflight = Inflight{};
    m_nextSeq = 1;
    m_link.peerFramed = false;
    clearProgramRun();
}

void HardwareIo::poll() {
    if (!m_serial.isOpen()) return;
    // One tick after the final status frame was decoded, which is one tick after whatever follows
    // the robot had the chance to take it up.
    if (m_finalStatusPending && m_finalStatusSeen) {
        m_finalStatusPending = false;
        m_finalStatusSeen = false;
    }
    m_serial.poll();

    std::string line;
    while (m_serial.takeLine(&line)) {
        if (line.empty()) continue;
        size_t payloadLength = line.size();
        const SerialFraming::CheckResult check =
            SerialFraming::verifyLine(line.data(), line.size(), &payloadLength);
        if (check == SerialFraming::CheckResult::Invalid) {
            ++m_link.badChecksumLines;
            appendLog("bad checksum, line dropped");
            continue;
        }
        if (check == SerialFraming::CheckResult::Valid) m_link.peerFramed = true;
        handleLine(line.substr(0, payloadLength));
    }

    // Before pumping, so a command that has just been abandoned lets the next one through.
    serviceInflight();
    pumpOutbox();

    // Deferred resend after queue_full, checked here so no timer or event loop is needed.
    if (m_programRetryPending && std::chrono::steady_clock::now() >= m_programRetryAt) {
        m_programRetryPending = false;
        sendNextProgramCommand();
    }

    // poll() closes the port on a failed read, which is how an unplugged board surfaces.
    if (!m_serial.isOpen()) {
        m_lastError = "Serial port closed unexpectedly.";
        appendLog(m_lastError);
        m_status = Status();
        m_finalStatusPending = false;
        m_finalStatusSeen = false;
        clearProgramRun();
    }
}

void HardwareIo::send(const std::string& json) {
    if (!m_serial.isOpen()) {
        m_lastError = "Not connected.";
        return;
    }
    m_outbox.push_back(json);
    pumpOutbox();
}

// Appends the sequence number and the checksum, then writes. The checksum goes on unconditionally:
// a robot that does not understand it ignores a suffix sitting after the closing brace, because its
// field lookup is a substring search that never reaches past the fields it wants.
void HardwareIo::writeFramed(const std::string& payload, int32_t seq) {
    std::string line = SerialFraming::withSequence(payload, seq);
    char suffix[SerialFraming::kChecksumTextLength + 2] = {};
    SerialFraming::formatChecksumSuffix(line.data(), line.size(), suffix, sizeof(suffix));
    line += suffix;

    std::string errorMessage;
    if (!m_serial.writeLine(line, &errorMessage)) {
        m_lastError = errorMessage;
        appendLog("write failed: " + m_lastError);
        return;
    }
    appendLog("> " + line);
}

void HardwareIo::pumpOutbox() {
    if (m_inflight.active || m_outbox.empty() || !m_serial.isOpen()) return;

    const std::string payload = m_outbox.front();
    m_outbox.pop_front();

    // Enable sequencing only after the peer sends a valid framed message; older firmware remains
    // compatible with unsequenced commands.
    if (!m_link.peerFramed) {
        writeFramed(payload, -1);
        return;
    }

    m_inflight.payload = payload;
    m_inflight.seq = m_nextSeq++;
    if (m_nextSeq < 0) m_nextSeq = 1;
    m_inflight.attempts = 1;
    m_inflight.sentAt = std::chrono::steady_clock::now();
    m_inflight.active = true;
    // Belongs to the command that has just left the slot, not to this one.
    m_inflight.retryAt = {};
    writeFramed(m_inflight.payload, m_inflight.seq);
}

void HardwareIo::serviceInflight() {
    if (!m_inflight.active) return;

    // queue_full is backpressure; retry without consuming the lost-line budget.
    if (m_inflight.retryAt != std::chrono::steady_clock::time_point{}) {
        if (std::chrono::steady_clock::now() < m_inflight.retryAt) return;
        m_inflight.retryAt = {};
        m_inflight.sentAt = std::chrono::steady_clock::now();
        appendLog("queue was full, resending seq " + std::to_string(m_inflight.seq));
        writeFramed(m_inflight.payload, m_inflight.seq);
        return;
    }

    if (std::chrono::steady_clock::now() - m_inflight.sentAt < kAckTimeout) return;

    if (m_inflight.attempts >= kMaxAttempts) {
        abandonInflight("no acknowledgement after " + std::to_string(kMaxAttempts) + " attempts");
        return;
    }
    // The robot rejects a repeat of a sequence number it has already run, so resending a motion
    // command whose acknowledgement went missing cannot move the arm twice.
    ++m_inflight.attempts;
    ++m_link.retries;
    m_inflight.sentAt = std::chrono::steady_clock::now();
    appendLog("retry seq " + std::to_string(m_inflight.seq));
    writeFramed(m_inflight.payload, m_inflight.seq);
}

void HardwareIo::abandonInflight(const std::string& reason) {
    if (!m_inflight.active) return;
    ++m_link.giveUps;
    const std::string what = "Command seq " + std::to_string(m_inflight.seq) + " failed: " + reason;
    m_inflight.active = false;
    m_lastError = what;
    appendLog(what);
    if (m_programActive) cancelProgram(what, true);
    pumpOutbox();
}

void HardwareIo::handleLine(const std::string& line) {
    const std::string text = line;
    const Json document = Json::parse(text, nullptr, /*allow_exceptions=*/false);
    if (document.is_discarded() || !document.is_object()) {
        if (m_messageCallback) m_messageCallback(line, Json::object());
        m_lastError = "Bad JSON: " + line.substr(0, 80);
        appendLog("< " + line);
        return;
    }

    const Json& object = document;
    // Before the built-in handling, so a watcher sees every message including robot_status,
    // which is deliberately kept out of the log.
    if (m_messageCallback) m_messageCallback(line, object);
    const std::string message = jsoncompat::fieldString(object, "msg");

    // robot_status arrives continuously when streaming; logging every one would bury the rest.
    if (message != "robot_status") appendLog("< " + line);

    if (message == "robot_status") {
        m_lastStatusObject = object;
        m_status.valid = true;
        const Json& jog = jsoncompat::fieldObject(object, "jog");
        // The firmware emits armed as an integer, not a JSON boolean: robot_status is built with
        // "\"jog\":{\"armed\":%u,...}". Reading it with fieldBool returned the fallback every
        // Status uses an integer while jog_arm_ack uses a JSON boolean; accept both.
        m_status.jogArmed = jsoncompat::fieldBool(jog, "armed", false) ||
                            jsoncompat::fieldInt(jog, "armed", 0) != 0;
        // Same integer encoding. The panel gates the jog and mastering controls on it so a
        // mastering sweep cannot be interrupted by another command.
        const Json& mastering = jsoncompat::fieldObject(object, "mastering");
        m_status.masteringActive = jsoncompat::fieldInt(mastering, "active", 0) == 1;
        readMasteringStatus(mastering);
        // limits_active is an array of nine per-switch flags, not a scalar, so a single bool
        // could only ever mean "any of them". Derived from the raw flags below instead.
        m_status.limitsActive = false;
        // Nine switches are reported; the first six are the joints the panel shows.
        const Json& raw = jsoncompat::fieldArray(object, "limits_raw");
        for (size_t i = 0; i < 6 && i < raw.size(); ++i) {
            m_status.limitPressed[i] = jsoncompat::toInt(raw[i]) != 0;
            if (m_status.limitPressed[i]) m_status.limitsActive = true;
        }
        m_status.firmwareSeconds = jsoncompat::fieldDouble(object, "ms") / 1000.0;
        m_status.limitPollMs =
            jsoncompat::fieldInt(jsoncompat::fieldObject(object, "limit_poll"), "period_ms", 0);
        ++m_status.statusCount;
        // Only noted here, not cleared. Clearing it now would drop the very frame it exists to
        // deliver: poll() runs before the panel reads status(), so the flag would already be false
        // by the time anything looked at the final position. poll() clears it on the following
        // tick instead.
        if (m_finalStatusPending) m_finalStatusSeen = true;
        return;
    }

    // Transport acknowledgements, handled before anything else and never surfaced to the UI: they
    // say a line arrived, not that the command did anything.
    if (message == "ack") {
        const int seq = jsoncompat::fieldInt(object, "seq", -1);
        if (m_inflight.active && seq == m_inflight.seq) {
            m_inflight.active = false;
            pumpOutbox();
        }
        return;
    }
    if (message == "nak") {
        const int seq = jsoncompat::fieldInt(object, "seq", -1);
        // A checksum NAK may not contain a trustworthy sequence number.
        if (m_inflight.active && (seq < 0 || seq == m_inflight.seq)) {
            // Let firmware lookahead drain before retrying queue_full.
            if (jsoncompat::fieldString(object, "code") == "queue_full") {
                m_inflight.retryAt = std::chrono::steady_clock::now() + kBackpressureRetryDelay;
                appendLog("nak queue_full, deferring seq " + std::to_string(m_inflight.seq));
            } else if (m_inflight.attempts >= kMaxAttempts) {
                abandonInflight("rejected: " + jsoncompat::fieldString(object, "code"));
            } else {
                ++m_inflight.attempts;
                ++m_link.retries;
                m_inflight.sentAt = std::chrono::steady_clock::now();
                appendLog("nak, resending seq " + std::to_string(m_inflight.seq));
                writeFramed(m_inflight.payload, m_inflight.seq);
            }
        }
        return;
    }

    // Fired as the arm passed the point, so the timestamp and state here are the robot's own rather
    // than the host's guess at when the move got there.
    if (message == "trigger") {
        TriggerReport report;
        report.id = jsoncompat::fieldInt(object, "id", -1);
        report.robotMillis = static_cast<uint32_t>(jsoncompat::fieldInt(object, "millis", 0));
        report.estop = jsoncompat::fieldInt(object, "estop", 0) != 0;
        const Json& steps = jsoncompat::fieldArray(object, "steps");
        for (size_t i = 0; i < 6 && i < steps.size(); ++i) {
            report.steps[i] = static_cast<int32_t>(jsoncompat::toInt(steps[i], 0));
        }
        const auto found = m_triggerMessages.find(report.id);
        report.message = found != m_triggerMessages.end() ? found->second : std::string();
        // Bounded, for the same reason the protocol log is: a long run must not grow without limit.
        constexpr size_t kMaxTriggerReports = 200;
        if (m_triggerReports.size() >= kMaxTriggerReports) m_triggerReports.erase(m_triggerReports.begin());
        m_triggerReports.push_back(report);
        appendLog(strutil::format("trigger %1: %2").arg(report.id).arg(report.message).str());
        return;
    }

    if (message == "motion_settings") {
        m_lastMotionSettings = object;
        m_motionSettingsReceived = true;
        return;
    }

    // One line per schedule, so the table fills in over eight replies. Only flagged as received
    // once the last index has landed, or the panel would load a half-read table.
    if (message == "weave_schedule") {
        const int schema = jsoncompat::fieldInt(object, "weave_schedule_schema", -1);
        if (schema != static_cast<int>(RobotMotionCore::kWeaveScheduleSchemaVersion)) {
            m_lastError = "Robot returned weave schedules with unsupported schema " +
                          std::to_string(schema) + ".";
            return;
        }
        const int index = jsoncompat::fieldInt(object, "index", -1);
        if (index < 0 || index >= static_cast<int>(RobotMotionCore::kMaxWeaveSchedules)) return;
        const int shape = jsoncompat::fieldInt(object, "shape", 0);
        const int rateMode = jsoncompat::fieldInt(object, "rate_mode", 0);
        if (shape < 0 || shape > static_cast<int>(RobotMotionCore::WeaveShape::LShape) ||
            rateMode < 0 || rateMode > static_cast<int>(RobotMotionCore::WeaveRateMode::Wavelength)) {
            return;
        }
        RobotMotionCore::WeaveParams weave = RobotMotionCore::defaultWeaveParams();
        weave.shape = static_cast<RobotMotionCore::WeaveShape>(shape);
        weave.rateMode = static_cast<RobotMotionCore::WeaveRateMode>(rateMode);
        weave.frequencyHz = jsoncompat::fieldDouble(object, "freq_hz", 0.0);
        weave.wavelengthMm = jsoncompat::fieldDouble(object, "wavelength_mm", 0.0);
        weave.amplitudeLeftMm = jsoncompat::fieldDouble(object, "amp_left_mm", 0.0);
        weave.amplitudeRightMm = jsoncompat::fieldDouble(object, "amp_right_mm", 0.0);
        weave.elevationMm = jsoncompat::fieldDouble(object, "elevation_mm", 0.0);
        weave.planeAngleDeg = jsoncompat::fieldDouble(object, "plane_angle_deg", 0.0);
        weave.biasMm = jsoncompat::fieldDouble(object, "bias_mm", 0.0);
        weave.dwellLeft = jsoncompat::fieldDouble(object, "dwell_left", 0.0);
        weave.dwellCenter = jsoncompat::fieldDouble(object, "dwell_center", 0.0);
        weave.dwellRight = jsoncompat::fieldDouble(object, "dwell_right", 0.0);
        weave.scheduleIndex = static_cast<uint8_t>(index);
        m_readWeaveSchedules.schedules[index] = weave;
        m_readWeaveSchedules.valid[index] = jsoncompat::fieldInt(object, "valid", 0) != 0 ? 1 : 0;
        if (index + 1 == static_cast<int>(RobotMotionCore::kMaxWeaveSchedules)) {
            m_weaveSchedulesReceived = true;
        }
        return;
    }

    if (message == "weave_schedule_set_failed") {
        m_lastError = "Robot rejected a weave schedule: " + jsoncompat::fieldString(object, "code");
        return;
    }

    if (message == "robot_model_load_done") {
        m_robotModelAccepted = true;
        m_lastError.clear();
        requestMotionSettings();
        return;
    }
    if (message == "robot_model_load_failed") {
        m_robotModelAccepted = false;
        m_lastError = "Robot model load failed: " + jsoncompat::fieldString(object, "code", "unknown");
        return;
    }

    // Motion acks, which drive a program run forward one command at a time. Each returns true
    // when it consumed the message on behalf of a running program; otherwise the reply belongs
    // to a single MoveJ/MoveL from the Run tab and only needs a status refresh.
    if (message == "movej_queued" || message == "movel_queued") {
        handleProgramQueued(object);
        return;
    }
    if (message == "movej_done" || message == "movel_done") {
        m_moveActive = false;
        m_finalStatusPending = true;
        m_finalStatusSeen = false;
        if (!handleProgramMoveDone(object, message == "movej_done" ? "MoveJ" : "MoveL")) {
            m_lastError.clear();
        }
        requestStatus();
        return;
    }
    if (message == "movej_rejected" || message == "movel_rejected") {
        m_moveActive = false;
        m_finalStatusPending = true;
        m_finalStatusSeen = false;
        // queue_full is not a failure: the firmware is simply busy, and the same command is
        // resent. Anything else ends the run.
        if (!handleProgramQueueFull(object)) {
            handleProgramMoveFailed(
                object, strutil::format("%1 rejected: %2")
                            .arg(message == "movej_rejected" ? "MoveJ" : "MoveL")
                            .arg(jsoncompat::fieldString(object, "code", "unknown")));
        }
        requestStatus();
        return;
    }
    if (message == "movej_stopped") {
        m_moveActive = false;
        m_finalStatusPending = true;
        m_finalStatusSeen = false;
        handleProgramMoveFailed(object, "MoveJ stopped");
        requestStatus();
        return;
    }
    if (message == "movel_aborted") {
        m_moveActive = false;
        m_finalStatusPending = true;
        m_finalStatusSeen = false;
        handleProgramMoveFailed(object,
                                strutil::format("MoveL aborted: %1 at sample %2")
                                    .arg(jsoncompat::fieldString(object, "code", "unknown"))
                                    .arg(jsoncompat::fieldInt(object, "sample", 0)));
        requestStatus();
        return;
    }

    if (jsoncompat::contains(object, "error")) {
        m_lastError = jsoncompat::fieldString(object, "error");
    }
}

// robot_status nests all of this under "mastering". Reading it from the top level - as this class
// did - found nothing, so every joint read as unmastered, which greyed out MoveJ, MoveL and Run
// program permanently, and the reported angles stayed at zero.
void HardwareIo::readMasteringStatus(const Json& mastering) {
    const Json& mastered = jsoncompat::fieldArray(mastering, "mastered");
    const Json& currentSteps = jsoncompat::fieldArray(mastering, "current_steps");
    const Json& zeroSteps = jsoncompat::fieldArray(mastering, "zero_steps");
    const Json& limitSteps = jsoncompat::fieldArray(mastering, "master_limit_steps");
    const Json& positionsCoarse = jsoncompat::fieldArray(mastering, "position_deg_x100");
    const Json& positionsFine = jsoncompat::fieldArray(mastering, "position_deg_x10000000");
    const Json& offsetsCoarse = jsoncompat::fieldArray(mastering, "offset_deg_x100");
    const Json& offsetsFine = jsoncompat::fieldArray(mastering, "offset_deg_x10000000");
    const Json& offsetsPlain = jsoncompat::fieldArray(mastering, "offset_deg");
    const Json& stepsPerDegX1000 = jsoncompat::fieldArray(mastering, "steps_per_deg_x1000");
    const Json& stepsPerDegFine = jsoncompat::fieldArray(mastering, "steps_per_deg_x10000000");
    const Json& stepsPerDegPlain = jsoncompat::fieldArray(mastering, "steps_per_deg");
    const Json& masterDir = jsoncompat::fieldArray(mastering, "master_dir");

    m_status.allMastered = mastered.size() >= 6;
    for (size_t i = 0; i < 6; ++i) {
        m_status.jointMastered[i] = arrayIntAt(mastered, i, 0) == 1;
        if (!m_status.jointMastered[i]) m_status.allMastered = false;

        const int currentStep = arrayIntAt(currentSteps, i, 0);
        // Defaulting zero to current means an unmastered joint reads as being at zero rather
        // than at some arbitrary distance from a zero that was never set.
        const int zeroStep = arrayIntAt(zeroSteps, i, currentStep);
        m_status.currentSteps[i] = currentStep;
        m_status.zeroSteps[i] = zeroStep;
        m_status.masterLimitSteps[i] = arrayIntAt(limitSteps, i, 0);

        // Each of these prefers the most precise encoding the firmware sent and falls back
        // through the coarser ones, ending at a value computed from the steps themselves. That
        // ordering matters: the x100 fields round to a hundredth of a degree, which is coarser
        // than the mastering resolution the panel edits to seven decimals.
        const double plainStepsPerDeg = arrayDoubleAt(stepsPerDegPlain, i, 0.0);
        const double stepsPerDegFallback = arrayIntAt(stepsPerDegX1000, i, -1) > 0
                                               ? arrayIntAt(stepsPerDegX1000, i, -1) / 1000.0
                                               : plainStepsPerDeg;
        m_status.reportedStepsPerDegree[i] =
            arrayScaledAt(stepsPerDegFine, i, 10000000.0, stepsPerDegFallback);

        const double computedDeg =
            plainStepsPerDeg > 0.0 ? static_cast<double>(currentStep - zeroStep) / plainStepsPerDeg : 0.0;
        const double coarseDeg =
            positionsCoarse.empty() ? computedDeg : arrayIntAt(positionsCoarse, i, 0) / 100.0;
        m_status.jointsDeg[i] = arrayScaledAt(positionsFine, i, 10000000.0, coarseDeg);

        const double plainOffset = arrayDoubleAt(offsetsPlain, i, arrayIntAt(offsetsCoarse, i, 0) / 100.0);
        m_status.reportedOffsetDeg[i] = arrayScaledAt(offsetsFine, i, 10000000.0, plainOffset);

        if (i < masterDir.size()) {
            m_status.reportedMasterDirection[i] = jsoncompat::toInt(masterDir[i], 1) >= 0 ? 1 : -1;
        }
    }
}

bool HardwareIo::startProgram(std::vector<Json> commands, std::string* errorMessage) {
    const auto fail = [&](const std::string& reason) {
        if (errorMessage) *errorMessage = reason;
        m_lastError = reason;
        return false;
    };
    if (!m_serial.isOpen()) return fail("Not connected.");
    if (!m_status.jogArmed) return fail("Arm jog before running a program.");
    if (!allJointsMastered()) return fail("Run program requires all six joints mastered.");
    // Without the kinematic model the firmware rejects every move with model_not_loaded, so this
    // is caught here with something that says what to do about it.
    if (!hasRobotModel()) return fail("Run program rejected: no robot model loaded.");
    if (commands.empty()) return fail("Run program rejected: program has no motion commands.");

    clearProgramRun();
    m_programCommands = std::move(commands);
    for (Json& command : m_programCommands) {
        command["id"] = m_nextMotionCommandId++;
        if (m_nextMotionCommandId <= 0) m_nextMotionCommandId = 1;
    }
    m_programActive = true;
    appendLog(strutil::format("program start: %1 command(s)").arg(m_programCommands.size()).str());
    m_lastError = strutil::format("Program starting: %1 command(s).").arg(m_programCommands.size()).str();
    sendNextProgramCommand();
    return true;
}

void HardwareIo::sendNextProgramCommand() {
    if (!m_programActive || !m_serial.isOpen()) return;
    if (m_programAwaitingAck) return;
    const int total = static_cast<int>(m_programCommands.size());
    if (m_programNextToSend >= total) return;

    Json command = m_programCommands[static_cast<size_t>(m_programNextToSend)];
    command["program_total"] = total;
    command["program_global_index"] = m_programNextToSend;
    command["program_final"] = m_programNextToSend + 1 >= total ? 1 : 0;
    send(compactJson(command));
    m_programAwaitingAck = true;
}

void HardwareIo::clearProgramRun() {
    m_moveActive = false;
    m_programActive = false;
    m_programCommands.clear();
    m_programNextToSend = 0;
    m_programCompleted = 0;
    m_programAwaitingAck = false;
    m_programRetryPending = false;
}

void HardwareIo::cancelProgram(const std::string& reason, bool sendStop) {
    const bool wasActive = m_programActive;
    clearProgramRun();
    if (sendStop && wasActive && m_serial.isOpen()) send("{\"cmd\":\"stop\"}");
    if (!reason.empty()) {
        m_lastError = reason;
        appendLog("program cancelled: " + reason);
    }
}

int HardwareIo::programExecutingRow() const {
    if (!m_programActive || m_programCompleted < 0 ||
        m_programCompleted >= static_cast<int>(m_programCommands.size())) {
        return -1;
    }
    return jsoncompat::fieldInt(m_programCommands[static_cast<size_t>(m_programCompleted)], "program_row", -1);
}

int HardwareIo::programPendingRow() const {
    if (!m_programActive) return -1;
    const int total = static_cast<int>(m_programCommands.size());
    // The firmware holds a lookahead queue, so commands are sent ahead of the one executing.
    // Reporting the send cursor directly would run the highlight off the end of what the robot
    // is actually doing, so it is clamped to the lookahead window, as Qt's marker code did.
    const int lookahead =
        std::min(total - 1,
                 m_programCompleted + static_cast<int>(RobotMotionCore::kMotionLookaheadQueuedCommands) - 1);
    const int index = std::min(m_programNextToSend, lookahead);
    if (index < 0 || index >= total) return -1;
    return jsoncompat::fieldInt(m_programCommands[static_cast<size_t>(index)], "program_row", -1);
}

bool HardwareIo::handleProgramQueued(const Json& object) {
    if (!m_programActive) return false;
    const int id = jsoncompat::fieldInt(object, "id", -1);
    if (!m_programAwaitingAck || m_programNextToSend < 0 ||
        m_programNextToSend >= static_cast<int>(m_programCommands.size())) {
        cancelProgram(strutil::format("Firmware queued unexpected command id %1.").arg(id).str(), true);
        return true;
    }
    const int expectedId =
        jsoncompat::fieldInt(m_programCommands[static_cast<size_t>(m_programNextToSend)], "id", -1);
    if (id != expectedId) {
        cancelProgram(strutil::format("Firmware queued command out of order: expected id %1, got id %2.")
                          .arg(expectedId)
                          .arg(id)
                          .str(),
                      true);
        return true;
    }

    m_programAwaitingAck = false;
    ++m_programNextToSend;
    m_lastError = strutil::format("Program queued command %1/%2")
                      .arg(m_programNextToSend)
                      .arg(m_programCommands.size())
                      .str();
    sendNextProgramCommand();
    return true;
}

bool HardwareIo::handleProgramQueueFull(const Json& object) {
    if (!m_programActive || jsoncompat::fieldString(object, "code") != "queue_full") return false;
    const int id = jsoncompat::fieldInt(object, "id", -1);
    if (!m_programAwaitingAck || m_programNextToSend < 0 ||
        m_programNextToSend >= static_cast<int>(m_programCommands.size())) {
        cancelProgram(
            strutil::format("Firmware reported queue full for unexpected command id %1.").arg(id).str(), true);
        return true;
    }
    const int expectedId =
        jsoncompat::fieldInt(m_programCommands[static_cast<size_t>(m_programNextToSend)], "id", -1);
    if (id != expectedId) {
        cancelProgram(strutil::format("Firmware reported queue full out of order: expected id %1, got id %2.")
                          .arg(expectedId)
                          .arg(id)
                          .str(),
                      true);
        return true;
    }

    m_programAwaitingAck = false;
    m_lastError = strutil::format("Firmware motion queue full; retrying command %1/%2.")
                      .arg(m_programNextToSend + 1)
                      .arg(m_programCommands.size())
                      .str();
    m_programRetryPending = true;
    m_programRetryAt = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    return true;
}

bool HardwareIo::handleProgramMoveDone(const Json& object, const char* label) {
    if (!m_programActive) return false;
    const int id = jsoncompat::fieldInt(object, "id", -1);
    if (m_programCompleted < 0 || m_programCompleted >= static_cast<int>(m_programCommands.size())) {
        cancelProgram(strutil::format("%1 completed after program state was invalid.").arg(label).str(), false);
        return true;
    }
    const int expectedId =
        jsoncompat::fieldInt(m_programCommands[static_cast<size_t>(m_programCompleted)], "id", -1);
    if (id != expectedId) {
        cancelProgram(strutil::format("%1 completed out of order: expected id %2, got id %3.")
                          .arg(label)
                          .arg(expectedId)
                          .arg(id)
                          .str(),
                      true);
        return true;
    }

    ++m_programCompleted;
    if (m_programCompleted >= static_cast<int>(m_programCommands.size())) {
        const int total = static_cast<int>(m_programCommands.size());
        clearProgramRun();
        m_lastError = strutil::format("Program complete: %1 command(s).").arg(total).str();
        appendLog(m_lastError);
        return true;
    }
    m_lastError = strutil::format("Program running: %1/%2 complete.")
                      .arg(m_programCompleted)
                      .arg(m_programCommands.size())
                      .str();
    sendNextProgramCommand();
    return true;
}

bool HardwareIo::handleProgramMoveFailed(const Json& object, const std::string& message) {
    if (!m_programActive) {
        m_lastError = message;
        return false;
    }
    const int id = jsoncompat::fieldInt(object, "id", -1);
    cancelProgram(id >= 0 ? strutil::format("%1 (id %2).").arg(message).arg(id).str() : message, true);
    return true;
}

void HardwareIo::appendLog(const std::string& entry) {
    m_log.push_back(entry);
    if (m_log.size() > kMaxLogLines) {
        m_log.erase(m_log.begin(), m_log.begin() + static_cast<std::ptrdiff_t>(m_log.size() - kMaxLogLines));
    }
}

void HardwareIo::sendCommand(const Json& command) {
    send(compactJson(command));
}

void HardwareIo::requestStatus() {
    send("{\"cmd\":\"get_status\"}");
}

void HardwareIo::setStatusStreaming(bool enabled, int periodMs) {
    m_status.streaming = enabled;
    send(strutil::format("{\"cmd\":\"stream_status\",\"enabled\":%1,\"period_ms\":%2}")
             .arg(enabled ? "true" : "false")
             .arg(std::max(10, periodMs)));
}

void HardwareIo::setLimitPollPeriod(int periodMs) {
    send(strutil::format("{\"cmd\":\"set_limit_poll\",\"period_ms\":%1}").arg(std::max(0, periodMs)));
}

void HardwareIo::armJog(bool armed) {
    send(armed ? "{\"cmd\":\"jog_arm\",\"enabled\":true}" : "{\"cmd\":\"jog_disarm\"}");
    requestStatus();
}

// The firmware reads "steps", not "pulse_us": handleJogJoint takes readIntField(..., "steps")
// and rejects the command with bad_steps when it is absent or outside 1..kJogMaxSteps (500).
// Sending pulse_us meant every jog was refused, so jogging could not have worked at all.
void HardwareIo::jogJoint(int jointIndex, int direction, int steps) {
    if (jointIndex < 0 || jointIndex > 5) return;
    Json command = Json::object();
    command["cmd"] = "jog_joint";
    command["joint"] = jointIndex + 1;  // firmware numbers joints from one
    command["steps"] = std::max(1, std::min(500, steps));
    command["dir"] = direction >= 0 ? 1 : -1;
    send(compactJson(command));
}

void HardwareIo::stopMotion() {
    // Stop abandons a program run as well as the current motion; leaving it active would keep
    // feeding commands to a robot the user has just told to stop.
    clearProgramRun();
    send("{\"cmd\":\"stop\"}");
}

// Seven decimals on steps_per_deg and offset_deg deliberately: mastering resolution depends on them.
void HardwareIo::masterJoint(int joint, double stepsPerDegree, double offsetDeg, int backoffSteps,
                             int masterDirection) {
    if (joint < 1 || joint > 6) return;
    send(strutil::format("{\"cmd\":\"master_joint\",\"joint\":%1,\"offset_deg\":%2,\"backoff_steps\":%3,\"steps_per_deg\":%4,\"master_dir\":%5}")
             .arg(joint)
             .arg(offsetDeg, 0, 'f', 7)
             .arg(backoffSteps)
             .arg(stepsPerDegree, 0, 'f', 7)
             .arg(masterDirection >= 0 ? 1 : -1)
             .str());
}

void HardwareIo::setJointZero(int joint, double stepsPerDegree, double offsetDeg) {
    if (joint < 1 || joint > 6) return;
    send(strutil::format("{\"cmd\":\"set_joint_zero\",\"joint\":%1,\"steps_per_deg\":%2,\"offset_deg\":%3}")
             .arg(joint)
             .arg(stepsPerDegree, 0, 'f', 7)
             .arg(offsetDeg, 0, 'f', 7)
             .str());
}

void HardwareIo::sendJogDirectionInvert(const std::array<bool, 6>& invert) {
    Json command = Json::object();
    command["cmd"] = "load_mastering";
    Json values = Json::array();
    for (bool value : invert) values.push_back(value ? 1 : 0);
    command["jog_dir_invert"] = values;
    send(compactJson(command));
    requestStatus();
}

void HardwareIo::moveJointsTo(const std::array<double, 6>& jointsRad, double speedDegPerSec) {
    Json command = Json::object();
    command["cmd"] = "movej";
    // Numbered like the program commands are, from the same counter. The firmware echoes the id in
    // movej_queued and movej_done, so without one every reply comes back as id -1 and cannot be
    // told apart from any other move's.
    command["id"] = m_nextMotionCommandId++;
    if (m_nextMotionCommandId <= 0) m_nextMotionCommandId = 1;
    command["target_deg"] = jointsAsDegrees(jointsRad);
    command["speed_deg_s"] = speedDegPerSec;
    m_moveActive = true;
    send(compactJson(command));
}

void HardwareIo::moveLinearTo(const std::array<double, 6>& jointsRad,
                              const RobotMotionCore::Transform& targetTcp,
                              double speedMmPerSec) {
    Json command = Json::object();
    command["cmd"] = "movel";
    command["id"] = m_nextMotionCommandId++;
    if (m_nextMotionCommandId <= 0) m_nextMotionCommandId = 1;
    // target_tcp is not optional for movel: enqueueMotionCommand rejects the command with bad_pose
    // when it is missing, since a linear move is defined by the tool path and the joint values only
    // pick which IK solution to arrive in.
    command["target_tcp"] = jsonArrayFromTransform(targetTcp);
    command["target_deg"] = jointsAsDegrees(jointsRad);
    command["speed_mm_s"] = speedMmPerSec;
    m_moveActive = true;
    send(compactJson(command));
}

void HardwareIo::setRobotModelCommand(Json command) {
    m_robotModelCommand = std::move(command);
    sendRobotModelIfConnected();
}

void HardwareIo::sendRobotModelIfConnected() {
    if (!m_serial.isOpen() || m_robotModelCommand.empty()) return;
    const std::string text = compactJson(m_robotModelCommand);
    // The firmware assembles a line into a 1536-byte buffer and drops anything that does not fit,
    // answering rx_line_too_long - after which every move comes back model_not_loaded. Sending it
    // anyway would just produce that at a distance, so it is refused here where the reason is
    // still visible.
    if (text.size() > 1535) {
        m_lastError = strutil::format("Robot model command is %1 bytes; the firmware accepts 1535.")
                          .arg(text.size())
                          .str();
        appendLog(m_lastError);
        return;
    }
    send(text);
}

void HardwareIo::sendMotionSettings(const Json& settings) {
    Json command = settings;
    command["cmd"] = "set_motion_settings";
    send(compactJson(command));
}

void HardwareIo::requestMotionSettings() {
    send("{\"cmd\":\"get_motion_settings\"}");
}

void HardwareIo::requestWeaveSchedules() {
    m_weaveSchedulesReceived = false;
    send("{\"cmd\":\"get_weave_schedules\"}");
}

void HardwareIo::sendWeaveSchedulesIfConnected() {
    if (!m_serial.isOpen() || !m_weaveSchedulesKnown) return;
    sendWeaveSchedules(m_weaveSchedules);
}

void HardwareIo::sendWeaveSchedules(const RobotMotionCore::WeaveScheduleTable& schedules) {
    // Retained so a reconnect can replay them without the panel having to notice, the same way the
    // robot model command is held.
    m_weaveSchedules = schedules;
    m_weaveSchedulesKnown = true;
    if (!m_serial.isOpen()) return;
    for (uint8_t index = 0; index < RobotMotionCore::kMaxWeaveSchedules; ++index) {
        const RobotMotionCore::WeaveParams& weave = schedules.schedules[index];
        Json command = Json::object();
        command["cmd"] = "set_weave_schedule";
        command["weave_schedule_schema"] = static_cast<int>(RobotMotionCore::kWeaveScheduleSchemaVersion);
        command["index"] = static_cast<int>(index);
        command["valid"] = schedules.valid[index] ? 1 : 0;
        command["shape"] = static_cast<int>(weave.shape);
        command["rate_mode"] = static_cast<int>(weave.rateMode);
        command["freq_hz"] = weave.frequencyHz;
        command["wavelength_mm"] = weave.wavelengthMm;
        command["amp_left_mm"] = weave.amplitudeLeftMm;
        command["amp_right_mm"] = weave.amplitudeRightMm;
        command["elevation_mm"] = weave.elevationMm;
        command["plane_angle_deg"] = weave.planeAngleDeg;
        command["bias_mm"] = weave.biasMm;
        command["dwell_left"] = weave.dwellLeft;
        command["dwell_center"] = weave.dwellCenter;
        command["dwell_right"] = weave.dwellRight;
        send(compactJson(command));
    }
}
