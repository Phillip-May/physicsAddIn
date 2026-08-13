#include "MasteringIo.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "AppState.h"
#include "CliProgramCommands.h"
#include "ProgramTextIo.h"
#include "UnitsMath.h"

// Derives the package's motion profile. The joint-accel and joint-jerk keys are simulator-only and
// deliberately absent from the firmware wire list, so the controller protocol and its schema version
// stay untouched.
Json motionSettingsFromRobotData(const OPW6RobotData* robotData) {
    RobotMotionCore::MotionProgramSettings profile = RobotMotionCore::defaultMotionProgramSettings();
    if (robotData) {
        RobotMotionCore::applyJointDynamicsLimitsToMotionSettings(
            &profile,
            robotData->hasJointVelocityLimits ? robotData->jointVelocityMaxRadS.data() : nullptr,
            robotData->hasJointAccelerationLimits ? robotData->jointAccelerationMaxRadS2.data() : nullptr,
            robotData->hasJointJerkLimits ? robotData->jointJerkMaxRadS3.data() : nullptr);
        if (robotData->controlPeriodSec > 0.0) profile.controlPeriodSec = robotData->controlPeriodSec;
        if (robotData->defaultJointSpeedDegPerSec > 0.0) profile.defaultJointSpeedDegPerSec = robotData->defaultJointSpeedDegPerSec;
        if (robotData->defaultLinearSpeedMmPerSec > 0.0) profile.defaultLinearSpeedMmPerSec = robotData->defaultLinearSpeedMmPerSec;
        if (robotData->defaultLinearAccelerationMmSec2 > 0.0) profile.defaultLinearAccelerationMmSec2 = robotData->defaultLinearAccelerationMmSec2;
        if (robotData->defaultLinearJerkMmSec3 > 0.0) profile.defaultLinearJerkMmSec3 = robotData->defaultLinearJerkMmSec3;
        if (robotData->defaultToolAngularSpeedRadSec > 0.0) profile.defaultToolAngularSpeedRadSec = robotData->defaultToolAngularSpeedRadSec;
        if (robotData->defaultToolAngularAccelerationRadSec2 > 0.0) profile.defaultToolAngularAccelerationRadSec2 = robotData->defaultToolAngularAccelerationRadSec2;
        if (robotData->defaultToolAngularJerkRadSec3 > 0.0) profile.defaultToolAngularJerkRadSec3 = robotData->defaultToolAngularJerkRadSec3;
        if (robotData->singularityThresholdRad > 0.0) profile.singularityThresholdRad = robotData->singularityThresholdRad;
    }
    Json settings = Json::object();
    settings["motion_settings_schema"] = static_cast<int>(RobotMotionCore::kMotionSettingsSchemaVersion);
    settings["control_period_s"] = profile.controlPeriodSec;
    settings["default_joint_speed_deg_s"] = profile.defaultJointSpeedDegPerSec;
    settings["default_linear_speed_mm_s"] = profile.defaultLinearSpeedMmPerSec;
    settings["default_joint_accel_rad_s2"] = profile.defaultJointAccelerationRadSec2;
    settings["default_joint_jerk_rad_s3"] = profile.defaultJointJerkRadSec3;
    settings["default_linear_accel_mm_s2"] = profile.defaultLinearAccelerationMmSec2;
    settings["default_linear_jerk_mm_s3"] = profile.defaultLinearJerkMmSec3;
    settings["default_tool_angular_speed_rad_s"] = profile.defaultToolAngularSpeedRadSec;
    settings["default_tool_angular_accel_rad_s2"] = profile.defaultToolAngularAccelerationRadSec2;
    settings["default_tool_angular_jerk_rad_s3"] = profile.defaultToolAngularJerkRadSec3;
    settings["singularity_threshold_rad"] = profile.singularityThresholdRad;
    return settings;
}

// Rounds to a fixed number of significant digits, so the serialised number is short.
constexpr int kFirmwareModelDigits = 12;

double roundToSignificantDigits(double value, int digits) {
    if (!std::isfinite(value) || value == 0.0) return value;
    char buffer[64] = {0};
    std::snprintf(buffer, sizeof(buffer), "%.*g", digits, value);
    return std::strtod(buffer, nullptr);
}
// Copies the motion settings the firmware's load_robot_model accepts, skipping anything it does
// not read. The schema key is mandatory: handleLoadRobotModel rejects the whole command with
// bad_motion_schema unless motion_settings_schema matches kMotionSettingsSchemaVersion exactly.
void appendMotionSettingsForFirmware(Json* command, const Json& settings) {
    static const char* const kKeys[] = {
        "control_period_s",
        "default_joint_speed_deg_s",
        "default_linear_speed_mm_s",
        "default_linear_accel_mm_s2",
        "default_linear_jerk_mm_s3",
        "default_tool_angular_speed_rad_s",
        "default_tool_angular_accel_rad_s2",
        "default_tool_angular_jerk_rad_s3",
        "singularity_threshold_rad",
    };
    const int schema = jsoncompat::fieldInt(settings, "motion_settings_schema", 0);
    if (schema > 0) (*command)["motion_settings_schema"] = schema;
    for (const char* key : kKeys) {
        const double value = jsoncompat::fieldDouble(settings, key, 0.0);
        if (std::isfinite(value) && value > 0.0) (*command)[key] = roundToSignificantDigits(value, kFirmwareModelDigits);
    }
}


// The load_robot_model command: the kinematics the firmware plans against, plus the dynamics limits
// and defaults from the package profile.
Json robotModelCommandForPackage(const RobotPoseController& poseController,
                                 const RobotMotionCore::RobotModel& model) {
    if (!RobotMotionCore::modelIsValid(model)) return Json::object();

    const auto arrayOf = [](const double* values, int count) {
        Json array = Json::array();
        for (int i = 0; i < count; ++i) {
            array.push_back(roundToSignificantDigits(values[i], kFirmwareModelDigits));
        }
        return array;
    };
    const auto arrayOfSix = [](const std::array<double, 6>& values) {
        Json array = Json::array();
        for (double value : values) array.push_back(roundToSignificantDigits(value, kFirmwareModelDigits));
        return array;
    };

    Json command = Json::object();
    command["cmd"] = "load_robot_model";
    command["dhm"] = arrayOf(model.dhm, 24);
    command["q_home"] = arrayOf(model.qHome, 6);
    command["q_min"] = arrayOf(model.qMin, 6);
    command["q_max"] = arrayOf(model.qMax, 6);
    command["dhm_signs"] = arrayOf(model.dhmSigns, 6);
    command["tool_bind"] = arrayOf(model.toolBindPose.values, 12);

    // Only sent when the package actually declares them; the firmware treats each as optional and
    // keeps its own AR4 defaults otherwise, which is better than being handed zeros.
    const auto scaled = [](const std::array<double, 6>& values, const std::array<double, 6>& scale) {
        std::array<double, 6> result = values;
        for (size_t i = 0; i < result.size(); ++i) {
            if (scale[i] > 0.0) result[i] *= scale[i];
        }
        return result;
    };
    if (const OPW6RobotData* robotData = poseController.robotData()) {
        if (robotData->hasJointVelocityLimits) {
            command["joint_velocity_max_rad_s"] =
                arrayOfSix(scaled(robotData->jointVelocityMaxRadS, activeRobot().motionDynamicLimitScale));
        }
        if (robotData->hasJointAccelerationLimits) {
            command["joint_acceleration_max_rad_s2"] =
                arrayOfSix(scaled(robotData->jointAccelerationMaxRadS2, activeRobot().motionDynamicLimitScale));
        }
        if (robotData->hasJointJerkLimits) {
            command["joint_jerk_max_rad_s3"] =
                arrayOfSix(scaled(robotData->jointJerkMaxRadS3, activeRobot().motionDynamicLimitScale));
        }
        // steps_per_deg_nominal is deliberately not sent: the firmware never reads it, in
        // load_robot_model or anywhere else, and it costs 69 bytes of a 1535-byte line.
        if (robotData->controllerMinTickGapUs > 0.0) {
            command["controller_min_tick_gap_us"] =
                roundToSignificantDigits(robotData->controllerMinTickGapUs, kFirmwareModelDigits);
        }
    }
    appendMotionSettingsForFirmware(&command, motionSettingsFromRobotData(poseController.robotData()));
    return command;
}

// Loads a settings object into the editors. Non-positive values are ignored, so a firmware
// readback that omits a key leaves whatever the package or the user last supplied.
void applyMotionSettingsToEditors(RobotInstance& robot, const Json& settings, const std::string& status) {
    if (settings.empty()) return;
    const int schema = jsoncompat::fieldInt(settings, "motion_settings_schema",
                                            static_cast<int>(RobotMotionCore::kMotionSettingsSchemaVersion));
    if (schema != static_cast<int>(RobotMotionCore::kMotionSettingsSchemaVersion)) {
        robot.motionSettingsStatus = "Unsupported motion settings schema " + std::to_string(schema) + ".";
        return;
    }
    const auto assign = [](double* target, double value) {
        if (target && std::isfinite(value) && value > 0.0) *target = value;
    };
    assign(&robot.motionControlPeriodMs, jsoncompat::fieldDouble(settings, "control_period_s") * 1000.0);
    assign(&robot.motionJointAccelDegS2, jsoncompat::fieldDouble(settings, "default_joint_accel_rad_s2") * kRadToDeg);
    assign(&robot.motionJointJerkDegS3, jsoncompat::fieldDouble(settings, "default_joint_jerk_rad_s3") * kRadToDeg);
    assign(&robot.motionLinearAccelMmS2, jsoncompat::fieldDouble(settings, "default_linear_accel_mm_s2"));
    assign(&robot.motionLinearJerkMmS3, jsoncompat::fieldDouble(settings, "default_linear_jerk_mm_s3"));
    assign(&robot.motionToolAngularSpeedDegS, jsoncompat::fieldDouble(settings, "default_tool_angular_speed_rad_s") * kRadToDeg);
    assign(&robot.motionToolAngularAccelDegS2, jsoncompat::fieldDouble(settings, "default_tool_angular_accel_rad_s2") * kRadToDeg);
    assign(&robot.motionToolAngularJerkDegS3, jsoncompat::fieldDouble(settings, "default_tool_angular_jerk_rad_s3") * kRadToDeg);
    assign(&robot.motionSingularityDeg, jsoncompat::fieldDouble(settings, "singularity_threshold_rad") * kRadToDeg);
    robot.motionSettingsStatus = status;
}

// The Motion Planner values in the units RobotProgramSimulator expects. Per instance, because a
// cell plans arms that are not the one whose panels happen to be open: the station's State row
// starts a run on whichever arm the row belongs to. The active-robot forms below are what the
// panels keep calling.
RobotProgramSimulator::MotionSettingsOverride motionSettingsOverrideFor(const RobotInstance& robot) {
    RobotProgramSimulator::MotionSettingsOverride override;
    override.controlPeriodSec = robot.motionControlPeriodMs / 1000.0;
    override.singularityThresholdRad = robot.motionSingularityDeg * kDegToRad;
    override.defaultJointAccelerationRadSec2 = robot.motionJointAccelDegS2 * kDegToRad;
    override.defaultJointJerkRadSec3 = robot.motionJointJerkDegS3 * kDegToRad;
    override.dynamicLimitScale = robot.motionDynamicLimitScale;
    override.defaultLinearAccelerationMmSec2 = robot.motionLinearAccelMmS2;
    override.defaultLinearJerkMmSec3 = robot.motionLinearJerkMmS3;
    override.defaultToolAngularSpeedRadSec = robot.motionToolAngularSpeedDegS * kDegToRad;
    override.defaultToolAngularAccelerationRadSec2 = robot.motionToolAngularAccelDegS2 * kDegToRad;
    override.defaultToolAngularJerkRadSec3 = robot.motionToolAngularJerkDegS3 * kDegToRad;
    return override;
}

RobotProgramSimulator::MotionSettingsOverride currentMotionSettingsOverride() {
    return motionSettingsOverrideFor(activeRobot());
}

// Single source for the singularity threshold used outside buildTrajectory, so the readouts and
// markers agree with the planner once the editor moves off the package default.
double effectiveSingularityThresholdRadFor(const RobotInstance& robot) {
    const double fromEditor = robot.motionSingularityDeg * kDegToRad;
    if (fromEditor > 0.0) return fromEditor;
    return robot.simulator.singularityThresholdRad();
}

double effectiveSingularityThresholdRad() {
    return effectiveSingularityThresholdRadFor(activeRobot());
}

// Used to suppress override warnings when the saved and package models match.
bool robotModelsMatch(const RobotMotionCore::RobotModel& a, const RobotMotionCore::RobotModel& b) {
    const auto same = [](const double* left, const double* right, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            const double scale = std::max(1.0, std::max(std::abs(left[i]), std::abs(right[i])));
            if (std::abs(left[i] - right[i]) > 1.0e-9 * scale) return false;
        }
        return true;
    };
    return same(a.dhm, b.dhm, 24) && same(a.qHome, b.qHome, 6) && same(a.qMin, b.qMin, 6) &&
           same(a.qMax, b.qMax, 6) && same(a.dhmSigns, b.dhmSigns, 6) &&
           same(a.toolBindPose.values, b.toolBindPose.values, 12);
}

bool robotModelOverrideDiffers(const RobotInstance& robot) {
    if (!robot.robotModelOverrideValid) return false;
    const RobotMotionCore::RobotModel packaged = robot.poseController.motionModel();
    if (!RobotMotionCore::modelIsValid(packaged)) return true;
    return !robotModelsMatch(robot.robotModelOverride, packaged);
}

// Named instance, because the two callers mean different arms: the simulator plans for the one on
// screen, and the firmware upload is for the one on the end of the cable.
RobotMotionCore::RobotModel currentRobotModelFor(const RobotInstance& robot) {
    if (robot.robotModelOverrideValid) return robot.robotModelOverride;
    return robot.poseController.motionModel();
}

// Everything above a node, without the node itself. What a placement held on that node has to be
// composed with to reach world, and what a world pose has to be divided by to be written back into
// it.
CadTransform parentWorldTransformOf(const CadNode* node) {
    CadTransform world;
    for (const CadNode* ancestor = node ? node->parent : nullptr; ancestor != nullptr;
         ancestor = ancestor->parent) {
        world = ancestor->loc * world;
    }
    return world;
}

// The node that holds this arm's placement: its base frame, or the arm itself when it has none -
// an arm that is the root of its own tree. Every reader and writer of a placement goes through
// here so neither case needs a second code path.
CadNode* baseFrameNodeFor(const RobotInstance& robot) {
    return robotBaseFrameNode(robot.poseController.robotNode());
}

Json jsonFromRobotModel(const RobotMotionCore::RobotModel& model) {
    const auto arrayOf = [](const double* values, int count) {
        Json array = Json::array();
        for (int i = 0; i < count; ++i) array.push_back(values[i]);
        return array;
    };
    Json object = Json::object();
    object["dhm"] = arrayOf(model.dhm, 24);
    object["q_home"] = arrayOf(model.qHome, 6);
    object["q_min"] = arrayOf(model.qMin, 6);
    object["q_max"] = arrayOf(model.qMax, 6);
    object["dhm_signs"] = arrayOf(model.dhmSigns, 6);
    object["tool_bind"] = arrayOf(model.toolBindPose.values, 12);
    return object;
}

// Every array has to be the right length and the result has to satisfy modelIsValid, because a
// half-read model is worse than none: it would be uploaded to the firmware and planned against.
bool robotModelFromJson(const Json& object, RobotMotionCore::RobotModel* model) {
    if (model == nullptr || !object.is_object()) return false;
    const auto read = [&object](const char* key, double* values, size_t count) {
        const Json& array = jsoncompat::fieldArray(object, key);
        if (array.size() != count) return false;
        for (size_t i = 0; i < count; ++i) values[i] = jsoncompat::toDouble(array[i]);
        return true;
    };
    RobotMotionCore::RobotModel parsed = {};
    if (!read("dhm", parsed.dhm, 24) || !read("q_home", parsed.qHome, 6) ||
        !read("q_min", parsed.qMin, 6) || !read("q_max", parsed.qMax, 6) ||
        !read("dhm_signs", parsed.dhmSigns, 6) ||
        !read("tool_bind", parsed.toolBindPose.values, 12)) {
        return false;
    }
    parsed.valid = 1;
    if (!RobotMotionCore::modelIsValid(parsed)) return false;
    *model = parsed;
    return true;
}

Json stationConfigForInstance(const RobotInstance& robot) {
    // Cleared by disconnect, so this is "a real robot has told us where it is", not merely
    // "a port was open at some point".
    const bool withHardwareReference = robot.hardware.status().valid;
    Json joints = Json::array();
    for (size_t i = 0; i < 6; ++i) {
        Json joint = Json::object();
        joint["joint"] = static_cast<int>(i) + 1;
        joint["steps_per_deg"] = robot.masterStepsPerDegree[i];
        joint["offset_deg"] = robot.masterOffsetDeg[i];
        joint["master_dir"] = robot.masterDirection[i];
        joint["jog_dir_invert"] = robot.jogDirInvert[i] ? 1 : 0;
        if (withHardwareReference) {
            joint["mastered"] = robot.hardware.status().jointMastered[i];
            joint["current_steps"] = robot.hardware.status().currentSteps[i];
            joint["zero_steps"] = robot.hardware.status().zeroSteps[i];
        }
        // Saved too, because Restore reference sends it: without it a restored file would put the
        // zero back but leave the firmware thinking the switch is at step zero, which is what
        // master_limit_distance_deg is derived from.
        if (withHardwareReference) joint["master_limit_steps"] = activeRobot().hardware.status().masterLimitSteps[i];
        joints.push_back(joint);
    }
    Json document = Json::object();
    document["backoff_steps"] = robot.masterBackoffSteps;
    document["joints"] = joints;
    document["motion_settings"] = motionSettingsForInstance(robot);
    Json limitScale = Json::array();
    for (double value : robot.motionDynamicLimitScale) limitScale.push_back(value);
    document["dynamic_limit_scale"] = limitScale;
    document["weave_schedule_schema"] = static_cast<int>(RobotMotionCore::kWeaveScheduleSchemaVersion);
    Json weaveSchedules = Json::array();
    for (int index = 0; index < static_cast<int>(RobotMotionCore::kMaxWeaveSchedules); ++index) {
        Json entry = jsonFromWeaveParams(robot.weaveSchedules.schedules[index]);
        entry["valid"] = robot.weaveSchedules.valid[index] ? 1 : 0;
        entry["name"] = robot.weaveScheduleNames[static_cast<size_t>(index)];
        weaveSchedules.push_back(entry);
    }
    document["weave_schedules"] = weaveSchedules;
    const RobotMotionCore::RobotModel model = currentRobotModelFor(robot);
    if (RobotMotionCore::modelIsValid(model)) document["robot_model"] = jsonFromRobotModel(model);
    return document;
}

Json masteringDocumentFromEditors() { return stationConfigForInstance(activeRobot()); }

void applyMasteringDocumentToEditors(RobotInstance& robot, const Json& document) {
    const Json mastering = masteringObjectFromDocument(document);
    const Json& stepsPerDeg = jsoncompat::fieldArray(mastering, "steps_per_deg");
    const Json& offsets = jsoncompat::fieldArray(mastering, "offset_deg");
    const Json& directions = jsoncompat::fieldArray(mastering, "master_dir");
    const Json& inverts = jsoncompat::fieldArray(mastering, "jog_dir_invert");
    for (size_t i = 0; i < 6; ++i) {
        if (i < stepsPerDeg.size()) {
            const double value = jsoncompat::toDouble(stepsPerDeg[i]);
            if (value > 0.0) robot.masterStepsPerDegree[i] = value;
        }
        if (i < offsets.size()) robot.masterOffsetDeg[i] = jsoncompat::toDouble(offsets[i]);
        if (i < directions.size()) {
            robot.masterDirection[i] = jsoncompat::toInt(directions[i], 1) >= 0 ? 1 : -1;
        }
        if (i < inverts.size()) {
            robot.jogDirInvert[i] = jsoncompat::toInt(inverts[i], 0) != 0;
        }
    }
    const int backoff = jsoncompat::fieldInt(document, "backoff_steps", robot.masterBackoffSteps);
    if (backoff >= 10 && backoff <= 5000) robot.masterBackoffSteps = backoff;

    const Json& settings = jsoncompat::fieldObject(document, "motion_settings");
    if (!settings.empty()) applyMotionSettingsToEditors(robot, settings, "Loaded from file.");

    // Backward-compatible field alias.
    const Json& limitScales = jsoncompat::contains(document, "dynamic_limit_scale")
        ? jsoncompat::fieldArray(document, "dynamic_limit_scale")
        : jsoncompat::fieldArray(document, "joint_limit_scale");
    for (size_t i = 0; i < 6 && i < limitScales.size(); ++i) {
        const double value = jsoncompat::toDouble(limitScales[i], 1.0);
        robot.motionDynamicLimitScale[i] = value > 0.0 ? value : 1.0;
    }

    const Json& weaveSchedules = jsoncompat::fieldArray(document, "weave_schedules");
    if (!weaveSchedules.empty()) {
        const int schema = jsoncompat::fieldInt(document, "weave_schedule_schema",
                                                static_cast<int>(RobotMotionCore::kWeaveScheduleSchemaVersion));
        if (schema != static_cast<int>(RobotMotionCore::kWeaveScheduleSchemaVersion)) {
            robot.weaveScheduleStatus =
                "Ignored the file's weave schedules: unsupported schema " + std::to_string(schema) + ".";
        } else {
            for (size_t i = 0; i < weaveSchedules.size() && i < RobotMotionCore::kMaxWeaveSchedules; ++i) {
                const Json& entry = weaveSchedules[i];
                if (!entry.is_object()) continue;
                RobotMotionCore::WeaveParams weave = {};
                if (!weaveParamsFromJson(entry, &weave)) continue;
                robot.weaveSchedules.schedules[i] = weave;
                robot.weaveSchedules.valid[i] = jsoncompat::fieldInt(entry, "valid", 0) != 0 ? 1 : 0;
                robot.weaveScheduleNames[i] = jsoncompat::fieldString(entry, "name");
            }
            robot.weaveScheduleStatus = "Weave schedules loaded from file.";
        }
    }

    const Json& modelObject = jsoncompat::fieldObject(document, "robot_model");
    if (!modelObject.empty()) {
        RobotMotionCore::RobotModel model = {};
        if (robotModelFromJson(modelObject, &model)) {
            robot.robotModelOverride = model;
            robot.robotModelOverrideValid = true;
            robot.robotModelOverrideSource = "file";
        } else {
            activeRobot().hardwareStatus = "Ignored the file's robot model: it is not a valid model.";
        }
    }
}

// Sends a loaded calibration to the firmware. This is the point of Load mastering: the editors
// alone change nothing, and it is the load_mastering command that restores the step reference so
// the joints read as mastered again without re-running a sweep.
void applyMasteringToFirmware(const Json& document, bool restoreReference) {
    const Json mastering = masteringObjectFromDocument(document);
    Json command = Json::object();
    command["cmd"] = "load_mastering";
    if (restoreReference) {
        for (const char* key : {"mastered", "current_steps", "zero_steps", "master_limit_steps"}) {
            if (jsoncompat::contains(mastering, key)) command[key] = jsoncompat::fieldArray(mastering, key);
        }
    }
    command["steps_per_deg"] = jsoncompat::fieldArray(mastering, "steps_per_deg");
    command["offset_deg"] = jsoncompat::fieldArray(mastering, "offset_deg");
    // Both direction tables are sent in both modes: which way a joint sweeps to find its switch,
    // and which way its motor turns for a commanded sign, are calibration, and have nothing to do
    // with whether the arm is standing at the saved zero.
    Json directions = Json::array();
    Json inverts = Json::array();
    for (size_t i = 0; i < 6; ++i) {
        directions.push_back(activeRobot().masterDirection[i]);
        inverts.push_back(activeRobot().jogDirInvert[i] ? 1 : 0);
    }
    command["master_dir"] = directions;
    command["jog_dir_invert"] = inverts;
    if (!activeRobot().hardware.isConnected()) {
        activeRobot().hardwareStatus = "Loaded into the editors; connect to send it to the robot.";
        return;
    }
    activeRobot().hardware.sendCommand(command);
    activeRobot().hardware.requestStatus();
    activeRobot().hardwareStatus = restoreReference ? "Mastering restored on the robot."
                                              : "Calibration sent to the robot.";
}

void applyLoadedMasteringDocument(const Json& document, const std::string& source) {
    applyMasteringDocumentToEditors(activeRobot(), document);
    activeRobot().hardware.setRobotModelCommand(
        robotModelCommandForPackage(activeRobot().poseController, currentRobotModelFor(activeRobot())));
    // Whether the arm is standing at the saved zero is a question only the operator can answer,
    // and getting it wrong means the robot moves against a wrong reference, so it is asked rather
    // than assumed, via a modal drawn next frame.
    activeRobot().pendingMasteringDocument = document;
    activeRobot().pendingMasteringSource = source;
    activeRobot().masteringConfirmPending = true;
    activeRobot().hardwareStatus = "Loaded mastering: " + source;
}

Json motionSettingsForInstance(const RobotInstance& robot) {
    Json settings = Json::object();
    settings["motion_settings_schema"] = static_cast<int>(RobotMotionCore::kMotionSettingsSchemaVersion);
    settings["control_period_s"] = robot.motionControlPeriodMs / 1000.0;
    settings["default_joint_speed_deg_s"] = jsoncompat::fieldDouble(
        robot.packageMotionSettings, "default_joint_speed_deg_s",
        RobotMotionCore::defaultMotionProgramSettings().defaultJointSpeedDegPerSec);
    settings["default_linear_speed_mm_s"] = jsoncompat::fieldDouble(
        robot.packageMotionSettings, "default_linear_speed_mm_s",
        RobotMotionCore::defaultMotionProgramSettings().defaultLinearSpeedMmPerSec);
    settings["default_linear_accel_mm_s2"] = robot.motionLinearAccelMmS2;
    settings["default_linear_jerk_mm_s3"] = robot.motionLinearJerkMmS3;
    settings["default_tool_angular_speed_rad_s"] = robot.motionToolAngularSpeedDegS * kDegToRad;
    settings["default_tool_angular_accel_rad_s2"] = robot.motionToolAngularAccelDegS2 * kDegToRad;
    settings["default_tool_angular_jerk_rad_s3"] = robot.motionToolAngularJerkDegS3 * kDegToRad;
    settings["singularity_threshold_rad"] = robot.motionSingularityDeg * kDegToRad;
    return settings;
}

Json motionSettingsFromEditors() { return motionSettingsForInstance(activeRobot()); }
