#include "ProgramTextIo.h"

#include <cmath>
#include <fstream>

#include "RobotProgramSimulator.h"
#include "RobotRuntime.h"
#include "StringUtil.h"

using strutil::operator<<;

Json jsonArrayFromTransform(const RobotMotionCore::Transform& transform) {
    Json array = Json::array();
    for (int i = 0; i < 12; ++i) array.push_back(transform.values[i]);
    return array;
}

Json jsonFromWeaveParams(const RobotMotionCore::WeaveParams& weave) {
    Json object = Json::object();
    object["shape"] = static_cast<int>(weave.shape);
    object["rate_mode"] = static_cast<int>(weave.rateMode);
    object["freq_hz"] = weave.frequencyHz;
    object["wavelength_mm"] = weave.wavelengthMm;
    object["amp_left_mm"] = weave.amplitudeLeftMm;
    object["amp_right_mm"] = weave.amplitudeRightMm;
    object["elevation_mm"] = weave.elevationMm;
    object["plane_angle_deg"] = weave.planeAngleDeg;
    object["bias_mm"] = weave.biasMm;
    object["dwell_left"] = weave.dwellLeft;
    object["dwell_center"] = weave.dwellCenter;
    object["dwell_right"] = weave.dwellRight;
    return object;
}

bool weaveParamsFromJson(const Json& object, RobotMotionCore::WeaveParams* weave) {
    if (!weave || !object.is_object()) return false;
    *weave = RobotMotionCore::defaultWeaveParams();
    const int shape = jsoncompat::fieldInt(object, "shape", 0);
    const int rateMode = jsoncompat::fieldInt(object, "rate_mode", 0);
    if (shape < 0 || shape > static_cast<int>(RobotMotionCore::WeaveShape::LShape)) return false;
    if (rateMode < 0 || rateMode > static_cast<int>(RobotMotionCore::WeaveRateMode::Wavelength)) return false;
    weave->shape = static_cast<RobotMotionCore::WeaveShape>(shape);
    weave->rateMode = static_cast<RobotMotionCore::WeaveRateMode>(rateMode);
    weave->frequencyHz = jsoncompat::fieldDouble(object, "freq_hz", 0.0);
    weave->wavelengthMm = jsoncompat::fieldDouble(object, "wavelength_mm", 0.0);
    weave->amplitudeLeftMm = jsoncompat::fieldDouble(object, "amp_left_mm", 0.0);
    weave->amplitudeRightMm = jsoncompat::fieldDouble(object, "amp_right_mm", 0.0);
    weave->elevationMm = jsoncompat::fieldDouble(object, "elevation_mm", 0.0);
    weave->planeAngleDeg = jsoncompat::fieldDouble(object, "plane_angle_deg", 0.0);
    weave->biasMm = jsoncompat::fieldDouble(object, "bias_mm", 0.0);
    weave->dwellLeft = jsoncompat::fieldDouble(object, "dwell_left", 0.0);
    weave->dwellCenter = jsoncompat::fieldDouble(object, "dwell_center", 0.0);
    weave->dwellRight = jsoncompat::fieldDouble(object, "dwell_right", 0.0);
    return true;
}



std::string programNumber(double value) {
    return strutil::formatShortest(value);
}

bool parseDoubleToken(const std::string& token, double* value) {
    double parsed = 0.0;
    if (!strutil::parseDouble(token, &parsed) || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

bool parseJointDegrees(const std::vector<std::string>& tokens, int firstToken, std::array<double, 6>* joints) {
    if (tokens.size() < firstToken + 6) return false;
    for (int i = 0; i < 6; ++i) {
        double degrees = 0.0;
        if (!parseDoubleToken(tokens[firstToken + i], &degrees)) return false;
        (*joints)[static_cast<size_t>(i)] = degrees * kDegToRad;
    }
    return true;
}

bool parseProgramTextLineHeadless(const std::string& line, int lineNumber, ParsedProgramInstruction* instruction, std::string* errorMessage) {
    const std::string trimmed = strutil::trimmed(line);
    if (trimmed.empty() || strutil::startsWith(trimmed, "#")) return false;
    const std::vector<std::string> tokens = strutil::splitSkippingEmpty(trimmed, ' ');
    if (tokens.empty()) return false;

    const std::string command = strutil::toLower(tokens[0]);
    if (command == "robotsimulatorprogram") return false;

    if (command == "movej" || command == "movel") {
        if (tokens.size() != 7 && tokens.size() != 8) {
            if (errorMessage) *errorMessage = strutil::format(
                "Line %1: %2 expects 6 joint values in degrees and optional J7 in mm.")
                .arg(lineNumber).arg(tokens[0]);
            return false;
        }
        std::array<double, 6> joints{};
        if (!parseJointDegrees(tokens, 1, &joints)) {
            if (errorMessage) *errorMessage = strutil::format("Line %1: invalid %2 joint value.").arg(lineNumber).arg(tokens[0]);
            return false;
        }
        double externalAxisMm = 0.0;
        const bool hasExternalAxis = tokens.size() == 8;
        if (hasExternalAxis && !parseDoubleToken(tokens[7], &externalAxisMm)) {
            if (errorMessage) *errorMessage = strutil::format(
                "Line %1: invalid J7 position.").arg(lineNumber);
            return false;
        }
        if (command == "movej") {
            instruction->type = RobotProgramNodeType::MoveJ;
            instruction->data = MoveJData{joints, hasExternalAxis, externalAxisMm};
        } else {
            instruction->type = RobotProgramNodeType::MoveL;
            instruction->data = MoveLData{joints, hasExternalAxis, externalAxisMm};
        }
        return true;
    }

    if (command == "movec") {
        if (errorMessage) *errorMessage = strutil::format("Line %1: MoveC is disabled because firmware does not support it.").arg(lineNumber);
        return false;
    }

    if (command == "setspeed") {
        if (tokens.size() != 3) {
            if (errorMessage) *errorMessage = strutil::format("Line %1: SetSpeed expects joint_deg_per_sec and linear_mm_per_sec.").arg(lineNumber);
            return false;
        }
        double jointDegPerSec = 0.0;
        double linearMmPerSec = 0.0;
        if (!parseDoubleToken(tokens[1], &jointDegPerSec) || !parseDoubleToken(tokens[2], &linearMmPerSec) ||
            jointDegPerSec <= 0.0 || linearMmPerSec <= 0.0) {
            if (errorMessage) *errorMessage = strutil::format("Line %1: invalid SetSpeed value.").arg(lineNumber);
            return false;
        }
        instruction->type = RobotProgramNodeType::SetSpeed;
        instruction->data = SetSpeedData{jointDegPerSec * kDegToRad, linearMmPerSec};
        return true;
    }

    if (command == "actuate") {
        if (tokens.size() != 4) {
            if (errorMessage) *errorMessage = strutil::format(
                "Line %1: Actuate expects mechanism_id actuator_id position.").arg(lineNumber);
            return false;
        }
        double position = 0.0;
        if (!parseDoubleToken(tokens[3], &position)) {
            if (errorMessage) *errorMessage = strutil::format(
                "Line %1: invalid Actuate position.").arg(lineNumber);
            return false;
        }
        instruction->type = RobotProgramNodeType::Actuate;
        instruction->data = ActuateData{tokens[1], tokens[2], position};
        return true;
    }

    if (command == "settool") {
        if (tokens.size() != 3) {
            if (errorMessage) *errorMessage = strutil::format(
                "Line %1: SetTool expects tool_id and zero-based tcp_index.").arg(lineNumber);
            return false;
        }
        char* end = nullptr;
        const long tcpIndex = std::strtol(tokens[2].c_str(), &end, 10);
        if (!end || *end != '\0' || tcpIndex < 0 || tcpIndex > 1024) {
            if (errorMessage) *errorMessage = strutil::format(
                "Line %1: invalid SetTool TCP index.").arg(lineNumber);
            return false;
        }
        instruction->type = RobotProgramNodeType::SetTool;
        instruction->data = SetToolData{tokens[1], static_cast<int>(tcpIndex)};
        return true;
    }

    if (command == "stop") {
        if (tokens.size() != 1) {
            if (errorMessage) *errorMessage = strutil::format(
                "Line %1: Stop does not take arguments.").arg(lineNumber);
            return false;
        }
        instruction->type = RobotProgramNodeType::Stop;
        instruction->data = std::monostate{};
        return true;
    }

    if (command == "trigger") {
        bool unterminated = false;
        const std::vector<std::string> parts = strutil::splitQuotedTokens(trimmed, &unterminated);
        if (unterminated) {
            if (errorMessage) *errorMessage = strutil::format("Line %1: unterminated quote.").arg(lineNumber);
            return false;
        }

        TriggerData data;
        bool sawAction = false;
        for (size_t i = 1; i < parts.size(); ++i) {
            const std::string& token = parts[i];
            const size_t equals = token.find('=');
            if (equals == std::string::npos) {
                // The action, followed by its argument. ShowMessage is the only one so far.
                if (strutil::toLower(token) != "showmessage") {
                    if (errorMessage) {
                        *errorMessage = strutil::format("Line %1: unknown trigger action '%2'.").arg(lineNumber).arg(token);
                    }
                    return false;
                }
                if (i + 1 >= parts.size()) {
                    if (errorMessage) {
                        *errorMessage = strutil::format("Line %1: ShowMessage needs a quoted message.").arg(lineNumber);
                    }
                    return false;
                }
                data.message = parts[i + 1];
                sawAction = true;
                ++i;
                continue;
            }

            const std::string key = strutil::toLower(token.substr(0, equals));
            const std::string value = token.substr(equals + 1);
            if (key == "ref") {
                const std::string reference = strutil::toLower(value);
                if (reference == "start") {
                    data.referenceStart = true;
                } else if (reference == "end") {
                    data.referenceStart = false;
                } else {
                    if (errorMessage) {
                        *errorMessage = strutil::format("Line %1: trigger ref must be start or end.").arg(lineNumber);
                    }
                    return false;
                }
                continue;
            }
            double number = 0.0;
            if (!parseDoubleToken(value, &number)) {
                if (errorMessage) *errorMessage = strutil::format("Line %1: trigger %2 is not a number.").arg(lineNumber).arg(key);
                return false;
            }
            if (key == "dist") {
                data.distanceMm = number;
            } else if (key == "time") {
                data.timeMs = number;
            } else {
                if (errorMessage) *errorMessage = strutil::format("Line %1: unknown trigger key '%2'.").arg(lineNumber).arg(key);
                return false;
            }
        }
        if (!sawAction) {
            if (errorMessage) {
                *errorMessage = strutil::format("Line %1: Trigger needs an action, e.g. ShowMessage \"text\".").arg(lineNumber);
            }
            return false;
        }
        instruction->type = RobotProgramNodeType::Trigger;
        instruction->data = data;
        return true;
    }

    if (command == "weaveon") {
        // WeaveOn <schedule>, or a bare WeaveOn to run whatever the last SetWeave established.
        if (tokens.size() > 2) {
            if (errorMessage) *errorMessage = strutil::format("Line %1: WeaveOn takes at most one schedule index.").arg(lineNumber);
            return false;
        }
        WeaveOnData data;
        if (tokens.size() == 2) {
            double index = 0.0;
            if (!parseDoubleToken(tokens[1], &index) || index < 0.0 ||
                index >= static_cast<double>(RobotMotionCore::kMaxWeaveSchedules) ||
                index != std::floor(index)) {
                if (errorMessage) {
                    *errorMessage = strutil::format("Line %1: WeaveOn schedule must be a whole number from 0 to %2.")
                                        .arg(lineNumber)
                                        .arg(RobotMotionCore::kMaxWeaveSchedules - 1);
                }
                return false;
            }
            data.scheduleIndex = static_cast<int>(index);
        }
        instruction->type = RobotProgramNodeType::WeaveOn;
        instruction->data = data;
        return true;
    }

    if (command == "weaveoff") {
        if (tokens.size() != 1) {
            if (errorMessage) *errorMessage = strutil::format("Line %1: WeaveOff takes no arguments.").arg(lineNumber);
            return false;
        }
        instruction->type = RobotProgramNodeType::WeaveOff;
        instruction->data = std::monostate{};
        return true;
    }

    if (command == "setweave") {
        RobotMotionCore::WeaveParams weave = RobotMotionCore::defaultWeaveParams();
        weave.shape = RobotMotionCore::WeaveShape::Sine;
        bool sawFrequency = false;
        bool sawWavelength = false;
        for (size_t i = 1; i < tokens.size(); ++i) {
            const std::string& token = tokens[i];
            const size_t equals = token.find('=');
            if (equals == std::string::npos || equals == 0 || equals + 1 >= token.size()) {
                if (errorMessage) *errorMessage = strutil::format("Line %1: SetWeave expects key=value, got '%2'.").arg(lineNumber).arg(token);
                return false;
            }
            const std::string key = strutil::toLower(token.substr(0, equals));
            const std::string value = token.substr(equals + 1);

            if (key == "shape") {
                const std::string shape = strutil::toLower(value);
                if (shape == "none") weave.shape = RobotMotionCore::WeaveShape::None;
                else if (shape == "sine") weave.shape = RobotMotionCore::WeaveShape::Sine;
                else if (shape == "zigzag") weave.shape = RobotMotionCore::WeaveShape::Zigzag;
                else if (shape == "trapezoid") weave.shape = RobotMotionCore::WeaveShape::Trapezoid;
                else if (shape == "circular") weave.shape = RobotMotionCore::WeaveShape::Circular;
                else if (shape == "figure8") weave.shape = RobotMotionCore::WeaveShape::FigureEight;
                else if (shape == "lshape") weave.shape = RobotMotionCore::WeaveShape::LShape;
                else {
                    if (errorMessage) *errorMessage = strutil::format("Line %1: unknown weave shape '%2'.").arg(lineNumber).arg(value);
                    return false;
                }
                continue;
            }

            double number = 0.0;
            if (!parseDoubleToken(value, &number)) {
                if (errorMessage) *errorMessage = strutil::format("Line %1: SetWeave %2 is not a number.").arg(lineNumber).arg(key);
                return false;
            }
            if (key == "freq") {
                weave.frequencyHz = number;
                weave.rateMode = RobotMotionCore::WeaveRateMode::Frequency;
                sawFrequency = true;
            } else if (key == "wavelength") {
                weave.wavelengthMm = number;
                weave.rateMode = RobotMotionCore::WeaveRateMode::Wavelength;
                sawWavelength = true;
            } else if (key == "amp") {
                weave.amplitudeLeftMm = number;
                weave.amplitudeRightMm = number;
            } else if (key == "amp_l") {
                weave.amplitudeLeftMm = number;
            } else if (key == "amp_r") {
                weave.amplitudeRightMm = number;
            } else if (key == "elev") {
                weave.elevationMm = number;
            } else if (key == "angle") {
                weave.planeAngleDeg = number;
            } else if (key == "bias") {
                weave.biasMm = number;
            } else if (key == "dwell_l") {
                weave.dwellLeft = number;
            } else if (key == "dwell_c") {
                weave.dwellCenter = number;
            } else if (key == "dwell_r") {
                weave.dwellRight = number;
            } else {
                if (errorMessage) *errorMessage = strutil::format("Line %1: unknown SetWeave key '%2'.").arg(lineNumber).arg(key);
                return false;
            }
        }
        if (sawFrequency && sawWavelength) {
            if (errorMessage) {
                *errorMessage = strutil::format("Line %1: SetWeave takes freq or wavelength, not both.").arg(lineNumber);
            }
            return false;
        }
        instruction->type = RobotProgramNodeType::SetWeave;
        instruction->data = SetWeaveData{weave};
        return true;
    }

    if (command == "setblending") {
        if (tokens.size() != 2) {
            if (errorMessage) *errorMessage = strutil::format("Line %1: SetBlending expects endpoint_deviation_mm.").arg(lineNumber);
            return false;
        }
        double radiusMm = 0.0;
        if (!parseDoubleToken(tokens[1], &radiusMm)) {
            if (errorMessage) *errorMessage = strutil::format("Line %1: invalid SetBlending endpoint deviation.").arg(lineNumber);
            return false;
        }
        instruction->type = RobotProgramNodeType::SetBlending;
        instruction->data = SetBlendingData{radiusMm};
        return true;
    }

    if (errorMessage) *errorMessage = strutil::format("Line %1: unknown instruction '%2'.").arg(lineNumber).arg(tokens[0]);
    return false;
}

void appendInstructionToRoot(RobotProgramNode* root, RobotProgramNodeType type, RobotProgramNodeData data) {
    auto node = std::make_unique<RobotProgramNode>();
    node->type = type;
    node->data = std::move(data);
    node->parent = root;
    root->children.push_back(std::move(node));
}

bool loadProgramTextFromStream(std::istream& file, RobotProgramNode* root, std::string* errorMessage) {
    int lineNumber = 0;
    std::string line;
    while (std::getline(file, line)) {
        // getline keeps a trailing carriage return on CRLF files; QTextStream stripped it.
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
        ++lineNumber;
        ParsedProgramInstruction instruction;
        std::string localError;
        const bool parsed = parseProgramTextLineHeadless(line, lineNumber, &instruction, &localError);
        if (!localError.empty()) {
            if (errorMessage) *errorMessage = localError;
            return false;
        }
        if (parsed) appendInstructionToRoot(root, instruction.type, instruction.data);
    }

    for (size_t i = 0; i < root->children.size(); ++i) {
        if (!root->children[i] || root->children[i]->type != RobotProgramNodeType::Trigger) continue;
        const bool followedByMove =
            i + 1 < root->children.size() && root->children[i + 1] &&
            (root->children[i + 1]->type == RobotProgramNodeType::MoveJ ||
             root->children[i + 1]->type == RobotProgramNodeType::MoveL ||
             root->children[i + 1]->type == RobotProgramNodeType::MoveC);
        if (!followedByMove) {
            if (errorMessage) {
                *errorMessage = strutil::format(
                                    "Instruction %1: a Trigger must be immediately followed by a move.")
                                    .arg(i + 1)
                                    .str();
            }
            return false;
        }
    }
    return true;
}

bool loadProgramTextHeadless(const std::string& fileName, RobotProgramNode* root, std::string* errorMessage) {
    std::ifstream file(fileName);
    if (!file) {
        if (errorMessage) *errorMessage = "Failed to open robot program file.";
        return false;
    }
    return loadProgramTextFromStream(file, root, errorMessage);
}

CadTransform withLocalRotation(const CadTransform& pose, int axis, double angleRadians) {
    const double c = std::cos(angleRadians);
    const double s = std::sin(angleRadians);
    CadTransform rotation;
    if (axis == 0) {
        rotation.values = {{1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0}};
    } else if (axis == 1) {
        rotation.values = {{c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0}};
    } else {
        rotation.values = {{c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0}};
    }
    return pose * rotation;
}

CadTransform composeLocalWpr(const CadTransform& basePose, const std::array<double, 3>& wprDegrees) {
    CadTransform pose = basePose;
    for (int axis = 0; axis < 3; ++axis) {
        pose = withLocalRotation(pose, axis, wprDegrees[static_cast<size_t>(axis)] * kDegToRad);
    }
    return pose;
}

double tcpDistance(const CadTransform& a, const CadTransform& b) {
    const double dx = a.values[3] - b.values[3];
    const double dy = a.values[7] - b.values[7];
    const double dz = a.values[11] - b.values[11];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string motionTypeName(RobotProgramNodeType type) {
    switch (type) {
    case RobotProgramNodeType::MoveJ:
        return "MoveJ";
    case RobotProgramNodeType::MoveL:
        return "MoveL";
    case RobotProgramNodeType::MoveC:
        return "MoveC";
    case RobotProgramNodeType::Root:
    case RobotProgramNodeType::SetSpeed:
    case RobotProgramNodeType::SetBlending:
    case RobotProgramNodeType::SetWeave:
    case RobotProgramNodeType::WeaveOn:
    case RobotProgramNodeType::WeaveOff:
    case RobotProgramNodeType::SetTool:
    case RobotProgramNodeType::Actuate:
    case RobotProgramNodeType::Stop:
    case RobotProgramNodeType::Trigger:
        break;
    }
    return "Motion";
}

bool collectProgramTcpPoints(const RobotProgramNode& root,
                             RobotPoseController* poseController,
                             std::vector<progtext::TcpFramePoint>* points) {
    points->clear();
    for (const auto& child : root.children) {
        if (!child || child->type != RobotProgramNodeType::MoveL) continue;
        const std::array<double, 6>* targetJoints = targetJointsForNode(*child);
        if (!targetJoints) continue;
        poseController->setJoints(*targetJoints);
        points->push_back({poseController->toolPose(), *targetJoints});
    }
    return points->size() >= 3;
}

std::string formatMoveLine(const char* command, const std::array<double, 6>& jointsRad) {
    std::vector<std::string> values;
    values << command;
    for (double joint : jointsRad) values << programNumber(joint * kRadToDeg);
    return strutil::join(values, " ");
}

std::string formatMoveLine(const char* command,
                           const std::array<double, 6>& jointsRad,
                           bool hasExternalAxis,
                           double externalAxisPositionMm) {
    std::string line = formatMoveLine(command, jointsRad);
    if (hasExternalAxis) line += " " + programNumber(externalAxisPositionMm);
    return line;
}

