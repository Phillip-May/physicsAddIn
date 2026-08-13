#pragma once

#include <string>

#include "CadNode.h"
#include "JsonCompat.h"
#include "RobotMotionCore.h"
#include "RobotProgramSimulator.h"

struct RobotInstance;
class RobotPoseController;
struct OPW6RobotData;

Json motionSettingsFromRobotData(const OPW6RobotData* robotData);
double roundToSignificantDigits(double value, int digits);
void appendMotionSettingsForFirmware(Json* command, const Json& settings);
Json robotModelCommandForPackage(const RobotPoseController& poseController,
                                 const RobotMotionCore::RobotModel& model);
void applyMotionSettingsToEditors(RobotInstance& robot, const Json& settings, const std::string& status);
RobotProgramSimulator::MotionSettingsOverride motionSettingsOverrideFor(const RobotInstance& robot);
RobotProgramSimulator::MotionSettingsOverride currentMotionSettingsOverride();
double effectiveSingularityThresholdRadFor(const RobotInstance& robot);
double effectiveSingularityThresholdRad();
bool robotModelsMatch(const RobotMotionCore::RobotModel& a, const RobotMotionCore::RobotModel& b);
bool robotModelOverrideDiffers(const RobotInstance& robot);
RobotMotionCore::RobotModel currentRobotModelFor(const RobotInstance& robot);
CadTransform parentWorldTransformOf(const CadNode* node);
CadNode* baseFrameNodeFor(const RobotInstance& robot);
Json jsonFromRobotModel(const RobotMotionCore::RobotModel& model);
bool robotModelFromJson(const Json& object, RobotMotionCore::RobotModel* model);
Json stationConfigForInstance(const RobotInstance& robot);
Json masteringDocumentFromEditors();
void applyMasteringDocumentToEditors(RobotInstance& robot, const Json& document);
void applyMasteringToFirmware(const Json& document, bool restoreReference);
void applyLoadedMasteringDocument(const Json& document, const std::string& source);
Json motionSettingsForInstance(const RobotInstance& robot);
Json motionSettingsFromEditors();
