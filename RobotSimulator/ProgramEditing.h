#pragma once

#include "AppState.h"
#include "RobotProgramModel.h"
#include "RobotProgramSimulator.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

std::array<double, 6>* editableMoveTargetJointsForNode(RobotProgramNode& node);
bool editableMoveExternalAxisForNode(RobotProgramNode& node,
                                     bool** hasExternalAxis,
                                     double** positionMm);

std::string programTextForInstruction(const RobotProgramNode& node);
std::string programTextForCurrentProgram();
void saveProgramToPath(const std::string& fileName);
int findProgramByName(const RobotInstance& robot, const std::string& name);
std::string uniqueProgramName(const RobotInstance& robot, const std::string& base);
void clearPlannedTrajectory(RobotInstance& robot);
void finishProgramLoad(RobotInstance& robot, int index, const char* what);
void loadProgramFromPath(const std::string& fileName);
RobotProgramSimulator::ProgramState programStateFor(const RobotInstance& robot);
RobotProgramSimulator::ProgramState currentProgramState();

void drawProgramRunMarkers(bool pending, bool executing);
void refreshAfterProgramEdit(RobotInstance& robot, int selectedInstruction,
                             const char* action);
RobotProgramNode* editableProgramMoveAt(RobotInstance& robot, int instruction);
bool previewProgramMoveTarget(RobotInstance& robot, int instruction);
bool editProgramMoveTargetFromWorldPose(RobotInstance& robot, int instruction,
                                        const CadTransform& worldPose,
                                        std::string* errorMessage);
int insertProgramInstruction(RobotInstance& robot, RobotProgramNodeType type,
                             RobotProgramNodeData data);
bool deleteProgramInstruction(RobotInstance& robot, int index);

struct RobotToolChoice {
    CadNode* node = nullptr;
    RobotToolData* data = nullptr;
    int tcpIndex = 0;
    std::string label;
};
std::vector<RobotToolChoice> robotToolChoices(RobotInstance& robot);
bool activateRobotToolChoice(RobotInstance& robot, const RobotToolChoice& choice);

std::unique_ptr<RobotProgramNode> cloneProgramNode(const RobotProgramNode& source,
                                                   RobotProgramNode* parent);
void selectProgram(RobotInstance& robot, int index);
void drawProgramLoadConflictModal(RobotInstance& robot);
void drawProgramList();
