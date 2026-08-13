#pragma once

#include "AppState.h"

bool drawActiveToolState(RobotInstance& robot, const char* comboId, bool editing);
void drawProgramState(RobotInstance& robot, bool editing, const char* id);
void drawProgramInstructionTable(RobotInstance& robot, float tableHeight, bool editing);
bool hasSelectedEditableMove(RobotInstance& robot);
void drawSelectedMoveTargetEditor(RobotInstance& robot, bool editing);
void drawProgramInstructionEditor(RobotInstance& robot, float reservedRowsAfterEditor);
