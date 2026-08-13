#pragma once

#include <memory>

#include "CadNode.h"
#include "StationPackage.h"

bool rebuildStationRollerConveyor(CadNode* root, const StationDocument& station);
bool rebuildAr4Table(CadNode* root);
bool rebuildParametricAccessory(CadNode* root, const StationDocument& station);
void rebuildParametricAccessories(CadNode* node, const StationDocument& station);
void includeSceneVisualMinY(const CadNode* node, const CadTransform& parentTransform,
                            bool parentVisible, double& minimumY, double& maximumAbsXZ,
                            bool& foundVisual);
std::shared_ptr<CadNode> makeDefaultFloor(double topY, double requestedHalfExtentMm);
CadNode* appendDefaultFloor(const std::shared_ptr<CadNode>& root);
