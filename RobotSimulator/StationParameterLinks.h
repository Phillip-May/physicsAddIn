#pragma once

#include <map>
#include <string>
#include <vector>

#include "CadNode.h"
#include "MountingSnap.h"
#include "SceneMath.h"
#include "StationPackage.h"

bool libraryNodeIsDescendantOf(const CadNode* node, const CadNode* ancestor);
bool stationParameterEndpointEquals(const StationParameterEndpoint& a,
                                    const StationParameterEndpoint& b);
std::string stationParameterEndpointKey(const StationParameterEndpoint& endpoint);
StationAccessoryInstance* stationAccessoryById(StationDocument& station,
                                               const std::string& id,
                                               size_t* index = nullptr);
const StationAccessoryInstance* stationAccessoryById(const StationDocument& station,
                                                     const std::string& id,
                                                     size_t* index = nullptr);
double* stationAccessoryParameter(StationAccessoryInstance& accessory,
                                  const std::string& parameter);
const double* stationAccessoryParameter(const StationAccessoryInstance& accessory,
                                        const std::string& parameter);
bool stationParameterBounds(const std::string& parameter, double* minimum, double* maximum);
bool applyStationLinkedParameter(StationDocument& station,
                                 const std::string& accessoryId,
                                 const std::string& parameter,
                                 double requestedValue,
                                 std::vector<size_t>* affectedAccessories,
                                 std::string* errorMessage,
                                 bool rebuildModels = true);
void removeStationParameterLinksForAccessory(StationDocument& station,
                                             const std::string& accessoryId);
void removeStationParameterLinksForEndpoint(StationDocument& station,
                                            const StationParameterEndpoint& endpoint);
StationAccessoryInstance* stationAccessoryContainingNode(StationDocument& station,
                                                         const CadNode* node);
int createStationParameterLinksForSnap(StationDocument& station,
                                       StationAccessoryInstance& sourceAccessory,
                                       const mountingsnap::Interface& sourceInterface,
                                       CadNode* targetMountingNode);
std::string stationLinkedParameterDescription(const StationDocument& station,
                                              const std::string& accessoryId,
                                              const std::string& parameter);
