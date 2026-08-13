#include "StationParameterLinks.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>

#include "AccessoryBuilders.h"

bool libraryNodeIsDescendantOf(const CadNode* node, const CadNode* ancestor) {
    for (const CadNode* current = node; current; current = current->parent) {
        if (current == ancestor) return true;
    }
    return false;
}

bool stationParameterEndpointEquals(const StationParameterEndpoint& a,
                                    const StationParameterEndpoint& b) {
    return a.accessoryId == b.accessoryId && a.parameter == b.parameter;
}

std::string stationParameterEndpointKey(const StationParameterEndpoint& endpoint) {
    return endpoint.accessoryId + "\n" + endpoint.parameter;
}

StationAccessoryInstance* stationAccessoryById(StationDocument& station,
                                               const std::string& id,
                                               size_t* index) {
    for (size_t i = 0; i < station.accessories.size(); ++i) {
        if (station.accessories[i].id != id) continue;
        if (index) *index = i;
        return &station.accessories[i];
    }
    return nullptr;
}

const StationAccessoryInstance* stationAccessoryById(const StationDocument& station,
                                                     const std::string& id,
                                                     size_t* index) {
    for (size_t i = 0; i < station.accessories.size(); ++i) {
        if (station.accessories[i].id != id) continue;
        if (index) *index = i;
        return &station.accessories[i];
    }
    return nullptr;
}

double* stationAccessoryParameter(StationAccessoryInstance& accessory,
                                  const std::string& parameter) {
    TransformNodeData* data = accessory.node ? accessory.node->asTransform() : nullptr;
    if (!data || data->accessoryGenerator != "roller_conveyor") return nullptr;
    if (parameter == "startHeightMm") return &data->accessoryStartHeightMm;
    if (parameter == "endHeightMm") return &data->accessoryEndHeightMm;
    return nullptr;
}

const double* stationAccessoryParameter(const StationAccessoryInstance& accessory,
                                        const std::string& parameter) {
    const TransformNodeData* data = accessory.node ? accessory.node->asTransform() : nullptr;
    if (!data || data->accessoryGenerator != "roller_conveyor") return nullptr;
    if (parameter == "startHeightMm") return &data->accessoryStartHeightMm;
    if (parameter == "endHeightMm") return &data->accessoryEndHeightMm;
    return nullptr;
}

bool stationParameterBounds(const std::string& parameter, double* minimum, double* maximum) {
    if (parameter != "startHeightMm" && parameter != "endHeightMm") return false;
    if (minimum) *minimum = 300.0;
    if (maximum) *maximum = 5000.0;
    return true;
}

// Applies one linked edit as an affine constraint transaction. Every reachable value is first
// represented as m*source+c, the source range is intersected with every member's limits, and only
// then are values written. This makes reverse edits and cycles deterministic and prevents a clamp
// on one conveyor from silently opening the seam on its neighbour.
bool applyStationLinkedParameter(StationDocument& station,
                                 const std::string& accessoryId,
                                 const std::string& parameter,
                                 double requestedValue,
                                 std::vector<size_t>* affectedAccessories,
                                 std::string* errorMessage,
                                 bool rebuildModels) {
    StationParameterEndpoint source;
    source.accessoryId = accessoryId;
    source.parameter = parameter;
    const std::string sourceKey = stationParameterEndpointKey(source);

    struct AffineValue {
        StationParameterEndpoint endpoint;
        double multiplier = 1.0;
        double constant = 0.0;
    };
    std::map<std::string, AffineValue> values;
    values[sourceKey] = {source, 1.0, 0.0};

    bool added = true;
    while (added) {
        added = false;
        for (const StationParameterLink& link : station.parameterLinks) {
            if (!std::isfinite(link.scale) || std::abs(link.scale) < 1.0e-12 ||
                !std::isfinite(link.offset)) {
                if (errorMessage) *errorMessage = "Parameter link has an invalid equation.";
                return false;
            }
            const std::string aKey = stationParameterEndpointKey(link.a);
            const std::string bKey = stationParameterEndpointKey(link.b);
            const auto a = values.find(aKey);
            const auto b = values.find(bKey);
            if (a != values.end() && b == values.end()) {
                values[bKey] = {link.b,
                                link.scale * a->second.multiplier,
                                link.scale * a->second.constant + link.offset};
                added = true;
            } else if (b != values.end() && a == values.end()) {
                values[aKey] = {link.a,
                                b->second.multiplier / link.scale,
                                (b->second.constant - link.offset) / link.scale};
                added = true;
            } else if (a != values.end() && b != values.end()) {
                const double expectedMultiplier = link.scale * a->second.multiplier;
                const double expectedConstant = link.scale * a->second.constant + link.offset;
                if (std::abs(b->second.multiplier - expectedMultiplier) > 1.0e-8 ||
                    std::abs(b->second.constant - expectedConstant) > 1.0e-6) {
                    if (errorMessage) {
                        *errorMessage = "Parameter links contain a contradictory cycle at '" +
                            link.id + "'.";
                    }
                    return false;
                }
            }
        }
    }

    double sourceMinimum = -std::numeric_limits<double>::infinity();
    double sourceMaximum = std::numeric_limits<double>::infinity();
    struct ResolvedValue {
        size_t accessoryIndex = 0;
        StationAccessoryInstance* accessory = nullptr;
        std::string parameter;
        double* address = nullptr;
        double multiplier = 1.0;
        double constant = 0.0;
    };
    std::vector<ResolvedValue> resolved;
    resolved.reserve(values.size());
    for (const auto& item : values) {
        const AffineValue& value = item.second;
        size_t accessoryIndex = 0;
        StationAccessoryInstance* accessory =
            stationAccessoryById(station, value.endpoint.accessoryId, &accessoryIndex);
        double* address = accessory
            ? stationAccessoryParameter(*accessory, value.endpoint.parameter) : nullptr;
        if (!address) {
            if (errorMessage) {
                *errorMessage = "Linked parameter does not resolve: " +
                    value.endpoint.accessoryId + "." + value.endpoint.parameter;
            }
            return false;
        }
        double minimum = 0.0, maximum = 0.0;
        if (!stationParameterBounds(value.endpoint.parameter, &minimum, &maximum) ||
            std::abs(value.multiplier) < 1.0e-12) {
            if (errorMessage) *errorMessage = "Linked parameter has no usable limits.";
            return false;
        }
        double localMinimum = (minimum - value.constant) / value.multiplier;
        double localMaximum = (maximum - value.constant) / value.multiplier;
        if (localMinimum > localMaximum) std::swap(localMinimum, localMaximum);
        sourceMinimum = std::max(sourceMinimum, localMinimum);
        sourceMaximum = std::min(sourceMaximum, localMaximum);
        resolved.push_back({accessoryIndex, accessory, value.endpoint.parameter, address,
                            value.multiplier, value.constant});
    }
    if (sourceMinimum > sourceMaximum) {
        if (errorMessage) *errorMessage = "Linked parameter limits have no common range.";
        return false;
    }

    const double sourceValue = std::max(sourceMinimum, std::min(sourceMaximum, requestedValue));
    std::vector<size_t> dirty;
    for (const ResolvedValue& value : resolved) {
        const double previous = *value.address;
        const double next = value.multiplier * sourceValue + value.constant;
        *value.address = next;
        // start/endHeightMm is the compatibility and linking value for the centerline.  Preserve
        // an existing cross-slope by translating both corresponding corners by the same delta.
        if (value.accessory && value.accessory->node) {
            if (TransformNodeData* data = value.accessory->node->asTransform()) {
                const double delta = next - previous;
                if (value.parameter == "startHeightMm") {
                    data->accessoryStartLeftHeightMm += delta;
                    data->accessoryStartRightHeightMm += delta;
                } else if (value.parameter == "endHeightMm") {
                    data->accessoryEndLeftHeightMm += delta;
                    data->accessoryEndRightHeightMm += delta;
                }
            }
        }
        if (std::find(dirty.begin(), dirty.end(), value.accessoryIndex) == dirty.end()) {
            dirty.push_back(value.accessoryIndex);
        }
    }
    if (rebuildModels) {
        for (size_t index : dirty) {
            StationAccessoryInstance& accessory = station.accessories[index];
            if (rebuildParametricAccessory(accessory.node, station)) {
                if (const TransformNodeData* data = accessory.node->asTransform()) {
                    accessory.name = accessory.node->name;
                    accessory.parameters = data->accessoryParametersJson();
                }
            }
        }
    }
    if (affectedAccessories) *affectedAccessories = dirty;
    if (errorMessage) errorMessage->clear();
    return true;
}

void removeStationParameterLinksForAccessory(StationDocument& station,
                                             const std::string& accessoryId) {
    station.parameterLinks.erase(
        std::remove_if(station.parameterLinks.begin(), station.parameterLinks.end(),
                       [&](const StationParameterLink& link) {
                           return link.a.accessoryId == accessoryId ||
                                  link.b.accessoryId == accessoryId;
                       }),
        station.parameterLinks.end());
}

void removeStationParameterLinksForEndpoint(StationDocument& station,
                                            const StationParameterEndpoint& endpoint) {
    station.parameterLinks.erase(
        std::remove_if(station.parameterLinks.begin(), station.parameterLinks.end(),
                       [&](const StationParameterLink& link) {
                           return stationParameterEndpointEquals(link.a, endpoint) ||
                                  stationParameterEndpointEquals(link.b, endpoint);
                       }),
        station.parameterLinks.end());
}

StationAccessoryInstance* stationAccessoryContainingNode(StationDocument& station,
                                                         const CadNode* node) {
    for (StationAccessoryInstance& accessory : station.accessories) {
        if (accessory.node && libraryNodeIsDescendantOf(node, accessory.node)) return &accessory;
    }
    return nullptr;
}

int createStationParameterLinksForSnap(StationDocument& station,
                                       StationAccessoryInstance& sourceAccessory,
                                       const mountingsnap::Interface& sourceInterface,
                                       CadNode* targetMountingNode) {
    if (sourceInterface.interfaceId.empty() || !targetMountingNode ||
        targetMountingNode->mountingHoles.interfaceId.empty()) return 0;
    StationAccessoryInstance* targetAccessory =
        stationAccessoryContainingNode(station, targetMountingNode);
    if (!targetAccessory || targetAccessory == &sourceAccessory) return 0;

    int created = 0;
    for (const auto& sourceBinding : sourceInterface.parameterBindings) {
        const auto targetBinding =
            targetMountingNode->mountingHoles.parameterBindings.find(sourceBinding.first);
        if (targetBinding == targetMountingNode->mountingHoles.parameterBindings.end()) continue;
        StationParameterEndpoint a{sourceAccessory.id, sourceInterface.interfaceId,
                                   sourceBinding.second};
        StationParameterEndpoint b{targetAccessory->id,
                                   targetMountingNode->mountingHoles.interfaceId,
                                   targetBinding->second};
        double* aValue = stationAccessoryParameter(sourceAccessory, a.parameter);
        double* bValue = stationAccessoryParameter(*targetAccessory, b.parameter);
        if (!aValue || !bValue) continue;
        removeStationParameterLinksForEndpoint(station, a);
        removeStationParameterLinksForEndpoint(station, b);
        StationParameterLink link;
        link.id = a.accessoryId + ":" + a.port + "__" + b.accessoryId + ":" + b.port +
                  "__" + sourceBinding.first;
        link.channel = sourceBinding.first;
        link.a = std::move(a);
        link.b = std::move(b);
        link.scale = 1.0;
        link.offset = *bValue - *aValue;
        station.parameterLinks.push_back(std::move(link));
        ++created;
    }
    return created;
}

std::string stationLinkedParameterDescription(const StationDocument& station,
                                              const std::string& accessoryId,
                                              const std::string& parameter) {
    StationParameterEndpoint endpoint;
    endpoint.accessoryId = accessoryId;
    endpoint.parameter = parameter;
    for (const StationParameterLink& link : station.parameterLinks) {
        const StationParameterEndpoint* other = nullptr;
        if (stationParameterEndpointEquals(link.a, endpoint)) other = &link.b;
        else if (stationParameterEndpointEquals(link.b, endpoint)) other = &link.a;
        if (!other) continue;
        const StationAccessoryInstance* accessory = stationAccessoryById(station,
                                                                          other->accessoryId);
        const std::string name = accessory && !accessory->name.empty()
            ? accessory->name : other->accessoryId;
        return name + " / " + (other->port.empty() ? other->parameter : other->port);
    }
    return {};
}

