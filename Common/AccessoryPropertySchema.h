#pragma once

#include "CadNode.h"

#include <string>
#include <vector>

enum class AccessoryFieldKind {
    Number,         // a double, held to [minimum, maximum]
    Integer,        // a count
    Toggle,         // a flag
    Choice,         // one of `choices`, stored as the string it carries
    ItemReference,  // names something else in the cell; each host offers its own candidates
};

struct AccessoryFieldChoice {
    const char* value;  // what the parameter block stores
    const char* label;  // what an operator reads
};

enum class AccessoryFieldGroup { Behaviour, Queue, Dimensions, Corners, Appearance };

struct AccessoryField {
    const char* key;    // the accessory parameter's own JSON key, which is also its stable id
    const char* label;
    const char* unit;   // empty rather than null when there is none
    const char* tooltip;
    AccessoryFieldGroup group = AccessoryFieldGroup::Dimensions;
    AccessoryFieldKind kind = AccessoryFieldKind::Number;
    // Numeric range used by both editors and geometry validation.
    double minimum = 0.0;
    double maximum = 0.0;
    double step = 1.0;
    int decimals = 1;
    std::vector<AccessoryFieldChoice> choices;
    // Null applies to every generator or role.
    const char* generator = nullptr;
    const char* role = nullptr;
    bool editableWhileRunning = false;

    // Exactly one accessor is set, matching kind.
    double* (*number)(TransformNodeData&) = nullptr;
    int* (*integer)(TransformNodeData&) = nullptr;
    bool* (*flag)(TransformNodeData&) = nullptr;
    std::string* (*text)(TransformNodeData&) = nullptr;

    // Applies coupled updates, such as corner and centre heights.
    void (*couple)(TransformNodeData&, double previous) = nullptr;
};

const std::vector<AccessoryField>& accessoryPropertySchema();

const AccessoryField* accessoryField(const char* key);

bool accessoryFieldApplies(const AccessoryField& field, const TransformNodeData& parameters);

double clampAccessoryNumber(const char* key, double value);
int clampAccessoryInteger(const char* key, int value);
bool accessoryChoiceIsValid(const char* key, const std::string& value);
