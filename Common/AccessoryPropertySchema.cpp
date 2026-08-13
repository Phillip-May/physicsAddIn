#include "AccessoryPropertySchema.h"

#include <algorithm>

namespace {

// Shared editor and geometry ranges. Turn angle includes the 720-degree spiral fixture.
const std::vector<AccessoryField>& schema() {
    static const std::vector<AccessoryField> fields = [] {
        std::vector<AccessoryField> table;

        AccessoryField field;
        const auto add = [&table](AccessoryField entry) { table.push_back(std::move(entry)); };

        // ---- Behaviour -------------------------------------------------------------------------
        field = AccessoryField();
        field.key = "simulationMode";
        field.label = "Simulation";
        field.unit = "";
        field.tooltip = "Logical transport carries a workpiece by its pose and costs the solver "
                        "nothing. PhysX gives it a body and lets contact decide where it goes.";
        field.group = AccessoryFieldGroup::Behaviour;
        field.kind = AccessoryFieldKind::Choice;
        field.choices = {{"global", "Global default"}, {"logical", "Logical push/pull"},
                         {"physx", "PhysX"}};
        field.generator = "roller_conveyor";
        field.text = [](TransformNodeData& p) { return &p.accessoryConveyorMode; };
        add(field);

        field = AccessoryField();
        field.key = "role";
        field.label = "Role";
        field.unit = "";
        field.tooltip = "Transport moves workpieces along. A spawner emits them at its entry, a "
                        "deleter consumes them at its far half, and a pick feeder narrows the deck "
                        "into an open pocket a robot can reach into.";
        field.group = AccessoryFieldGroup::Behaviour;
        field.kind = AccessoryFieldKind::Choice;
        field.choices = {{"normal", "Transport"}, {"spawner", "Object spawner"},
                         {"deleter", "Deleter"}, {"pick_feeder", "Robot pick feeder"}};
        field.generator = "roller_conveyor";
        field.text = [](TransformNodeData& p) { return &p.accessoryConveyorRole; };
        add(field);

        field = AccessoryField();
        field.key = "speedMmS";
        field.label = "Speed";
        field.unit = "mm/s";
        field.tooltip = "";
        field.group = AccessoryFieldGroup::Behaviour;
        field.minimum = 0.0;
        field.maximum = 5000.0;
        field.step = 10.0;
        field.generator = "roller_conveyor";
        field.number = [](TransformNodeData& p) { return &p.accessoryConveyorSpeedMmS; };
        add(field);

        field = AccessoryField();
        field.key = "spawnObjectId";
        field.label = "Workpiece";
        field.unit = "";
        field.tooltip = "What this spawner clones. Named rather than inferred from whatever the "
                        "products land on: product geometry is never guessed from the conveyor.";
        field.group = AccessoryFieldGroup::Behaviour;
        field.kind = AccessoryFieldKind::ItemReference;
        field.generator = "roller_conveyor";
        field.role = "spawner";
        field.text = [](TransformNodeData& p) { return &p.accessorySpawnObjectId; };
        add(field);

        field = AccessoryField();
        field.key = "spawnIntervalSeconds";
        field.label = "Spawn interval";
        field.unit = "s";
        field.tooltip = "";
        field.group = AccessoryFieldGroup::Behaviour;
        field.minimum = 0.05;
        field.maximum = 3600.0;
        field.step = 0.1;
        field.decimals = 2;
        field.generator = "roller_conveyor";
        field.role = "spawner";
        field.number = [](TransformNodeData& p) { return &p.accessorySpawnIntervalSeconds; };
        add(field);

        field = AccessoryField();
        field.key = "maxActiveSpawns";
        field.label = "Max active";
        field.unit = "";
        field.tooltip = "How many products from this spawner may be alive anywhere in the cell. "
                        "Products a deleter retires free capacity again. Zero is unlimited.";
        field.group = AccessoryFieldGroup::Behaviour;
        field.kind = AccessoryFieldKind::Integer;
        field.minimum = 0.0;
        field.maximum = 100000.0;
        field.generator = "roller_conveyor";
        field.role = "spawner";
        field.integer = [](TransformNodeData& p) { return &p.accessoryMaxActiveSpawns; };
        add(field);

        // ---- The queue an accumulating conveyor forms ------------------------------------------
        field = AccessoryField();
        field.key = "initialWorkpieceEndInsetMm";
        field.label = "End stop inset";
        field.unit = "mm";
        field.tooltip = "How far short of the path's end a workpiece with nowhere to go comes to "
                        "rest - where the end stop stands. A stated zero means the far end of the "
                        "path is the stop, which is what a lane whose end is a delivery pose needs.";
        field.group = AccessoryFieldGroup::Queue;
        field.minimum = 0.0;
        field.maximum = 2000.0;
        field.step = 5.0;
        field.generator = "roller_conveyor";
        field.number = [](TransformNodeData& p) {
            return &p.accessoryInitialWorkpieceEndInsetMm;
        };
        add(field);

        field = AccessoryField();
        field.key = "initialWorkpieceSpacingMm";
        field.label = "Queue pitch";
        field.unit = "mm";
        field.tooltip = "How far apart accumulated workpieces come to rest. Smaller than the "
                        "workpiece is a queue of boxes standing inside one another, which makes a "
                        "gripper take whichever is nearest rather than the one at the gate.";
        field.group = AccessoryFieldGroup::Queue;
        field.minimum = 0.0;
        field.maximum = 2000.0;
        field.step = 5.0;
        field.generator = "roller_conveyor";
        field.number = [](TransformNodeData& p) {
            return &p.accessoryInitialWorkpieceSpacingMm;
        };
        add(field);

        field = AccessoryField();
        field.key = "initialWorkpieceCount";
        field.label = "Primed workpieces";
        field.unit = "";
        field.tooltip = "How many workpieces the conveyor opens a run already holding, laid out "
                        "backwards from its end stop at the pitch above.";
        field.group = AccessoryFieldGroup::Queue;
        field.kind = AccessoryFieldKind::Integer;
        field.minimum = 0.0;
        field.maximum = 100.0;
        field.generator = "roller_conveyor";
        field.integer = [](TransformNodeData& p) { return &p.accessoryInitialWorkpieceCount; };
        add(field);

        // ---- Dimensions ------------------------------------------------------------------------
        field = AccessoryField();
        field.key = "lengthMm";
        field.label = "Length";
        field.unit = "mm";
        field.tooltip = "How far the lane runs. Where the conveyor stands is its own pose, so this "
                        "and that pose are the whole path.";
        field.group = AccessoryFieldGroup::Dimensions;
        field.minimum = 600.0;
        field.maximum = 6000.0;
        field.step = 10.0;
        field.number = [](TransformNodeData& p) { return &p.accessoryLengthMm; };
        add(field);

        field = AccessoryField();
        field.key = "widthMm";
        field.label = "Width";
        field.unit = "mm";
        field.tooltip = "";
        field.group = AccessoryFieldGroup::Dimensions;
        field.minimum = 300.0;
        field.maximum = 2000.0;
        field.step = 10.0;
        field.number = [](TransformNodeData& p) { return &p.accessoryWidthMm; };
        add(field);

        field = AccessoryField();
        field.key = "startHeightMm";
        field.label = "Start height";
        field.unit = "mm";
        field.tooltip = "The deck's height above the conveyor's own feet at the entry. This and its "
                        "two corners are one thing described twice: moving it carries both corners.";
        field.group = AccessoryFieldGroup::Dimensions;
        field.minimum = 300.0;
        field.maximum = 5000.0;
        field.step = 10.0;
        field.generator = "roller_conveyor";
        field.number = [](TransformNodeData& p) { return &p.accessoryStartHeightMm; };
        field.couple = [](TransformNodeData& p, double previous) {
            const double delta = p.accessoryStartHeightMm - previous;
            p.accessoryStartLeftHeightMm += delta;
            p.accessoryStartRightHeightMm += delta;
        };
        add(field);

        field.key = "endHeightMm";
        field.label = "End height";
        field.tooltip = "The deck's height above the conveyor's own feet at the far end.";
        field.number = [](TransformNodeData& p) { return &p.accessoryEndHeightMm; };
        field.couple = [](TransformNodeData& p, double previous) {
            const double delta = p.accessoryEndHeightMm - previous;
            p.accessoryEndLeftHeightMm += delta;
            p.accessoryEndRightHeightMm += delta;
        };
        add(field);

        field = AccessoryField();
        field.key = "heightMm";
        field.label = "Height";
        field.unit = "mm";
        field.tooltip = "";
        field.group = AccessoryFieldGroup::Dimensions;
        field.minimum = 300.0;
        field.maximum = 5000.0;
        field.step = 10.0;
        field.number = [](TransformNodeData& p) { return &p.accessoryHeightMm; };
        add(field);

        field = AccessoryField();
        field.key = "turnAngleDeg";
        field.label = "Turn angle";
        field.unit = "deg";
        field.tooltip = "A curved deck instead of a straight one. Zero is straight; the sign is which "
                        "way it turns.";
        field.group = AccessoryFieldGroup::Dimensions;
        field.minimum = -1080.0;
        field.maximum = 1080.0;
        field.step = 5.0;
        field.generator = "roller_conveyor";
        field.number = [](TransformNodeData& p) { return &p.accessoryTurnAngleDeg; };
        add(field);

        field = AccessoryField();
        field.key = "curveRadiusMm";
        field.label = "Curve radius";
        field.unit = "mm";
        field.tooltip = "Read only when the turn angle is not zero. Held to at least half the deck's "
                        "width plus a margin, because a curve tighter than the deck is not one.";
        field.group = AccessoryFieldGroup::Dimensions;
        field.minimum = 400.0;
        field.maximum = 5000.0;
        field.step = 25.0;
        field.generator = "roller_conveyor";
        field.number = [](TransformNodeData& p) { return &p.accessoryCurveRadiusMm; };
        add(field);

        field = AccessoryField();
        field.key = "rollerPitchMm";
        field.label = "Roller pitch";
        field.unit = "mm";
        field.tooltip = "";
        field.group = AccessoryFieldGroup::Dimensions;
        field.minimum = 10.0;
        field.maximum = 300.0;
        field.step = 5.0;
        field.generator = "roller_conveyor";
        field.number = [](TransformNodeData& p) { return &p.accessoryRollerPitchMm; };
        add(field);

        field = AccessoryField();
        field.key = "holePitchMm";
        field.label = "Hole pitch";
        field.unit = "mm";
        field.tooltip = "";
        field.group = AccessoryFieldGroup::Dimensions;
        field.minimum = 10.0;
        field.maximum = 500.0;
        field.step = 5.0;
        field.number = [](TransformNodeData& p) { return &p.accessoryHolePitchMm; };
        add(field);

        // ---- The four deck corners -------------------------------------------------------------
        struct Corner {
            const char* key;
            const char* label;
            double* (*member)(TransformNodeData&);
            void (*couple)(TransformNodeData&, double);
        };
        const Corner corners[] = {
            {"startLeftHeightMm", "Start left",
             [](TransformNodeData& p) { return &p.accessoryStartLeftHeightMm; },
             [](TransformNodeData& p, double) {
                 p.accessoryStartHeightMm = 0.5 * (p.accessoryStartLeftHeightMm +
                                                   p.accessoryStartRightHeightMm);
             }},
            {"startRightHeightMm", "Start right",
             [](TransformNodeData& p) { return &p.accessoryStartRightHeightMm; },
             [](TransformNodeData& p, double) {
                 p.accessoryStartHeightMm = 0.5 * (p.accessoryStartLeftHeightMm +
                                                   p.accessoryStartRightHeightMm);
             }},
            {"endLeftHeightMm", "End left",
             [](TransformNodeData& p) { return &p.accessoryEndLeftHeightMm; },
             [](TransformNodeData& p, double) {
                 p.accessoryEndHeightMm = 0.5 * (p.accessoryEndLeftHeightMm +
                                                 p.accessoryEndRightHeightMm);
             }},
            {"endRightHeightMm", "End right",
             [](TransformNodeData& p) { return &p.accessoryEndRightHeightMm; },
             [](TransformNodeData& p, double) {
                 p.accessoryEndHeightMm = 0.5 * (p.accessoryEndLeftHeightMm +
                                                 p.accessoryEndRightHeightMm);
             }},
        };
        for (const Corner& corner : corners) {
            field = AccessoryField();
            field.key = corner.key;
            field.label = corner.label;
            field.unit = "mm";
            field.tooltip = "Left and right are seen looking from the start toward the end. Unequal "
                            "corners tilt the deck, which is how parts accumulate at one pick corner.";
            field.group = AccessoryFieldGroup::Corners;
            field.minimum = 300.0;
            field.maximum = 5000.0;
            field.step = 10.0;
            field.generator = "roller_conveyor";
            field.number = corner.member;
            field.couple = corner.couple;
            add(field);
        }

        // ---- Appearance ------------------------------------------------------------------------
        struct Flag {
            const char* key;
            const char* label;
            const char* tooltip;
            bool* (*member)(TransformNodeData&);
        };
        const Flag flags[] = {
            {"supportBracesEnabled", "Lower side braces",
             "The pair of longitudinal braces below the deck. Spiral decks leave these off and use "
             "perimeter towers instead.",
             [](TransformNodeData& p) { return &p.accessorySupportBracesEnabled; }},
            {"rollerCoverEnabled", "Black roller cover",
             "A belt surface over the rollers. In PhysX mode it is the only powered contact surface, "
             "and it raises the height a workpiece rests at.",
             [](TransformNodeData& p) { return &p.accessoryRollerCoverEnabled; }},
            {"endStopEnabled", "End stop",
             "A barrier at the far end, which also closes that mounting interface. A lane that "
             "accumulates needs one for anything to come to rest against.",
             [](TransformNodeData& p) { return &p.accessoryEndStopEnabled; }},
        };
        for (const Flag& flag : flags) {
            field = AccessoryField();
            field.key = flag.key;
            field.label = flag.label;
            field.unit = "";
            field.tooltip = flag.tooltip;
            field.group = AccessoryFieldGroup::Appearance;
            field.kind = AccessoryFieldKind::Toggle;
            field.generator = "roller_conveyor";
            field.flag = flag.member;
            add(field);
        }

        return table;
    }();
    return fields;
}

} // namespace

const std::vector<AccessoryField>& accessoryPropertySchema() { return schema(); }

const AccessoryField* accessoryField(const char* key) {
    if (!key) return nullptr;
    const std::string wanted(key);
    for (const AccessoryField& field : schema()) {
        if (wanted == field.key) return &field;
    }
    return nullptr;
}

bool accessoryFieldApplies(const AccessoryField& field, const TransformNodeData& parameters) {
    if (field.generator && parameters.accessoryGenerator != field.generator) return false;
    // `heightMm` is the one field a conveyor does not have: its height is the mean of its four deck
    // corners, so offering it would be a second way to say something the corners already say.
    if (parameters.accessoryGenerator == "roller_conveyor" &&
        std::string(field.key) == "heightMm") {
        return false;
    }
    if (parameters.accessoryGenerator == "roller_conveyor" &&
        std::string(field.key) == "holePitchMm") {
        return false;
    }
    if (field.role && parameters.accessoryConveyorRole != field.role) return false;
    return true;
}

double clampAccessoryNumber(const char* key, double value) {
    const AccessoryField* field = accessoryField(key);
    if (!field) return value;
    return std::max(field->minimum, std::min(field->maximum, value));
}

int clampAccessoryInteger(const char* key, int value) {
    const AccessoryField* field = accessoryField(key);
    if (!field) return value;
    return std::max(static_cast<int>(field->minimum),
                    std::min(static_cast<int>(field->maximum), value));
}

bool accessoryChoiceIsValid(const char* key, const std::string& value) {
    const AccessoryField* field = accessoryField(key);
    if (!field) return true;
    for (const AccessoryFieldChoice& choice : field->choices) {
        if (value == choice.value) return true;
    }
    return false;
}
