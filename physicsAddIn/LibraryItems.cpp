#include "LibraryItems.h"

#include "CadNodePackage.h"
#include "PlacedItemAxes.h"
#include "RoboDkBridge.h"

#include "iitem.h"
#include "irobodk.h"

#include <QDir>
#include <QFileInfo>

const char* const kLibraryItemParam = "PhysicsLibraryItem";

namespace {

Item addGenericItem(RoboDK* rdk, const QString& name) {
    const QList<Item> before = rdk->getItemList(IItem::ITEM_TYPE_GENERIC);
    rdk->Command(QStringLiteral("AddItem"), name);
    for (Item candidate : rdk->getItemList(IItem::ITEM_TYPE_GENERIC)) {
        if (candidate && !before.contains(candidate)) return candidate;
    }
    return nullptr;
}

Item stubLink(RoboDK* rdk, const QString& name) {
    // Three points, one per column: `AddShape` reads a 3xN matrix of vertices and makes one triangle per
    // three of them.
    tMatrix2D* points = Matrix2D_Create();
    Matrix2D_Set_Size(points, 3, 3);
    const double corners[3][3] = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    for (int corner = 0; corner < 3; ++corner) {
        for (int axis = 0; axis < 3; ++axis) {
            Matrix2D_Set_ij(points, axis, corner, corners[corner][axis]);
        }
    }
    Item shape = rdk->AddShape(points);
    Matrix2D_Delete(&points);
    if (shape) shape->setName(name);
    return shape;
}

} // namespace

Item createRailMechanismItem(RoboDK* rdk, const QString& name, double lowerMm, double upperMm,
                             double homeMm, QString* error) {
    const auto fail = [&](const QString& message) -> Item {
        if (error) *error = message;
        return nullptr;
    };
    if (!rdk) return fail(QStringLiteral("No RoboDK."));
    Item base = stubLink(rdk, name + QStringLiteral(" link stub 0"));
    Item moving = stubLink(rdk, name + QStringLiteral(" link stub 1"));
    if (!base || !moving) {
        if (base) base->Delete();
        if (moving) moving->Delete();
        return fail(QStringLiteral("RoboDK would not add the shapes a mechanism is built from."));
    }
    QList<Item> links;
    links << base << moving;
    const tJoints build(&homeMm, 1);
    const tJoints home(&homeMm, 1);
    const double sense = 1.0;
    const tJoints senses(&sense, 1);
    const tJoints low(&lowerMm, 1);
    const tJoints high(&upperMm, 1);
    // No parameters: a 1T needs none, and translates about Z. Which way that Z points in the cell is the
    // base frame's business, and `PlacedItemAxes::railBaseInTree` is what turns it onto the package's own
    // axis of travel.
    constexpr int kRoboDkModel1T = 4;
    Item mechanism = rdk->BuildMechanism(kRoboDkModel1T, links, nullptr, build, home, senses,
                                        low, high, Mat(), Mat(), name);
    // The stubs, gone. RoboDK has copied whatever it needed out of them by now - measured: the mechanism
    // stays valid, drivable and keeps its custom data after both are deleted.
    base->Delete();
    moving->Delete();
    if (!mechanism || !rdk->Valid(mechanism)) {
        return fail(QStringLiteral("RoboDK would not build a linear mechanism for '%1'.").arg(name));
    }
    mechanism->setName(name);
    if (error) error->clear();
    return mechanism;
}

Item railMechanismBaseFrame(RoboDK* rdk, Item mechanism) {
    // The frame `BuildMechanism` makes beside the mechanism and parents it to - `<name> Base`. Where a
    // mechanism *stands* is that frame's pose, not the item's own: a robot item's `Pose` is its flange.
    if (!rdk || !mechanism || !rdk->Valid(mechanism)) return nullptr;
    Item parent = mechanism->Parent();
    if (!parent || !rdk->Valid(parent) || parent->Type() != IItem::ITEM_TYPE_FRAME) return nullptr;
    return parent;
}

bool placedItemIsRailMechanism(RoboDK* rdk, Item item) {
    return rdk && item && rdk->Valid(item) && item->Type() == IItem::ITEM_TYPE_ROBOT;
}

Item createLibraryItem(RoboDK* rdk, const QString& name, QString* error) {
    if (!rdk) return nullptr;
    Item node = addGenericItem(rdk, name);
    if (!node) {
        if (error) {
            *error = QStringLiteral("RoboDK would not add a generic item for '%1'.").arg(name);
        }
        return nullptr;
    }
    // Set last, because `AddItem` is a command with a name in it and this is the API that means it.
    node->setName(name);
    if (error) error->clear();
    return node;
}

bool libraryItemSpecFromItem(RoboDK* rdk, Item item, LibraryItemSpec* out, QString* error) {
    const auto fail = [&](const QString& message) {
        if (error) *error = message;
        return false;
    };
    if (!rdk || !item || !out) return fail(QStringLiteral("No item."));
    QByteArray blob;
    if (!item->getParam(kLibraryItemParam, blob) || blob.trimmed().isEmpty()) {
        return fail(QStringLiteral("'%1' carries no library item description.").arg(item->Name()));
    }
    const Json parameters = Json::parse(blob.constData(), blob.constData() + blob.size(), nullptr,
                                        false);
    if (!parameters.is_object()) {
        return fail(QStringLiteral("'%1' has a library item description that is not JSON.")
                        .arg(item->Name()));
    }

    LibraryItemSpec spec;
    spec.packageRef = QString::fromStdString(jsoncompat::fieldString(parameters, "package"));
    if (spec.packageRef.isEmpty()) {
        return fail(QStringLiteral("'%1' names no package.").arg(item->Name()));
    }
    spec.variantId = QString::fromStdString(jsoncompat::fieldString(parameters, "variant"));
    spec.parameters = jsoncompat::fieldObject(parameters, "parameters");
    // The placement, and the only statement of it there is: the item's own pose is identity, because a
    // generic node answers PoseAbs as identity however its pose is set, and there is no second item
    // standing in the cell. One with no `pose` stands at its parent's origin until something places it.
    spec.hasPose = false;
    if (jsoncompat::contains(parameters, "pose")) {
        const Json& pose = parameters["pose"];
        if (pose.is_array() && pose.size() >= 12) {
            for (int cell = 0; cell < 12; ++cell) {
                spec.poseLocal.values[cell] = pose[static_cast<size_t>(cell)].get<double>();
            }
            spec.hasPose = true;
        }
    }
    if (jsoncompat::contains(parameters, "jointsDeg")) {
        const Json& joints = parameters["jointsDeg"];
        if (joints.is_array() && joints.size() == 6) {
            for (const Json& angle : joints) spec.axes.push_back(angle.get<double>());
        }
    } else if (jsoncompat::contains(parameters, "railMm")) {
        const Json& position = parameters["railMm"];
        if (position.is_number()) spec.axes.push_back(position.get<double>());
    }
    *out = spec;
    if (error) error->clear();
    return true;
}

bool writeLibraryItemSpec(RoboDK* rdk, Item item, const LibraryItemSpec& spec, QByteArray* blob) {
    if (!item || (rdk && !rdk->Valid(item))) return false;
    Json parameters = Json::object();
    parameters["package"] = spec.packageRef.toStdString();
    if (!spec.variantId.isEmpty()) parameters["variant"] = spec.variantId.toStdString();
    if (spec.parameters.is_object() && !spec.parameters.empty()) {
        parameters["parameters"] = spec.parameters;
    }
    // Twelve numbers, row-major, because a placement is rigid and the last row is never anything else.
    Json pose = Json::array();
    for (int cell = 0; cell < 12; ++cell) pose.push_back(spec.poseLocal.values[cell]);
    parameters["pose"] = pose;
    // Written only for a mechanism that has some, so nothing that is not one grows a key - which keeps
    // every existing item's bytes identical and its station undirtied by having been opened.
    if (spec.axes.size() == 6) {
        Json joints = Json::array();
        for (double angle : spec.axes) joints.push_back(angle);
        parameters["jointsDeg"] = joints;
    } else if (spec.axes.size() == 1) {
        parameters["railMm"] = spec.axes.front();
    }

    const std::string text = parameters.dump();
    const QByteArray written(text.data(), static_cast<int>(text.size()));
    if (blob && written == *blob) return false;
    item->setParam(kLibraryItemParam, written);
    if (blob) *blob = written;
    return true;
}

std::shared_ptr<CadNode> loadLibraryItemTree(const LibraryItemSpec& spec, const QString& libraryRoot,
                                             QString* error) {
    QString path = spec.packageRef;
    if (QFileInfo(path).isRelative() || !QFileInfo::exists(path)) {
        const QString beside = QDir(libraryRoot).filePath(QFileInfo(spec.packageRef).fileName());
        if (QFileInfo::exists(beside)) path = beside;
    }
    if (!QFileInfo::exists(path)) {
        if (error) {
            *error = QStringLiteral("Package '%1' is not in the library at '%2'.")
                         .arg(spec.packageRef, libraryRoot);
        }
        return {};
    }

    std::string packageError;
    std::shared_ptr<CadNode> root = loadCadNodePackage(path.toStdString(), &packageError);
    if (!root) {
        if (error) {
            *error = QStringLiteral("Package '%1' could not be read: %2")
                         .arg(path, QString::fromStdString(packageError));
        }
        return {};
    }
    // Apply catalogue presets consistently across both hosts.
    if (spec.parameters.is_object() && !spec.parameters.empty()) {
        if (TransformNodeData* transform = root->asTransform()) {
            transform->applyAccessoryParametersJson(spec.parameters);
        }
    }
    // The item pose owns placement; geometry and mounting interfaces share an identity root.
    root->loc = CadTransform();
    if (error) error->clear();
    return root;
}

void bindPlacedItemAxes(PlacedLibraryItem& placed, QString* warning) {
    if (warning) warning->clear();
    placed.axes.reset();
    if (!placed.scenery.root) return;
    auto axes = std::make_shared<PlacedItemAxes>();
    axes->bind(placed.scenery.root.get());
    if (!axes->drives()) return;
    placed.axes = std::move(axes);
    if (placed.spec.axes.empty()) return;

    QString error;
    if (placed.axes->setValues(placed.spec.axes, &error)) return;
    if (warning) {
        *warning = QStringLiteral("'%1' was saved with axes this package will not take (%2), so it "
                                  "stands at its home instead.")
                       .arg(placed.name, error);
    }
    placed.spec.axes = placed.axes->values();
}
