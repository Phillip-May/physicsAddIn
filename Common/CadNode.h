#ifndef QTCADVIEWER_CADNODE_H
#define QTCADVIEWER_CADNODE_H
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <unordered_set>
#include <functional>
#include "JsonCompat.h"
#include <array>
#include <mutex>
#include <cstdint>
#include <cmath>

#ifdef CADNODE_ENABLE_OCCT
#include <TopLoc_Location.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#endif

struct CadNode;

struct CadTransform {
    std::array<double, 12> values{{1.0, 0.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0}};

    CadTransform() = default;

#ifdef CADNODE_ENABLE_OCCT
    CadTransform(const TopLoc_Location& loc) {
        const gp_Trsf& trsf = loc.Transformation();
        for (int row = 1; row <= 3; ++row) {
            for (int col = 1; col <= 4; ++col) {
                const size_t index = static_cast<size_t>((row - 1) * 4 + (col - 1));
                values[index] = (col <= 3) ? trsf.Value(row, col) : trsf.TranslationPart().GetData()[row - 1];
            }
        }
    }

    CadTransform(const gp_Trsf& trsf) : CadTransform(TopLoc_Location(trsf)) {}

    CadTransform& operator=(const TopLoc_Location& loc) {
        *this = CadTransform(loc);
        return *this;
    }

    CadTransform& operator=(const gp_Trsf& trsf) {
        *this = CadTransform(trsf);
        return *this;
    }

    gp_Trsf Transformation() const {
        gp_Trsf trsf;
        trsf.SetValues(values[0], values[1], values[2], values[3],
                       values[4], values[5], values[6], values[7],
                       values[8], values[9], values[10], values[11]);
        return trsf;
    }

    TopLoc_Location toTopLoc() const {
        return TopLoc_Location(Transformation());
    }

    operator TopLoc_Location() const {
        return toTopLoc();
    }
#endif

    bool IsIdentity() const {
        static const std::array<double, 12> identity{{1.0, 0.0, 0.0, 0.0,
                                                     0.0, 1.0, 0.0, 0.0,
                                                     0.0, 0.0, 1.0, 0.0}};
        return values == identity;
    }

    double value(int row, int column) const {
        return values[static_cast<size_t>(row * 4 + column)];
    }

    CadTransform operator*(const CadTransform& rhs) const {
        CadTransform out;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                double value = (col == 3) ? values[static_cast<size_t>(row * 4 + 3)] : 0.0;
                for (int k = 0; k < 3; ++k) {
                    value += values[static_cast<size_t>(row * 4 + k)] * rhs.values[static_cast<size_t>(k * 4 + col)];
                }
                out.values[static_cast<size_t>(row * 4 + col)] = value;
            }
        }
        return out;
    }

    CadTransform rigidInverse() const {
        CadTransform out;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                out.values[static_cast<size_t>(row * 4 + col)] = values[static_cast<size_t>(col * 4 + row)];
            }
        }
        for (int row = 0; row < 3; ++row) {
            double translation = 0.0;
            for (int k = 0; k < 3; ++k) {
                translation -= values[static_cast<size_t>(k * 4 + row)] * values[static_cast<size_t>(k * 4 + 3)];
            }
            out.values[static_cast<size_t>(row * 4 + 3)] = translation;
        }
        return out;
    }
};

#ifdef CADNODE_ENABLE_OCCT
using CadXcafShapeType = TopAbs_ShapeEnum;
constexpr CadXcafShapeType kCadXcafUnknownShape = TopAbs_SHAPE;
#endif

inline Json topLocToJson(const CadTransform& loc) {
    Json arr = Json::array();
    for (double value : loc.values) arr.push_back(value);
    return arr;
}

inline CadTransform topLocFromJson(const Json& arr) {
    CadTransform transform;
    for (int i = 0; i < arr.size() && i < 12; ++i) {
        transform.values[static_cast<size_t>(i)] = jsoncompat::toDouble(arr[i]);
    }
    return transform;
}

struct CadVec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    CadVec3() = default;
    CadVec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    double X() const { return x; }
    double Y() const { return y; }
    double Z() const { return z; }
};

inline double dot(const CadVec3& a, const CadVec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline CadVec3 cross(const CadVec3& a, const CadVec3& b) {
    return CadVec3(a.y * b.z - a.z * b.y,
                   a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x);
}

inline double lengthOf(const CadVec3& v) { return std::sqrt(dot(v, v)); }

inline CadVec3 scaled(const CadVec3& v, double by) {
    return CadVec3(v.x * by, v.y * by, v.z * by);
}

inline CadVec3 difference(const CadVec3& a, const CadVec3& b) {
    return CadVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline CadVec3 normalized(const CadVec3& v) {
    const double length = lengthOf(v);
    return length < 1.0e-30 ? CadVec3() : scaled(v, 1.0 / length);
}

// Placement applied to a point: rotation plus translation.
inline CadVec3 operator*(const CadTransform& transform, const CadVec3& point) {
    return CadVec3(
        transform.values[0] * point.x + transform.values[1] * point.y +
            transform.values[2] * point.z + transform.values[3],
        transform.values[4] * point.x + transform.values[5] * point.y +
            transform.values[6] * point.z + transform.values[7],
        transform.values[8] * point.x + transform.values[9] * point.y +
            transform.values[10] * point.z + transform.values[11]);
}

// Rotation only, for directions.
inline CadVec3 rotate(const CadTransform& transform, const CadVec3& direction) {
    return CadVec3(
        transform.values[0] * direction.x + transform.values[1] * direction.y +
            transform.values[2] * direction.z,
        transform.values[4] * direction.x + transform.values[5] * direction.y +
            transform.values[6] * direction.z,
        transform.values[8] * direction.x + transform.values[9] * direction.y +
            transform.values[10] * direction.z);
}

template <size_t N>
inline Json stdArrayToJson(const std::array<double, N>& values) {
    Json arr = Json::array();
    for (double value : values) arr.push_back(value);
    return arr;
}

template <size_t N>
inline void stdArrayFromJson(const Json& arr, std::array<double, N>& values) {
    for (int i = 0; i < arr.size() && i < static_cast<int>(N); ++i) {
        values[static_cast<size_t>(i)] = jsoncompat::toDouble(arr[i]);
    }
}

struct CADNodeColor {
    float r, g, b, a;
    CADNodeColor(float r_ = 1.0f, float g_ = 1.0f, float b_ = 1.0f, float a_ = 1.0f) : r(r_), g(g_), b(b_), a(a_) {}
    static CADNodeColor fromSRGB(int r_, int g_, int b_, int a_ = 255) {
        return CADNodeColor(r_ / 255.0f, g_ / 255.0f, b_ / 255.0f, a_ / 255.0f);
    }
    // Conversion to QColor deliberately lives outside this header, in CadNodeQtAdapter.h.
    // It was the only thing that pulled QtGui in, and this header is shared with
    // RobotSimulator, which links no Qt at all.
};

#ifdef CADNODE_ENABLE_OCCT
struct ColoredFace {
    TopoDS_Face face;
    CADNodeColor color;
    TopLoc_Location loc;
};

union CADNodeData {
    TopoDS_Face face;
    TopoDS_Edge edge;
    TopoDS_Shape shape;
    
    CADNodeData() : shape() {}
    
    ~CADNodeData() {}
    
    CADNodeData(const CADNodeData& other) : shape(other.shape) {}
    
    CADNodeData& operator=(const CADNodeData& other) {
        if (this != &other) {
            shape = other.shape;
        }
        return *this;
    }
};
#endif

enum class CadNodeType {
    Unknown = 0, // Default type, type should always be explicitly set
    XCAF,
    Custom, //For components of other CadNodes
    Rail,
    Turnable,
    Physics, // New: Physics object node
    ConnectionPoint, // New: Connection point node
    Transform, // New: Transform node type
    MutexRoot, // New: Mutex root node type
    OPW6Robot,
    RobotLink,
    RobotTool,
    MeshGeometry,
    GantryMechanism,
    DragChainMechanism,
};

struct CadNodeDataBase {
    virtual ~CadNodeDataBase() = default;
    virtual Json toJson() const { return Json::object(); }
    static std::shared_ptr<CadNodeDataBase> fromJson(const Json&) { return nullptr; }
};

#ifdef CADNODE_ENABLE_OCCT
struct XCAFNodeData : public CadNodeDataBase {
    CadXcafShapeType type = kCadXcafUnknownShape;
    std::vector<int> labelPath; // Path from root to label
    int shapeIndex = -1;        // Index of face/edge under the label
    TopoDS_Shape shape;
    TopoDS_Shape originalXCAFShape; // Not serialized, used for relinking
    bool hasFace() const { return type == TopAbs_FACE && !shape.IsNull(); }
    bool hasEdge() const { return type == TopAbs_EDGE && !shape.IsNull(); }
    TopoDS_Face getFace() const { return (type == TopAbs_FACE) ? TopoDS::Face(shape) : TopoDS_Face(); }
    TopoDS_Edge getEdge() const { return (type == TopAbs_EDGE) ? TopoDS::Edge(shape) : TopoDS_Edge(); }
};
#endif

struct CustomNodeData : public CadNodeDataBase {
    int customProperty = 0;

    Json toJson() const override {
        Json obj = Json::object();
        obj["customProperty"] = customProperty;
        return obj;
    }
    static std::shared_ptr<CustomNodeData> fromJson(const Json& obj) {
        auto data = std::make_shared<CustomNodeData>();
        data->customProperty = jsoncompat::fieldInt(obj, "customProperty");
        return data;
    }
};

struct ConvexHullData {
    std::vector<std::array<double, 3>> vertices; // x, y, z
    std::vector<std::array<uint32_t, 3>> indices; // triangle indices

    Json toJson() const {
        Json hullObj = Json::object();
        Json vertsArr = Json::array();
        for (const auto& v : vertices) {
            Json vArr = Json::array();
            vArr.push_back(v[0]);
            vArr.push_back(v[1]);
            vArr.push_back(v[2]);
            vertsArr.push_back(vArr);
        }
        hullObj["vertices"] = vertsArr;

        Json indsArr = Json::array();
        for (const auto& tri : indices) {
            Json triArr = Json::array();
            triArr.push_back(static_cast<int64_t>(tri[0]));
            triArr.push_back(static_cast<int64_t>(tri[1]));
            triArr.push_back(static_cast<int64_t>(tri[2]));
            indsArr.push_back(triArr);
        }
        hullObj["indices"] = indsArr;
        return hullObj;
    }

    static ConvexHullData fromJson(const Json& hullObj) {
        ConvexHullData hull;
        Json vertsArr = jsoncompat::fieldArray(hullObj, "vertices");
        for (const auto& vVal : vertsArr) {
            Json vArr = jsoncompat::toArray(vVal);
            if (vArr.size() == 3) {
                hull.vertices.push_back({jsoncompat::toDouble(vArr[0]), jsoncompat::toDouble(vArr[1]), jsoncompat::toDouble(vArr[2])});
            }
        }

        Json indsArr = jsoncompat::fieldArray(hullObj, "indices");
        for (const auto& triVal : indsArr) {
            Json triArr = jsoncompat::toArray(triVal);
            if (triArr.size() == 3) {
                hull.indices.push_back({
                    static_cast<uint32_t>(jsoncompat::toInt(triArr[0])),
                    static_cast<uint32_t>(jsoncompat::toInt(triArr[1])),
                    static_cast<uint32_t>(jsoncompat::toInt(triArr[2]))
                });
            }
        }
        return hull;
    }
};

// Snap locations for fixture plates and other models with mounting holes. Irregular holes remain
// explicit, while a rectangular lattice is represented by one origin, two step vectors and two
// counts so a large tooling plate does not serialize thousands of repeated coordinates.
struct MountingHoleGridData {
    CadVec3 originMm;
    CadVec3 uStepMm;
    CadVec3 vStepMm;
    uint32_t uCount = 0;
    uint32_t vCount = 0;

    Json toJson() const {
        Json obj = Json::object();
        obj["originMm"] = Json::array({originMm.x, originMm.y, originMm.z});
        obj["uStepMm"] = Json::array({uStepMm.x, uStepMm.y, uStepMm.z});
        obj["vStepMm"] = Json::array({vStepMm.x, vStepMm.y, vStepMm.z});
        obj["uCount"] = static_cast<int64_t>(uCount);
        obj["vCount"] = static_cast<int64_t>(vCount);
        return obj;
    }

    static MountingHoleGridData fromJson(const Json& obj) {
        MountingHoleGridData grid;
        const auto readVec = [&](const char* key, CadVec3& value) {
            const Json arr = jsoncompat::fieldArray(obj, key);
            if (arr.size() == 3) {
                value = CadVec3(jsoncompat::toDouble(arr[0]), jsoncompat::toDouble(arr[1]),
                                jsoncompat::toDouble(arr[2]));
            }
        };
        readVec("originMm", grid.originMm);
        readVec("uStepMm", grid.uStepMm);
        readVec("vStepMm", grid.vStepMm);
        const int uCount = jsoncompat::fieldInt(obj, "uCount");
        const int vCount = jsoncompat::fieldInt(obj, "vCount");
        if (uCount > 0) grid.uCount = static_cast<uint32_t>(uCount);
        if (vCount > 0) grid.vCount = static_cast<uint32_t>(vCount);
        return grid;
    }
};

struct MountingHoleData {
    std::vector<CadVec3> pointsMm;
    std::vector<MountingHoleGridData> grids;
    // Optional visualization-only offset from the mating/snap plane to the visible hole opening.
    // Snapping continues to use pointsMm and grids unchanged.
    CadVec3 markerOffsetMm;
    // A robot can expose accessory and tool interfaces in addition to its floor pattern. Only a
    // flagged interface participates when the whole robot is dragged in from the library.
    bool placementSource = false;
    // End-to-end interfaces have outward-facing normals. Mating them aligns those normals in
    // opposite directions; horizontal feet/fixture holes leave this false and align normals.
    bool mateOpposite = false;
    std::string interfaceId;
    std::map<std::string, std::string> parameterBindings;
    std::string comment;

    bool empty() const { return pointsMm.empty() && grids.empty(); }

    Json toJson() const {
        Json obj = Json::object();
        if (!pointsMm.empty()) {
            Json points = Json::array();
            for (const CadVec3& point : pointsMm) {
                points.push_back(Json::array({point.x, point.y, point.z}));
            }
            obj["pointsMm"] = points;
        }
        if (!grids.empty()) {
            Json gridArray = Json::array();
            for (const MountingHoleGridData& grid : grids) gridArray.push_back(grid.toJson());
            obj["grids"] = gridArray;
        }
        if (markerOffsetMm.x != 0.0 || markerOffsetMm.y != 0.0 || markerOffsetMm.z != 0.0) {
            obj["markerOffsetMm"] = Json::array(
                {markerOffsetMm.x, markerOffsetMm.y, markerOffsetMm.z});
        }
        if (placementSource) obj["placementSource"] = true;
        if (mateOpposite) obj["mateOpposite"] = true;
        if (!interfaceId.empty()) obj["interfaceId"] = interfaceId;
        if (!parameterBindings.empty()) {
            Json bindings = Json::object();
            for (const auto& binding : parameterBindings) {
                bindings[binding.first] = binding.second;
            }
            obj["parameterBindings"] = bindings;
        }
        if (!comment.empty()) obj["comment"] = comment;
        return obj;
    }

    static MountingHoleData fromJson(const Json& obj) {
        MountingHoleData holes;
        const Json points = jsoncompat::fieldArray(obj, "pointsMm");
        for (const Json& pointValue : points) {
            const Json point = jsoncompat::toArray(pointValue);
            if (point.size() == 3) {
                holes.pointsMm.emplace_back(jsoncompat::toDouble(point[0]),
                                            jsoncompat::toDouble(point[1]),
                                            jsoncompat::toDouble(point[2]));
            }
        }
        const Json grids = jsoncompat::fieldArray(obj, "grids");
        for (const Json& gridValue : grids) {
            if (!gridValue.is_object()) continue;
            MountingHoleGridData grid =
                MountingHoleGridData::fromJson(jsoncompat::toObject(gridValue));
            if (grid.uCount > 0 && grid.vCount > 0) holes.grids.push_back(grid);
        }
        const Json markerOffset = jsoncompat::fieldArray(obj, "markerOffsetMm");
        if (markerOffset.size() == 3) {
            holes.markerOffsetMm = CadVec3(jsoncompat::toDouble(markerOffset[0]),
                                               jsoncompat::toDouble(markerOffset[1]),
                                               jsoncompat::toDouble(markerOffset[2]));
        }
        holes.placementSource = jsoncompat::fieldBool(obj, "placementSource", false);
        holes.mateOpposite = jsoncompat::fieldBool(obj, "mateOpposite", false);
        holes.interfaceId = jsoncompat::fieldString(obj, "interfaceId");
        const Json& bindings = jsoncompat::fieldObject(obj, "parameterBindings");
        for (auto it = bindings.begin(); it != bindings.end(); ++it) {
            if (it.value().is_string()) {
                holes.parameterBindings[it.key()] = it.value().get<std::string>();
            }
        }
        holes.comment = jsoncompat::fieldString(obj, "comment");
        return holes;
    }
};

struct PhysicsNodeData : public CadNodeDataBase {
    bool convexHullGenerated = false;
    std::vector<ConvexHullData> hulls;
    bool collisionMeshVisible = true;
    bool isPhysicsActive = false;

    float mass = 100.0f;
    float staticFriction = 0.8f;
    float dynamicFriction = 0.6f;
    float restitution = 0.1f;
    CadVec3 centerOfMass;
    std::string materialName;
    bool useCustomProperties = false;
    bool useCustomCenterOfMass = false;

    Json toJson() const override {
        Json obj = Json::object();
        obj["convexHullGenerated"] = convexHullGenerated;
        obj["collisionMeshVisible"] = collisionMeshVisible;
        // Serialize hulls
        Json hullsArr = Json::array();
        for (const auto& hull : hulls) {
            hullsArr.push_back(hull.toJson());
        }
        obj["hulls"] = hullsArr;
        obj["mass"] = mass;
        obj["staticFriction"] = staticFriction;
        obj["dynamicFriction"] = dynamicFriction;
        obj["restitution"] = restitution;
        Json comArr = Json::array({centerOfMass.X(), centerOfMass.Y(), centerOfMass.Z()});
        obj["centerOfMass"] = comArr;
        obj["materialName"] = materialName;
        obj["useCustomProperties"] = useCustomProperties;
        obj["useCustomCenterOfMass"] = useCustomCenterOfMass;
        return obj;
    }
    static std::shared_ptr<PhysicsNodeData> fromJson(const Json& obj) {
        auto data = std::make_shared<PhysicsNodeData>();
        data->convexHullGenerated = jsoncompat::fieldBool(obj, "convexHullGenerated");
        data->collisionMeshVisible = jsoncompat::fieldBool(obj, "collisionMeshVisible", true);
        // Deserialize hulls
        Json hullsArr = jsoncompat::fieldArray(obj, "hulls");
        for (const auto& hullVal : hullsArr) {
            data->hulls.push_back(ConvexHullData::fromJson(jsoncompat::toObject(hullVal)));
        }
        data->mass = static_cast<float>(jsoncompat::fieldDouble(obj, "mass", 100.0));
        data->staticFriction = static_cast<float>(jsoncompat::fieldDouble(obj, "staticFriction", 0.8));
        data->dynamicFriction = static_cast<float>(jsoncompat::fieldDouble(obj, "dynamicFriction", 0.6));
        data->restitution = static_cast<float>(jsoncompat::fieldDouble(obj, "restitution", 0.1));
        Json comArr = jsoncompat::fieldArray(obj, "centerOfMass");
        if (comArr.size() == 3) data->centerOfMass = CadVec3(jsoncompat::toDouble(comArr[0]), jsoncompat::toDouble(comArr[1]), jsoncompat::toDouble(comArr[2]));
        data->materialName = jsoncompat::fieldString(obj, "materialName");
        data->useCustomProperties = jsoncompat::fieldBool(obj, "useCustomProperties", false);
        data->useCustomCenterOfMass = jsoncompat::fieldBool(obj, "useCustomCenterOfMass", false);
        return data;
    }
};

struct RailNodeData : public CadNodeDataBase {
    CadVec3 axisOfTravel{1, 0, 0};
    CadVec3 buildJointPosition{0, 0, 0};
    double travelLength = 0.0;
    int numSegments = 10;

    Json toJson() const override {
        Json obj = Json::object();
        Json axisArr = Json::array({axisOfTravel.x, axisOfTravel.y, axisOfTravel.z});
        Json jointArr = Json::array({buildJointPosition.x, buildJointPosition.y, buildJointPosition.z});
        obj["axisOfTravel"] = axisArr;
        obj["buildJointPosition"] = jointArr;
        obj["travelLength"] = travelLength;
        obj["numSegments"] = numSegments;
        return obj;
    }
    static std::shared_ptr<RailNodeData> fromJson(const Json& obj) {
        auto data = std::make_shared<RailNodeData>();
        Json axisArr = jsoncompat::fieldArray(obj, "axisOfTravel");
        if (axisArr.size() == 3) data->axisOfTravel = CadVec3(jsoncompat::toDouble(axisArr[0]), jsoncompat::toDouble(axisArr[1]), jsoncompat::toDouble(axisArr[2]));
        Json jointArr = jsoncompat::fieldArray(obj, "buildJointPosition");
        if (jointArr.size() == 3) data->buildJointPosition = CadVec3(jsoncompat::toDouble(jointArr[0]), jsoncompat::toDouble(jointArr[1]), jsoncompat::toDouble(jointArr[2]));
        data->travelLength = jsoncompat::fieldDouble(obj, "travelLength");
        data->numSegments = jsoncompat::fieldInt(obj, "numSegments");
        return data;
    }
};

// A kinematic linear axis carrying an arbitrary subtree. Unlike RailNodeData, which describes the
// old CAD rail tiler, this is a live mechanism: movingFrame is translated along axisOfTravel and
// every child below that frame (normally a robot) follows through ordinary scene-tree transforms.
struct GantryMechanismData : public CadNodeDataBase {
    static constexpr size_t kMaxBaseCollisionHulls = 128;

    CadVec3 axisOfTravel{1, 0, 0};
    double positionMm = 0.0;
    double homePositionMm = 0.0;
    double lowerLimitMm = 0.0;
    double upperLimitMm = 0.0;
    double velocityMaxMmS = 0.0;
    // Offline-baked, gantry-local convex decomposition of the stationary base. Keeping these in
    // the package avoids running decomposition when a station is opened.
    std::vector<ConvexHullData> baseCollisionHulls;
    Json baseCollisionHullBake = Json::object();

    // Resolved after the complete tree has loaded. The serialized path is from the package root,
    // following the same convention as robot geometryPaths and activeToolPath.
    CadNode* movingFrame = nullptr;
    Json movingFramePath = Json::array();
    bool positionApplied = false; // Runtime-only; keeps controller rebinding idempotent.

    Json toJson() const override {
        Json obj = Json::object();
        obj["axisOfTravel"] = Json::array({axisOfTravel.x, axisOfTravel.y, axisOfTravel.z});
        obj["positionMm"] = positionMm;
        obj["homePositionMm"] = homePositionMm;
        obj["lowerLimitMm"] = lowerLimitMm;
        obj["upperLimitMm"] = upperLimitMm;
        if (velocityMaxMmS > 0.0) obj["velocityMaxMmS"] = velocityMaxMmS;
        Json hulls = Json::array();
        for (const ConvexHullData& hull : baseCollisionHulls) hulls.push_back(hull.toJson());
        if (!hulls.empty()) obj["baseCollisionHulls"] = hulls;
        if (!baseCollisionHullBake.empty()) obj["baseCollisionHullBake"] = baseCollisionHullBake;
        obj["movingFramePath"] = movingFramePath;
        return obj;
    }

    static std::shared_ptr<GantryMechanismData> fromJson(const Json& obj) {
        auto data = std::make_shared<GantryMechanismData>();
        const Json axis = jsoncompat::fieldArray(obj, "axisOfTravel");
        if (axis.size() == 3) {
            data->axisOfTravel = CadVec3(jsoncompat::toDouble(axis[0]),
                                         jsoncompat::toDouble(axis[1]),
                                         jsoncompat::toDouble(axis[2]));
        }
        data->positionMm = jsoncompat::fieldDouble(obj, "positionMm");
        data->homePositionMm = jsoncompat::fieldDouble(obj, "homePositionMm");
        data->lowerLimitMm = jsoncompat::fieldDouble(obj, "lowerLimitMm");
        data->upperLimitMm = jsoncompat::fieldDouble(obj, "upperLimitMm");
        data->velocityMaxMmS = jsoncompat::fieldDouble(obj, "velocityMaxMmS");
        const Json hulls = jsoncompat::fieldArray(obj, "baseCollisionHulls");
        for (const auto& hullValue : hulls) {
            if (hullValue.is_object()) {
                data->baseCollisionHulls.push_back(
                    ConvexHullData::fromJson(jsoncompat::toObject(hullValue)));
            }
        }
        data->baseCollisionHullBake = jsoncompat::fieldObject(obj, "baseCollisionHullBake");
        data->movingFramePath = jsoncompat::fieldArray(obj, "movingFramePath");
        return data;
    }
};

// A cable carrier rendered and simulated as instances of one canonical member. The package owns
// exactly one MeshGeometry node; linkFrames carry the repeated poses without duplicating either
// the mesh payload, the decoded vertices, or the renderer's GPU buffer.
struct DragChainMechanismData : public CadNodeDataBase {
    CadVec3 fixedAnchorMm{0, 0, 0};
    CadVec3 movingAnchorMm{0, 0, 0}; // In movingFrame coordinates at every gantry position.
    CadVec3 travelAxis{1, 0, 0};
    CadVec3 hingeAxis{0, 0, -1};
    double departureAxisSign = 0.0; // Optional fixed departure direction along travelAxis.
    double pitchMm = 110.26;
    double bendRadiusMm = 200.0;
    double linkMassKg = 1.0;
    double maxJointRotationDeg = 35.0;
    // Large supported carriers can use the deterministic contour when a long free-body solve is
    // undesirable. Existing packages default to the PhysX revolute-chain backend.
    bool physicsEnabled = true;
    // Present the same local hinge to both terminal connectors. The final member is placed from
    // the fixed anchor back toward the chain, so its local-zero hinge meets the fixed connector.
    bool reverseFixedEndMember = false;
    // Visual/body offset from the fixed hinge line. Joint frames subtract this offset, allowing a
    // terminal member to sit on the matching connector face without changing the chain contour.
    CadVec3 fixedEndMemberOffsetMm{0, 0, 0};

    CadNode* movingFrame = nullptr;
    CadNode* prototypeGeometry = nullptr;
    std::vector<CadNode*> linkFrames; // Ordered moving end to fixed end.
    Json movingFramePath = Json::array();
    Json prototypeGeometryPath = Json::array();
    Json linkFramePaths = Json::array();

    Json toJson() const override {
        Json obj = Json::object();
        obj["fixedAnchorMm"] = Json::array({fixedAnchorMm.x, fixedAnchorMm.y, fixedAnchorMm.z});
        obj["movingAnchorMm"] = Json::array({movingAnchorMm.x, movingAnchorMm.y, movingAnchorMm.z});
        obj["travelAxis"] = Json::array({travelAxis.x, travelAxis.y, travelAxis.z});
        obj["hingeAxis"] = Json::array({hingeAxis.x, hingeAxis.y, hingeAxis.z});
        obj["departureAxisSign"] = departureAxisSign;
        obj["pitchMm"] = pitchMm;
        obj["bendRadiusMm"] = bendRadiusMm;
        obj["linkMassKg"] = linkMassKg;
        obj["maxJointRotationDeg"] = maxJointRotationDeg;
        obj["physicsEnabled"] = physicsEnabled;
        obj["reverseFixedEndMember"] = reverseFixedEndMember;
        obj["fixedEndMemberOffsetMm"] = Json::array({fixedEndMemberOffsetMm.x,
                                                      fixedEndMemberOffsetMm.y,
                                                      fixedEndMemberOffsetMm.z});
        obj["movingFramePath"] = movingFramePath;
        obj["prototypeGeometryPath"] = prototypeGeometryPath;
        obj["linkFramePaths"] = linkFramePaths;
        return obj;
    }

    static std::shared_ptr<DragChainMechanismData> fromJson(const Json& obj) {
        auto data = std::make_shared<DragChainMechanismData>();
        const auto readVec = [&](const char* key, CadVec3& value) {
            const Json arr = jsoncompat::fieldArray(obj, key);
            if (arr.size() == 3) value = CadVec3(jsoncompat::toDouble(arr[0]),
                                                 jsoncompat::toDouble(arr[1]),
                                                 jsoncompat::toDouble(arr[2]));
        };
        readVec("fixedAnchorMm", data->fixedAnchorMm);
        readVec("movingAnchorMm", data->movingAnchorMm);
        readVec("travelAxis", data->travelAxis);
        readVec("hingeAxis", data->hingeAxis);
        data->departureAxisSign = jsoncompat::fieldDouble(obj, "departureAxisSign");
        data->pitchMm = jsoncompat::fieldDouble(obj, "pitchMm", 110.26);
        data->bendRadiusMm = jsoncompat::fieldDouble(obj, "bendRadiusMm", 200.0);
        data->linkMassKg = jsoncompat::fieldDouble(obj, "linkMassKg", 1.0);
        data->maxJointRotationDeg = jsoncompat::fieldDouble(obj, "maxJointRotationDeg", 35.0);
        data->physicsEnabled = jsoncompat::fieldBool(obj, "physicsEnabled", true);
        data->reverseFixedEndMember = jsoncompat::fieldBool(obj, "reverseFixedEndMember", false);
        readVec("fixedEndMemberOffsetMm", data->fixedEndMemberOffsetMm);
        data->movingFramePath = jsoncompat::fieldArray(obj, "movingFramePath");
        data->prototypeGeometryPath = jsoncompat::fieldArray(obj, "prototypeGeometryPath");
        data->linkFramePaths = jsoncompat::fieldArray(obj, "linkFramePaths");
        return data;
    }
};

struct TransformNodeData : public CadNodeDataBase {
    bool isRobotBaseFrame = false;

    // Optional recipe for accessories whose geometry can be edited in the station properties
    // panel. The package supplies the defaults; a station may override the four dimensions per
    // instance without embedding another copy of the mesh package.
    std::string accessoryGenerator;
    double accessoryWidthMm = 0.0;
    double accessoryLengthMm = 0.0;
    double accessoryHeightMm = 0.0;
    double accessoryHolePitchMm = 0.0;
    double accessoryRollerPitchMm = 100.0;
    double accessoryStartHeightMm = 0.0;
    double accessoryEndHeightMm = 0.0;
    // Absolute deck heights at the four plan-view corners. Zero inherits the corresponding
    // centerline height. Left/right are viewed from start toward end.
    double accessoryStartLeftHeightMm = 0.0;
    double accessoryStartRightHeightMm = 0.0;
    double accessoryEndLeftHeightMm = 0.0;
    double accessoryEndRightHeightMm = 0.0;
    double accessoryTurnAngleDeg = 0.0;
    double accessoryCurveRadiusMm = 0.0;
    bool accessorySupportBracesEnabled = true;
    bool accessoryRollerCoverEnabled = false;
    bool accessoryEndStopEnabled = false;
    // Conveyor behavior is stored with the same per-instance recipe as its geometry. "global"
    // inherits the station default; the other supported values are "logical" and "physx".
    std::string accessoryConveyorMode = "global";
    // "normal" transports workpieces, "spawner" emits them at the start interface, "deleter"
    // consumes them, and "pick_feeder" narrows the shared transport into an open robot pocket.
    std::string accessoryConveyorRole = "normal";
    double accessoryConveyorSpeedMmS = 250.0;
    double accessorySpawnIntervalSeconds = 1.0;
    // Maximum live products originating from this spawner. Zero means unlimited.
    int accessoryMaxActiveSpawns = 0;
    // Stable id of a hidden station accessory used as the workpiece prototype. An empty or stale
    // id leaves the spawner unconfigured; product geometry is never inferred from the conveyor.
    std::string accessorySpawnObjectId;
    int accessoryInitialWorkpieceCount = 0;
    double accessoryInitialWorkpieceEndInsetMm = 70.0;
    double accessoryInitialWorkpieceSpacingMm = 90.0;

    bool hasParametricAccessory() const { return !accessoryGenerator.empty(); }

    Json accessoryParametersJson() const {
        Json parameters = Json::object();
        if (!hasParametricAccessory()) return parameters;
        parameters["generator"] = accessoryGenerator;
        parameters["widthMm"] = accessoryWidthMm;
        parameters["lengthMm"] = accessoryLengthMm;
        parameters["heightMm"] = accessoryHeightMm;
        if (accessoryGenerator == "roller_conveyor") {
            parameters["rollerPitchMm"] = accessoryRollerPitchMm;
            parameters["startHeightMm"] = accessoryStartHeightMm;
            parameters["endHeightMm"] = accessoryEndHeightMm;
            parameters["startLeftHeightMm"] = accessoryStartLeftHeightMm;
            parameters["startRightHeightMm"] = accessoryStartRightHeightMm;
            parameters["endLeftHeightMm"] = accessoryEndLeftHeightMm;
            parameters["endRightHeightMm"] = accessoryEndRightHeightMm;
            parameters["turnAngleDeg"] = accessoryTurnAngleDeg;
            parameters["curveRadiusMm"] = accessoryCurveRadiusMm;
            parameters["supportBracesEnabled"] = accessorySupportBracesEnabled;
            parameters["rollerCoverEnabled"] = accessoryRollerCoverEnabled;
            parameters["endStopEnabled"] = accessoryEndStopEnabled;
            parameters["simulationMode"] = accessoryConveyorMode;
            parameters["role"] = accessoryConveyorRole;
            parameters["speedMmS"] = accessoryConveyorSpeedMmS;
            parameters["spawnIntervalSeconds"] = accessorySpawnIntervalSeconds;
            parameters["maxActiveSpawns"] = accessoryMaxActiveSpawns;
            if (!accessorySpawnObjectId.empty()) {
                parameters["spawnObjectId"] = accessorySpawnObjectId;
            }
            if (accessoryInitialWorkpieceCount > 0) {
                parameters["initialWorkpieceCount"] = accessoryInitialWorkpieceCount;
            }
            // Always, and not only when there are initial workpieces to lay out: these two are also
            // the *queue* an accumulating conveyor forms - conveyorSpecFrom reads endInsetMm and
            // productPitchMm off them - so a conveyor that writes itself out and reads itself back
            // would otherwise silently take the 70/90 defaults. For a lane whose far end is already
            // the delivery point that moves every product 70 mm short of the pick.
            parameters["initialWorkpieceEndInsetMm"] = accessoryInitialWorkpieceEndInsetMm;
            parameters["initialWorkpieceSpacingMm"] = accessoryInitialWorkpieceSpacingMm;
        } else {
            parameters["holePitchMm"] = accessoryHolePitchMm;
        }
        return parameters;
    }

    void applyAccessoryParametersJson(const Json& parameters) {
        if (!parameters.is_object()) return;
        accessoryGenerator = jsoncompat::fieldString(parameters, "generator", accessoryGenerator);
        accessoryWidthMm = jsoncompat::fieldDouble(parameters, "widthMm", accessoryWidthMm);
        accessoryLengthMm = jsoncompat::fieldDouble(parameters, "lengthMm", accessoryLengthMm);
        accessoryHeightMm = jsoncompat::fieldDouble(parameters, "heightMm", accessoryHeightMm);
        accessoryHolePitchMm =
            jsoncompat::fieldDouble(parameters, "holePitchMm", accessoryHolePitchMm);
        accessoryRollerPitchMm =
            jsoncompat::fieldDouble(parameters, "rollerPitchMm", accessoryRollerPitchMm);
        if (accessoryGenerator == "roller_conveyor") {
            const bool overridesStartHeight = parameters.contains("startHeightMm");
            const bool overridesEndHeight = parameters.contains("endHeightMm");
            const bool overridesStartLeft = parameters.contains("startLeftHeightMm");
            const bool overridesStartRight = parameters.contains("startRightHeightMm");
            const bool overridesEndLeft = parameters.contains("endLeftHeightMm");
            const bool overridesEndRight = parameters.contains("endRightHeightMm");
            const double legacyHeight = accessoryHeightMm;
            accessoryStartHeightMm = jsoncompat::fieldDouble(
                parameters, "startHeightMm",
                accessoryStartHeightMm > 0.0 ? accessoryStartHeightMm : legacyHeight);
            accessoryEndHeightMm = jsoncompat::fieldDouble(
                parameters, "endHeightMm",
                accessoryEndHeightMm > 0.0 ? accessoryEndHeightMm : legacyHeight);
    // A scalar endpoint-height override applies to both corners unless it also supplies a
    // corner-specific value.
            accessoryStartLeftHeightMm = overridesStartLeft
                ? jsoncompat::fieldDouble(parameters, "startLeftHeightMm",
                                          accessoryStartLeftHeightMm)
                : (overridesStartHeight ? accessoryStartHeightMm : accessoryStartLeftHeightMm);
            accessoryStartRightHeightMm = overridesStartRight
                ? jsoncompat::fieldDouble(parameters, "startRightHeightMm",
                                          accessoryStartRightHeightMm)
                : (overridesStartHeight ? accessoryStartHeightMm : accessoryStartRightHeightMm);
            accessoryEndLeftHeightMm = overridesEndLeft
                ? jsoncompat::fieldDouble(parameters, "endLeftHeightMm",
                                          accessoryEndLeftHeightMm)
                : (overridesEndHeight ? accessoryEndHeightMm : accessoryEndLeftHeightMm);
            accessoryEndRightHeightMm = overridesEndRight
                ? jsoncompat::fieldDouble(parameters, "endRightHeightMm",
                                          accessoryEndRightHeightMm)
                : (overridesEndHeight ? accessoryEndHeightMm : accessoryEndRightHeightMm);
            accessoryTurnAngleDeg = jsoncompat::fieldDouble(
                parameters, "turnAngleDeg", accessoryTurnAngleDeg);
            accessoryCurveRadiusMm = jsoncompat::fieldDouble(
                parameters, "curveRadiusMm", accessoryCurveRadiusMm);
            accessorySupportBracesEnabled = jsoncompat::fieldBool(
                parameters, "supportBracesEnabled", accessorySupportBracesEnabled);
            accessoryRollerCoverEnabled = jsoncompat::fieldBool(
                parameters, "rollerCoverEnabled", accessoryRollerCoverEnabled);
            accessoryEndStopEnabled = jsoncompat::fieldBool(
                parameters, "endStopEnabled", accessoryEndStopEnabled);
            accessoryConveyorMode = jsoncompat::fieldString(
                parameters, "simulationMode", accessoryConveyorMode);
            accessoryConveyorRole = jsoncompat::fieldString(
                parameters, "role", accessoryConveyorRole);
            accessoryConveyorSpeedMmS = jsoncompat::fieldDouble(
                parameters, "speedMmS", accessoryConveyorSpeedMmS);
            accessorySpawnIntervalSeconds = jsoncompat::fieldDouble(
                parameters, "spawnIntervalSeconds", accessorySpawnIntervalSeconds);
            accessoryMaxActiveSpawns = jsoncompat::fieldInt(
                parameters, "maxActiveSpawns", accessoryMaxActiveSpawns);
            accessorySpawnObjectId = jsoncompat::fieldString(
                parameters, "spawnObjectId", accessorySpawnObjectId);
            accessoryInitialWorkpieceCount = jsoncompat::fieldInt(
                parameters, "initialWorkpieceCount", accessoryInitialWorkpieceCount);
            accessoryInitialWorkpieceEndInsetMm = jsoncompat::fieldDouble(
                parameters, "initialWorkpieceEndInsetMm", accessoryInitialWorkpieceEndInsetMm);
            accessoryInitialWorkpieceSpacingMm = jsoncompat::fieldDouble(
                parameters, "initialWorkpieceSpacingMm", accessoryInitialWorkpieceSpacingMm);
        }
    }

    Json toJson() const override {
        Json obj = Json::object();
        if (isRobotBaseFrame) obj["isRobotBaseFrame"] = isRobotBaseFrame;
        if (hasParametricAccessory()) obj["parametricAccessory"] = accessoryParametersJson();
        return obj;
    }
    static std::shared_ptr<TransformNodeData> fromJson(const Json& obj) {
        auto data = std::make_shared<TransformNodeData>();
        data->isRobotBaseFrame = jsoncompat::fieldBool(obj, "isRobotBaseFrame");
        data->applyAccessoryParametersJson(
            jsoncompat::fieldObject(obj, "parametricAccessory"));
        return data;
    }
};

struct MutexRootNodeData : public CadNodeDataBase {
    mutable std::mutex mutex;
    
    // Ground plane properties
    bool groundPlaneVisible = true;
    double groundPlaneY = -50.0;  // Y position of the ground plane
    double groundPlaneSize = 50000.0;  // Size of the ground plane (half-width)
    double groundPlaneThickness = 0.1;  // Thickness of the ground plane
    CADNodeColor groundPlaneColor = CADNodeColor(0.7f, 0.7f, 0.7f, 0.8f);  // Light gray with transparency
    
    MutexRootNodeData() = default;
    
    Json toJson() const override {
        Json obj = Json::object();
        obj["groundPlaneVisible"] = groundPlaneVisible;
        obj["groundPlaneY"] = groundPlaneY;
        obj["groundPlaneSize"] = groundPlaneSize;
        obj["groundPlaneThickness"] = groundPlaneThickness;
        Json colorArr = Json::array();
        colorArr.push_back(groundPlaneColor.r);
        colorArr.push_back(groundPlaneColor.g);
        colorArr.push_back(groundPlaneColor.b);
        colorArr.push_back(groundPlaneColor.a);
        obj["groundPlaneColor"] = colorArr;
        return obj;
    }
    
    static std::shared_ptr<MutexRootNodeData> fromJson(const Json& obj) {
        auto data = std::make_shared<MutexRootNodeData>();
        data->groundPlaneVisible = jsoncompat::fieldBool(obj, "groundPlaneVisible", false);
        data->groundPlaneY = jsoncompat::fieldDouble(obj, "groundPlaneY", -50.0);
        data->groundPlaneSize = jsoncompat::fieldDouble(obj, "groundPlaneSize", 10000.0);
        data->groundPlaneThickness = jsoncompat::fieldDouble(obj, "groundPlaneThickness", 0.1);
        Json colorArr = jsoncompat::fieldArray(obj, "groundPlaneColor");
        if (colorArr.size() == 4) {
            data->groundPlaneColor = CADNodeColor(
                static_cast<float>(jsoncompat::toDouble(colorArr[0], 0.7)),
                static_cast<float>(jsoncompat::toDouble(colorArr[1], 0.7)),
                static_cast<float>(jsoncompat::toDouble(colorArr[2], 0.7)),
                static_cast<float>(jsoncompat::toDouble(colorArr[3], 0.8))
            );
        }
        return data;
    }
};

struct RobotCollisionIgnore {
    int a = 0;
    int b = 0;

    Json toJson() const {
        Json arr = Json::array();
        arr.push_back(a);
        arr.push_back(b);
        return arr;
    }

    static RobotCollisionIgnore fromJson(const Json& arr) {
        RobotCollisionIgnore ignore;
        if (arr.size() >= 2) {
            ignore.a = jsoncompat::toInt(arr[0]);
            ignore.b = jsoncompat::toInt(arr[1]);
        }
        return ignore;
    }
};

struct RobotTcp {
    std::string name;
    CadTransform loc; // tool-to-TCP
    bool visible = true;

    Json toJson() const {
        Json obj = Json::object();
        obj["name"] = name;
        if (!loc.IsIdentity()) obj["loc"] = topLocToJson(loc);
        if (!visible) obj["visible"] = visible;
        return obj;
    }

    static RobotTcp fromJson(const Json& obj) {
        RobotTcp tcp;
        tcp.name = jsoncompat::fieldString(obj, "name");
        tcp.loc = topLocFromJson(jsoncompat::fieldArray(obj, "loc"));
        tcp.visible = jsoncompat::fieldBool(obj, "visible", true);
        return tcp;
    }
};

// A scalar mechanism coordinate and the scene nodes driven by it. Shared by rails, grippers and
// future tooling so command/state and transform application do not grow mechanism-specific copies.
struct MechanismActuatorBinding {
    CadNode* node = nullptr;
    Json nodePath = Json::array();
    CadVec3 translationAxis{0, 0, 0};
    double mmPerUnit = 0.0;

    Json toJson() const {
        Json obj = Json::object();
        obj["nodePath"] = nodePath;
        obj["translationAxis"] = Json::array(
            {translationAxis.x, translationAxis.y, translationAxis.z});
        obj["mmPerUnit"] = mmPerUnit;
        return obj;
    }

    static MechanismActuatorBinding fromJson(const Json& obj) {
        MechanismActuatorBinding binding;
        binding.nodePath = jsoncompat::fieldArray(obj, "nodePath");
        const Json axis = jsoncompat::fieldArray(obj, "translationAxis");
        if (axis.size() == 3) {
            binding.translationAxis = CadVec3(jsoncompat::toDouble(axis[0]),
                                               jsoncompat::toDouble(axis[1]),
                                               jsoncompat::toDouble(axis[2]));
        }
        binding.mmPerUnit = jsoncompat::fieldDouble(obj, "mmPerUnit");
        return binding;
    }
};

struct MechanismActuatorData {
    std::string id;
    std::string name;
    double position = 0.0;
    double lowerLimit = 0.0;
    double upperLimit = 1.0;
    double velocityUnitsPerSecond = 1.0;
    double effortLimit = 0.0;
    // "none", "logical-grasp", or "physics-grasp". Physics never reparents the product.
    std::string interaction = "none";
    int tcpIndex = -1;
    double captureRadiusMm = 90.0;
    std::vector<MechanismActuatorBinding> bindings;

    Json toJson() const {
        Json obj = Json::object();
        obj["id"] = id;
        obj["name"] = name;
        obj["position"] = position;
        obj["lowerLimit"] = lowerLimit;
        obj["upperLimit"] = upperLimit;
        obj["velocityUnitsPerSecond"] = velocityUnitsPerSecond;
        if (effortLimit > 0.0) obj["effortLimit"] = effortLimit;
        if (interaction != "none") obj["interaction"] = interaction;
        if (tcpIndex >= 0) obj["tcpIndex"] = tcpIndex;
        if (captureRadiusMm != 90.0) obj["captureRadiusMm"] = captureRadiusMm;
        Json values = Json::array();
        for (const MechanismActuatorBinding& binding : bindings) values.push_back(binding.toJson());
        obj["bindings"] = values;
        return obj;
    }

    static MechanismActuatorData fromJson(const Json& obj) {
        MechanismActuatorData data;
        data.id = jsoncompat::fieldString(obj, "id");
        data.name = jsoncompat::fieldString(obj, "name");
        data.position = jsoncompat::fieldDouble(obj, "position");
        data.lowerLimit = jsoncompat::fieldDouble(obj, "lowerLimit");
        data.upperLimit = jsoncompat::fieldDouble(obj, "upperLimit", 1.0);
        data.velocityUnitsPerSecond = jsoncompat::fieldDouble(obj, "velocityUnitsPerSecond", 1.0);
        data.effortLimit = jsoncompat::fieldDouble(obj, "effortLimit");
        data.interaction = jsoncompat::fieldString(obj, "interaction", "none");
        data.tcpIndex = jsoncompat::fieldInt(obj, "tcpIndex", -1);
        data.captureRadiusMm = jsoncompat::fieldDouble(obj, "captureRadiusMm", 90.0);
        for (const auto& value : jsoncompat::fieldArray(obj, "bindings")) {
            if (value.is_object()) data.bindings.push_back(
                MechanismActuatorBinding::fromJson(jsoncompat::toObject(value)));
        }
        return data;
    }
};

struct RobotDhmData {
    static constexpr size_t kJointCount = 6;
    static constexpr size_t kRowCount = 4;
    static constexpr size_t kValueCount = kJointCount * kRowCount;

    // Row-major modified DH table: theta offsets, a, alpha, d.
    std::array<double, kValueCount> values{};
    int sourceValueCount = static_cast<int>(kValueCount);

    double thetaOffset(size_t joint) const { return values[joint]; }
    double a(size_t joint) const { return values[kJointCount + joint]; }
    double alpha(size_t joint) const { return values[2 * kJointCount + joint]; }
    double d(size_t joint) const { return values[3 * kJointCount + joint]; }

    bool isValid() const {
        if (sourceValueCount != static_cast<int>(kValueCount)) return false;
        bool anyNonZero = false;
        for (double value : values) {
            if (!std::isfinite(value)) return false;
            anyNonZero = anyNonZero || std::abs(value) > 1.0e-12;
        }
        return anyNonZero;
    }

    Json toJson() const {
        return stdArrayToJson(values);
    }

    static RobotDhmData fromJson(const Json& arr) {
        RobotDhmData data;
        data.sourceValueCount = static_cast<int>(arr.size());
        stdArrayFromJson(arr, data.values);
        return data;
    }
};

struct OPW6RobotData : public CadNodeDataBase {
    RobotDhmData dhm;
    std::array<double, 6> q{};
    std::array<double, 6> qHome{};
    std::array<double, 6> qMin{};
    std::array<double, 6> qMax{};
    std::array<double, 6> jointVelocityMaxRadS{};
    std::array<double, 6> jointAccelerationMaxRadS2{};
    std::array<double, 6> jointJerkMaxRadS3{};
    std::array<double, 6> stepsPerDegreeNominal{};
    std::array<double, 6> driveMicrostepsPerRev{};
    std::array<double, 6> jointZeroOffsetRad{};
    std::array<double, 24> dhmCorrection{};
    CadTransform toolCalibration;
    CadTransform baseCalibration;
    bool hasJointVelocityLimits = false;
    bool hasJointAccelerationLimits = false;
    bool hasJointJerkLimits = false;
    bool hasNominalStepsPerDegree = false;
    bool hasDriveMicrostepsPerRev = false;
    bool hasJointZeroOffsets = false;
    bool hasDhmCorrection = false;
    bool hasToolCalibration = false;
    bool hasBaseCalibration = false;
    double controllerMinTickGapUs = 0.0;
    double hmiMinSpeedDelayUs = 0.0;
    double hmiVirtualSpeedScale = 0.0;
    double controlPeriodSec = 0.0;
    double defaultJointSpeedDegPerSec = 0.0;
    double defaultLinearSpeedMmPerSec = 0.0;
    double defaultLinearAccelerationMmSec2 = 0.0;
    double defaultLinearJerkMmSec3 = 0.0;
    double defaultToolAngularSpeedRadSec = 0.0;
    double defaultToolAngularAccelerationRadSec2 = 0.0;
    double defaultToolAngularJerkRadSec3 = 0.0;
    double singularityThresholdRad = 0.0;
    Json motionLimitDerivation = Json::object();

    CadNode* activeTool = nullptr;
    Json activeToolPath = Json::array();

    std::vector<RobotCollisionIgnore> collisionIgnores = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}
    };

    Json toJson() const override {
        Json obj = Json::object();
        obj["dhm"] = dhm.toJson();
        obj["qHome"] = stdArrayToJson(qHome);
        obj["qMin"] = stdArrayToJson(qMin);
        obj["qMax"] = stdArrayToJson(qMax);
        if (hasJointVelocityLimits) obj["jointVelocityMaxRadS"] = stdArrayToJson(jointVelocityMaxRadS);
        if (hasJointAccelerationLimits) obj["jointAccelerationMaxRadS2"] = stdArrayToJson(jointAccelerationMaxRadS2);
        if (hasJointJerkLimits) obj["jointJerkMaxRadS3"] = stdArrayToJson(jointJerkMaxRadS3);
        if (hasNominalStepsPerDegree) obj["stepsPerDegreeNominal"] = stdArrayToJson(stepsPerDegreeNominal);
        if (hasDriveMicrostepsPerRev) obj["driveMicrostepsPerRev"] = stdArrayToJson(driveMicrostepsPerRev);
        if (hmiMinSpeedDelayUs > 0.0) obj["hmiMinSpeedDelayUs"] = hmiMinSpeedDelayUs;
        if (hmiVirtualSpeedScale > 0.0) obj["hmiVirtualSpeedScale"] = hmiVirtualSpeedScale;
        if (controllerMinTickGapUs > 0.0) obj["controllerMinTickGapUs"] = controllerMinTickGapUs;
        if (controlPeriodSec > 0.0) obj["controlPeriodSec"] = controlPeriodSec;
        if (defaultJointSpeedDegPerSec > 0.0) obj["defaultJointSpeedDegPerSec"] = defaultJointSpeedDegPerSec;
        if (defaultLinearSpeedMmPerSec > 0.0) obj["defaultLinearSpeedMmPerSec"] = defaultLinearSpeedMmPerSec;
        if (defaultLinearAccelerationMmSec2 > 0.0) obj["defaultLinearAccelerationMmSec2"] = defaultLinearAccelerationMmSec2;
        if (defaultLinearJerkMmSec3 > 0.0) obj["defaultLinearJerkMmSec3"] = defaultLinearJerkMmSec3;
        if (defaultToolAngularSpeedRadSec > 0.0) obj["defaultToolAngularSpeedRadSec"] = defaultToolAngularSpeedRadSec;
        if (defaultToolAngularAccelerationRadSec2 > 0.0) obj["defaultToolAngularAccelerationRadSec2"] = defaultToolAngularAccelerationRadSec2;
        if (defaultToolAngularJerkRadSec3 > 0.0) obj["defaultToolAngularJerkRadSec3"] = defaultToolAngularJerkRadSec3;
        if (singularityThresholdRad > 0.0) obj["singularityThresholdRad"] = singularityThresholdRad;
        if (hasJointZeroOffsets) obj["jointZeroOffsetRad"] = stdArrayToJson(jointZeroOffsetRad);
        if (hasDhmCorrection) obj["dhmCorrection"] = stdArrayToJson(dhmCorrection);
        if (hasToolCalibration) obj["toolCalibration"] = topLocToJson(toolCalibration);
        if (hasBaseCalibration) obj["baseCalibration"] = topLocToJson(baseCalibration);
        if (!motionLimitDerivation.empty()) obj["motionLimitDerivation"] = motionLimitDerivation;
        obj["activeToolPath"] = activeToolPath;
        Json ignoresArr = Json::array();
        for (const auto& ignore : collisionIgnores) ignoresArr.push_back(ignore.toJson());
        obj["collisionIgnores"] = ignoresArr;
        return obj;
    }

    static std::shared_ptr<OPW6RobotData> fromJson(const Json& obj) {
        auto data = std::make_shared<OPW6RobotData>();
        data->dhm = RobotDhmData::fromJson(jsoncompat::fieldArray(obj, "dhm"));
        stdArrayFromJson(jsoncompat::fieldArray(obj, "q"), data->q);
        stdArrayFromJson(jsoncompat::fieldArray(obj, "qHome"), data->qHome);
        stdArrayFromJson(jsoncompat::fieldArray(obj, "qMin"), data->qMin);
        stdArrayFromJson(jsoncompat::fieldArray(obj, "qMax"), data->qMax);
        const Json velocityArr = jsoncompat::fieldArray(obj, "jointVelocityMaxRadS");
        data->hasJointVelocityLimits = velocityArr.size() == 6;
        if (data->hasJointVelocityLimits) stdArrayFromJson(velocityArr, data->jointVelocityMaxRadS);
        const Json accelerationArr = jsoncompat::fieldArray(obj, "jointAccelerationMaxRadS2");
        data->hasJointAccelerationLimits = accelerationArr.size() == 6;
        if (data->hasJointAccelerationLimits) stdArrayFromJson(accelerationArr, data->jointAccelerationMaxRadS2);
        const Json jerkArr = jsoncompat::fieldArray(obj, "jointJerkMaxRadS3");
        data->hasJointJerkLimits = jerkArr.size() == 6;
        if (data->hasJointJerkLimits) stdArrayFromJson(jerkArr, data->jointJerkMaxRadS3);
        const Json stepsPerDegreeArr = jsoncompat::fieldArray(obj, "stepsPerDegreeNominal");
        data->hasNominalStepsPerDegree = stepsPerDegreeArr.size() == 6;
        if (data->hasNominalStepsPerDegree) stdArrayFromJson(stepsPerDegreeArr, data->stepsPerDegreeNominal);
        const Json driveMicrostepsArr = jsoncompat::fieldArray(obj, "driveMicrostepsPerRev");
        data->hasDriveMicrostepsPerRev = driveMicrostepsArr.size() == 6;
        if (data->hasDriveMicrostepsPerRev) stdArrayFromJson(driveMicrostepsArr, data->driveMicrostepsPerRev);
        data->hmiMinSpeedDelayUs = jsoncompat::fieldDouble(obj, "hmiMinSpeedDelayUs", 0.0);
        data->hmiVirtualSpeedScale = jsoncompat::fieldDouble(obj, "hmiVirtualSpeedScale", 0.0);
        data->controllerMinTickGapUs = jsoncompat::fieldDouble(obj, "controllerMinTickGapUs", 0.0);
        data->controlPeriodSec = jsoncompat::fieldDouble(obj, "controlPeriodSec", 0.0);
        data->defaultJointSpeedDegPerSec = jsoncompat::fieldDouble(obj, "defaultJointSpeedDegPerSec", 0.0);
        data->defaultLinearSpeedMmPerSec = jsoncompat::fieldDouble(obj, "defaultLinearSpeedMmPerSec", 0.0);
        data->defaultLinearAccelerationMmSec2 = jsoncompat::fieldDouble(obj, "defaultLinearAccelerationMmSec2", 0.0);
        data->defaultLinearJerkMmSec3 = jsoncompat::fieldDouble(obj, "defaultLinearJerkMmSec3", 0.0);
        data->defaultToolAngularSpeedRadSec = jsoncompat::fieldDouble(obj, "defaultToolAngularSpeedRadSec", 0.0);
        data->defaultToolAngularAccelerationRadSec2 = jsoncompat::fieldDouble(obj, "defaultToolAngularAccelerationRadSec2", 0.0);
        data->defaultToolAngularJerkRadSec3 = jsoncompat::fieldDouble(obj, "defaultToolAngularJerkRadSec3", 0.0);
        data->singularityThresholdRad = jsoncompat::fieldDouble(obj, "singularityThresholdRad", 0.0);
        const Json zeroOffsetArr = jsoncompat::fieldArray(obj, "jointZeroOffsetRad");
        data->hasJointZeroOffsets = zeroOffsetArr.size() == 6;
        if (data->hasJointZeroOffsets) stdArrayFromJson(zeroOffsetArr, data->jointZeroOffsetRad);
        const Json dhmCorrectionArr = jsoncompat::fieldArray(obj, "dhmCorrection");
        data->hasDhmCorrection = dhmCorrectionArr.size() == 24;
        if (data->hasDhmCorrection) stdArrayFromJson(dhmCorrectionArr, data->dhmCorrection);
        const Json toolCalibrationArr = jsoncompat::fieldArray(obj, "toolCalibration");
        data->hasToolCalibration = toolCalibrationArr.size() == 12;
        if (data->hasToolCalibration) data->toolCalibration = topLocFromJson(toolCalibrationArr);
        const Json baseCalibrationArr = jsoncompat::fieldArray(obj, "baseCalibration");
        data->hasBaseCalibration = baseCalibrationArr.size() == 12;
        if (data->hasBaseCalibration) data->baseCalibration = topLocFromJson(baseCalibrationArr);
        data->motionLimitDerivation = jsoncompat::fieldObject(obj, "motionLimitDerivation");
        if (!obj.contains("q")) data->q = data->qHome;
        data->activeToolPath = jsoncompat::fieldArray(obj, "activeToolPath");
        data->collisionIgnores.clear();
        Json ignoresArr = jsoncompat::fieldArray(obj, "collisionIgnores");
        for (const auto& ignoreVal : ignoresArr) {
            if (ignoreVal.is_array()) data->collisionIgnores.push_back(RobotCollisionIgnore::fromJson(jsoncompat::toArray(ignoreVal)));
        }
        if (data->collisionIgnores.empty()) {
            data->collisionIgnores = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}};
        }
        return data;
    }
};

struct RobotLinkData : public CadNodeDataBase {
    std::vector<CadNode*> geometryNodes;
    std::vector<Json> geometryPaths;
    std::vector<ConvexHullData> collisionHulls;
    bool visualEnabled = true;
    bool collisionHullsVisible = true;

    Json toJson() const override {
        Json obj = Json::object();
        Json pathsArr = Json::array();
        for (const auto& path : geometryPaths) pathsArr.push_back(path);
        obj["geometryPaths"] = pathsArr;
        Json hullsArr = Json::array();
        for (const auto& hull : collisionHulls) hullsArr.push_back(hull.toJson());
        if (!hullsArr.empty()) obj["collisionHulls"] = hullsArr;
        if (!visualEnabled) obj["visualEnabled"] = visualEnabled;
        if (!collisionHullsVisible) obj["collisionHullsVisible"] = collisionHullsVisible;
        return obj;
    }

    static std::shared_ptr<RobotLinkData> fromJson(const Json& obj) {
        auto data = std::make_shared<RobotLinkData>();
        Json pathsArr = jsoncompat::fieldArray(obj, "geometryPaths");
        for (const auto& pathVal : pathsArr) {
            if (pathVal.is_array()) data->geometryPaths.push_back(jsoncompat::toArray(pathVal));
        }
        Json hullsArr = jsoncompat::fieldArray(obj, "collisionHulls");
        for (const auto& hullVal : hullsArr) {
            if (hullVal.is_object()) data->collisionHulls.push_back(ConvexHullData::fromJson(jsoncompat::toObject(hullVal)));
        }
        data->visualEnabled = jsoncompat::fieldBool(obj, "visualEnabled", true);
        data->collisionHullsVisible = jsoncompat::fieldBool(obj, "collisionHullsVisible", true);
        return data;
    }
};

struct RobotToolData : public CadNodeDataBase {
    std::vector<CadNode*> geometryNodes;
    std::vector<Json> geometryPaths;
    std::vector<ConvexHullData> collisionHulls;
    std::vector<RobotTcp> tcps;
    std::vector<MechanismActuatorData> actuators;
    int activeTcpIndex = 0;
    bool visualEnabled = true;
    bool collisionHullsVisible = true;

    Json toJson() const override {
        Json obj = Json::object();
        Json pathsArr = Json::array();
        for (const auto& path : geometryPaths) pathsArr.push_back(path);
        obj["geometryPaths"] = pathsArr;
        Json hullsArr = Json::array();
        for (const auto& hull : collisionHulls) hullsArr.push_back(hull.toJson());
        if (!hullsArr.empty()) obj["collisionHulls"] = hullsArr;
        Json tcpsArr = Json::array();
        for (const auto& tcp : tcps) tcpsArr.push_back(tcp.toJson());
        if (!tcpsArr.empty()) obj["tcps"] = tcpsArr;
        Json actuatorArray = Json::array();
        for (const MechanismActuatorData& actuator : actuators) {
            actuatorArray.push_back(actuator.toJson());
        }
        if (!actuatorArray.empty()) obj["actuators"] = actuatorArray;
        if (activeTcpIndex != 0) obj["activeTcpIndex"] = activeTcpIndex;
        if (!visualEnabled) obj["visualEnabled"] = visualEnabled;
        if (!collisionHullsVisible) obj["collisionHullsVisible"] = collisionHullsVisible;
        return obj;
    }

    static std::shared_ptr<RobotToolData> fromJson(const Json& obj) {
        auto data = std::make_shared<RobotToolData>();
        Json pathsArr = jsoncompat::fieldArray(obj, "geometryPaths");
        for (const auto& pathVal : pathsArr) {
            if (pathVal.is_array()) data->geometryPaths.push_back(jsoncompat::toArray(pathVal));
        }
        Json hullsArr = jsoncompat::fieldArray(obj, "collisionHulls");
        for (const auto& hullVal : hullsArr) {
            if (hullVal.is_object()) data->collisionHulls.push_back(ConvexHullData::fromJson(jsoncompat::toObject(hullVal)));
        }
        Json tcpsArr = jsoncompat::fieldArray(obj, "tcps");
        for (const auto& tcpVal : tcpsArr) {
            if (tcpVal.is_object()) data->tcps.push_back(RobotTcp::fromJson(jsoncompat::toObject(tcpVal)));
        }
        for (const auto& actuatorValue : jsoncompat::fieldArray(obj, "actuators")) {
            if (actuatorValue.is_object()) data->actuators.push_back(
                MechanismActuatorData::fromJson(jsoncompat::toObject(actuatorValue)));
        }
        data->activeTcpIndex = jsoncompat::fieldInt(obj, "activeTcpIndex", 0);
        data->visualEnabled = jsoncompat::fieldBool(obj, "visualEnabled", true);
        data->collisionHullsVisible = jsoncompat::fieldBool(obj, "collisionHullsVisible", true);
        return data;
    }
};

struct MeshGeometryData : public CadNodeDataBase {
    std::string meshSource;
    std::vector<float> vertices; // xyz triples
    std::vector<uint32_t> indices; // triangle indices, 3 per face
    std::array<float, 6> bounds{{0, 0, 0, 0, 0, 0}};
    bool loaded = false;

    Json toJson() const override {
        Json obj = Json::object();
        obj["meshSource"] = meshSource;
        return obj;
    }

    static std::shared_ptr<MeshGeometryData> fromJson(const Json& obj) {
        auto data = std::make_shared<MeshGeometryData>();
        // "source" is the legacy key name; kept as a fallback so older packages still load.
        data->meshSource =
            jsoncompat::fieldString(obj, "meshSource", jsoncompat::fieldString(obj, "source"));
        Json boundsArr = jsoncompat::fieldArray(obj, "bounds");
        for (int i = 0; i < boundsArr.size() && i < 6; ++i) {
            data->bounds[static_cast<size_t>(i)] = static_cast<float>(jsoncompat::toDouble(boundsArr[i]));
        }
        return data;
    }
};

enum class ConnectionFlags : uint32_t {
    None = 0,
    Cables = 1 << 0,      // Can connect cables
    Conveyors = 1 << 1,   // Can connect conveyors
    All = Cables | Conveyors
};

inline ConnectionFlags operator|(ConnectionFlags a, ConnectionFlags b) {
    return static_cast<ConnectionFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline ConnectionFlags operator&(ConnectionFlags a, ConnectionFlags b) {
    return static_cast<ConnectionFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline ConnectionFlags operator^(ConnectionFlags a, ConnectionFlags b) {
    return static_cast<ConnectionFlags>(static_cast<uint32_t>(a) ^ static_cast<uint32_t>(b));
}

inline ConnectionFlags operator~(ConnectionFlags a) {
    return static_cast<ConnectionFlags>(~static_cast<uint32_t>(a));
}

inline ConnectionFlags& operator|=(ConnectionFlags& a, ConnectionFlags b) {
    return a = a | b;
}

inline ConnectionFlags& operator&=(ConnectionFlags& a, ConnectionFlags b) {
    return a = a & b;
}

inline ConnectionFlags& operator^=(ConnectionFlags& a, ConnectionFlags b) {
    return a = a ^ b;
}

struct ConnectionPointData : public CadNodeDataBase {
    ConnectionFlags connectionFlags = ConnectionFlags::Cables; // Default to cables
    std::string description; // Optional description of the connection point
    
    bool canConnectCables() const {
        return (connectionFlags & ConnectionFlags::Cables) != ConnectionFlags::None;
    }
    
    bool canConnectConveyors() const {
        return (connectionFlags & ConnectionFlags::Conveyors) != ConnectionFlags::None;
    }
    
    void setCanConnectCables(bool enable) {
        if (enable) {
            connectionFlags |= ConnectionFlags::Cables;
        } else {
            connectionFlags &= ~ConnectionFlags::Cables;
        }
    }
    
    void setCanConnectConveyors(bool enable) {
        if (enable) {
            connectionFlags |= ConnectionFlags::Conveyors;
        } else {
            connectionFlags &= ~ConnectionFlags::Conveyors;
        }
    }

    Json toJson() const override {
        Json obj = Json::object();
        obj["connectionFlags"] = static_cast<int>(connectionFlags);
        obj["description"] = description;
        return obj;
    }
    static std::shared_ptr<ConnectionPointData> fromJson(const Json& obj) {
        auto data = std::make_shared<ConnectionPointData>();
        data->connectionFlags = static_cast<ConnectionFlags>(jsoncompat::fieldInt(obj, "connectionFlags"));
        data->description = jsoncompat::fieldString(obj, "description");
        return data;
    }
};

struct CadNode {
    std::string name;
    CADNodeColor color;
    CadTransform loc;
    CadTransform globalLoc; // Cached global transform
    std::vector<std::shared_ptr<CadNode>> children;
    bool visible = true;
    bool excludedFromDecomposition = false; // Exclude from VHACD/CoACD mesh generation
    MountingHoleData mountingHoles;
    bool needsGlobalLocUpdate = false;

    CadNodeType type = CadNodeType::Unknown; // Must be set explicitly
    std::shared_ptr<CadNodeDataBase> data; // Holds type-specific data

    CadNode* parent = nullptr;

#ifdef CADNODE_ENABLE_OCCT
    XCAFNodeData* asXCAF() {
        return type == CadNodeType::XCAF ? static_cast<XCAFNodeData*>(data.get()) : nullptr;
    }
    const XCAFNodeData* asXCAF() const {
        return type == CadNodeType::XCAF ? static_cast<const XCAFNodeData*>(data.get()) : nullptr;
    }
#endif

    CustomNodeData* asCustom() {
        return type == CadNodeType::Custom ? static_cast<CustomNodeData*>(data.get()) : nullptr;
    }
    const CustomNodeData* asCustom() const {
        return type == CadNodeType::Custom ? static_cast<const CustomNodeData*>(data.get()) : nullptr;
    }

    PhysicsNodeData* asPhysics() {
        return type == CadNodeType::Physics ? static_cast<PhysicsNodeData*>(data.get()) : nullptr;
    }
    const PhysicsNodeData* asPhysics() const {
        return type == CadNodeType::Physics ? static_cast<const PhysicsNodeData*>(data.get()) : nullptr;
    }

    RailNodeData* asRail() {
        return type == CadNodeType::Rail ? static_cast<RailNodeData*>(data.get()) : nullptr;
    }
    const RailNodeData* asRail() const {
        return type == CadNodeType::Rail ? static_cast<const RailNodeData*>(data.get()) : nullptr;
    }

    GantryMechanismData* asGantryMechanism() {
        return type == CadNodeType::GantryMechanism ? static_cast<GantryMechanismData*>(data.get()) : nullptr;
    }
    const GantryMechanismData* asGantryMechanism() const {
        return type == CadNodeType::GantryMechanism ? static_cast<const GantryMechanismData*>(data.get()) : nullptr;
    }

    DragChainMechanismData* asDragChainMechanism() {
        return type == CadNodeType::DragChainMechanism ? static_cast<DragChainMechanismData*>(data.get()) : nullptr;
    }
    const DragChainMechanismData* asDragChainMechanism() const {
        return type == CadNodeType::DragChainMechanism ? static_cast<const DragChainMechanismData*>(data.get()) : nullptr;
    }

    TransformNodeData* asTransform() {
        return type == CadNodeType::Transform ? static_cast<TransformNodeData*>(data.get()) : nullptr;
    }
    const TransformNodeData* asTransform() const {
        return type == CadNodeType::Transform ? static_cast<const TransformNodeData*>(data.get()) : nullptr;
    }

    ConnectionPointData* asConnectionPoint() {
        return type == CadNodeType::ConnectionPoint ? static_cast<ConnectionPointData*>(data.get()) : nullptr;
    }
    const ConnectionPointData* asConnectionPoint() const {
        return type == CadNodeType::ConnectionPoint ? static_cast<const ConnectionPointData*>(data.get()) : nullptr;
    }

    MutexRootNodeData* asMutexRoot() {
        return type == CadNodeType::MutexRoot ? static_cast<MutexRootNodeData*>(data.get()) : nullptr;
    }
    const MutexRootNodeData* asMutexRoot() const {
        return type == CadNodeType::MutexRoot ? static_cast<const MutexRootNodeData*>(data.get()) : nullptr;
    }

    OPW6RobotData* asOPW6Robot() {
        return type == CadNodeType::OPW6Robot ? static_cast<OPW6RobotData*>(data.get()) : nullptr;
    }
    const OPW6RobotData* asOPW6Robot() const {
        return type == CadNodeType::OPW6Robot ? static_cast<const OPW6RobotData*>(data.get()) : nullptr;
    }

    RobotLinkData* asRobotLink() {
        return type == CadNodeType::RobotLink ? static_cast<RobotLinkData*>(data.get()) : nullptr;
    }
    const RobotLinkData* asRobotLink() const {
        return type == CadNodeType::RobotLink ? static_cast<const RobotLinkData*>(data.get()) : nullptr;
    }

    RobotToolData* asRobotTool() {
        return type == CadNodeType::RobotTool ? static_cast<RobotToolData*>(data.get()) : nullptr;
    }
    const RobotToolData* asRobotTool() const {
        return type == CadNodeType::RobotTool ? static_cast<const RobotToolData*>(data.get()) : nullptr;
    }

    MeshGeometryData* asMeshGeometry() {
        return type == CadNodeType::MeshGeometry ? static_cast<MeshGeometryData*>(data.get()) : nullptr;
    }
    const MeshGeometryData* asMeshGeometry() const {
        return type == CadNodeType::MeshGeometry ? static_cast<const MeshGeometryData*>(data.get()) : nullptr;
    }

    void setVisibleRecursive(bool vis) {
        visible = vis;
        for (auto& child : children) {
            if (child) child->setVisibleRecursive(vis);
        }
    }

    // Recursively apply a transformation to this node and all children
#ifdef CADNODE_ENABLE_OCCT
    void applyTransform(const gp_Trsf& trsf) {
        loc = TopLoc_Location(trsf * loc.Transformation());
        for (auto& child : children) {
            if (child) child->applyTransform(trsf);
        }
    }
#endif

    const CadNode* root() const {
        const CadNode* current = this;
        while (current->parent) current = current->parent;
        return current;
    }

    CadNode* root() {
        CadNode* current = this;
        while (current->parent) current = current->parent;
        return current;
    }

    static Json pathToNode(const CadNode* rootNode, const CadNode* target) {
        Json arr = Json::array();
        if (!rootNode || !target) return arr;
        std::vector<int> path;
        if (!findPathToNode(rootNode, target, path)) return arr;
        for (int index : path) arr.push_back(index);
        return arr;
    }

    static CadNode* nodeAtPath(CadNode* rootNode, const Json& path) {
        if (path.empty()) return nullptr;
        CadNode* current = rootNode;
        for (const auto& pathVal : path) {
            if (!current) return nullptr;
            int index = jsoncompat::toInt(pathVal, -1);
            if (index < 0 || index >= static_cast<int>(current->children.size())) return nullptr;
            current = current->children[static_cast<size_t>(index)].get();
        }
        return current;
    }

    static void resolveRobotReferences(CadNode* rootNode) {
        if (!rootNode) return;
        resolveRobotReferencesRecursive(rootNode, rootNode);
    }

    Json toJson() const {
        Json obj = Json::object();
        obj["name"] = name;
        obj["color"] = Json::array({color.r, color.g, color.b, color.a});
        if (!loc.IsIdentity()) obj["loc"] = topLocToJson(loc);
        if (!visible) obj["visible"] = visible;
        if (excludedFromDecomposition) obj["excludedFromDecomposition"] = excludedFromDecomposition;
        if (!mountingHoles.empty()) obj["mountingHoles"] = mountingHoles.toJson();
        obj["type"] = static_cast<int>(type);
        Json dataObj = Json::object();
        if (type == CadNodeType::Rail && data) dataObj = static_cast<RailNodeData*>(data.get())->toJson();
        else if (type == CadNodeType::GantryMechanism && data) {
            GantryMechanismData* gantryData = static_cast<GantryMechanismData*>(data.get());
            gantryData->movingFramePath = pathToNode(root(), gantryData->movingFrame);
            dataObj = gantryData->toJson();
        }
        else if (type == CadNodeType::DragChainMechanism && data) {
            DragChainMechanismData* chainData = static_cast<DragChainMechanismData*>(data.get());
            chainData->movingFramePath = pathToNode(root(), chainData->movingFrame);
            chainData->prototypeGeometryPath = pathToNode(root(), chainData->prototypeGeometry);
            chainData->linkFramePaths = Json::array();
            for (CadNode* frame : chainData->linkFrames) {
                chainData->linkFramePaths.push_back(pathToNode(root(), frame));
            }
            dataObj = chainData->toJson();
        }
        else if (type == CadNodeType::Physics && data) dataObj = static_cast<PhysicsNodeData*>(data.get())->toJson();
        else if (type == CadNodeType::ConnectionPoint && data) dataObj = static_cast<ConnectionPointData*>(data.get())->toJson();
        else if (type == CadNodeType::Transform && data) dataObj = static_cast<TransformNodeData*>(data.get())->toJson();
        else if (type == CadNodeType::MutexRoot && data) dataObj = static_cast<MutexRootNodeData*>(data.get())->toJson();
        else if (type == CadNodeType::OPW6Robot && data) {
            OPW6RobotData* robotData = static_cast<OPW6RobotData*>(data.get());
            robotData->activeToolPath = pathToNode(root(), robotData->activeTool);
            dataObj = robotData->toJson();
        }
        else if (type == CadNodeType::RobotLink && data) {
            RobotLinkData* linkData = static_cast<RobotLinkData*>(data.get());
            linkData->geometryPaths.clear();
            for (CadNode* geometryNode : linkData->geometryNodes) {
                linkData->geometryPaths.push_back(pathToNode(root(), geometryNode));
            }
            dataObj = linkData->toJson();
        }
        else if (type == CadNodeType::RobotTool && data) {
            RobotToolData* toolData = static_cast<RobotToolData*>(data.get());
            toolData->geometryPaths.clear();
            for (CadNode* geometryNode : toolData->geometryNodes) {
                toolData->geometryPaths.push_back(pathToNode(root(), geometryNode));
            }
            for (MechanismActuatorData& actuator : toolData->actuators) {
                for (MechanismActuatorBinding& binding : actuator.bindings) {
                    binding.nodePath = pathToNode(root(), binding.node);
                }
            }
            dataObj = toolData->toJson();
        }
        else if (type == CadNodeType::MeshGeometry && data) dataObj = static_cast<MeshGeometryData*>(data.get())->toJson();
#ifdef CADNODE_ENABLE_OCCT
        else if (type == CadNodeType::XCAF && asXCAF()) {
            const XCAFNodeData* xData = asXCAF();
            Json labelPathArr = Json::array();
            for (int tag : xData->labelPath) labelPathArr.push_back(tag);
            dataObj["labelPath"] = labelPathArr;
            dataObj["shapeIndex"] = xData->shapeIndex;
            dataObj["shapeType"] = static_cast<int>(xData->type);
        }
#endif
        obj["data"] = dataObj;
        Json childrenArr = Json::array();
        for (const auto& child : children) {
            if (child) childrenArr.push_back(child->toJson());
        }
        obj["children"] = childrenArr;
        return obj;
    }
    static std::shared_ptr<CadNode> fromJson(const Json& obj) {
        auto node = std::make_shared<CadNode>();
        node->name = jsoncompat::fieldString(obj, "name");
        Json colorArr = jsoncompat::fieldArray(obj, "color");
        if (colorArr.size() == 4) {
            node->color = CADNodeColor(
                static_cast<float>(jsoncompat::toDouble(colorArr[0])),
                static_cast<float>(jsoncompat::toDouble(colorArr[1])),
                static_cast<float>(jsoncompat::toDouble(colorArr[2])),
                static_cast<float>(jsoncompat::toDouble(colorArr[3])));
        }
        Json locArr = jsoncompat::fieldArray(obj, "loc");
        if (locArr.size() == 12) node->loc = topLocFromJson(locArr);
        node->visible = jsoncompat::fieldBool(obj, "visible", true);
        node->excludedFromDecomposition = jsoncompat::fieldBool(obj, "excludedFromDecomposition", false);
        node->mountingHoles =
            MountingHoleData::fromJson(jsoncompat::fieldObject(obj, "mountingHoles"));
        node->type = static_cast<CadNodeType>(jsoncompat::fieldInt(obj, "type"));
        Json dataObj = jsoncompat::fieldObject(obj, "data");
        if (node->type == CadNodeType::Rail) node->data = RailNodeData::fromJson(dataObj);
        else if (node->type == CadNodeType::GantryMechanism) node->data = GantryMechanismData::fromJson(dataObj);
        else if (node->type == CadNodeType::DragChainMechanism) node->data = DragChainMechanismData::fromJson(dataObj);
        else if (node->type == CadNodeType::Physics) node->data = PhysicsNodeData::fromJson(dataObj);
        else if (node->type == CadNodeType::ConnectionPoint) node->data = ConnectionPointData::fromJson(dataObj);
        else if (node->type == CadNodeType::Transform) node->data = TransformNodeData::fromJson(dataObj);
        else if (node->type == CadNodeType::MutexRoot) node->data = MutexRootNodeData::fromJson(dataObj);
        else if (node->type == CadNodeType::OPW6Robot) node->data = OPW6RobotData::fromJson(dataObj);
        else if (node->type == CadNodeType::RobotLink) node->data = RobotLinkData::fromJson(dataObj);
        else if (node->type == CadNodeType::RobotTool) node->data = RobotToolData::fromJson(dataObj);
        else if (node->type == CadNodeType::MeshGeometry) node->data = MeshGeometryData::fromJson(dataObj);
#ifdef CADNODE_ENABLE_OCCT
        else if (node->type == CadNodeType::XCAF && dataObj.contains("labelPath")) {
            auto xData = std::make_shared<XCAFNodeData>();
            Json labelPathArr = jsoncompat::fieldArray(dataObj, "labelPath");
            for (const auto& v : labelPathArr) xData->labelPath.push_back(jsoncompat::toInt(v));
            xData->shapeIndex = jsoncompat::fieldInt(dataObj, "shapeIndex", -1);
            xData->type = static_cast<TopAbs_ShapeEnum>(jsoncompat::fieldInt(dataObj, "shapeType", static_cast<int>(TopAbs_SHAPE)));
            node->data = xData;
        }
#endif
        Json childrenArr = jsoncompat::fieldArray(obj, "children");
        for (const auto& childVal : childrenArr) {
            if (childVal.is_object()) {
                std::shared_ptr<CadNode> child = fromJson(jsoncompat::toObject(childVal));
                if (child) {
                    child->parent = node.get();
                    node->children.push_back(child);
                }
            }
        }
        resolveRobotReferences(node.get());
        return node;
    }

private:
    static bool findPathToNode(const CadNode* current, const CadNode* target, std::vector<int>& path) {
        if (current == target) return true;
        for (size_t i = 0; i < current->children.size(); ++i) {
            path.push_back(static_cast<int>(i));
            if (current->children[i] && findPathToNode(current->children[i].get(), target, path)) return true;
            path.pop_back();
        }
        return false;
    }

    static void resolveRobotReferencesRecursive(CadNode* rootNode, CadNode* current) {
        if (!current) return;
        if (current->type == CadNodeType::GantryMechanism && current->data) {
            GantryMechanismData* gantryData = static_cast<GantryMechanismData*>(current->data.get());
            gantryData->movingFrame = nodeAtPath(rootNode, gantryData->movingFramePath);
        } else if (current->type == CadNodeType::DragChainMechanism && current->data) {
            DragChainMechanismData* chainData = static_cast<DragChainMechanismData*>(current->data.get());
            chainData->movingFrame = nodeAtPath(rootNode, chainData->movingFramePath);
            chainData->prototypeGeometry = nodeAtPath(rootNode, chainData->prototypeGeometryPath);
            chainData->linkFrames.clear();
            for (const Json& path : chainData->linkFramePaths) {
                if (CadNode* frame = nodeAtPath(rootNode, path)) chainData->linkFrames.push_back(frame);
            }
        } else if (current->type == CadNodeType::OPW6Robot && current->data) {
            OPW6RobotData* robotData = static_cast<OPW6RobotData*>(current->data.get());
            robotData->activeTool = nodeAtPath(rootNode, robotData->activeToolPath);
        } else if (current->type == CadNodeType::RobotLink && current->data) {
            RobotLinkData* linkData = static_cast<RobotLinkData*>(current->data.get());
            linkData->geometryNodes.clear();
            for (const Json& path : linkData->geometryPaths) {
                if (CadNode* node = nodeAtPath(rootNode, path)) linkData->geometryNodes.push_back(node);
            }
        } else if (current->type == CadNodeType::RobotTool && current->data) {
            RobotToolData* toolData = static_cast<RobotToolData*>(current->data.get());
            toolData->geometryNodes.clear();
            for (const Json& path : toolData->geometryPaths) {
                if (CadNode* node = nodeAtPath(rootNode, path)) toolData->geometryNodes.push_back(node);
            }
            for (MechanismActuatorData& actuator : toolData->actuators) {
                for (MechanismActuatorBinding& binding : actuator.bindings) {
                    binding.node = nodeAtPath(rootNode, binding.nodePath);
                }
            }
        }
        for (auto& child : current->children) {
            resolveRobotReferencesRecursive(rootNode, child.get());
        }
    }
};

inline size_t countUniqueCadNodes(const CadNode* root) {
    std::unordered_set<const CadNode*> visited;
    std::function<void(const CadNode*)> visit = [&](const CadNode* node) {
        if (!node || visited.count(node)) return;
        visited.insert(node);
        for (const auto& child : node->children) {
            visit(child.get());
        }
    };
    visit(root);
    return visited.size();
}

#endif // QTCADVIEWER_CADNODE_H
