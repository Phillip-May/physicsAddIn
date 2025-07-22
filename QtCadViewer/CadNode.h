#ifndef QTCADVIEWER_CADNODE_H
#define QTCADVIEWER_CADNODE_H
#include <vector>
#include <string>
#include <TopLoc_Location.hxx>
#include <TopoDS_Face.hxx>
#include <QtGui/QColor>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS.hxx>
#include <memory>
#include <QVector3D>
#include <QString>
#include <unordered_set>
#include <functional>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <array>

// Define a simple RGBA color struct
struct CADNodeColor {
    float r, g, b, a;
    CADNodeColor(float r_ = 1.0f, float g_ = 1.0f, float b_ = 1.0f, float a_ = 1.0f) : r(r_), g(g_), b(b_), a(a_) {}
    static CADNodeColor fromSRGB(int r_, int g_, int b_, int a_ = 255) {
        return CADNodeColor(r_ / 255.0f, g_ / 255.0f, b_ / 255.0f, a_ / 255.0f);
    }
    QColor toQColor() const {
        return QColor::fromRgbF(r, g, b, a);
    }
};

// Struct to hold a face, its color, and its transformation
struct ColoredFace {
    TopoDS_Face face;
    CADNodeColor color;
    TopLoc_Location loc;
};

// Union to hold different types of shape data
union CADNodeData {
    TopoDS_Face face;
    TopoDS_Edge edge;
    TopoDS_Shape shape;
    
    // Default constructor
    CADNodeData() : shape() {}
    
    // Destructor - needed for proper cleanup
    ~CADNodeData() {}
    
    // Copy constructor
    CADNodeData(const CADNodeData& other) : shape(other.shape) {}
    
    // Assignment operator
    CADNodeData& operator=(const CADNodeData& other) {
        if (this != &other) {
            shape = other.shape;
        }
        return *this;
    }
};

// 1. Node type enum
enum class CadNodeType {
    Unknown = 0, // Default type, type should always be explicitly set
    XCAF,
    Custom, //For components of other CadNodes
    Rail,
    Turnable,
    Physics, // New: Physics object node
    ConnectionPoint, // New: Connection point node
    Transform, // New: Transform node type
    // ... add more as needed
};

// 2. Base class for node-specific data
struct CadNodeDataBase {
    virtual ~CadNodeDataBase() = default;
    virtual QJsonObject toJson() const { return QJsonObject(); }
    static std::shared_ptr<CadNodeDataBase> fromJson(const QJsonObject& obj) {
        return nullptr; // Default implementation
    }
};

// 3. XCAF-specific data
struct XCAFNodeData : public CadNodeDataBase {
    TopAbs_ShapeEnum type = TopAbs_SHAPE;
    TopoDS_Shape shape;
    // Optionally: TopoDS_Face, TopoDS_Edge, etc.
    // Add more XCAF-specific fields as needed

    // Utility functions
    bool hasFace() const { return type == TopAbs_FACE && !shape.IsNull(); }
    bool hasEdge() const { return type == TopAbs_EDGE && !shape.IsNull(); }
    TopoDS_Face getFace() const { return (type == TopAbs_FACE) ? TopoDS::Face(shape) : TopoDS_Face(); }
    TopoDS_Edge getEdge() const { return (type == TopAbs_EDGE) ? TopoDS::Edge(shape) : TopoDS_Edge(); }
    // No toJson/fromJson for XCAFNodeData!
};

// 4. Custom node data (example)
struct CustomNodeData : public CadNodeDataBase {
    // Add custom fields here
    int customProperty = 0;
    // ...

    QJsonObject toJson() const override {
        QJsonObject obj;
        obj["customProperty"] = customProperty;
        return obj;
    }
    static std::shared_ptr<CustomNodeData> fromJson(const QJsonObject& obj) {
        auto data = std::make_shared<CustomNodeData>();
        data->customProperty = obj["customProperty"].toInt();
        return data;
    }
};

// 4b. Physics node data
struct ConvexHullData {
    std::vector<std::array<double, 3>> vertices; // x, y, z
    std::vector<std::array<uint32_t, 3>> indices; // triangle indices
};

struct PhysicsNodeData : public CadNodeDataBase {
    bool convexHullGenerated = false;
    std::vector<ConvexHullData> hulls;
    // Add more fields as needed (e.g., material, mass, etc.)
    // Add this field for collision mesh visibility
    bool collisionMeshVisible = true;

    QJsonObject toJson() const override {
        QJsonObject obj;
        obj["convexHullGenerated"] = convexHullGenerated;
        obj["collisionMeshVisible"] = collisionMeshVisible;
        // Hulls not serialized for now (can add if needed)
        return obj;
    }
    static std::shared_ptr<PhysicsNodeData> fromJson(const QJsonObject& obj) {
        auto data = std::make_shared<PhysicsNodeData>();
        data->convexHullGenerated = obj["convexHullGenerated"].toBool();
        data->collisionMeshVisible = obj["collisionMeshVisible"].toBool(true);
        return data;
    }
};

// 4c. Rail node data
struct RailNodeData : public CadNodeDataBase {
    QVector3D axisOfTravel{1,0,0};
    QVector3D buildJointPosition{0,0,0};
    double travelLength = 0.0;
    int numSegments = 1;
    QString jsonString; // For custom JSON editing

    QJsonObject toJson() const override {
        QJsonObject obj;
        QJsonArray axisArr; axisArr << axisOfTravel.x() << axisOfTravel.y() << axisOfTravel.z();
        QJsonArray jointArr; jointArr << buildJointPosition.x() << buildJointPosition.y() << buildJointPosition.z();
        obj["axisOfTravel"] = axisArr;
        obj["buildJointPosition"] = jointArr;
        obj["travelLength"] = travelLength;
        obj["numSegments"] = numSegments;
        obj["jsonString"] = jsonString;
        return obj;
    }
    static std::shared_ptr<RailNodeData> fromJson(const QJsonObject& obj) {
        auto data = std::make_shared<RailNodeData>();
        QJsonArray axisArr = obj["axisOfTravel"].toArray();
        if (axisArr.size() == 3) data->axisOfTravel = QVector3D(axisArr[0].toDouble(), axisArr[1].toDouble(), axisArr[2].toDouble());
        QJsonArray jointArr = obj["buildJointPosition"].toArray();
        if (jointArr.size() == 3) data->buildJointPosition = QVector3D(jointArr[0].toDouble(), jointArr[1].toDouble(), jointArr[2].toDouble());
        data->travelLength = obj["travelLength"].toDouble();
        data->numSegments = obj["numSegments"].toInt();
        data->jsonString = obj["jsonString"].toString();
        return data;
    }
};

// 4d. Transform node data (for future extensibility)
struct TransformNodeData : public CadNodeDataBase {
    // Currently empty, but can be extended for animation, etc.

    QJsonObject toJson() const override {
        QJsonObject obj;
        return obj;
    }
    static std::shared_ptr<TransformNodeData> fromJson(const QJsonObject& obj) {
        return std::make_shared<TransformNodeData>();
    }
};

// Connection point flags (bit flags)
enum class ConnectionFlags : uint32_t {
    None = 0,
    Cables = 1 << 0,      // Can connect cables
    Conveyors = 1 << 1,   // Can connect conveyors
    All = Cables | Conveyors
};

// Bitwise operators for ConnectionFlags
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

// Connection point data
struct ConnectionPointData : public CadNodeDataBase {
    ConnectionFlags connectionFlags = ConnectionFlags::Cables; // Default to cables
    std::string description; // Optional description of the connection point
    
    // Helper functions
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

    QJsonObject toJson() const override {
        QJsonObject obj;
        obj["connectionFlags"] = static_cast<int>(connectionFlags);
        obj["description"] = QString::fromStdString(description);
        return obj;
    }
    static std::shared_ptr<ConnectionPointData> fromJson(const QJsonObject& obj) {
        auto data = std::make_shared<ConnectionPointData>();
        data->connectionFlags = static_cast<ConnectionFlags>(obj["connectionFlags"].toInt());
        data->description = obj["description"].toString().toStdString();
        return data;
    }
};

// 5. The generic CadNode struct
struct CadNode {
    std::string name;
    CADNodeColor color;
    TopLoc_Location loc;
    std::vector<std::shared_ptr<CadNode>> children;
    bool visible = true;
    bool excludedFromDecomposition = false; // Exclude from VHACD/CoACD mesh generation

    CadNodeType type = CadNodeType::Unknown; // Must be set explicitly
    std::shared_ptr<CadNodeDataBase> data; // Holds type-specific data

    // New: Optional XCAF label tag for relinking geometry
    int xcafLabelTag = -1; // -1 means not linked to XCAF

    // Helper to get XCAF data safely
    XCAFNodeData* asXCAF() {
        return type == CadNodeType::XCAF ? static_cast<XCAFNodeData*>(data.get()) : nullptr;
    }
    const XCAFNodeData* asXCAF() const {
        return type == CadNodeType::XCAF ? static_cast<const XCAFNodeData*>(data.get()) : nullptr;
    }

    // Helper to get Custom data safely
    CustomNodeData* asCustom() {
        return type == CadNodeType::Custom ? static_cast<CustomNodeData*>(data.get()) : nullptr;
    }
    const CustomNodeData* asCustom() const {
        return type == CadNodeType::Custom ? static_cast<const CustomNodeData*>(data.get()) : nullptr;
    }

    // Helper to get Physics data safely
    PhysicsNodeData* asPhysics() {
        return type == CadNodeType::Physics ? static_cast<PhysicsNodeData*>(data.get()) : nullptr;
    }
    const PhysicsNodeData* asPhysics() const {
        return type == CadNodeType::Physics ? static_cast<const PhysicsNodeData*>(data.get()) : nullptr;
    }

    // Helper to get Rail data safely
    RailNodeData* asRail() {
        return type == CadNodeType::Rail ? static_cast<RailNodeData*>(data.get()) : nullptr;
    }
    const RailNodeData* asRail() const {
        return type == CadNodeType::Rail ? static_cast<const RailNodeData*>(data.get()) : nullptr;
    }

    // Helper to get Transform data safely
    TransformNodeData* asTransform() {
        return type == CadNodeType::Transform ? static_cast<TransformNodeData*>(data.get()) : nullptr;
    }
    const TransformNodeData* asTransform() const {
        return type == CadNodeType::Transform ? static_cast<const TransformNodeData*>(data.get()) : nullptr;
    }

    // Helper to get ConnectionPoint data safely
    ConnectionPointData* asConnectionPoint() {
        return type == CadNodeType::ConnectionPoint ? static_cast<ConnectionPointData*>(data.get()) : nullptr;
    }
    const ConnectionPointData* asConnectionPoint() const {
        return type == CadNodeType::ConnectionPoint ? static_cast<const ConnectionPointData*>(data.get()) : nullptr;
    }

    // ... add more helpers for other node types as needed

    // Visibility helper
    void setVisibleRecursive(bool vis) {
        visible = vis;
        for (auto& child : children) {
            if (child) child->setVisibleRecursive(vis);
        }
    }

    // Recursively apply a transformation to this node and all children
    void applyTransform(const gp_Trsf& trsf) {
        loc = TopLoc_Location(trsf * loc.Transformation());
        for (auto& child : children) {
            if (child) child->applyTransform(trsf);
        }
    }

    QJsonObject toJson() const {
        QJsonObject obj;
        obj["name"] = QString::fromStdString(name);
        obj["color"] = QJsonArray{color.r, color.g, color.b, color.a};
        // Serialize transform as a 12-element array (3x4 matrix)
        const gp_Trsf& trsf = loc.Transformation();
        QJsonArray locArr;
        for (int row = 1; row <= 3; ++row) {
            for (int col = 1; col <= 4; ++col) {
                if (col <= 3) locArr.append(trsf.Value(row, col));
                else locArr.append(trsf.TranslationPart().GetData()[row-1]);
            }
        }
        obj["loc"] = locArr;
        obj["visible"] = visible;
        obj["excludedFromDecomposition"] = excludedFromDecomposition;
        obj["type"] = static_cast<int>(type);
        if (xcafLabelTag >= 0) obj["xcafLabelTag"] = xcafLabelTag;
        // Data
        QJsonObject dataObj;
        if (type == CadNodeType::Rail && data) dataObj = static_cast<RailNodeData*>(data.get())->toJson();
        else if (type == CadNodeType::Physics && data) dataObj = static_cast<PhysicsNodeData*>(data.get())->toJson();
        else if (type == CadNodeType::ConnectionPoint && data) dataObj = static_cast<ConnectionPointData*>(data.get())->toJson();
        obj["data"] = dataObj;
        // Children
        QJsonArray childrenArr;
        for (const auto& child : children) {
            if (child) childrenArr.append(child->toJson());
        }
        obj["children"] = childrenArr;
        return obj;
    }
    static std::shared_ptr<CadNode> fromJson(const QJsonObject& obj) {
        auto node = std::make_shared<CadNode>();
        node->name = obj["name"].toString().toStdString();
        QJsonArray colorArr = obj["color"].toArray();
        if (colorArr.size() == 4) node->color = CADNodeColor(colorArr[0].toDouble(), colorArr[1].toDouble(), colorArr[2].toDouble(), colorArr[3].toDouble());
        QJsonArray locArr = obj["loc"].toArray();
        if (locArr.size() == 12) {
            gp_Trsf trsf;
            trsf.SetValues(
                locArr[0].toDouble(), locArr[1].toDouble(), locArr[2].toDouble(), locArr[3].toDouble(),
                locArr[4].toDouble(), locArr[5].toDouble(), locArr[6].toDouble(), locArr[7].toDouble(),
                locArr[8].toDouble(), locArr[9].toDouble(), locArr[10].toDouble(), locArr[11].toDouble()
            );
            node->loc = TopLoc_Location(trsf);
        }
        node->visible = obj["visible"].toBool(true);
        node->excludedFromDecomposition = obj["excludedFromDecomposition"].toBool(false);
        node->type = static_cast<CadNodeType>(obj["type"].toInt());
        node->xcafLabelTag = obj.contains("xcafLabelTag") ? obj["xcafLabelTag"].toInt() : -1;
        QJsonObject dataObj = obj["data"].toObject();
        if (node->type == CadNodeType::Rail) node->data = RailNodeData::fromJson(dataObj);
        else if (node->type == CadNodeType::Physics) node->data = PhysicsNodeData::fromJson(dataObj);
        else if (node->type == CadNodeType::ConnectionPoint) node->data = ConnectionPointData::fromJson(dataObj);
        // Children
        QJsonArray childrenArr = obj["children"].toArray();
        for (const auto& childVal : childrenArr) {
            if (childVal.isObject()) node->children.push_back(fromJson(childVal.toObject()));
        }
        return node;
    }
};

// Utility: Count unique CadNode pointers in a tree
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


