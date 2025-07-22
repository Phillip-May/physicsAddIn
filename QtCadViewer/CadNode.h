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
};

// 4. Custom node data (example)
struct CustomNodeData : public CadNodeDataBase {
    // Add custom fields here
    int customProperty = 0;
    // ...
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
};

// 4c. Rail node data
struct RailNodeData : public CadNodeDataBase {
    QVector3D axisOfTravel{1,0,0};
    QVector3D buildJointPosition{0,0,0};
    double travelLength = 0.0;
    int numSegments = 1;
    QString jsonString; // For custom JSON editing
};

// 4d. Transform node data (for future extensibility)
struct TransformNodeData : public CadNodeDataBase {
    // Currently empty, but can be extended for animation, etc.
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


