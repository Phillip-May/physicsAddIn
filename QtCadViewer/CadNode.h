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
    Unknown = 0, // Default type, must be explicitly set
    XCAF,
    Custom,
    Rail,
    Turnable,
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

// 5. The generic CadNode struct
struct CadNode {
    std::string name;
    CADNodeColor color;
    TopLoc_Location loc;
    std::vector<std::shared_ptr<CadNode>> children;
    bool visible = true;

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

    // ... add more helpers for other node types as needed

    // Visibility helper
    void setVisibleRecursive(bool vis) {
        visible = vis;
        for (auto& child : children) {
            if (child) child->setVisibleRecursive(vis);
        }
    }
};

#endif // QTCADVIEWER_CADNODE_H


