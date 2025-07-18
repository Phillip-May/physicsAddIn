#ifndef QTCADVIEWER_CADNODE_H
#define QTCADVIEWER_CADNODE_H
#include <vector>
#include <string>
#include <TopLoc_Location.hxx>
#include <TopoDS_Face.hxx>
#include <QtGui/QColor>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Edge.hxx>

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

// Tree node for XCAF structure
struct TreeNode {
    std::string name;
    CADNodeColor color;
    TopLoc_Location loc;
    std::vector<std::unique_ptr<TreeNode>> children;
    TopAbs_ShapeEnum type;
    
    // Union data for specific shape types
    CADNodeData shapeData;
    bool visible = true; // Add visibility flag, default true
    
    // Constructor
    TreeNode() : type(TopAbs_SHAPE), shapeData() {}
    
    // Destructor
    ~TreeNode() {
        // The union will be automatically cleaned up
    }
    
    // Helper methods to safely access shape data
    bool hasFace() const {
        return type == TopAbs_FACE;
    }
    
    bool hasEdge() const {
        return type == TopAbs_EDGE;
    }
    
    TopoDS_Face getFace() const {
        if (hasFace()) {
            return shapeData.face;
        }
        return TopoDS_Face(); // Return null face
    }
    
    TopoDS_Edge getEdge() const {
        if (hasEdge()) {
            return shapeData.edge;
        }
        return TopoDS_Edge(); // Return null edge
    }
    
    void setFace(const TopoDS_Face& face) {
        type = TopAbs_FACE;
        shapeData.face = face;
    }
    
    void setEdge(const TopoDS_Edge& edge) {
        type = TopAbs_EDGE;
        shapeData.edge = edge;
    }
    
    // Clear shape data
    void clearShapeData() {
        type = TopAbs_SHAPE;
        shapeData = CADNodeData();
    }

    void setVisibleRecursive(bool vis) {
        visible = vis;
        for (auto& child : children) {
            if (child) child->setVisibleRecursive(vis);
        }
    }
};

#endif // QTCADVIEWER_CADNODE_H


