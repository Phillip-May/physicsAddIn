#pragma once
#include <QAbstractItemModel>
#include <TDF_Label.hxx>
#include <vector>
#include <memory>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopLoc_Location.hxx>

// Node for XCAF label tree
struct XCAFLabelNode {
    TDF_Label label;
    std::vector<std::unique_ptr<XCAFLabelNode>> children;
    XCAFLabelNode(const TDF_Label& lbl) : label(lbl) {}
};

class XCAFLabelTreeModel : public QAbstractItemModel {
    Q_OBJECT
public:
    XCAFLabelTreeModel(std::unique_ptr<XCAFLabelNode> root, QObject* parent = nullptr);
    QModelIndex index(int row, int column, const QModelIndex& parent) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent) const override;
    int columnCount(const QModelIndex&) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    XCAFLabelNode* getNode(const QModelIndex& index) const;
    XCAFLabelNode* getRootNodePointer() const { return m_root.get(); }
    
    // Set tools for enhanced information display
    void setShapeTool(const Handle(XCAFDoc_ShapeTool)& shapeTool) { m_shapeTool = shapeTool; }
    void setColorTool(const Handle(XCAFDoc_ColorTool)& colorTool) { m_colorTool = colorTool; }
    
private:
    std::unique_ptr<XCAFLabelNode> m_root;
    Handle(XCAFDoc_ShapeTool) m_shapeTool;
    Handle(XCAFDoc_ColorTool) m_colorTool;
    
    // Helper methods for detailed information
    QString getShapeTypeString(const TDF_Label& label) const;
    QString getReferenceInfo(const TDF_Label& label) const;
    QString getTransformInfo(const TDF_Label& label) const;
    QString getColorInfo(const TDF_Label& label) const;
    QString getAssemblyInfo(const TDF_Label& label) const;
    QString getProductInfo(const TDF_Label& label) const;
    QString getVisibilityInfo(const TDF_Label& label) const;
    QString getLayerInfo(const TDF_Label& label) const;
    QString getStep214Info(const TDF_Label& label) const;
    QString getGeometryLocationInfo(const TDF_Label& label) const;
}; 