#include "XCAFLabelTreeModel.h"
#include <TDataStd_Name.hxx>
#include <QString>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <TDF_ChildIterator.hxx>
#include <Quantity_Color.hxx>
#include <gp_Trsf.hxx>
#include <gp_Mat.hxx>
#include <gp_XYZ.hxx>

XCAFLabelTreeModel::XCAFLabelTreeModel(std::unique_ptr<XCAFLabelNode> root, QObject* parent)
    : QAbstractItemModel(parent), m_root(std::move(root)) {}

QModelIndex XCAFLabelTreeModel::index(int row, int column, const QModelIndex& parent) const {
    auto parentNode = getNode(parent);
    if (!parentNode || row < 0 || row >= static_cast<int>(parentNode->children.size()))
        return QModelIndex();
    return createIndex(row, column, parentNode->children[row].get());
}

QModelIndex XCAFLabelTreeModel::parent(const QModelIndex& index) const {
    XCAFLabelNode* node = getNode(index);
    if (!node || node == m_root.get())
        return QModelIndex();
    // No parent pointer, so traverse from root
    std::function<QModelIndex(XCAFLabelNode*, XCAFLabelNode*)> findParent = [&](XCAFLabelNode* parent, XCAFLabelNode* child) -> QModelIndex {
        for (size_t i = 0; i < parent->children.size(); ++i) {
            if (parent->children[i].get() == child)
                return createIndex(static_cast<int>(i), 0, parent);
            QModelIndex idx = findParent(parent->children[i].get(), child);
            if (idx.isValid()) return idx;
        }
        return QModelIndex();
    };
    return findParent(m_root.get(), node);
}

int XCAFLabelTreeModel::rowCount(const QModelIndex& parent) const {
    auto parentNode = getNode(parent);
    return parentNode ? static_cast<int>(parentNode->children.size()) : 0;
}

int XCAFLabelTreeModel::columnCount(const QModelIndex&) const {
    return 1;
}

QVariant XCAFLabelTreeModel::data(const QModelIndex& index, int role) const {
    auto node = getNode(index);
    if (!node) return QVariant();
    
    if (role == Qt::DisplayRole) {
        QString baseName;
        
        // Handle special "All Shapes" node (empty label)
        if (node->label.IsNull()) {
            baseName = "All Shapes";
        } else {
            Handle(TDataStd_Name) nameAttr;
            if (node->label.FindAttribute(TDataStd_Name::GetID(), nameAttr) && !nameAttr.IsNull()) {
                baseName = QString::fromStdWString(nameAttr->Get().ToWideString());
            } else {
                baseName = QString("Label %1").arg(node->label.Tag());
            }
        }
        
        // Build comprehensive diagnostic information
        QStringList info;
        info << baseName;
        
        // Shape type information
        QString shapeInfo = getShapeTypeString(node->label);
        if (!shapeInfo.isEmpty()) {
            info << shapeInfo;
        }
        
        // Reference information
        QString refInfo = getReferenceInfo(node->label);
        if (!refInfo.isEmpty()) {
            info << refInfo;
        }
        
        // Transform information
        QString transformInfo = getTransformInfo(node->label);
        if (!transformInfo.isEmpty()) {
            info << transformInfo;
        }
        
        // Assembly information
        QString assemblyInfo = getAssemblyInfo(node->label);
        if (!assemblyInfo.isEmpty()) {
            info << assemblyInfo;
        }
        
        // Color information
        QString colorInfo = getColorInfo(node->label);
        if (!colorInfo.isEmpty()) {
            info << colorInfo;
        }
        
        // Product information (STEP 214 specific)
        QString productInfo = getProductInfo(node->label);
        if (!productInfo.isEmpty()) {
            info << productInfo;
        }
        
        // Visibility information
        QString visibilityInfo = getVisibilityInfo(node->label);
        if (!visibilityInfo.isEmpty()) {
            info << visibilityInfo;
        }
        
        // Layer information
        QString layerInfo = getLayerInfo(node->label);
        if (!layerInfo.isEmpty()) {
            info << layerInfo;
        }
        
        // STEP 214 specific information
        QString step214Info = getStep214Info(node->label);
        if (!step214Info.isEmpty()) {
            info << step214Info;
        }
        
        // Geometry location information
        QString geometryInfo = getGeometryLocationInfo(node->label);
        if (!geometryInfo.isEmpty()) {
            info << geometryInfo;
        }
        
        return info.join(" | ");
    }
    
    return QVariant();
}

Qt::ItemFlags XCAFLabelTreeModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) return Qt::NoItemFlags;
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

XCAFLabelNode* XCAFLabelTreeModel::getNode(const QModelIndex& index) const {
    if (!index.isValid()) return m_root.get();
    return static_cast<XCAFLabelNode*>(index.internalPointer());
}

QString XCAFLabelTreeModel::getShapeTypeString(const TDF_Label& label) const {
    if (m_shapeTool.IsNull() || label.IsNull()) return "";
    
    TopoDS_Shape shape = m_shapeTool->GetShape(label);
    if (shape.IsNull()) return "NO_SHAPE";
    
    QString typeStr;
    switch (shape.ShapeType()) {
        case TopAbs_COMPOUND: typeStr = "COMPOUND"; break;
        case TopAbs_COMPSOLID: typeStr = "COMPSOLID"; break;
        case TopAbs_SOLID: typeStr = "SOLID"; break;
        case TopAbs_SHELL: typeStr = "SHELL"; break;
        case TopAbs_FACE: typeStr = "FACE"; break;
        case TopAbs_WIRE: typeStr = "WIRE"; break;
        case TopAbs_EDGE: typeStr = "EDGE"; break;
        case TopAbs_VERTEX: typeStr = "VERTEX"; break;
        case TopAbs_SHAPE: typeStr = "SHAPE"; break;
        default: typeStr = "UNKNOWN"; break;
    }
    
    // Count sub-shapes for compounds
    if (shape.ShapeType() == TopAbs_COMPOUND) {
        int solidCount = 0, shellCount = 0, faceCount = 0, edgeCount = 0;
        for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next()) solidCount++;
        for (TopExp_Explorer exp(shape, TopAbs_SHELL); exp.More(); exp.Next()) shellCount++;
        for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) faceCount++;
        for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next()) edgeCount++;
        
        QStringList counts;
        if (solidCount > 0) counts << QString("S%1").arg(solidCount);
        if (shellCount > 0) counts << QString("Sh%1").arg(shellCount);
        if (faceCount > 0) counts << QString("F%1").arg(faceCount);
        if (edgeCount > 0) counts << QString("E%1").arg(edgeCount);
        
        if (!counts.isEmpty()) {
            typeStr += QString(" [%1]").arg(counts.join(","));
        }
    }
    
    return QString("Type: %1").arg(typeStr);
}

QString XCAFLabelTreeModel::getReferenceInfo(const TDF_Label& label) const {
    if (m_shapeTool.IsNull() || label.IsNull()) return "";
    
    if (m_shapeTool->IsReference(label)) {
        TDF_Label refLabel;
        if (m_shapeTool->GetReferredShape(label, refLabel)) {
            return QString("REF->%1").arg(refLabel.Tag());
        } else {
            return "REF->INVALID";
        }
    }
    
    return "";
}

QString XCAFLabelTreeModel::getTransformInfo(const TDF_Label& label) const {
    if (m_shapeTool.IsNull() || label.IsNull()) return "";
    
    TopLoc_Location loc = m_shapeTool->GetLocation(label);
    if (loc.IsIdentity()) return "";
    
    const gp_Trsf& trsf = loc.Transformation();
    const gp_Mat& mat = trsf.VectorialPart();
    const gp_XYZ& trans = trsf.TranslationPart();
    
    // Check if it's just a translation (identity matrix)
    if (mat.Value(1,1) == 1.0 && mat.Value(1,2) == 0.0 && mat.Value(1,3) == 0.0 &&
        mat.Value(2,1) == 0.0 && mat.Value(2,2) == 1.0 && mat.Value(2,3) == 0.0 &&
        mat.Value(3,1) == 0.0 && mat.Value(3,2) == 0.0 && mat.Value(3,3) == 1.0) {
        return QString("T[%1,%2,%3]").arg(trans.X(), 0, 'f', 2).arg(trans.Y(), 0, 'f', 2).arg(trans.Z(), 0, 'f', 2);
    }
    
    // Check if it's just a rotation (no translation)
    if (trans.X() == 0 && trans.Y() == 0 && trans.Z() == 0) {
        return "ROTATION";
    }
    
    return "TRANSFORM";
}

QString XCAFLabelTreeModel::getColorInfo(const TDF_Label& label) const {
    if (m_colorTool.IsNull() || label.IsNull()) return "";
    
    Quantity_Color color;
    if (m_colorTool->GetColor(label, XCAFDoc_ColorSurf, color) ||
        m_colorTool->GetColor(label, XCAFDoc_ColorGen, color) ||
        m_colorTool->GetColor(label, XCAFDoc_ColorCurv, color)) {
        return QString("RGB(%1,%2,%3)").arg(color.Red()).arg(color.Green()).arg(color.Blue());
    }
    
    return "";
}

QString XCAFLabelTreeModel::getAssemblyInfo(const TDF_Label& label) const {
    if (m_shapeTool.IsNull() || label.IsNull()) return "";
    
    TopoDS_Shape shape = m_shapeTool->GetShape(label);
    if (shape.IsNull() || shape.ShapeType() != TopAbs_COMPOUND) return "";
    
    // Check if this compound has multiple direct shape children (assembly indicator)
    int shapeCount = 0;
    int subAssemblyCount = 0;
    
    if (!label.IsNull()) {
        for (TDF_ChildIterator it(label); it.More(); it.Next()) {
            TDF_Label childLabel = it.Value();
            if (!childLabel.IsNull()) {
                TopoDS_Shape childShape = m_shapeTool->GetShape(childLabel);
                if (!childShape.IsNull()) {
                    shapeCount++;
                    
                    // Check if this child is itself a sub-assembly
                    if (childShape.ShapeType() == TopAbs_COMPOUND) {
                        int childShapeCount = 0;
                        for (TDF_ChildIterator childIt(childLabel); childIt.More(); childIt.Next()) {
                            TDF_Label grandChildLabel = childIt.Value();
                            if (!grandChildLabel.IsNull()) {
                                TopoDS_Shape grandChildShape = m_shapeTool->GetShape(grandChildLabel);
                                if (!grandChildShape.IsNull()) {
                                    childShapeCount++;
                                    if (childShapeCount > 1) {
                                        subAssemblyCount++;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    
                    if (shapeCount > 1) {
                        QString result = QString("ASSEMBLY(%1 parts)").arg(shapeCount);
                        if (subAssemblyCount > 0) {
                            result += QString(" +%1 sub-assemblies").arg(subAssemblyCount);
                        }
                        return result;
                    }
                }
            }
        }
    }
    
    return "";
}

QString XCAFLabelTreeModel::getProductInfo(const TDF_Label& label) const {
    // STEP 214 specific product information
    // This would typically involve STEP 214 specific entities like PRODUCT_DEFINITION_CONTEXT
    // For now, we'll look for common STEP 214 patterns
    
    // Check if this label has children that might indicate product structure
    int childCount = 0;
    int shapeChildCount = 0;
    int refChildCount = 0;
    
    if (!label.IsNull()) {
        for (TDF_ChildIterator it(label); it.More(); it.Next()) {
            childCount++;
            TDF_Label childLabel = it.Value();
            
            if (m_shapeTool.IsNull() == false && !childLabel.IsNull()) {
                TopoDS_Shape childShape = m_shapeTool->GetShape(childLabel);
                if (!childShape.IsNull()) {
                    shapeChildCount++;
                }
                
                if (m_shapeTool->IsReference(childLabel)) {
                    refChildCount++;
                }
            }
        }
    }
    
    QStringList info;
    if (childCount > 0) {
        info << QString("CHILDREN:%1").arg(childCount);
    }
    if (shapeChildCount > 0) {
        info << QString("SHAPES:%1").arg(shapeChildCount);
    }
    if (refChildCount > 0) {
        info << QString("REFS:%1").arg(refChildCount);
    }
    
    if (!info.isEmpty()) {
        return QString("STEP214[%1]").arg(info.join(","));
    }
    
    return "";
}

QString XCAFLabelTreeModel::getVisibilityInfo(const TDF_Label& label) const {
    // Check visibility attributes
    // This is STEP 214 specific information
    return "";
}

QString XCAFLabelTreeModel::getLayerInfo(const TDF_Label& label) const {
    // Check layer information
    // This would involve XCAFDoc_LayerTool
    return "";
}

QString XCAFLabelTreeModel::getStep214Info(const TDF_Label& label) const {
    // STEP 214 specific diagnostic information
    QStringList info;
    
    // Check for common STEP 214 patterns
    if (m_shapeTool.IsNull() == false && !label.IsNull()) {
        TopoDS_Shape shape = m_shapeTool->GetShape(label);
        
        // Check if this is a compound with no direct shape (typical STEP 214 assembly pattern)
        if (!shape.IsNull() && shape.ShapeType() == TopAbs_COMPOUND) {
            bool hasDirectShape = false;
            int childCount = 0;
            int refChildCount = 0;
            int emptyChildCount = 0;
            
            if (!label.IsNull()) {
                for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                    childCount++;
                    TDF_Label childLabel = it.Value();
                    if (!childLabel.IsNull()) {
                        TopoDS_Shape childShape = m_shapeTool->GetShape(childLabel);
                        if (!childShape.IsNull()) {
                            hasDirectShape = true;
                        } else {
                            emptyChildCount++;
                        }
                        
                        // Check if this child is a reference
                        if (m_shapeTool->IsReference(childLabel)) {
                            refChildCount++;
                        }
                    }
                }
            }
            
            if (!hasDirectShape) {
                if (refChildCount > 0) {
                    info << QString("EMPTY_ASSEMBLY_WITH_REFS(%1 refs)").arg(refChildCount);
                } else if (emptyChildCount > 0) {
                    info << QString("EMPTY_ASSEMBLY_WITH_EMPTY_CHILDREN(%1 empty)").arg(emptyChildCount);
                } else if (childCount == 0) {
                    info << "EMPTY_ASSEMBLY_NO_CHILDREN";
                } else {
                    info << "ASSEMBLY_ONLY";
                }
            }
        }
        
        // Check for reference chains (common in STEP 214)
        if (m_shapeTool->IsReference(label)) {
            TDF_Label refLabel;
            if (m_shapeTool->GetReferredShape(label, refLabel)) {
                // Check if the referred shape is also a reference
                if (!refLabel.IsNull() && m_shapeTool->IsReference(refLabel)) {
                    info << "REF_CHAIN";
                }
                
                // Check if the referred shape has geometry
                if (!refLabel.IsNull()) {
                    TopoDS_Shape refShape = m_shapeTool->GetShape(refLabel);
                    if (refShape.IsNull()) {
                        info << "REF_TO_EMPTY";
                    }
                }
            }
        }
    }
    
    // Check for empty labels with children (STEP 214 product structure)
    if (label.IsNull() == false) {
        int childCount = 0;
        int emptyChildCount = 0;
        
        // Double-check label is not null before creating iterator
        if (!label.IsNull()) {
            for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                childCount++;
                TDF_Label childLabel = it.Value();
                
                if (m_shapeTool.IsNull() == false && !childLabel.IsNull()) {
                    TopoDS_Shape childShape = m_shapeTool->GetShape(childLabel);
                    if (childShape.IsNull()) {
                        emptyChildCount++;
                    }
                }
            }
        }
        
        if (emptyChildCount > 0 && emptyChildCount == childCount) {
            info << "PRODUCT_STRUCTURE";
        }
    }
    
    if (!info.isEmpty()) {
        return QString("STEP214_PATTERN[%1]").arg(info.join(","));
    }
    
    return "";
} 

QString XCAFLabelTreeModel::getGeometryLocationInfo(const TDF_Label& label) const {
    if (m_shapeTool.IsNull() || label.IsNull()) return "";
    
    TopoDS_Shape shape = m_shapeTool->GetShape(label);
    
    // If this label has no shape, try to find where the geometry might be
    if (shape.IsNull()) {
        QStringList locations;
        
        // Check if this is a reference
        if (m_shapeTool->IsReference(label)) {
            TDF_Label refLabel;
            if (m_shapeTool->GetReferredShape(label, refLabel)) {
                locations << QString("REF->%1").arg(refLabel.Tag());
                
                // Check if the referred shape also has no geometry
                TopoDS_Shape refShape = m_shapeTool->GetShape(refLabel);
                if (refShape.IsNull()) {
                    locations << "REF_TO_EMPTY";
                }
            }
        }
        
        // Check children for references
        if (!label.IsNull()) {
            int refChildren = 0;
            for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                TDF_Label childLabel = it.Value();
                if (!childLabel.IsNull() && m_shapeTool->IsReference(childLabel)) {
                    refChildren++;
                }
            }
            if (refChildren > 0) {
                locations << QString("HAS_%1_REF_CHILDREN").arg(refChildren);
            }
        }
        
        if (!locations.isEmpty()) {
            return QString("GEOMETRY_LOCATION[%1]").arg(locations.join(","));
        }
    }
    
    return "";
} 