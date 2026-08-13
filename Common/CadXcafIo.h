#pragma once

#include "CadNode.h"
#include "XCAFLabelTreeModel.h"

#include <QString>
#include <TDocStd_Document.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <memory>
#include <vector>

std::unique_ptr<XCAFLabelNode> buildLabelTreeWithReferences(
    const TDF_Label& label,
    const Handle(XCAFDoc_ShapeTool)& shapeTool = nullptr);

std::unique_ptr<XCAFLabelNode> buildDocumentLabelTree(
    const Handle(TDocStd_Document)& doc,
    const Handle(XCAFDoc_ShapeTool)& shapeTool);

uint qHash(const TDF_Label& label, uint seed = 0);
QString shapeTypeToString(TopAbs_ShapeEnum type);
CADNodeColor getEffectiveFaceColor(const TopoDS_Face& face,
                                   const Handle(XCAFDoc_ShapeTool)& shapeTool,
                                   const Handle(XCAFDoc_ColorTool)& colorTool);
CADNodeColor get_label_color(const TDF_Label& label,
                             const Handle(XCAFDoc_ColorTool)& colorTool,
                             const CADNodeColor& parentColor);
CADNodeColor get_shape_color(const TopoDS_Shape& shape,
                             const TDF_Label& label,
                             const Handle(XCAFDoc_ShapeTool)& shapeTool,
                             const Handle(XCAFDoc_ColorTool)& colorTool,
                             const CADNodeColor& parentColor);
bool hasSubAssembliesRecursive(const TDF_Label& label,
                               const Handle(XCAFDoc_ShapeTool)& shapeTool);
QString makeTransformString(const TopLoc_Location& loc);
bool isAssembly(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool);
std::vector<int> getLabelPath(const TDF_Label& label, const Handle(TDocStd_Document)& doc);
TDF_Label findLabelByPath(const Handle(TDocStd_Document)& doc, const std::vector<int>& path);
void debugPrintXCAFRelinkInfo(const CadNode* node,
                             const Handle(TDocStd_Document)& doc,
                             int depth = 0);

std::shared_ptr<CadNode> build_tree_xcaf(
    const TDF_Label& label,
    const Handle(XCAFDoc_ShapeTool)& shapeTool,
    const Handle(XCAFDoc_ColorTool)& colorTool,
    const CADNodeColor& parentColor,
    const TopLoc_Location& parentLoc,
    const Handle(TDocStd_Document)& doc);

bool saveXCAFToSTEP(const Handle(TDocStd_Document)& doc, const QString& filename);
bool saveXCAFToBinary(const Handle(TDocStd_Document)& doc, const QString& filename);
void compareFileSizes(const QString& baseName);

bool loadStepFile(const QString& stepFile,
                  Handle(TDocStd_Document)& doc,
                  std::shared_ptr<CadNode>& cadRoot,
                  std::unique_ptr<XCAFLabelNode>& labelRoot,
                  Handle(XCAFDoc_ShapeTool)& shapeTool,
                  Handle(XCAFDoc_ColorTool)& colorTool);

bool loadFromJsonAndBin(const QString& packageFile,
                        Handle(TDocStd_Document)& doc,
                        std::shared_ptr<CadNode>& cadRoot,
                        std::unique_ptr<XCAFLabelNode>& labelRoot,
                        Handle(XCAFDoc_ShapeTool)& shapeTool,
                        Handle(XCAFDoc_ColorTool)& colorTool,
                        std::shared_ptr<CadNode>& customModelRootContainer,
                        std::shared_ptr<CadNode>& customModelRoot);

TDF_Label findLabelForShape(const Handle(XCAFDoc_ShapeTool)& shapeTool,
                            const TDF_Label& label,
                            const TopoDS_Shape& targetShape);
TDF_Label findLabelForFaceOrEdge(const Handle(XCAFDoc_ShapeTool)& shapeTool,
                                 const TDF_Label& label,
                                 const TopoDS_Shape& targetShape);
TopoDS_Shape extrudeFace(const TopoDS_Face& face, double distance);
