// Qt wiring between the tree views and the GL viewers: selection sync, the context
// menus, and tab setup. The only part of this group that touches a widget.
#include "CadViewerWiring.h"

#include "CadDecomposition.h"
#include "CadNodePackage.h"
#include "CadNodeQtAdapter.h"
#include "CadNodeOps.h"
#include "CadOpenGLWidget.h"
#include "CadXcafIo.h"
#include "CustomModelTreeModel.h"
#include "RailJsonEditorDialog.h"
#include "RobotRuntime.h"
#include "SimulationManager.h"
#include "XCAFLabelTreeModel.h"

#include <QAbstractItemView>
#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QItemSelection>
#include <QItemSelectionModel>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QModelIndex>
#include <QPushButton>
#include <QSet>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextEdit>
#include <QTreeView>
#include <QVBoxLayout>
#include <TopExp_Explorer.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <gp_Ax1.hxx>

#include <chrono>
#include <functional>
#include <set>
#include <unordered_map>

template <typename ModelType>
void connectTreeAndViewer(QTreeView* tree, CadOpenGLWidget* viewer, ModelType* model) {
    auto accumulateTransform = [](CadNode* node, std::function<CadNode*(CadNode*)> getParent) -> TopLoc_Location {
        TopLoc_Location acc;
        std::vector<CadNode*> ancestry;
        CadNode* cur = node;
        while (cur) {
            ancestry.push_back(cur);
            cur = getParent(cur);
        }
        for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
            acc = acc * (*it)->loc;
        }
        return acc;
    };
    // Tree -> Viewer
    QObject::connect(tree->selectionModel(), &QItemSelectionModel::selectionChanged, viewer, [=](const QItemSelection &, const QItemSelection &) {
        viewer->clearSelection();
        QModelIndexList selectedIndexes = tree->selectionModel()->selectedIndexes();
        QSet<CadNode*> allNodesToSelect;
        for (const QModelIndex& index : selectedIndexes) {
            CadNode* node = const_cast<CadNode*>(model->getNode(index));
            if (node) {
                allNodesToSelect.insert(node);
                // Optionally add descendants
                std::function<void(CadNode*)> addDescendants = [&](CadNode* currentNode) {
                    for (const auto& child : currentNode->children) {
                        if (child) {
                            allNodesToSelect.insert(child.get());
                            addDescendants(child.get());
                        }
                    }
                };
                addDescendants(node);
            }
        }
        for (CadNode* node : allNodesToSelect) {
            TopLoc_Location accLoc = node->globalLoc;
            viewer->addToSelection(node, accLoc);
        }
    });

    // Viewer -> Tree
    QObject::connect(viewer, &CadOpenGLWidget::facePicked, tree, [=](CadNode* node) {
        QModelIndex idx = model->indexForNode(node);
        if (idx.isValid()) {
            QModelIndex parentIdx = idx.parent();
            while (parentIdx.isValid()) {
                tree->expand(parentIdx);
                parentIdx = parentIdx.parent();
            }
            // Multi-selection: add to selection instead of replacing
            QItemSelectionModel* selectionModel = tree->selectionModel();
            QItemSelection currentSelection = selectionModel->selection();
            currentSelection.select(idx, idx);
            selectionModel->select(currentSelection, QItemSelectionModel::Select);
        }
    });
}

template void connectTreeAndViewer(QTreeView* tree, CadOpenGLWidget* viewer, CustomModelTreeModel* model);

void setupComprehensiveContextMenu(QTreeView* treeView,
                                   QAbstractItemModel* model,
                                   const Handle(TDocStd_Document)& doc,
                                   CadOpenGLWidget* openGLViewer,
                                   const CadViewerWorkspace* workspace) {
    treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(treeView, &QTreeView::customContextMenuRequested, treeView, [=](const QPoint& pos) {
        QModelIndex idx = treeView->indexAt(pos);
        if (!idx.isValid()) return;

        QMenu menu;

        // Handle CadNode type (CustomModelTreeModel)
        CustomModelTreeModel* customModel = qobject_cast<CustomModelTreeModel*>(model);
        if (customModel) {
            CadNode* node = const_cast<CadNode*>(customModel->getNode(idx));
            if (!node) return;

            // --- Save/Load Node to/from JSON ---
            QAction* saveNodeAction = menu.addAction("Save Node to JSON...");
            QObject::connect(saveNodeAction, &QAction::triggered, treeView, [=]() {
                QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Node as JSON", "", "JSON Files (*.json)");
                if (fileName.isEmpty()) return;
                if (!fileName.endsWith(".json")) fileName += ".json";
                std::shared_ptr<CadNode> nodeToSave = std::make_shared<CadNode>(*node);
                // CadNode serialises through nlohmann now; the rest of this file's JSON is
                // unrelated Qt config and stays as it is.
                const std::string json = nodeToSave->toJson().dump(4);
                QFile file(fileName);
                if (!file.open(QIODevice::WriteOnly)) {
                    QMessageBox::warning(nullptr, "Error", "Failed to open file for writing: " + fileName);
                    return;
                }
                file.write(QByteArray::fromStdString(json));
                file.close();
                QMessageBox::information(nullptr, "Success", "Node saved to:\n" + fileName);
            });

            QAction* loadNodeAction = menu.addAction("Load Node from JSON/Package...");
            QObject::connect(loadNodeAction, &QAction::triggered, treeView, [=]() {
                QString fileName = QFileDialog::getOpenFileName(nullptr, "Load Node from JSON or Package", "", "Robot Packages (*.zip *.json);;JSON Files (*.json);;Zip Files (*.zip)");
                if (fileName.isEmpty()) return;
                std::string packageError;
                std::shared_ptr<CadNode> loadedNode =
                    loadCadNodePackage(fileName.toStdString(), &packageError);
                if (!loadedNode) {
                    QMessageBox::warning(nullptr, "Error", packageError.empty()
                        ? QStringLiteral("Failed to load node package.")
                        : QString::fromStdString(packageError));
                    return;
                }

                // Update node properties
                node->children.clear();
                for (auto& child : loadedNode->children) {
                    node->children.push_back(child);
                }
                node->name = loadedNode->name;
                node->type = loadedNode->type;
                node->color = loadedNode->color;
                node->loc = loadedNode->loc;
                node->visible = loadedNode->visible;
                node->excludedFromDecomposition = loadedNode->excludedFromDecomposition;
                node->data = loadedNode->data;
                customModel->dataChanged(idx, idx);
                setParentPointersRecursive(node);
                CadNode::resolveRobotReferences(node->root());

                // Relink XCAF nodes
                std::function<void(CadNode*)> relinkXCAF = [&](CadNode* n) {
                    if (!n) return;
                    if (n->type == CadNodeType::XCAF) {
                        XCAFNodeData* xData = n->asXCAF();
                        if (xData && !xData->labelPath.empty()) {
                            TDF_Label label = findLabelByPath(doc, xData->labelPath);
                            if (!label.IsNull()) {
                                auto shapeToolLocal = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
                                TopoDS_Shape occShape = shapeToolLocal->GetShape(label);
                                if (xData->type == TopAbs_FACE || xData->type == TopAbs_EDGE) {
                                    if (xData->shapeIndex >= 0) {
                                        int idx = 0;
                                        TopoDS_Shape foundShape;
                                        for (TopExp_Explorer exp(occShape, xData->type); exp.More(); exp.Next(), ++idx) {
                                            if (idx == xData->shapeIndex) {
                                                foundShape = exp.Current();
                                                break;
                                            }
                                        }
                                        xData->shape = foundShape;
                                    } else {
                                        xData->shape = occShape;
                                    }
                                } else {
                                    xData->shape = occShape;
                                }
                            } else {
                                xData->shape.Nullify();
                            }
                        }
                    }
                    for (auto& child : n->children) {
                        relinkXCAF(child.get());
                    }
                };
                relinkXCAF(node);
                if (openGLViewer) {
                    openGLViewer->markCacheDirty();
                    openGLViewer->update();
                }
            });

            // --- Edit Node Location/Transform ---
            QAction* editLocationAction = menu.addAction("Edit Location...");
            QObject::connect(editLocationAction, &QAction::triggered, treeView, [=]() {
                QDialog dialog(treeView);
                dialog.setWindowTitle("Edit Node Location - " + QString::fromStdString(node->name));
                dialog.setModal(true);
                dialog.resize(400, 300);

                QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);

                // Get current transform values
                const gp_Trsf& trsf = node->loc.Transformation();
                const gp_Mat& mat = trsf.VectorialPart();
                const gp_XYZ& trans = trsf.TranslationPart();

                // Translation group
                QGroupBox* transGroup = new QGroupBox("Translation");
                QGridLayout* transLayout = new QGridLayout(transGroup);

                QDoubleSpinBox* xTrans = new QDoubleSpinBox;
                QDoubleSpinBox* yTrans = new QDoubleSpinBox;
                QDoubleSpinBox* zTrans = new QDoubleSpinBox;

                xTrans->setRange(-10000, 10000);
                yTrans->setRange(-10000, 10000);
                zTrans->setRange(-10000, 10000);
                xTrans->setDecimals(3);
                yTrans->setDecimals(3);
                zTrans->setDecimals(3);
                xTrans->setValue(trans.X());
                yTrans->setValue(trans.Y());
                zTrans->setValue(trans.Z());

                transLayout->addWidget(new QLabel("X:"), 0, 0);
                transLayout->addWidget(xTrans, 0, 1);
                transLayout->addWidget(new QLabel("Y:"), 1, 0);
                transLayout->addWidget(yTrans, 1, 1);
                transLayout->addWidget(new QLabel("Z:"), 2, 0);
                transLayout->addWidget(zTrans, 2, 1);

                // Rotation group (Euler angles)
                QGroupBox* rotGroup = new QGroupBox("Rotation (Euler angles in degrees)");
                QGridLayout* rotLayout = new QGridLayout(rotGroup);

                QDoubleSpinBox* xRot = new QDoubleSpinBox;
                QDoubleSpinBox* yRot = new QDoubleSpinBox;
                QDoubleSpinBox* zRot = new QDoubleSpinBox;

                xRot->setRange(-360, 360);
                yRot->setRange(-360, 360);
                zRot->setRange(-360, 360);
                xRot->setDecimals(1);
                yRot->setDecimals(1);
                zRot->setDecimals(1);

                // Convert rotation matrix to Euler angles (simplified)
                // This is a basic conversion - for more complex cases you might want a quaternion editor
                double rx = atan2(mat.Value(3,2), mat.Value(3,3)) * 180.0 / M_PI;
                double ry = atan2(-mat.Value(3,1), sqrt(mat.Value(3,2)*mat.Value(3,2) + mat.Value(3,3)*mat.Value(3,3))) * 180.0 / M_PI;
                double rz = atan2(mat.Value(2,1), mat.Value(1,1)) * 180.0 / M_PI;

                xRot->setValue(rx);
                yRot->setValue(ry);
                zRot->setValue(rz);

                rotLayout->addWidget(new QLabel("X (Pitch):"), 0, 0);
                rotLayout->addWidget(xRot, 0, 1);
                rotLayout->addWidget(new QLabel("Y (Yaw):"), 1, 0);
                rotLayout->addWidget(yRot, 1, 1);
                rotLayout->addWidget(new QLabel("Z (Roll):"), 2, 0);
                rotLayout->addWidget(zRot, 2, 1);

                // Scale group
                QGroupBox* scaleGroup = new QGroupBox("Scale");
                QGridLayout* scaleLayout = new QGridLayout(scaleGroup);

                QDoubleSpinBox* xScale = new QDoubleSpinBox;
                QDoubleSpinBox* yScale = new QDoubleSpinBox;
                QDoubleSpinBox* zScale = new QDoubleSpinBox;

                xScale->setRange(0.001, 1000);
                yScale->setRange(0.001, 1000);
                zScale->setRange(0.001, 1000);
                xScale->setDecimals(3);
                yScale->setDecimals(3);
                zScale->setDecimals(3);

                // Extract scale from matrix (simplified)
                double sx = sqrt(mat.Value(1,1)*mat.Value(1,1) + mat.Value(2,1)*mat.Value(2,1) + mat.Value(3,1)*mat.Value(3,1));
                double sy = sqrt(mat.Value(1,2)*mat.Value(1,2) + mat.Value(2,2)*mat.Value(2,2) + mat.Value(3,2)*mat.Value(3,2));
                double sz = sqrt(mat.Value(1,3)*mat.Value(1,3) + mat.Value(2,3)*mat.Value(2,3) + mat.Value(3,3)*mat.Value(3,3));

                xScale->setValue(sx);
                yScale->setValue(sy);
                zScale->setValue(sz);

                scaleLayout->addWidget(new QLabel("X:"), 0, 0);
                scaleLayout->addWidget(xScale, 0, 1);
                scaleLayout->addWidget(new QLabel("Y:"), 1, 0);
                scaleLayout->addWidget(yScale, 1, 1);
                scaleLayout->addWidget(new QLabel("Z:"), 2, 0);
                scaleLayout->addWidget(zScale, 2, 1);

                // Buttons
                QHBoxLayout* buttonLayout = new QHBoxLayout;
                QPushButton* okButton = new QPushButton("OK");
                QPushButton* cancelButton = new QPushButton("Cancel");
                QPushButton* resetButton = new QPushButton("Reset to Identity");

                buttonLayout->addWidget(resetButton);
                buttonLayout->addStretch();
                buttonLayout->addWidget(cancelButton);
                buttonLayout->addWidget(okButton);

                // Add all widgets to main layout
                mainLayout->addWidget(transGroup);
                mainLayout->addWidget(rotGroup);
                mainLayout->addWidget(scaleGroup);
                mainLayout->addLayout(buttonLayout);

                // Connect reset button
                QObject::connect(resetButton, &QPushButton::clicked, [&]() {
                    xTrans->setValue(0.0);
                    yTrans->setValue(0.0);
                    zTrans->setValue(0.0);
                    xRot->setValue(0.0);
                    yRot->setValue(0.0);
                    zRot->setValue(0.0);
                    xScale->setValue(1.0);
                    yScale->setValue(1.0);
                    zScale->setValue(1.0);
                });

                // Connect OK/Cancel buttons
                QObject::connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
                QObject::connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

                if (dialog.exec() == QDialog::Accepted) {
                    // Create new transform from the edited values
                    gp_Trsf newTrsf;

                    // Set translation
                    newTrsf.SetTranslationPart(gp_XYZ(xTrans->value(), yTrans->value(), zTrans->value()));

                    // Set rotation (convert Euler angles to rotation matrix)
                    double rxRad = xRot->value() * M_PI / 180.0;
                    double ryRad = yRot->value() * M_PI / 180.0;
                    double rzRad = zRot->value() * M_PI / 180.0;

                    // Create rotation transformations for each axis
                    gp_Trsf rotX, rotY, rotZ;
                    rotX.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(1,0,0)), rxRad);
                    rotY.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,1,0)), ryRad);
                    rotZ.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), rzRad);

                    // Combined rotation
                    gp_Trsf combinedRot = rotZ * rotY * rotX;

                    // Create translation transformation
                    gp_Trsf transTrsf;
                    transTrsf.SetTranslation(gp_Vec(xTrans->value(), yTrans->value(), zTrans->value()));

                    // Combine transformations: translation * rotation
                    newTrsf = transTrsf * combinedRot;

                    // Apply the new transform
                    node->loc = TopLoc_Location(newTrsf);
                    customModel->dataChanged(idx, idx);
                    if (openGLViewer) {
                        openGLViewer->markCacheDirty();
                        openGLViewer->update();
                    }
                }
            });

            // --- VHACD and CoACD generation actions ---
            QAction* vhacdAction = menu.addAction("Generate Collision Mesh (VHACD)");
            QObject::connect(vhacdAction, &QAction::triggered, treeView, [=]() {
                // Default parameters as JSON
                QJsonObject defaultParams{
                    {"maxConvexHulls", 256},
                    {"resolution", 500000},
                    {"minimumVolumePercentErrorAllowed", 0.000001},
                    {"maxRecursionDepth", 15},
                    {"shrinkWrap", true},
                    {"fillMode", 0}, // 0 = FLOOD_FILL
                    {"maxNumVerticesPerCH", 256},
                    {"asyncACD", true},
                    {"minEdgeLength", 0.0005},
                    {"findBestPlane", false}
                };
                QJsonDocument doc(defaultParams);
                RailJsonEditorDialog dlg(QString::fromUtf8(doc.toJson(QJsonDocument::Indented)), treeView);
                if (dlg.exec() == QDialog::Accepted) {
                    QJsonDocument userDoc = QJsonDocument::fromJson(dlg.getJsonString().toUtf8());
                    if (!userDoc.isObject()) {
                        QMessageBox::warning(nullptr, "VHACD", "Invalid JSON for VHACD parameters.");
                        return;
                    }
                    QJsonObject obj = userDoc.object();
                    generateVHACDStub(QString::fromStdString(node->name), obj, node);
                    if (openGLViewer) {
                        openGLViewer->markCacheDirty();
                    }
                    QMessageBox::information(nullptr, "VHACD", "VHACD collision mesh generation complete.");
                }
            });

            QAction* coacdAction = menu.addAction("Generate Collision Mesh (CoACD)");
            QObject::connect(coacdAction, &QAction::triggered, treeView, [=]() {
                // Default parameters as JSON
                QJsonObject defaultParams{
                    {"concavity", 0.0025},
                    {"alpha", 0.05},
                    {"beta", 0.05},
                    {"maxConvexHull", 16},
                    {"preprocess", "voxel"},
                    {"prepRes", 64},
                    {"sampleRes", 10000},
                    {"mctsNodes", 100},
                    {"mctsIter", 100},
                    {"mctsDepth", 10},
                    {"pca", true},
                    {"merge", true},
                    {"decimate", true},
                    {"maxChVertex", 64},
                    {"extrude", false},
                    {"extrudeMargin", 0.0},
                    {"apxMode", "fast"},
                    {"seed", 42}
                };
                QJsonDocument doc(defaultParams);
                RailJsonEditorDialog dlg(QString::fromUtf8(doc.toJson(QJsonDocument::Indented)), treeView);
                if (dlg.exec() == QDialog::Accepted) {
                    QJsonDocument userDoc = QJsonDocument::fromJson(dlg.getJsonString().toUtf8());
                    if (!userDoc.isObject()) {
                        QMessageBox::warning(nullptr, "CoACD", "Invalid JSON for CoACD parameters.");
                        return;
                    }
                    QJsonObject obj = userDoc.object();
                    double concavity = obj.value("concavity").toDouble(0.0025);
                    double alpha = obj.value("alpha").toDouble(0.05);
                    double beta = obj.value("beta").toDouble(0.05);
                    int maxConvexHull = obj.value("maxConvexHull").toInt(16);
                    std::string preprocess = obj.value("preprocess").toString("voxel").toStdString();
                    int prepRes = obj.value("prepRes").toInt(64);
                    int sampleRes = obj.value("sampleRes").toInt(10000);
                    int mctsNodes = obj.value("mctsNodes").toInt(100);
                    int mctsIter = obj.value("mctsIter").toInt(100);
                    int mctsDepth = obj.value("mctsDepth").toInt(10);
                    bool pca = obj.value("pca").toBool(true);
                    bool merge = obj.value("merge").toBool(true);
                    bool decimate = obj.value("decimate").toBool(true);
                    int maxChVertex = obj.value("maxChVertex").toInt(64);
                    bool extrude = obj.value("extrude").toBool(false);
                    double extrudeMargin = obj.value("extrudeMargin").toDouble(0.0);
                    std::string apxMode = obj.value("apxMode").toString("fast").toStdString();
                    int seed = obj.value("seed").toInt(42);
                    generateCoACDStub(QString::fromStdString(node->name), concavity, alpha, beta, node,
                        maxConvexHull, preprocess, prepRes, sampleRes, mctsNodes, mctsIter, mctsDepth,
                        pca, merge, decimate, maxChVertex, extrude, extrudeMargin, apxMode, seed);
                    if (openGLViewer) {
                        openGLViewer->markCacheDirty();
                    }
                    QMessageBox::information(nullptr, "CoACD", "CoACD collision mesh generation complete.");
                }
            });

            // --- Exclude by color submenu ---
            QMenu* excludeByColorMenu = menu.addMenu("Exclude By Color");
            std::set<QString> uniqueColorKeys;
            std::map<QString, QColor> colorKeyToColor;
            std::function<void(const CadNode*)> collectColors;
            collectColors = [&](const CadNode* n) {
                if (!n) return;
                QColor c = toQColor(n->color);
                QString key = QString("%1,%2,%3,%4")
                    .arg(int(c.redF() * 255))
                    .arg(int(c.greenF() * 255))
                    .arg(int(c.blueF() * 255))
                    .arg(int(c.alphaF() * 255));
                uniqueColorKeys.insert(key);
                colorKeyToColor[key] = c;
                for (const auto& child : n->children) collectColors(child.get());
            };
            collectColors(node);

            for (const QString& key : uniqueColorKeys) {
                QColor color = colorKeyToColor[key];
                QString colorText = QString("RGB(%1, %2, %3)")
                    .arg(int(color.redF() * 255))
                    .arg(int(color.greenF() * 255))
                    .arg(int(color.blueF() * 255));
                QAction* colorAction = new QAction(colorText, excludeByColorMenu);
                QPixmap pix(16, 16);
                pix.fill(color);
                colorAction->setIcon(QIcon(pix));
                excludeByColorMenu->addAction(colorAction);
                QObject::connect(colorAction, &QAction::triggered, treeView, [=]() {
                    std::vector<QModelIndex> changedIndices;
                    std::function<void(CadNode*)> excludeByColor;
                    excludeByColor = [&](CadNode* n) {
                        if (!n) return;
                        if (toQColor(n->color) == color && !n->excludedFromDecomposition) {
                            n->excludedFromDecomposition = true;
                            QModelIndex changedIdx = customModel->indexForNode(n);
                            if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                        }
                        for (auto& child : n->children) excludeByColor(child.get());
                    };
                    excludeByColor(const_cast<CadNode*>(node));
                    for (const QModelIndex& changedIdx : changedIndices) {
                        customModel->dataChanged(changedIdx, changedIdx);
                    }
                });
            }

            // --- Debug actions ---
            QAction* infoAction = menu.addAction(QString("Node type: %1").arg((int)node->type));
            infoAction->setEnabled(false);

            QAction* printPtrAction = menu.addAction("Print Node Pointer (Debug)");
            QObject::connect(printPtrAction, &QAction::triggered, treeView, [=]() {
                qDebug() << "[Debug] Node pointer:" << static_cast<const void*>(node);
                if (auto xData = node->asXCAF()) {
                    QString typeStr = shapeTypeToString(xData->type);
                    bool isNull = xData->shape.IsNull();
                    qDebug() << "[Debug]   XCAF info:";
                    qDebug() << "[Debug]     shapeIndex:" << xData->shapeIndex;
                    qDebug() << "[Debug]     shape type:" << typeStr;
                    qDebug() << "[Debug]     shape isNull:" << isNull;
                }
                if (auto physData = node->asPhysics()) {
                    qDebug() << "[Debug]   PHYSICS info:";
                    qDebug() << "[Debug]     convexHullGenerated:" << physData->convexHullGenerated;
                    qDebug() << "[Debug]     collisionMeshVisible:" << physData->collisionMeshVisible;
                    qDebug() << "[Debug]     hulls.size():" << physData->hulls.size();
                }
            });

            // --- Show/Hide node action ---
            QAction* showHideAction = nullptr;
            if (node->visible) {
                showHideAction = menu.addAction("Hide Node");
            } else {
                showHideAction = menu.addAction("Show Node");
            }
            QObject::connect(showHideAction, &QAction::triggered, treeView, [=]() {
                bool newVisibility = !node->visible;
                std::vector<QModelIndex> changedIndices;
                std::function<void(CadNode*)> setVisibility;
                setVisibility = [&](CadNode* n) {
                    if (!n) return;
                    n->visible = newVisibility;
                    QModelIndex changedIdx = customModel->indexForNode(n);
                    if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                    for (auto& child : n->children) setVisibility(child.get());
                };
                setVisibility(const_cast<CadNode*>(customModel->getNode(idx)));
                for (const QModelIndex& changedIdx : changedIndices) {
                    customModel->dataChanged(changedIdx, changedIdx);
                }
            });

            // --- Exclude/Include node action ---
            QAction* excludeIncludeAction = nullptr;
            if (!node->excludedFromDecomposition) {
                excludeIncludeAction = menu.addAction("Exclude Node (and Children)");
            } else {
                excludeIncludeAction = menu.addAction("Include Node (and Children)");
            }
            QObject::connect(excludeIncludeAction, &QAction::triggered, treeView, [=]() {
                bool newExcluded = !node->excludedFromDecomposition;
                std::vector<QModelIndex> changedIndices;
                std::function<void(CadNode*)> setExcluded;
                setExcluded = [&](CadNode* n) {
                    if (!n) return;
                    n->excludedFromDecomposition = newExcluded;
                    QModelIndex changedIdx = customModel->indexForNode(n);
                    if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                    for (auto& child : n->children) setExcluded(child.get());
                };
                setExcluded(const_cast<CadNode*>(customModel->getNode(idx)));
                for (const QModelIndex& changedIdx : changedIndices) {
                    customModel->dataChanged(changedIdx, changedIdx);
                }
            });

            // --- Delete Node action (not for root) ---
            if (node != customModel->getRootNodePointer()) {
                QAction* deleteAction = menu.addAction("Delete Node");
                QObject::connect(deleteAction, &QAction::triggered, treeView, [=]() {
                    customModel->removeNode(node);
                    QModelIndex parentIdx = customModel->indexForNode(customModel->getParentNode(node));
                    if (parentIdx.isValid()) {
                        treeView->setCurrentIndex(parentIdx);
                    } else {
                        treeView->clearSelection();
                    }
                });
            }

            // --- Toggle collision mesh visibility for Physics nodes ---
            if (node->type == CadNodeType::Physics && node->asPhysics()) {
                QAction* toggleCollisionAction = menu.addAction("Toggle Collision Mesh Visibility");
                QObject::connect(toggleCollisionAction, &QAction::triggered, treeView, [=]() {
                    PhysicsNodeData* physData = node->asPhysics();
                    physData->collisionMeshVisible = !physData->collisionMeshVisible;
                    customModel->dataChanged(idx, idx);
                    if (openGLViewer) {
                        openGLViewer->markCacheDirty();
                        openGLViewer->update();
                    }
                });
            }

            // --- Expand Rail to Other Trees ---
            if (node->type == CadNodeType::Rail && node->asRail()) {
                QAction* expandRailAction = menu.addAction("Expand Rail to Other Tree...");
                QObject::connect(expandRailAction, &QAction::triggered, treeView, [=]() {
                    // Create a dialog to select target tree
                    QDialog dialog(treeView);
                    dialog.setWindowTitle("Select Target Tree for Rail Expansion");
                    dialog.setModal(true);
                    dialog.resize(400, 200);

                    QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);

                    QLabel* label = new QLabel("Select the target tree to expand this rail into:");
                    mainLayout->addWidget(label);

                    QComboBox* treeComboBox = new QComboBox;
                    mainLayout->addWidget(treeComboBox);

                    // Populate combo box with available trees
                    const std::vector<CadViewerPane> panes = workspace
                        ? workspace->panes()
                        : std::vector<CadViewerPane>();
                    for (size_t i = 0; i < panes.size(); ++i) {
                        QTreeView* availableTreeView = panes[i].tree;
                        if (availableTreeView && availableTreeView != treeView) { // Don't include current tree
                            QString treeName = "Tree " + QString::number(i + 1);
                            // Try to get a more descriptive name from the tab widget
                            if (availableTreeView->parent()) {
                                QTabWidget* tabWidget = qobject_cast<QTabWidget*>(availableTreeView->parent()->parent());
                                if (tabWidget) {
                                    QWidget* parentWidget = qobject_cast<QWidget*>(availableTreeView->parent());
                                    if (parentWidget) {
                                        int tabIndex = tabWidget->indexOf(parentWidget);
                                        if (tabIndex >= 0) {
                                            treeName = tabWidget->tabText(tabIndex);
                                        }
                                    }
                                }
                            }
                            treeComboBox->addItem(treeName, QVariant::fromValue(static_cast<void*>(availableTreeView)));
                        }
                    }

                    if (treeComboBox->count() == 0) {
                        QMessageBox::warning(nullptr, "No Target Trees", "No other trees available for rail expansion.");
                        return;
                    }

                    // Buttons
                    QHBoxLayout* buttonLayout = new QHBoxLayout;
                    QPushButton* okButton = new QPushButton("Expand");
                    QPushButton* cancelButton = new QPushButton("Cancel");

                    buttonLayout->addStretch();
                    buttonLayout->addWidget(cancelButton);
                    buttonLayout->addWidget(okButton);
                    mainLayout->addLayout(buttonLayout);

                    // Connect buttons
                    QObject::connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
                    QObject::connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

                    if (dialog.exec() == QDialog::Accepted) {
                        QTreeView* targetTreeView = static_cast<QTreeView*>(treeComboBox->currentData().value<void*>());
                        if (targetTreeView) {
                            // Find the corresponding OpenGL viewer
                            CadOpenGLWidget* targetOpenGLViewer = nullptr;
                            for (const CadViewerPane& pane : panes) {
                                if (pane.tree == targetTreeView) {
                                    targetOpenGLViewer = pane.viewer;
                                    break;
                                }
                            }

                            // Get the target tree's root node
                            CustomModelTreeModel* targetModel = qobject_cast<CustomModelTreeModel*>(targetTreeView->model());
                            if (targetModel) {
                                std::shared_ptr<const CadNode> targetRootShared = targetModel->getRoot();
                                if (targetRootShared) {
                                    // Create a non-const shared_ptr for the expandRailInPhysicsPreview function
                                    std::shared_ptr<CadNode> targetRootNonConst = std::const_pointer_cast<CadNode>(targetRootShared);
                                    // Expand the rail to the target tree
                                    expandRailInPhysicsPreview(node, targetRootNonConst, targetOpenGLViewer);

                                    // Update the target tree view
                                    targetModel->dataChanged(targetModel->index(0, 0), targetModel->index(0, 0));

                                    QMessageBox::information(nullptr, "Success",
                                        QString("Rail '%1' expanded to target tree successfully.").arg(QString::fromStdString(node->name)));
                                }
                            }
                        }
                    }
                });
            }
        }

        // Handle XCAFLabelNode type (XCAFLabelTreeModel)
        XCAFLabelTreeModel* labelModel = qobject_cast<XCAFLabelTreeModel*>(model);
        if (labelModel) {
            XCAFLabelNode* labelNode = const_cast<XCAFLabelNode*>(labelModel->getNode(idx));
            if (!labelNode) return;

            // Add basic info action for XCAF nodes
            QAction* infoAction = menu.addAction(QString("XCAF Label Tag: %1").arg(labelNode->label.Tag()));
            infoAction->setEnabled(false);

            // Add debug action
            QAction* debugAction = menu.addAction("Debug XCAF Node");
            QObject::connect(debugAction, &QAction::triggered, treeView, [=]() {
                qDebug() << "[Debug] XCAF Node:";
                qDebug() << "[Debug]   Label Tag:" << labelNode->label.Tag();
                qDebug() << "[Debug]   Label Address:" << static_cast<const void*>(&labelNode->label);
                qDebug() << "[Debug]   Children count:" << labelNode->children.size();
            });
        }

        if (!menu.isEmpty()) {
            menu.exec(treeView->viewport()->mapToGlobal(pos));
        }
    });
}

CadViewerPane initTreeAndOpenGLWidget(std::shared_ptr<CadNode>& inputRoot,
                                     QTabWidget* treeTabWidget,
                                     QTabWidget* openGLTabWidget,
                                     const QString& name,
                                     const Handle(TDocStd_Document)& doc,
                                     CadViewerWorkspace& workspace,
                                     SimulationManager* simManager) {
    inputRoot->name = "Tree Root " + name.toStdString();
    QTreeView* treeView = new QTreeView;
    CustomModelTreeModel* qtModel = new CustomModelTreeModel(inputRoot, treeView);
    treeView->setModel(qtModel);
    treeView->setHeaderHidden(false);
    treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);

    CadOpenGLWidget* openGLViewer = new CadOpenGLWidget(inputRoot.get());
    if (simManager) {
        openGLViewer->setSimulationManager(simManager);
    }

    setupComprehensiveContextMenu(treeView, qtModel, doc, openGLViewer, &workspace);

    treeTabWidget->addTab(treeView, name + " Tree");
    openGLTabWidget->addTab(openGLViewer, name + " Preview");

    connectTreeAndViewer(treeView, openGLViewer, qtModel);

    const CadViewerPane pane{treeView, openGLViewer};
    workspace.addPane(pane);
    return pane;
}
