#ifndef QTCADVIEWER_CUSTOMMODEL_TREEMODEL_H
#define QTCADVIEWER_CUSTOMMODEL_TREEMODEL_H

#include <vector>
#include <string>
#include <memory>
#include "CadNode.h" // For CadNode

#ifdef Q_OS_WIN
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <QAbstractItemModel>



class CustomModelTreeModel : public QAbstractItemModel {
    Q_OBJECT
public:
    using NodeType = CadNode;

    // The model always has a persistent root node; all custom parts are children of this root.
    // This design makes data interoperability and tree walking easier.
    explicit CustomModelTreeModel(std::shared_ptr<NodeType> root, QObject* parent = nullptr);
    ~CustomModelTreeModel();

    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& child) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    // Access root node (shared_ptr, const)
    std::shared_ptr<const NodeType> getRoot() const { return root_; }
    // Access root node (raw pointer, const)
    const NodeType* getRootNodePointer() const { return root_.get(); }

    // Make these public for selection sync
    const NodeType* getNode(const QModelIndex& index) const;
    QModelIndex indexForNode(const NodeType* node, int column = 0) const;
    const NodeType* getParentNode(const NodeType* node) const;

    // Transform nodes are supported in the custom model tree. They apply a transform to all children without duplicating geometry.
    void resetModelAndAddNode(std::shared_ptr<NodeType> newNode);
    void addNodeWithReset(std::shared_ptr<NodeType> newNode);

    // Remove a node from the tree given a pointer to the node
    bool removeNode(const NodeType* node);

private:
    std::shared_ptr<NodeType> root_;
};

#ifdef Q_OS_WIN
#pragma warning(pop)
#endif

#endif // QTCADVIEWER_CUSTOMMODEL_TREEMODEL_H
