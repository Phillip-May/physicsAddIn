#ifndef QTCADVIEWER_CUSTOMMODEL_TREEMODEL_H
#define QTCADVIEWER_CUSTOMMODEL_TREEMODEL_H

#include <vector>
#include <string>
#include <memory>
#include "CadNode.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <QAbstractItemModel>

class CustomModelTreeModel : public QAbstractItemModel {
    Q_OBJECT
public:
    using NodeType = CadNode;

    // Custom parts are children of a persistent root node.
    explicit CustomModelTreeModel(std::shared_ptr<NodeType> root, QObject* parent = nullptr);
    ~CustomModelTreeModel();

    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& child) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    std::shared_ptr<const NodeType> getRoot() const { return root_; }
    const NodeType* getRootNodePointer() const { return root_.get(); }

    const NodeType* getNode(const QModelIndex& index) const;
    QModelIndex indexForNode(const NodeType* node, int column = 0) const;
    const NodeType* getParentNode(const NodeType* node) const;

    void resetModelAndAddNode(std::shared_ptr<NodeType> newNode);
    void addNodeWithReset(std::shared_ptr<NodeType> newNode);

    bool removeNode(const NodeType* node);

private:
    std::shared_ptr<NodeType> root_;
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // QTCADVIEWER_CUSTOMMODEL_TREEMODEL_H
