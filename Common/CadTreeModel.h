#pragma once
#include <QAbstractItemModel>
#include <memory>
#include <TopLoc_Location.hxx>
// Forward declaration instead of including main.cpp
struct CadNode;

struct CadTreeItem {
    CadNode* node;
    TopLoc_Location accumulatedLoc;
    std::vector<std::unique_ptr<CadTreeItem>> children;
    CadTreeItem* parent = nullptr;
    int row = 0;
};

class CadTreeModel : public QAbstractItemModel {
    Q_OBJECT
public:
    CadTreeModel(std::unique_ptr<CadNode> root, QObject* parent = nullptr);
    QModelIndex index(int row, int column, const QModelIndex& parent) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent) const override;
    int columnCount(const QModelIndex&) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    CadNode* getNode(const QModelIndex& index) const;
    TopLoc_Location getAccumulatedLoc(const QModelIndex& index) const;
    CadTreeItem* getItem(const QModelIndex& index) const;
    CadNode* getRootNodePointer() const { return m_root.get(); }
    QModelIndex indexForNode(CadNode* target) const;
    CadNode* getParentNode(const CadNode* node) const;
private:
    std::unique_ptr<CadNode> m_root;
    std::unique_ptr<CadTreeItem> m_rootItem;
    void buildTree(CadNode* node, const TopLoc_Location& parentLoc, CadTreeItem* parentItem);
}; 
