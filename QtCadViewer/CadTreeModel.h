#pragma once
#include <QAbstractItemModel>
#include <memory>
// Forward declaration instead of including main.cpp
struct CadNode;

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
    CadNode* nodeFromIndex(const QModelIndex& index) const;
    CadNode* getRootNodePointer() const { return m_root.get(); }
    QModelIndex indexForNode(CadNode* target) const;
private:
    std::unique_ptr<CadNode> m_root;
}; 
