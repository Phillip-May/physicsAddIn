#include "CadTreeModel.h"

#include <CadNode.h>

CadTreeModel::CadTreeModel(std::unique_ptr<TreeNode> root, QObject* parent)
    : QAbstractItemModel(parent), m_root(std::move(root)) {}

QModelIndex CadTreeModel::index(int row, int column, const QModelIndex& parent) const {
    auto parentNode = nodeFromIndex(parent);
    if (!parentNode || row < 0 || row >= static_cast<int>(parentNode->children.size()))
        return QModelIndex();
    return createIndex(row, column, parentNode->children[row].get());
}

QModelIndex CadTreeModel::parent(const QModelIndex& index) const {
    TreeNode* node = getNode(index);
    if (!node || node == m_root.get())
        return QModelIndex();
    // TreeNode does not have a parent pointer, so we must traverse from root
    std::function<QModelIndex(TreeNode*, TreeNode*, const QModelIndex&)> findParent = [&](TreeNode* parent, TreeNode* child, const QModelIndex& parentIndex) -> QModelIndex {
        for (size_t i = 0; i < parent->children.size(); ++i) {
            if (parent->children[i].get() == child)
                return createIndex(static_cast<int>(i), 0, parent);
            QModelIndex idx = findParent(parent->children[i].get(), child, createIndex(static_cast<int>(i), 0, parent->children[i].get()));
            if (idx.isValid()) return idx;
        }
        return QModelIndex();
    };
    return findParent(m_root.get(), node, QModelIndex());
}

int CadTreeModel::rowCount(const QModelIndex& parent) const {
    auto parentNode = nodeFromIndex(parent);
    return parentNode ? static_cast<int>(parentNode->children.size()) : 0;
}

int CadTreeModel::columnCount(const QModelIndex&) const {
    return 1;
}

QVariant CadTreeModel::data(const QModelIndex& index, int role) const {
    auto node = getNode(index);
    if (!node) return QVariant();
    if (role == Qt::DisplayRole) return QString::fromStdString(node->name);
    if (role == Qt::BackgroundRole) return node->color.toQColor();
    return QVariant();
}

Qt::ItemFlags CadTreeModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) return Qt::NoItemFlags;
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable;
}

TreeNode* CadTreeModel::getNode(const QModelIndex& index) const {
    if (!index.isValid()) return m_root.get();
    return static_cast<TreeNode*>(index.internalPointer());
}

TreeNode* CadTreeModel::nodeFromIndex(const QModelIndex& index) const {
    if (!index.isValid()) return m_root.get();
    return static_cast<TreeNode*>(index.internalPointer());
} 
