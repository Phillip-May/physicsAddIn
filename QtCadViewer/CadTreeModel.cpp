#include "CadTreeModel.h"

#include <CadNode.h>

CadTreeModel::CadTreeModel(std::unique_ptr<CadNode> root, QObject* parent)
    : QAbstractItemModel(parent), m_root(std::move(root)) {}

QModelIndex CadTreeModel::index(int row, int column, const QModelIndex& parent) const {
    auto parentNode = nodeFromIndex(parent);
    if (!parentNode || row < 0 || row >= static_cast<int>(parentNode->children.size()))
        return QModelIndex();
    return createIndex(row, column, parentNode->children[row].get());
}

QModelIndex CadTreeModel::parent(const QModelIndex& index) const {
    CadNode* childNode = getNode(index);
    if (!childNode || childNode == m_root.get())
        return QModelIndex();
    // Recursively search for the parent and its row
    std::function<QModelIndex(CadNode*, const QModelIndex&)> findParent = [&](CadNode* parent, const QModelIndex& parentIdx) -> QModelIndex {
        for (int row = 0; row < static_cast<int>(parent->children.size()); ++row) {
            CadNode* child = parent->children[row].get();
            if (child == childNode) {
                // Return the index of the parent node (as a child of its own parent)
                return parentIdx;
            }
            QModelIndex idx = this->index(row, 0, parentIdx);
            QModelIndex found = findParent(child, idx);
            if (found.isValid()) return found;
        }
        return QModelIndex();
    };
    return findParent(m_root.get(), QModelIndex());
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
    if (role == Qt::BackgroundRole) {
        QColor originalColor = node->color.toQColor();
        originalColor.setAlphaF(originalColor.alphaF() / 3.0f);
        return originalColor;
    }
    return QVariant();
}

Qt::ItemFlags CadTreeModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) return Qt::NoItemFlags;
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable;
}

CadNode* CadTreeModel::getNode(const QModelIndex& index) const {
    if (!index.isValid()) return m_root.get();
    return static_cast<CadNode*>(index.internalPointer());
}

CadNode* CadTreeModel::nodeFromIndex(const QModelIndex& index) const {
    if (!index.isValid()) return m_root.get();
    return static_cast<CadNode*>(index.internalPointer());
}

QModelIndex CadTreeModel::indexForNode(CadNode* target) const {
    std::function<QModelIndex(CadNode*, const QModelIndex&)> find = [&](CadNode* node, const QModelIndex& parent) -> QModelIndex {
        if (node == target) return parent;
        for (int row = 0; row < static_cast<int>(node->children.size()); ++row) {
            CadNode* child = node->children[row].get();
            QModelIndex idx = index(row, 0, parent);
            QModelIndex found = find(child, idx);
            if (found.isValid()) return found;
        }
        return QModelIndex();
    };
    return find(m_root.get(), QModelIndex());
} 
