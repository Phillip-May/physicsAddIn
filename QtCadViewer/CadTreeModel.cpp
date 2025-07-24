#include "CadTreeModel.h"

#include <CadNode.h>

CadTreeModel::CadTreeModel(std::unique_ptr<CadNode> root, QObject* parent)
    : QAbstractItemModel(parent), m_root(std::move(root)) {
    m_rootItem = std::make_unique<CadTreeItem>();
    m_rootItem->node = m_root.get();
    m_rootItem->accumulatedLoc = TopLoc_Location();
    m_rootItem->parent = nullptr;
    m_rootItem->row = 0;
    buildTree(m_root.get(), TopLoc_Location(), m_rootItem.get());
}

void CadTreeModel::buildTree(CadNode* node, const TopLoc_Location& parentLoc, CadTreeItem* parentItem) {
    int row = 0;
    for (const auto& child : node->children) {
        auto item = std::make_unique<CadTreeItem>();
        item->node = child.get();
        item->accumulatedLoc = parentLoc * child->loc;
        item->parent = parentItem;
        item->row = row++;
        buildTree(child.get(), item->accumulatedLoc, item.get());
        parentItem->children.push_back(std::move(item));
    }
}

CadTreeItem* CadTreeModel::getItem(const QModelIndex& index) const {
    if (!index.isValid()) return m_rootItem.get();
    return static_cast<CadTreeItem*>(index.internalPointer());
}

CadNode* CadTreeModel::getNode(const QModelIndex& index) const {
    return getItem(index)->node;
}

TopLoc_Location CadTreeModel::getAccumulatedLoc(const QModelIndex& index) const {
    return getItem(index)->accumulatedLoc;
}

QModelIndex CadTreeModel::index(int row, int column, const QModelIndex& parent) const {
    CadTreeItem* parentItem = getItem(parent);
    if (!parentItem || row < 0 || row >= static_cast<int>(parentItem->children.size()))
        return QModelIndex();
    return createIndex(row, column, parentItem->children[row].get());
}

QModelIndex CadTreeModel::parent(const QModelIndex& index) const {
    CadTreeItem* childItem = getItem(index);
    if (!childItem || !childItem->parent || childItem == m_rootItem.get())
        return QModelIndex();
    CadTreeItem* parentItem = childItem->parent;
    if (!parentItem->parent) return QModelIndex();
    return createIndex(parentItem->row, 0, parentItem);
}

int CadTreeModel::rowCount(const QModelIndex& parent) const {
    CadTreeItem* parentItem = getItem(parent);
    return parentItem ? static_cast<int>(parentItem->children.size()) : 0;
}

int CadTreeModel::columnCount(const QModelIndex&) const {
    return 1;
}

QVariant CadTreeModel::data(const QModelIndex& index, int role) const {
    auto item = getItem(index);
    if (!item || !item->node) return QVariant();
    if (role == Qt::DisplayRole) return QString::fromStdString(item->node->name);
    if (role == Qt::BackgroundRole) {
        QColor originalColor = item->node->color.toQColor();
        originalColor.setAlphaF(originalColor.alphaF() / 3.0f);
        return originalColor;
    }
    return QVariant();
}

Qt::ItemFlags CadTreeModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) return Qt::NoItemFlags;
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable;
}

QModelIndex CadTreeModel::indexForNode(CadNode* target) const {
    std::function<QModelIndex(CadTreeItem*, const QModelIndex&)> find = [&](CadTreeItem* item, const QModelIndex& parent) -> QModelIndex {
        if (item->node == target) return parent;
        for (int row = 0; row < static_cast<int>(item->children.size()); ++row) {
            CadTreeItem* child = item->children[row].get();
            QModelIndex idx = index(row, 0, parent);
            QModelIndex found = find(child, idx);
            if (found.isValid()) return found;
        }
        return QModelIndex();
    };
    return find(m_rootItem.get(), QModelIndex());
}

// Recursively find the parent of a given node
CadNode* CadTreeModel::getParentNode(const CadNode* node) const {
    if (!m_root || node == m_root.get()) return nullptr;
    std::function<CadNode*(CadTreeItem*)> findParent = [&](CadTreeItem* current) -> CadNode* {
        for (const auto& child : current->children) {
            if (child->node == node) return current->node;
            if (child) {
                CadNode* res = findParent(child.get());
                if (res) return res;
            }
        }
        return nullptr;
    };
    return findParent(m_rootItem.get());
} 
