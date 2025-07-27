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
    if (!parentItem) return QModelIndex();
    
    // Handle root node case - if parent is invalid, we're at the root level
    if (!parent.isValid()) {
        if (row == 0 && column == 0) {
            return createIndex(0, 0, m_rootItem.get());
        }
        return QModelIndex();
    }
    
    // If the parent is the root item, show its actual children (not itself)
    if (parentItem == m_rootItem.get()) {
        int validChildIndex = 0;
        for (size_t i = 0; i < m_rootItem->children.size(); ++i) {
            auto& childItem = m_rootItem->children[i];
            if (!childItem) continue;
            // Prevent circular reference - don't show root as its own child
            if (childItem->node == m_root.get()) continue;
            
            if (validChildIndex == row) {
                return createIndex(row, column, childItem.get());
            }
            validChildIndex++;
        }
        return QModelIndex();
    }
    
    // Handle regular child nodes
    if (row < 0 || row >= static_cast<int>(parentItem->children.size()))
        return QModelIndex();
    return createIndex(row, column, parentItem->children[row].get());
}

QModelIndex CadTreeModel::parent(const QModelIndex& index) const {
    CadTreeItem* childItem = getItem(index);
    if (!childItem) return QModelIndex();
    
    // If the child is the root item, return invalid index (root has no parent)
    if (childItem == m_rootItem.get()) return QModelIndex();
    
    CadTreeItem* parentItem = childItem->parent;
    if (!parentItem) return QModelIndex();
    
    // If the parent is the root item, return the root index
    if (parentItem == m_rootItem.get()) {
        return createIndex(0, 0, m_rootItem.get());
    }
    
    if (!parentItem->parent) return QModelIndex();
    return createIndex(parentItem->row, 0, parentItem);
}

int CadTreeModel::rowCount(const QModelIndex& parent) const {
    CadTreeItem* parentItem = getItem(parent);
    if (!parentItem) return 0;
    
    // If parent is invalid, we're at the root level - return 1 to show the root node
    if (!parent.isValid()) {
        return 1;
    }
    
    // If the parent is the root item, return the number of its actual children (excluding self)
    if (parentItem == m_rootItem.get()) {
        int count = 0;
        for (const auto& child : m_rootItem->children) {
            if (child->node != m_root.get()) {
                count++;
            }
        }
        return count;
    }
    
    return static_cast<int>(parentItem->children.size());
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
    // If target is the root node, return the root index
    if (target == m_root.get()) {
        return createIndex(0, 0, m_rootItem.get());
    }
    
    std::function<QModelIndex(CadTreeItem*, const QModelIndex&)> find = [&](CadTreeItem* item, const QModelIndex& parent) -> QModelIndex {
        if (item->node == target) return parent;
        for (int row = 0; row < static_cast<int>(item->children.size()); ++row) {
            CadTreeItem* child = item->children[row].get();
            // Skip if this is the root item and we're looking for root node
            if (item == m_rootItem.get() && child->node == m_root.get()) continue;
            
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
            // Skip if this is the root item and we're looking at the root node
            if (current == m_rootItem.get() && child->node == m_root.get()) continue;
            
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
