#include "CustomModelTreeModel.h"
#include <QString>

CustomModelTreeModel::CustomModelTreeModel(std::shared_ptr<NodeType> root, QObject* parent)
    : QAbstractItemModel(parent), root_(std::move(root)) {}

CustomModelTreeModel::~CustomModelTreeModel() = default;

const CustomModelTreeModel::NodeType* CustomModelTreeModel::getNode(const QModelIndex& index) const {
    if (!index.isValid()) return root_.get();
    return static_cast<const NodeType*>(index.internalPointer());
}

QModelIndex CustomModelTreeModel::index(int row, int column, const QModelIndex& parent) const {
    const NodeType* parentNode = getNode(parent);
    if (!parentNode) return QModelIndex();
    if (row < 0 || row >= static_cast<int>(parentNode->children.size())) return QModelIndex();
    auto& childPtr = parentNode->children[row];
    if (!childPtr) return QModelIndex();
    return createIndex(row, column, childPtr.get());
}

QModelIndex CustomModelTreeModel::parent(const QModelIndex& child) const {
    if (!child.isValid()) return QModelIndex();
    const NodeType* childNode = getNode(child);
    if (!childNode || childNode == root_.get()) return QModelIndex();
    const NodeType* parentNode = getParentNode(childNode);
    if (!parentNode) return QModelIndex();
    // Find the parent's parent and the row of this parent among its children
    const NodeType* grandParent = getParentNode(parentNode);
    if (!grandParent) return QModelIndex();
    for (size_t i = 0; i < grandParent->children.size(); ++i) {
        if (grandParent->children[i].get() == parentNode) {
            return createIndex(static_cast<int>(i), 0, const_cast<NodeType*>(parentNode));
        }
    }
    return QModelIndex();
}

int CustomModelTreeModel::rowCount(const QModelIndex& parent) const {
    const NodeType* parentNode = getNode(parent);
    if (!parentNode) return 0;
    return static_cast<int>(parentNode->children.size());
}

int CustomModelTreeModel::columnCount(const QModelIndex& /*parent*/) const {
    return 1;
}

QVariant CustomModelTreeModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid()) return QVariant();
    const NodeType* node = getNode(index);
    if (!node) return QVariant();
    if (role == Qt::DisplayRole) {
        // Display node name and type
        QString display = QString::fromStdString(node->name);
        return display;
    }
    if (role == Qt::BackgroundRole) {
        QColor c = node->color.toQColor();
        c.setAlphaF(c.alphaF() / 3.0); // Make it 1/3 as opaque
        return c;
    }
    return QVariant();
}

QVariant CustomModelTreeModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole) {
        if (section == 0) return QString("CAD Node");
    }
    return QVariant();
}

const CustomModelTreeModel::NodeType* CustomModelTreeModel::getParentNode(const NodeType* node) const {
    if (!root_) return nullptr;
    if (node == root_.get()) return nullptr;
    // Recursively search for the parent
    std::function<const NodeType*(const NodeType*)> findParent = [&](const NodeType* current) -> const NodeType* {
        for (const auto& child : current->children) {
            if (child.get() == node) return current;
            if (child) {
                const NodeType* res = findParent(child.get());
                if (res) return res;
            }
        }
        return nullptr;
    };
    return findParent(root_.get());
}

QModelIndex CustomModelTreeModel::indexForNode(const NodeType* node, int column) const {
    if (!node || node == root_.get()) return QModelIndex();
    const NodeType* parent = getParentNode(node);
    if (!parent) return QModelIndex();
    for (size_t i = 0; i < parent->children.size(); ++i) {
        if (parent->children[i].get() == node) {
            return createIndex(static_cast<int>(i), column, const_cast<NodeType*>(node));
        }
    }
    return QModelIndex();
}

void CustomModelTreeModel::resetModelAndAddNode(std::shared_ptr<NodeType> newNode) {
    beginResetModel();
    if (root_->children.size() == 1) {
        root_->children[0]->children.push_back(newNode);
    } else {
        root_->children.push_back(newNode);
    }
    endResetModel();
} 

// Remove a node from the tree given a pointer to the node
bool CustomModelTreeModel::removeNode(const NodeType* node) {
    if (!node || node == root_.get()) return false;
    NodeType* parent = const_cast<NodeType*>(getParentNode(node));
    if (!parent) return false;
    auto& siblings = parent->children;
    auto it = std::find_if(siblings.begin(), siblings.end(), [node](const std::shared_ptr<NodeType>& child) {
        return child.get() == node;
    });
    if (it != siblings.end()) {
        int row = static_cast<int>(std::distance(siblings.begin(), it));
        beginRemoveRows(indexForNode(parent), row, row);
        siblings.erase(it);
        endRemoveRows();
        return true;
    }
    return false;
} 