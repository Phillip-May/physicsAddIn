#include "XCAFLabelTreeModel.h"
#include <TDataStd_Name.hxx>
#include <QString>

XCAFLabelTreeModel::XCAFLabelTreeModel(std::unique_ptr<XCAFLabelNode> root, QObject* parent)
    : QAbstractItemModel(parent), m_root(std::move(root)) {}

QModelIndex XCAFLabelTreeModel::index(int row, int column, const QModelIndex& parent) const {
    auto parentNode = getNode(parent);
    if (!parentNode || row < 0 || row >= static_cast<int>(parentNode->children.size()))
        return QModelIndex();
    return createIndex(row, column, parentNode->children[row].get());
}

QModelIndex XCAFLabelTreeModel::parent(const QModelIndex& index) const {
    XCAFLabelNode* node = getNode(index);
    if (!node || node == m_root.get())
        return QModelIndex();
    // No parent pointer, so traverse from root
    std::function<QModelIndex(XCAFLabelNode*, XCAFLabelNode*)> findParent = [&](XCAFLabelNode* parent, XCAFLabelNode* child) -> QModelIndex {
        for (size_t i = 0; i < parent->children.size(); ++i) {
            if (parent->children[i].get() == child)
                return createIndex(static_cast<int>(i), 0, parent);
            QModelIndex idx = findParent(parent->children[i].get(), child);
            if (idx.isValid()) return idx;
        }
        return QModelIndex();
    };
    return findParent(m_root.get(), node);
}

int XCAFLabelTreeModel::rowCount(const QModelIndex& parent) const {
    auto parentNode = getNode(parent);
    return parentNode ? static_cast<int>(parentNode->children.size()) : 0;
}

int XCAFLabelTreeModel::columnCount(const QModelIndex&) const {
    return 1;
}

QVariant XCAFLabelTreeModel::data(const QModelIndex& index, int role) const {
    auto node = getNode(index);
    if (!node) return QVariant();
    if (role == Qt::DisplayRole) {
        Handle(TDataStd_Name) nameAttr;
        if (node->label.FindAttribute(TDataStd_Name::GetID(), nameAttr) && !nameAttr.IsNull()) {
            return QString::fromStdWString(nameAttr->Get().ToWideString());
        }
        return QString("Label %1").arg(node->label.Tag());
    }
    return QVariant();
}

Qt::ItemFlags XCAFLabelTreeModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) return Qt::NoItemFlags;
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

XCAFLabelNode* XCAFLabelTreeModel::getNode(const QModelIndex& index) const {
    if (!index.isValid()) return m_root.get();
    return static_cast<XCAFLabelNode*>(index.internalPointer());
} 