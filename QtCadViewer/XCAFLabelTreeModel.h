#pragma once
#include <QAbstractItemModel>
#include <TDF_Label.hxx>
#include <vector>
#include <memory>

// Node for XCAF label tree
struct XCAFLabelNode {
    TDF_Label label;
    std::vector<std::unique_ptr<XCAFLabelNode>> children;
    XCAFLabelNode(const TDF_Label& lbl) : label(lbl) {}
};

class XCAFLabelTreeModel : public QAbstractItemModel {
    Q_OBJECT
public:
    XCAFLabelTreeModel(std::unique_ptr<XCAFLabelNode> root, QObject* parent = nullptr);
    QModelIndex index(int row, int column, const QModelIndex& parent) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent) const override;
    int columnCount(const QModelIndex&) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    XCAFLabelNode* getNode(const QModelIndex& index) const;
    XCAFLabelNode* getRootNodePointer() const { return m_root.get(); }
private:
    std::unique_ptr<XCAFLabelNode> m_root;
}; 