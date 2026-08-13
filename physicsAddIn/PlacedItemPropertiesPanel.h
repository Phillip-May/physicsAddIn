#ifndef PLACEDITEMPROPERTIESPANEL_H
#define PLACEDITEMPROPERTIESPANEL_H

#include "PlacedItem.h"
#include "PlacedMechanismSchema.h"

#include <QWidget>

#include <vector>

class QFormLayout;
class QLabel;
class QLineEdit;
class QDoubleSpinBox;
class QPushButton;

// A placed package's properties: where it stands, and where its axes are.
class PlacedItemPropertiesPanel : public QWidget {
    Q_OBJECT

public:
    explicit PlacedItemPropertiesPanel(QWidget* parent = nullptr);

    void showPlacedItem(const PlacedItem* placed, bool runStopped);
    void showNothing();

signals:
    // Where it now stands, relative to the RoboDK parent its item hangs under - which is what its stored
    // placement is measured against. Emitted only when one of the six numbers actually changed.
    void placementEdited(Item item, const CadTransform& poseLocal);
    // Where its axes now stand: one position in millimetres for a rail, six angles in degrees for an arm.
    void axesEdited(Item item, const std::vector<double>& values);
    // A rename of the item, and so of the name every verb that takes one will answer to.
    void renameRequested(Item item, const QString& wanted);

private:
    struct NumberRow {
        QLabel* label = nullptr;
        QDoubleSpinBox* editor = nullptr;
        double shown = 0.0;
    };

    void buildPlacementRows();
    void rebuildAxisRows(const std::vector<placedmechanism::AxisField>& fields);
    void commitPlacement();
    void commitAxes();
    bool operatorIsTyping() const;

    QFormLayout* m_form = nullptr;
    QLabel* m_heading = nullptr;
    QLineEdit* m_name = nullptr;
    QLabel* m_runNote = nullptr;
    QLabel* m_axisHeading = nullptr;
    QLabel* m_travelNote = nullptr;
    NumberRow m_placement[6];
    // As many as the package has: one for a rail, six for an arm, none for a pedestal. Owned rows, torn
    // down and rebuilt when the item on show changes kind.
    std::vector<NumberRow> m_axes;
    std::vector<placedmechanism::AxisField> m_axisFields;
    Item m_item = nullptr;
    bool m_syncing = false;
};

#endif // PLACEDITEMPROPERTIESPANEL_H
