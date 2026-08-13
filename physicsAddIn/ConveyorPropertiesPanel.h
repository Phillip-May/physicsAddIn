#ifndef CONVEYORPROPERTIESPANEL_H
#define CONVEYORPROPERTIESPANEL_H

#include "RoboDkConveyorHost.h"

#include <QWidget>

#include <vector>

class QComboBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;

struct AccessoryField;

// The selected conveyor's properties, as a rendering of `Common/AccessoryPropertySchema`.
class ConveyorPropertiesPanel : public QWidget {
    Q_OBJECT

public:
    explicit ConveyorPropertiesPanel(QWidget* parent = nullptr);

    // Shows this conveyor, or "nothing selected" for a null name. Called at the panel timer's rate, so
    // it syncs rows in place and never rebuilds: a rebuild at event rate would take the operator's
    // cursor out of the box they are typing in. Widgets an operator is editing are left alone.
    void showConveyor(const RoboDkConveyorHost::Segment* segment, bool runStopped,
                      const QStringList& workpieceCandidates);
    void showNothing();

signals:
    // One committed edit: the conveyor that was on show, as it now reads. Emitted on Enter or focus-out
    // for a number and immediately for a choice or a toggle - never on every keystroke, because
    // adopting an edit re-exports the prototype's mesh to measure the workpiece.
    void parametersEdited(const QString& conveyor, const TransformNodeData& parameters);
    // A rename, which is a rename of the conveyor's *item* and so of its three IO variables.
    void renameRequested(const QString& conveyor, const QString& wanted);
    // Where the conveyor now stands, relative to the RoboDK parent its item hangs under - which is what
    // the stored placement is measured against. Emitted only when one of the six numbers actually
    // changed; see `commitPlacement`.
    void placementEdited(const QString& conveyor, const CadTransform& poseLocal);

private:
    struct Row {
        const AccessoryField* field = nullptr;
        QLabel* label = nullptr;
        QWidget* editor = nullptr;
    };

    // One of the six numbers a placement is shown as, and what was last synced into it - which is how a
    // focus-out that changed nothing is told from an edit.
    struct PlacementRow {
        QLabel* label = nullptr;
        QDoubleSpinBox* editor = nullptr;
        double shown = 0.0;
    };

    void buildRows();
    void buildPlacementRows();
    void commit();
    void commitPlacement();
    // Whether any editor currently has the keyboard, in which case its value is the operator's and not
    // ours to overwrite.
    bool operatorIsTyping() const;

    QFormLayout* m_form = nullptr;
    QLabel* m_heading = nullptr;
    QLineEdit* m_name = nullptr;
    QLabel* m_renameWarning = nullptr;
    QLabel* m_runNote = nullptr;
    std::vector<Row> m_rows;
    // X, Y, Z in mm and then the three euler angles in degrees, in RoboDK's own `ToXYZRPW` order and
    // convention - the same six numbers its own pose dialog shows, so an operator reads one thing.
    PlacementRow m_placement[6];
    QString m_conveyor;
    TransformNodeData m_parameters;
    bool m_syncing = false;
};

#endif // CONVEYORPROPERTIESPANEL_H
