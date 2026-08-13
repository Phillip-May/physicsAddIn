#include "PlacedItemPropertiesPanel.h"

#include "PlacedItemAxes.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

PlacedItemPropertiesPanel::PlacedItemPropertiesPanel(QWidget* parent) : QWidget(parent) {
    setWindowTitle(tr("Placed item properties"));

    auto* column = new QVBoxLayout(this);
    m_heading = new QLabel(this);
    m_heading->setWordWrap(true);
    column->addWidget(m_heading);

    m_form = new QFormLayout();
    column->addLayout(m_form);

    m_name = new QLineEdit(this);
    // On Enter or focus-out, like every other committed edit here: renaming per keystroke would ask
    // RoboDK for a free name once per letter and leave the node called `WS20`.
    connect(m_name, &QLineEdit::editingFinished, this, [this]() {
        if (m_syncing || !m_item) return;
        emit renameRequested(m_item, m_name->text());
    });
    m_form->addRow(tr("Name"), m_name);

    buildPlacementRows();

    m_axisHeading = new QLabel(this);
    m_axisHeading->setEnabled(false);
    m_form->addRow(m_axisHeading);

    m_travelNote = new QLabel(this);
    m_travelNote->setWordWrap(true);
    m_travelNote->setEnabled(false);
    column->addWidget(m_travelNote);

    m_runNote = new QLabel(tr("A run is going. Moving this moves what is drawn; the solver was handed "
                              "its shapes when the run started."), this);
    m_runNote->setWordWrap(true);
    m_runNote->setEnabled(false);
    column->addWidget(m_runNote);

    column->addStretch(1);
    showNothing();
}

void PlacedItemPropertiesPanel::buildPlacementRows() {
    auto* separator = new QLabel(tr("Placement"), this);
    separator->setEnabled(false);
    m_form->addRow(separator);
    // Said on every row, because it is the difference between these numbers being wrong and their being
    // relative: dragging the frame the item hangs under moves it and leaves all six exactly as they are.
    const QString explanation =
        tr("Where this stands, relative to the RoboDK frame its item hangs under. Moving that frame "
           "moves this and leaves these unchanged. The three angles are RoboDK's own Rx, Ry, Rz.");
    const std::array<placedmechanism::PlacementField, 6>& fields = placedmechanism::placementFields();
    for (int at = 0; at < 6; ++at) {
        const placedmechanism::PlacementField& field = fields[static_cast<size_t>(at)];
        NumberRow& row = m_placement[at];
        row.label = new QLabel(QStringLiteral("%1 (%2)").arg(QString::fromUtf8(field.label),
                                                            QString::fromUtf8(field.unit)), this);
        auto* editor = new QDoubleSpinBox(this);
        editor->setRange(-field.limit, field.limit);
        editor->setSingleStep(field.step);
        editor->setDecimals(field.decimals);
        editor->setKeyboardTracking(false);
        connect(editor, &QAbstractSpinBox::editingFinished, this,
                &PlacedItemPropertiesPanel::commitPlacement);
        row.editor = editor;
        row.label->setToolTip(explanation);
        editor->setToolTip(explanation);
        m_form->addRow(row.label, editor);
    }
}

void PlacedItemPropertiesPanel::rebuildAxisRows(
    const std::vector<placedmechanism::AxisField>& fields) {
    for (NumberRow& row : m_axes) {
        // Taken out of the form as well as deleted: `removeRow` on the label takes its field with it, and
        // deleting the widgets alone would leave the layout holding rows that are gone.
        if (row.label) m_form->removeRow(row.label);
    }
    m_axes.clear();
    m_axisFields = fields;
    for (const placedmechanism::AxisField& field : fields) {
        NumberRow row;
        row.label = new QLabel(QStringLiteral("%1 (%2)").arg(QString::fromStdString(field.label),
                                                            QString::fromUtf8(field.unit)), this);
        auto* editor = new QDoubleSpinBox(this);
        // The package's own range, which is the point of the shared table: this is the same pair of
        // numbers `PlacedItemAxes::setValues` refuses on, so the box cannot offer a position the
        // mechanism will then reject.
        editor->setRange(field.minimum, field.maximum);
        editor->setSingleStep(field.step);
        editor->setDecimals(field.decimals);
        editor->setKeyboardTracking(false);
        connect(editor, &QAbstractSpinBox::editingFinished, this,
                &PlacedItemPropertiesPanel::commitAxes);
        row.editor = editor;
        const QString explanation = field.limited
            ? tr("%1 to %2 %3, which is what this package states.")
                  .arg(field.minimum, 0, 'f', field.decimals)
                  .arg(field.maximum, 0, 'f', field.decimals)
                  .arg(QString::fromUtf8(field.unit))
            : tr("This package states no limit for this axis, so nothing here is held to one.");
        row.label->setToolTip(explanation);
        editor->setToolTip(explanation);
        m_form->addRow(row.label, editor);
        m_axes.push_back(row);
    }
}

bool PlacedItemPropertiesPanel::operatorIsTyping() const {
    for (const NumberRow& row : m_placement) {
        if (row.editor && row.editor->hasFocus()) return true;
    }
    for (const NumberRow& row : m_axes) {
        if (row.editor && row.editor->hasFocus()) return true;
    }
    return m_name && m_name->hasFocus();
}

void PlacedItemPropertiesPanel::showNothing() {
    m_item = nullptr;
    m_syncing = true;
    m_heading->setText(tr("Nothing selected. Double-click a placed package in the station tree."));
    m_name->clear();
    m_name->setEnabled(false);
    for (const NumberRow& row : m_placement) {
        row.label->setVisible(false);
        row.editor->setVisible(false);
    }
    rebuildAxisRows({});
    m_axisHeading->setVisible(false);
    m_travelNote->setVisible(false);
    m_runNote->setVisible(false);
    m_syncing = false;
}

void PlacedItemPropertiesPanel::showPlacedItem(const PlacedItem* placed, bool runStopped) {
    if (!placed || !placed->item) {
        showNothing();
        return;
    }
    const bool sameItem = m_item == placed->item;
    m_item = placed->item;
    // A value the operator is part way through typing is theirs, not ours to overwrite.
    if (sameItem && operatorIsTyping()) return;

    const std::vector<placedmechanism::AxisField> fields =
        placed->axes ? placed->axes->fields()
                     : placedmechanism::axisFields(placed->scenery.root.get());
    m_syncing = true;

    bool shapeChanged = !sameItem || fields.size() != m_axisFields.size();
    for (size_t at = 0; !shapeChanged && at < fields.size(); ++at) {
        shapeChanged = fields[at].key != m_axisFields[at].key ||
                       fields[at].minimum != m_axisFields[at].minimum ||
                       fields[at].maximum != m_axisFields[at].maximum;
    }
    if (shapeChanged) rebuildAxisRows(fields);

    const placedmechanism::AxisKind kind =
        placedmechanism::axisKindOf(placed->scenery.root.get());
    m_heading->setText(kind == placedmechanism::AxisKind::Rail
                           ? tr("Linear rail")
                           : (kind == placedmechanism::AxisKind::Robot ? tr("Robot arm")
                                                                      : tr("Placed package")));
    m_name->setEnabled(true);
    if (m_name->text() != placed->name) m_name->setText(placed->name);

    const std::array<double, 6> six = placedmechanism::placementValues(placed->poseLocal);
    for (int at = 0; at < 6; ++at) {
        NumberRow& row = m_placement[at];
        row.label->setVisible(true);
        row.editor->setVisible(true);
        row.shown = six[static_cast<size_t>(at)];
        if (row.editor->value() != row.shown) row.editor->setValue(row.shown);
    }

    const std::vector<double> values = placed->axes ? placed->axes->values() : std::vector<double>();
    m_axisHeading->setVisible(!m_axes.empty());
    m_axisHeading->setText(kind == placedmechanism::AxisKind::Rail ? tr("Axis") : tr("Axes"));
    for (size_t at = 0; at < m_axes.size(); ++at) {
        NumberRow& row = m_axes[at];
        row.shown = at < values.size() ? values[at] : 0.0;
        if (row.editor->value() != row.shown) row.editor->setValue(row.shown);
    }
    if (m_axes.size() == 1 && m_axisFields.size() == 1 && m_axisFields[0].limited) {
        m_travelNote->setVisible(true);
        m_travelNote->setText(tr("Travel %1 to %2 %3.")
                                  .arg(m_axisFields[0].minimum, 0, 'f', m_axisFields[0].decimals)
                                  .arg(m_axisFields[0].maximum, 0, 'f', m_axisFields[0].decimals)
                                  .arg(QString::fromUtf8(m_axisFields[0].unit)));
    } else {
        m_travelNote->setVisible(false);
    }
    m_runNote->setVisible(!runStopped);
    m_syncing = false;
}

void PlacedItemPropertiesPanel::commitPlacement() {
    if (m_syncing || !m_item) return;
    std::array<double, 6> values{};
    bool moved = false;
    for (int at = 0; at < 6; ++at) {
        values[static_cast<size_t>(at)] = m_placement[at].editor->value();
        if (values[static_cast<size_t>(at)] != m_placement[at].shown) moved = true;
    }
    // Nothing typed is nothing to write, and this is not an optimisation: a committed move re-bakes the
    // item's triangles, so an unchanged focus-out would move it by its own rounding and dirty the station
    // every time the window was clicked through.
    if (!moved) return;
    for (int at = 0; at < 6; ++at) m_placement[at].shown = values[static_cast<size_t>(at)];
    emit placementEdited(m_item, placedmechanism::placementPose(values));
}

void PlacedItemPropertiesPanel::commitAxes() {
    if (m_syncing || !m_item || m_axes.empty()) return;
    std::vector<double> values;
    values.reserve(m_axes.size());
    bool moved = false;
    for (NumberRow& row : m_axes) {
        values.push_back(row.editor->value());
        if (values.back() != row.shown) moved = true;
    }
    if (!moved) return;
    for (size_t at = 0; at < m_axes.size(); ++at) m_axes[at].shown = values[at];
    emit axesEdited(m_item, values);
}
