#include "ConveyorPropertiesPanel.h"

#include "AccessoryPropertySchema.h"
#include "PlacedMechanismSchema.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QVBoxLayout>

namespace {

QString groupHeading(AccessoryFieldGroup group) {
    switch (group) {
    case AccessoryFieldGroup::Behaviour: return QObject::tr("Behaviour");
    case AccessoryFieldGroup::Queue: return QObject::tr("Queue");
    case AccessoryFieldGroup::Dimensions: return QObject::tr("Dimensions");
    case AccessoryFieldGroup::Corners: return QObject::tr("Deck corners");
    case AccessoryFieldGroup::Appearance: return QObject::tr("Appearance");
    }
    return QString();
}

QString labelFor(const AccessoryField& field) {
    const QString label = QString::fromUtf8(field.label);
    const QString unit = QString::fromUtf8(field.unit);
    return unit.isEmpty() ? label : QStringLiteral("%1 (%2)").arg(label, unit);
}

} // namespace

ConveyorPropertiesPanel::ConveyorPropertiesPanel(QWidget* parent) : QWidget(parent) {
    auto* column = new QVBoxLayout(this);
    m_heading = new QLabel(this);
    m_heading->setWordWrap(true);
    column->addWidget(m_heading);

    m_form = new QFormLayout();
    column->addLayout(m_form);

    m_name = new QLineEdit(this);
    m_form->addRow(tr("Name"), m_name);
    connect(m_name, &QLineEdit::editingFinished, this, [this]() {
        if (m_syncing || m_conveyor.isEmpty()) return;
        const QString wanted = m_name->text().trimmed();
        if (!wanted.isEmpty() && wanted != m_conveyor) emit renameRequested(m_conveyor, wanted);
    });

    // Said where the name is edited, because it is not recoverable by looking. A station's native
    // Wait for I/O holds "Box feed ready" as a literal string in the instruction, so renaming the
    // conveyor renames the variable and leaves the Wait naming one nothing writes.
    m_renameWarning = new QLabel(
        tr("Renaming this conveyor renames its three IO variables. A native Wait for I/O holds the "
           "old name as text and has to be edited by hand."), this);
    m_renameWarning->setWordWrap(true);
    m_renameWarning->setEnabled(false);
    column->addWidget(m_renameWarning);

    // Before the schema's rows, because where a machine stands is the first thing about it - and
    // because it is the only place a conveyor can be moved from by hand. RoboDK's own move tools reach
    // an item's pose, and a conveyor's item is a generic node whose pose RoboDK ignores.
    buildPlacementRows();
    buildRows();

    m_runNote = new QLabel(tr("Stop the run to edit these."), this);
    m_runNote->setWordWrap(true);
    m_runNote->setEnabled(false);
    column->addWidget(m_runNote);
    column->addStretch(1);

    showNothing();
}

void ConveyorPropertiesPanel::buildPlacementRows() {
    auto* separator = new QLabel(tr("Placement"), this);
    separator->setEnabled(false);
    m_form->addRow(separator);
    // Said once, on every row, because it is the difference between these numbers being wrong and their
    // being relative: dragging the frame a conveyor hangs under moves the conveyor and leaves all six
    // exactly as they are.
    const QString explanation =
        tr("Where the conveyor stands, relative to the RoboDK frame its item hangs under. Moving that "
           "frame moves the conveyor and leaves these unchanged.");
    const auto& fields = placedmechanism::placementFields();
    for (int at = 0; at < 6; ++at) {
        const placedmechanism::PlacementField& field = fields[static_cast<size_t>(at)];
        PlacementRow& row = m_placement[at];
        row.label = new QLabel(QStringLiteral("%1 (%2)").arg(QString::fromUtf8(field.label),
                                                             QString::fromUtf8(field.unit)), this);
        auto* editor = new QDoubleSpinBox(this);
        editor->setRange(-field.limit, field.limit);
        editor->setSingleStep(field.step);
        // Three, which is what RoboDK's own pose dialog shows and what the stored pose is compared at -
        // a placement rounded coarser than that would drift every time it was shown and committed.
        editor->setDecimals(3);
        editor->setKeyboardTracking(false);
        // On Enter or focus-out, never per keystroke: a committed move re-bakes the conveyor's geometry
        // and re-exports its prototype's mesh.
        connect(editor, &QAbstractSpinBox::editingFinished, this,
                &ConveyorPropertiesPanel::commitPlacement);
        row.editor = editor;
        row.label->setToolTip(explanation);
        editor->setToolTip(explanation);
        m_form->addRow(row.label, editor);
    }
}

void ConveyorPropertiesPanel::buildRows() {
    AccessoryFieldGroup heading = AccessoryFieldGroup::Behaviour;
    bool first = true;
    for (const AccessoryField& field : accessoryPropertySchema()) {
        if (first || field.group != heading) {
            auto* separator = new QLabel(groupHeading(field.group), this);
            separator->setEnabled(false);
            m_form->addRow(separator);
            heading = field.group;
            first = false;
        }

        Row row;
        row.field = &field;
        row.label = new QLabel(labelFor(field), this);
        switch (field.kind) {
        case AccessoryFieldKind::Number: {
            auto* editor = new QDoubleSpinBox(this);
            // Straight from the schema, which is where the geometry's own clamp reads them too, so a
            // value this box refuses is a value the builder would have refused.
            editor->setRange(field.minimum, field.maximum);
            editor->setSingleStep(field.step);
            editor->setDecimals(field.decimals);
            editor->setKeyboardTracking(false);
            // On Enter or focus-out, never per keystroke: a committed edit re-declares the conveyor,
            // which re-exports its prototype's mesh.
            connect(editor, &QAbstractSpinBox::editingFinished, this,
                    &ConveyorPropertiesPanel::commit);
            row.editor = editor;
            break;
        }
        case AccessoryFieldKind::Integer: {
            auto* editor = new QSpinBox(this);
            editor->setRange(static_cast<int>(field.minimum), static_cast<int>(field.maximum));
            editor->setKeyboardTracking(false);
            connect(editor, &QAbstractSpinBox::editingFinished, this,
                    &ConveyorPropertiesPanel::commit);
            row.editor = editor;
            break;
        }
        case AccessoryFieldKind::Toggle: {
            auto* editor = new QCheckBox(this);
            connect(editor, &QCheckBox::clicked, this, &ConveyorPropertiesPanel::commit);
            row.editor = editor;
            break;
        }
        case AccessoryFieldKind::Choice: {
            auto* editor = new QComboBox(this);
            for (const AccessoryFieldChoice& choice : field.choices) {
                editor->addItem(QString::fromUtf8(choice.label), QString::fromUtf8(choice.value));
            }
            // Overloaded in Qt 5.15 - int and QString - so the one that is wanted has to be named.
            connect(editor, QOverload<int>::of(&QComboBox::activated), this,
                    [this](int) { commit(); });
            row.editor = editor;
            break;
        }
        case AccessoryFieldKind::ItemReference: {
            auto* editor = new QComboBox(this);
            editor->setEditable(true);
            // Overloaded in Qt 5.15 - int and QString - so the one that is wanted has to be named.
            connect(editor, QOverload<int>::of(&QComboBox::activated), this,
                    [this](int) { commit(); });
            connect(editor->lineEdit(), &QLineEdit::editingFinished, this,
                    &ConveyorPropertiesPanel::commit);
            row.editor = editor;
            break;
        }
        }
        if (!QString::fromUtf8(field.tooltip).isEmpty()) {
            row.label->setToolTip(QString::fromUtf8(field.tooltip));
            row.editor->setToolTip(QString::fromUtf8(field.tooltip));
        }
        m_form->addRow(row.label, row.editor);
        m_rows.push_back(row);
    }
}

bool ConveyorPropertiesPanel::operatorIsTyping() const {
    for (const Row& row : m_rows) {
        if (row.editor && row.editor->hasFocus()) return true;
    }
    for (const PlacementRow& row : m_placement) {
        if (row.editor && row.editor->hasFocus()) return true;
    }
    return m_name->hasFocus();
}

void ConveyorPropertiesPanel::showNothing() {
    m_conveyor.clear();
    m_syncing = true;
    m_heading->setText(tr("No conveyor selected. Click one in the station tree."));
    m_name->clear();
    m_name->setEnabled(false);
    m_renameWarning->setVisible(false);
    m_runNote->setVisible(false);
    for (const Row& row : m_rows) {
        row.label->setVisible(false);
        row.editor->setVisible(false);
    }
    for (const PlacementRow& row : m_placement) {
        row.label->setVisible(false);
        row.editor->setVisible(false);
    }
    m_syncing = false;
}

void ConveyorPropertiesPanel::showConveyor(const RoboDkConveyorHost::Segment* segment,
                                           bool runStopped,
                                           const QStringList& workpieceCandidates) {
    if (!segment) {
        showNothing();
        return;
    }
    const bool sameConveyor = m_conveyor == segment->name;
    m_conveyor = segment->name;
    m_parameters = segment->parameters;
    if (sameConveyor && operatorIsTyping()) return;

    m_syncing = true;
    m_heading->setText(tr("Conveyor"));
    m_name->setEnabled(true);
    if (m_name->text() != segment->name) m_name->setText(segment->name);
    m_renameWarning->setVisible(true);
    m_runNote->setVisible(!runStopped);

    // A `spawner=` declaration has no item and no machine in the cell, so it has nowhere to stand and
    // these rows are about nothing. Every other conveyor has all six, mid-run included: the same move is
    // available by dragging the frame it hangs under, and its products are placed every step anyway.
    const std::array<double, 6> placement =
        placedmechanism::placementValues(segment->poseLocal);
    for (int at = 0; at < 6; ++at) {
        PlacementRow& row = m_placement[at];
        row.label->setVisible(segment->item != nullptr);
        row.editor->setVisible(segment->item != nullptr);
        if (!segment->item) continue;
        row.editor->setValue(placement[at]);
        row.shown = row.editor->value();
    }

    for (const Row& row : m_rows) {
        const AccessoryField& field = *row.field;
        const bool applies = accessoryFieldApplies(field, m_parameters);
        row.label->setVisible(applies);
        row.editor->setVisible(applies);
        if (!applies) continue;
        row.editor->setEnabled(runStopped || field.editableWhileRunning);
        row.label->setEnabled(row.editor->isEnabled());

        TransformNodeData& parameters = m_parameters;
        switch (field.kind) {
        case AccessoryFieldKind::Number:
            static_cast<QDoubleSpinBox*>(row.editor)->setValue(*field.number(parameters));
            break;
        case AccessoryFieldKind::Integer:
            static_cast<QSpinBox*>(row.editor)->setValue(*field.integer(parameters));
            break;
        case AccessoryFieldKind::Toggle:
            static_cast<QCheckBox*>(row.editor)->setChecked(*field.flag(parameters));
            break;
        case AccessoryFieldKind::Choice: {
            auto* editor = static_cast<QComboBox*>(row.editor);
            const QString value = QString::fromStdString(*field.text(parameters));
            const int at = editor->findData(value);
            if (at >= 0) editor->setCurrentIndex(at);
            break;
        }
        case AccessoryFieldKind::ItemReference: {
            auto* editor = static_cast<QComboBox*>(row.editor);
            const QString value = QString::fromStdString(*field.text(parameters));
            QStringList wanted = workpieceCandidates;
            if (!value.isEmpty() && !wanted.contains(value)) wanted.prepend(value);
            QStringList current;
            for (int index = 0; index < editor->count(); ++index) current << editor->itemText(index);
            if (current != wanted) {
                editor->clear();
                editor->addItems(wanted);
            }
            editor->setCurrentText(value);
            break;
        }
        }
    }
    m_syncing = false;
}

void ConveyorPropertiesPanel::commit() {
    if (m_syncing || m_conveyor.isEmpty()) return;
    TransformNodeData edited = m_parameters;
    for (const Row& row : m_rows) {
        const AccessoryField& field = *row.field;
        if (!row.editor->isVisible() || !row.editor->isEnabled()) continue;
        switch (field.kind) {
        case AccessoryFieldKind::Number: {
            double* member = field.number(edited);
            const double previous = *member;
            *member = static_cast<QDoubleSpinBox*>(row.editor)->value();
            if (field.couple && *member != previous) field.couple(edited, previous);
            break;
        }
        case AccessoryFieldKind::Integer:
            *field.integer(edited) = static_cast<QSpinBox*>(row.editor)->value();
            break;
        case AccessoryFieldKind::Toggle:
            *field.flag(edited) = static_cast<QCheckBox*>(row.editor)->isChecked();
            break;
        case AccessoryFieldKind::Choice:
            *field.text(edited) =
                static_cast<QComboBox*>(row.editor)->currentData().toString().toStdString();
            break;
        case AccessoryFieldKind::ItemReference:
            *field.text(edited) =
                static_cast<QComboBox*>(row.editor)->currentText().trimmed().toStdString();
            break;
        }
    }
    m_parameters = edited;
    emit parametersEdited(m_conveyor, edited);
}

void ConveyorPropertiesPanel::commitPlacement() {
    if (m_syncing || m_conveyor.isEmpty()) return;
    std::array<double, 6> values{};
    bool moved = false;
    for (int at = 0; at < 6; ++at) {
        values[at] = m_placement[at].editor->value();
        if (values[at] != m_placement[at].shown) moved = true;
    }
    // Nothing typed is nothing to write, and this is not an optimisation. Six numbers read back out of
    // the boxes they were rounded into do not rebuild the pose they came from, so committing a
    // focus-out that changed nothing would move the conveyor a little and mark the station dirty -
    // every time an operator clicked through the panel.
    if (!moved) return;
    for (int at = 0; at < 6; ++at) m_placement[at].shown = values[at];
    emit placementEdited(m_conveyor, placedmechanism::placementPose(values));
}
