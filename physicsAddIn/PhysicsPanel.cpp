#include "PhysicsPanel.h"

#include "iitem.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>

#include <unordered_map>

PhysicsPanel::PhysicsPanel(PhysicsWorld* world, QWidget* parent)
    : QDockWidget(tr("Physics"), parent), m_world(world) {
    setObjectName(QStringLiteral("PhysicsSimulationDock"));
    setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    buildUi();
    refresh();
}

void PhysicsPanel::buildUi() {
    QWidget* contents = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(contents);

    m_items = new QTreeWidget(contents);
    m_items->setColumnCount(3);
    m_items->setHeaderLabels({tr("Item"), tr("Role"), tr("State")});
    m_items->setRootIsDecorated(false);
    m_items->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_items->header()->setStretchLastSection(true);
    layout->addWidget(m_items, 1);

    QHBoxLayout* buttons = new QHBoxLayout();
    m_start = new QPushButton(tr("Start"), contents);
    m_pause = new QPushButton(tr("Pause"), contents);
    m_stop = new QPushButton(tr("Stop"), contents);
    m_remove = new QPushButton(tr("Remove"), contents);
    buttons->addWidget(m_start);
    buttons->addWidget(m_pause);
    buttons->addWidget(m_stop);
    buttons->addStretch();
    buttons->addWidget(m_remove);
    layout->addLayout(buttons);

    QFormLayout* settings = new QFormLayout();
    m_gravity = new QDoubleSpinBox(contents);
    m_gravity->setRange(-50.0, 50.0);
    m_gravity->setDecimals(2);
    m_gravity->setSingleStep(0.5);
    // RoboDK is Z-up, so gravity is the Z component and downward is negative.
    m_gravity->setValue(-9.81);
    m_gravity->setSuffix(tr(" m/s2 (Z)"));
    settings->addRow(tr("Gravity"), m_gravity);
    layout->addLayout(settings);

    m_status = new QLabel(contents);
    m_status->setWordWrap(true);
    layout->addWidget(m_status);

    setWidget(contents);

    connect(m_start, &QPushButton::clicked, this, [this]() {
        QString error;
        applyGravity();
        m_world->start(&error);
        refresh();
        emit runStateChanged();
    });

    connect(m_pause, &QPushButton::clicked, this, [this]() {
        if (m_world->state() == PhysicsWorld::State::Paused) {
            m_world->resume();
        } else {
            m_world->pause();
        }
        refresh();
        emit runStateChanged();
    });

    connect(m_stop, &QPushButton::clicked, this, [this]() {
        m_world->stop();
        refresh();
        emit runStateChanged();
    });

    connect(m_remove, &QPushButton::clicked, this, [this]() {
        removeSelected();
        refresh();
        emit runStateChanged();
    });
}

void PhysicsPanel::applyGravity() {
    m_world->setGravity(CadVec3{0.0, 0.0, m_gravity->value()});
}

void PhysicsPanel::removeSelected() {
    std::vector<Item> doomed;
    for (QTreeWidgetItem* row : m_items->selectedItems()) {
        const auto stored = row->data(0, Qt::UserRole).value<quintptr>();
        if (stored != 0) doomed.push_back(reinterpret_cast<Item>(stored));
    }
    for (Item item : doomed) m_world->forget(item);
}

void PhysicsPanel::syncItems() {
    // Rows are keyed by the participant's Item, which is the only identity a row has: names repeat
    // in this station and a spawned clone is not in the tree at all until it exists.
    std::unordered_map<quintptr, const PhysicsWorld::Participant*> live;
    for (const PhysicsWorld::Participant& participant : m_world->participants()) {
        live.emplace(reinterpret_cast<quintptr>(participant.item), &participant);
    }

    for (int index = m_items->topLevelItemCount() - 1; index >= 0; --index) {
        QTreeWidgetItem* row = m_items->topLevelItem(index);
        const auto key = row->data(0, Qt::UserRole).value<quintptr>();
        const auto hit = live.find(key);
        if (hit == live.end()) {
            delete m_items->takeTopLevelItem(index);
            continue;
        }
        const PhysicsWorld::Participant& participant = *hit->second;
        // Written only when they differ: setText on an unchanged cell still repaints the row.
        if (row->text(0) != participant.name) row->setText(0, participant.name);
        const QString role = PhysicsWorld::roleName(participant.role);
        if (row->text(1) != role) row->setText(1, role);
        if (row->text(2) != participant.status) row->setText(2, participant.status);
        live.erase(hit);
    }

    if (!live.empty()) {
        for (const PhysicsWorld::Participant& participant : m_world->participants()) {
            const auto key = reinterpret_cast<quintptr>(participant.item);
            if (live.find(key) == live.end()) continue;
            QTreeWidgetItem* row = new QTreeWidgetItem(m_items);
            row->setText(0, participant.name);
            row->setText(1, PhysicsWorld::roleName(participant.role));
            row->setText(2, participant.status);
            row->setData(0, Qt::UserRole, QVariant::fromValue(key));
        }
        m_items->resizeColumnToContents(0);
        m_items->resizeColumnToContents(1);
    }
}

void PhysicsPanel::refresh() {
    m_items->clear();
    syncItems();
    m_items->resizeColumnToContents(0);
    m_items->resizeColumnToContents(1);

    const PhysicsWorld::State state = m_world->state();
    const bool stopped = state == PhysicsWorld::State::Stopped;
    m_start->setEnabled(stopped || state == PhysicsWorld::State::Paused);
    m_pause->setEnabled(!stopped);
    m_pause->setText(state == PhysicsWorld::State::Paused ? tr("Resume") : tr("Pause"));
    m_stop->setEnabled(!stopped);
    // Editing the cast while it is running would desynchronise the world from the scene it built.
    m_remove->setEnabled(stopped);
    m_gravity->setEnabled(stopped);
    refreshStatus();
}

void PhysicsPanel::refreshStatus() {
    m_status->setText(m_world->summary());
}
