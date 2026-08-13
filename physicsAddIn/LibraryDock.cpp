#include "LibraryDock.h"

#include "LibraryPlacer.h"
#include "PhysicsIcons.h"
#include "PhysicsWorld.h"

#include <QBrush>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace {

// reload() invalidates entry pointers, so rows store indices.
constexpr int kEntryRole = Qt::UserRole + 1;
constexpr int kRefusalRole = Qt::UserRole + 2;

QString standingHint() {
    return LibraryDock::tr("Click an entry to arm it, then point in the 3D view. The wheel turns it, Esc "
                           "cancels. It mates this plugin's own conveyors and packages, and the floor - "
                           "RoboDK's own objects carry no mounting holes.");
}

} // namespace

LibraryDock::LibraryDock(PhysicsWorld* world, QWidget* parent)
    : QDockWidget(tr("Physics Library"), parent), m_world(world) {
    setObjectName(QStringLiteral("PhysicsLibraryDock"));
    setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    buildUi();
}

void LibraryDock::buildUi() {
    QWidget* contents = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(contents);

    m_root = new QLabel(contents);
    m_root->setWordWrap(true);
    layout->addWidget(m_root);

    QHBoxLayout* top = new QHBoxLayout();
    m_search = new QLineEdit(contents);
    m_search->setPlaceholderText(tr("Search"));
    m_search->setClearButtonEnabled(true);
    m_reload = new QPushButton(tr("Rescan"), contents);
    top->addWidget(m_search, 1);
    top->addWidget(m_reload);
    layout->addLayout(top);

    m_list = new QTreeWidget(contents);
    m_list->setColumnCount(3);
    m_list->setHeaderLabels({tr("Type"), tr("Name"), tr("Notes")});
    m_list->setRootIsDecorated(false);
    m_list->setSelectionMode(QAbstractItemView::SingleSelection);
    m_list->header()->setStretchLastSection(true);
    layout->addWidget(m_list, 1);

    m_status = new QLabel(contents);
    m_status->setWordWrap(true);
    m_status->setText(standingHint());
    layout->addWidget(m_status);

    setWidget(contents);

    connect(m_search, &QLineEdit::textChanged, this, [this]() { applyFilter(); });
    connect(m_reload, &QPushButton::clicked, this, [this]() { reload(); });
    // Arm on press so drag-out placement starts before the pointer leaves the row.
    connect(m_list, &QTreeWidget::itemPressed, this,
            [this](QTreeWidgetItem* row, int) { chose(row); });
    connect(m_list, &QTreeWidget::itemActivated, this,
            [this](QTreeWidgetItem* row, int) { chose(row); });
    m_list->viewport()->installEventFilter(this);
}

bool LibraryDock::eventFilter(QObject* watched, QEvent* event) {
    // Preserve the pressed row while dragging out of the list.
    if (m_list && watched == m_list->viewport() && event && event->type() == QEvent::MouseMove &&
        static_cast<QMouseEvent*>(event)->buttons() != Qt::NoButton) {
        return true;
    }
    return QDockWidget::eventFilter(watched, event);
}

void LibraryDock::reload() {
    if (!m_world || !m_list) return;
    const QString root = m_world->libraryRoot();
    m_root->setText(root.isEmpty()
                        ? tr("No library root. Set the station parameter PhysicsLibraryRoot, or send the "
                             "plugin command 'libraryroot <path>'.")
                        : tr("Library: %1").arg(root));

    m_entries = m_world->libraryCatalogue();
    m_list->clear();
    for (size_t index = 0; index < m_entries.size(); ++index) {
        const librarycatalogue::Entry& entry = m_entries[index];
        const QString refusal = dockRefusal(entry);
        auto* row = new QTreeWidgetItem(m_list);
        row->setText(0, QString::fromLatin1(librarycatalogue::categoryLabel(entry.category)));
        row->setIcon(0, libraryCategoryIcon(entry.category));
        row->setText(1, QString::fromStdString(entry.name));
        row->setText(2, refusal);
        row->setData(0, kEntryRole, static_cast<int>(index));
        row->setData(0, kRefusalRole, refusal);
        if (!refusal.isEmpty()) row->setForeground(1, QBrush(Qt::gray));
        const QString detail = QString::fromStdString(entry.path) +
            (entry.variantId == "default" ? QString()
                                          : QStringLiteral("\n%1").arg(
                                                QString::fromStdString(entry.variantId)));
        for (int column = 0; column < 3; ++column) {
            row->setToolTip(column, refusal.isEmpty() ? detail
                                                      : detail + QStringLiteral("\n\n") + refusal);
        }
    }
    m_list->resizeColumnToContents(0);
    applyFilter();
}

void LibraryDock::applyFilter() {
    if (!m_list) return;
    const std::string wanted = m_search ? m_search->text().trimmed().toStdString() : std::string();
    for (int at = 0; at < m_list->topLevelItemCount(); ++at) {
        QTreeWidgetItem* row = m_list->topLevelItem(at);
        const int index = row->data(0, kEntryRole).toInt();
        const bool shown = index >= 0 && index < static_cast<int>(m_entries.size()) &&
                           librarycatalogue::matches(m_entries[static_cast<size_t>(index)], nullptr,
                                                     wanted);
        row->setHidden(!shown);
    }
}

void LibraryDock::chose(QTreeWidgetItem* row) {
    if (!row) return;
    const QString refusal = row->data(0, kRefusalRole).toString();
    if (!refusal.isEmpty()) {
        setStatus(tr("%1 cannot be placed as a drawn item: %2").arg(row->text(1), refusal));
        return;
    }
    const int index = row->data(0, kEntryRole).toInt();
    if (index < 0 || index >= static_cast<int>(m_entries.size())) return;
    emit armRequested(m_entries[static_cast<size_t>(index)]);
}

void LibraryDock::setStatus(const QString& text) {
    if (m_status) m_status->setText(text.isEmpty() ? standingHint() : text);
}

void LibraryDock::clearArmedRow() {
    if (m_list) m_list->clearSelection();
}
