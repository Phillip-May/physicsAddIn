#ifndef LIBRARYDOCK_H
#define LIBRARYDOCK_H

#include "LibraryCatalogue.h"

#include <QDockWidget>
#include <QString>

#include <vector>

class QLabel;
class QLineEdit;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;
class PhysicsWorld;

class LibraryDock : public QDockWidget {
    Q_OBJECT

public:
    LibraryDock(PhysicsWorld* world, QWidget* parent = nullptr);

    // Re-scans the library. Costs a package load per entry, so it is called when the dock is shown, when
    // the library root changes and when the button is pressed - never on a timer.
    void reload();
    void setStatus(const QString& text);
    // Drops the selection, for when a placement has been committed or cancelled: a row that stays
    // highlighted says something is still armed.
    void clearArmedRow();

signals:
    // An armable row was chosen. The entry is one of this dock's own, which outlives the signal.
    void armRequested(const librarycatalogue::Entry& entry);

protected:
    // Keeps a drag that began on a row from walking the selection across the list. See the definition.
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void buildUi();
    void applyFilter();
    void chose(QTreeWidgetItem* row);

    PhysicsWorld* m_world = nullptr;
    QTreeWidget* m_list = nullptr;
    QLineEdit* m_search = nullptr;
    QPushButton* m_reload = nullptr;
    QLabel* m_root = nullptr;
    QLabel* m_status = nullptr;
    // Held because a row points into it by index, and because an armed entry's tree is what the ghost
    // draws. Rebuilt only by `reload()`.
    std::vector<librarycatalogue::Entry> m_entries;
};

#endif // LIBRARYDOCK_H
