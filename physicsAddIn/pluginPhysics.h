#ifndef PLUGINPHYSICS_H
#define PLUGINPHYSICS_H


#include <QObject>
#include <QtPlugin>
#include <QDockWidget>
#include "iapprobodk.h"
#include "robodktypes.h"
#include "PhysXEngine.h"
#include "MaterialManager.h"
#include "ObjectPropertiesManager.h"
#include "SceneConfigurationDialog.h"

class QToolBar;
class QMenu;
class QAction;
class IRoboDK;
class IItem;

///
/// \brief The PluginPhysics class provides physics simulation capabilities for RoboDK.
/// A RoboDK plugin must implement the IAppRoboDK and the QObject class.
///
class PluginPhysics : public QObject, IAppRoboDK
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "RoboDK.IAppRoboDK")// FILE "metadatalugin.json")
    Q_INTERFACES(IAppRoboDK)

public:
    //------------------------------- RoboDK Plug-in Interface commands ------------------------------
    static QString getPluginName();
    QString PluginName(void) override;    
    virtual QString PluginLoad(QMainWindow *mw, QMenuBar *menubar, QStatusBar *statusbar, RoboDK *rdk, const QString &settings="") override;
    virtual void PluginUnload() override;
    virtual void PluginLoadToolbar(QMainWindow *mw, int icon_size) override;
    virtual bool PluginItemClick(Item item, QMenu *menu, TypeClick click_type) override;
    virtual QString PluginCommand(const QString &command, const QString &value) override;
    virtual void PluginEvent(TypeEvent event_type) override;

    //----------------------------------------------------------------------------------

// Recommended pointers to use in your plugin:
public:
    /// RoboDK's <strong>main window</strong> pointer.
    QMainWindow *MainWindow;

    /// RoboDK's main <strong>status bar</strong> pointer.
    QStatusBar *StatusBar;

    /// Pointer to the <strong>RoboDK API</strong> interface.
    RoboDK *RDK;

    QAction *actionSceneConfig;
    QAction *actionMaterialManager;
    QAction *actionCreateSoftBody;

public slots:

private:
    void showMaterialManager();
    void showObjectProperties(Item item = nullptr);
    void showSoftBodyDialog();
    void testVHACDIntegration();
    
    // Physics engine and managers
    PhysXEngine* m_physicsEngine;
    MaterialManager* m_materialManager;
    ObjectPropertiesManager* m_objectPropertiesManager;
    SceneConfigurationDialog* m_sceneConfigDialog;

};
//! [0]


#endif // PLUGINPHYSICS_H 