#ifndef PLUGINMAIN_H
#define PLUGINMAIN_H

#include <QObject>
#include <memory>
#include "iapprobodk.h"
#include "robodktypes.h"

// Forward declarations
class QMainWindow;
class QMenuBar;
class QStatusBar;
class QAction;
class QMenu;
class RoboDK;
class Item;

// Module forward declarations
class PhysicsEngine;
class RoboDKPhysicsBridge;
class PluginUI;
class ConfigManager;

/**
 * @brief Main plugin class - simplified and focused on plugin lifecycle
 * 
 * This class handles the plugin lifecycle and delegates specific functionality
 * to specialized modules. It serves as the main entry point and coordinator.
 */
class PluginMain : public QObject, IAppRoboDK
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "RoboDK.IAppRoboDK")
    Q_INTERFACES(IAppRoboDK)

public:
    explicit PluginMain(QObject* parent = nullptr);
    ~PluginMain();

    // RoboDK Plugin Interface
    static QString getPluginName();
    QString PluginName() override;
    QString PluginLoad(QMainWindow* mw, QMenuBar* menubar, QStatusBar* statusbar, 
                      RoboDK* rdk, const QString& settings = "") override;
    void PluginUnload() override;
    void PluginLoadToolbar(QMainWindow* mw, int icon_size) override;
    bool PluginItemClick(Item item, QMenu* menu, TypeClick click_type) override;
    QString PluginCommand(const QString& command, const QString& value) override;
    void PluginEvent(TypeEvent event_type) override;

    // Plugin state
    bool isInitialized() const;
    bool isSimulationRunning() const;

    // Configuration
    void loadConfiguration();
    void saveConfiguration();

signals:
    void pluginInitialized();
    void pluginUnloaded();
    void simulationStarted();
    void simulationStopped();
    void error(const QString& error);

private slots:
    void onPhysicsEngineError(const QString& error);
    void onBridgeError(const QString& error);
    void onSimulationStarted();
    void onSimulationStopped();

private:
    // Core components
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<RoboDKPhysicsBridge> m_bridge;
    std::unique_ptr<PluginUI> m_ui;
    std::unique_ptr<ConfigManager> m_config;
    
    // RoboDK references
    RoboDK* m_robodk;
    QMainWindow* m_mainWindow;
    QStatusBar* m_statusBar;
    
    // Plugin state
    bool m_initialized;
    bool m_simulationRunning;
    
    // Initialization helpers
    bool initializeComponents();
    void cleanupComponents();
    bool setupUI();
    bool setupPhysics();
    bool setupBridge();
    
    // Configuration helpers
    void loadDefaultConfiguration();
    void validateConfiguration();
    
    // Error handling
    void handleError(const QString& error, const QString& context);
    void logError(const QString& error);
    
    // Constants
    static constexpr const char* PLUGIN_NAME = "Physics Simulation";
    static constexpr const char* CONFIG_FILE = "physics_config.json";
};

#endif // PLUGINMAIN_H 