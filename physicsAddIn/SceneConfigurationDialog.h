#ifndef SCENECONFIGURATIONDIALOG_H
#define SCENECONFIGURATIONDIALOG_H

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <QSlider>
#include <QTabWidget>
#include <QSpinBox>
#include <QMessageBox>
#include "PxPhysicsAPI.h"
#include "IPhysicsEngine.h"
#include "MaterialManager.h"

class IPhysicsEngine;
class MaterialManager;

/**
 * @brief Dialog for configuring scene-level physics settings
 */
class SceneConfigurationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SceneConfigurationDialog(IPhysicsEngine* physicsEngine, MaterialManager* materialManager = nullptr, QWidget* parent = nullptr);
    ~SceneConfigurationDialog();

    // Configuration getters
    PxVec3 getGravity() const;
    float getGlobalStaticFriction() const;
    float getGlobalDynamicFriction() const;
    float getGlobalRestitution() const;
    QString getSelectedDefaultMaterial() const;
    int getSolverIterations() const;
    int getSolverVelocityIterations() const;
    bool getPCMEnabled() const;
    bool getStabilizationEnabled() const;
    float getContactOffset() const;
    float getRestOffset() const;
    float getWakeDistance() const;

signals:
    void configurationChanged();

private slots:
    void onApplyClicked();
    void onResetClicked();
    void onCancelClicked();
    void onEngineTypeChanged(int index);
    void onGravityChanged();
    void onGlobalMaterialChanged();
    void onDefaultMaterialChanged(int index);
    void onSolverSettingsChanged();
    void onAdvancedSettingsChanged();
    void onMaterialAdded(const QString& materialName);
    void onMaterialRemoved(const QString& materialName);
    void onMaterialUpdated(const QString& materialName);
    void saveCustomMaterial();
    void onDebugVisualizationChanged(bool enabled);

private:
    void setupUI();
    void setupGeneralTab();
    void setupSolverTab();
    void setupAdvancedTab();
    void connectSignals();
    void loadCurrentConfiguration();
    void applyConfiguration();
    void validateInputs();
    void updateMaterialDropdown();
    
    // UI components
    QTabWidget* m_tabWidget;
    
    // General tab
    QWidget* m_generalTab;
    QComboBox* m_engineTypeCombo;
    QDoubleSpinBox* m_gravityXSpin;
    QDoubleSpinBox* m_gravityYSpin;
    QDoubleSpinBox* m_gravityZSpin;
    QComboBox* m_defaultMaterialCombo;
    QDoubleSpinBox* m_globalStaticFrictionSpin;
    QDoubleSpinBox* m_globalDynamicFrictionSpin;
    QDoubleSpinBox* m_globalRestitutionSpin;
    
    // Solver tab
    QWidget* m_solverTab;
    QSpinBox* m_solverIterationsSpin;
    QSpinBox* m_solverVelocityIterationsSpin;
    QCheckBox* m_pcmEnabledCheck;
    QCheckBox* m_stabilizationEnabledCheck;
    QDoubleSpinBox* m_contactOffsetSpin;
    QDoubleSpinBox* m_restOffsetSpin;
    
    // Advanced tab
    QWidget* m_advancedTab;
    QDoubleSpinBox* m_sleepThresholdSpin;
    QDoubleSpinBox* m_stabilizationThresholdSpin;
    QDoubleSpinBox* m_wakeDistanceSpin;
    QCheckBox* m_ccdEnabledCheck;
    QCheckBox* m_debugVisualizationCheck;
    QSpinBox* m_maxSubStepsSpin;
    QDoubleSpinBox* m_fixedTimeStepSpin;
    
    // Buttons
    QPushButton* m_applyButton;
    QPushButton* m_resetButton;
    QPushButton* m_cancelButton;
    
    // Data
    IPhysicsEngine* m_physicsEngine;
    MaterialManager* m_materialManager;
    
    // Original values for reset
    PxVec3 m_originalGravity;
    float m_originalGlobalStaticFriction;
    float m_originalGlobalDynamicFriction;
    float m_originalGlobalRestitution;
    int m_originalSolverIterations;
    int m_originalSolverVelocityIterations;
    bool m_originalPCMEnabled;
    bool m_originalStabilizationEnabled;
    float m_originalContactOffset;
    float m_originalRestOffset;
    float m_originalSleepThreshold;
    float m_originalStabilizationThreshold;
    float m_originalWakeDistance;
    bool m_originalCCDEnabled;
    bool m_originalDebugVisualization;
    int m_originalMaxSubSteps;
    float m_originalFixedTimeStep;
    
    // Constants
    static constexpr float MIN_GRAVITY = -10000.0f;
    static constexpr float MAX_GRAVITY = 10000.0f;
    static constexpr float MIN_FRICTION = 0.0f;
    static constexpr float MAX_FRICTION = 2.0f;
    static constexpr float MIN_RESTITUTION = 0.0f;
    static constexpr float MAX_RESTITUTION = 1.0f;
    static constexpr int MIN_SOLVER_ITERATIONS = 1;
    static constexpr int MAX_SOLVER_ITERATIONS = 255;
    static constexpr int MIN_SOLVER_VELOCITY_ITERATIONS = 1;
    static constexpr int MAX_SOLVER_VELOCITY_ITERATIONS = 255;
    static constexpr float MIN_OFFSET = 0.0f;
    static constexpr float MAX_OFFSET = 100.0f;
    static constexpr float MIN_THRESHOLD = 0.0f;
    static constexpr float MAX_THRESHOLD = 1000.0f;
    static constexpr float MIN_WAKE_DISTANCE = 0.0f;
    static constexpr float MAX_WAKE_DISTANCE = 10000.0f;
    static constexpr int MIN_SUB_STEPS = 1;
    static constexpr int MAX_SUB_STEPS = 100;
    static constexpr float MIN_TIME_STEP = 0.001f;
    static constexpr float MAX_TIME_STEP = 1.0f;
};

#endif // SCENECONFIGURATIONDIALOG_H 