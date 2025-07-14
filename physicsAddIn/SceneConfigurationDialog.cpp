#include "SceneConfigurationDialog.h"
#include "PhysXEngine.h"
#include "MaterialManager.h"
#include <QDebug>
#include <QMessageBox>

SceneConfigurationDialog::SceneConfigurationDialog(IPhysicsEngine* physicsEngine, MaterialManager* materialManager, QWidget* parent)
    : QDialog(parent)
    , m_physicsEngine(physicsEngine)
    , m_materialManager(materialManager)
    , m_tabWidget(nullptr)
    , m_generalTab(nullptr)
    , m_solverTab(nullptr)
    , m_advancedTab(nullptr)
    , m_applyButton(nullptr)
    , m_resetButton(nullptr)
    , m_cancelButton(nullptr)
{
    setWindowTitle(tr("Scene Default Configuration"));
    setModal(true);
    setMinimumSize(500, 400);
    
    setupUI();
    connectSignals();
    loadCurrentConfiguration();
}

SceneConfigurationDialog::~SceneConfigurationDialog()
{
}

void SceneConfigurationDialog::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    
    // Create tab widget
    m_tabWidget = new QTabWidget(this);
    mainLayout->addWidget(m_tabWidget);
    
    // Setup tabs
    setupGeneralTab();
    setupSolverTab();
    setupAdvancedTab();
    
    // Create button layout
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    
    m_applyButton = new QPushButton(tr("Apply"), this);
    m_resetButton = new QPushButton(tr("Reset"), this);
    m_cancelButton = new QPushButton(tr("Cancel"), this);
    
    buttonLayout->addWidget(m_applyButton);
    buttonLayout->addWidget(m_resetButton);
    buttonLayout->addWidget(m_cancelButton);
    
    mainLayout->addLayout(buttonLayout);
}

void SceneConfigurationDialog::setupGeneralTab()
{
    m_generalTab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(m_generalTab);
    
    // Engine type group
    QGroupBox* engineGroup = new QGroupBox(tr("Physics Engine"), m_generalTab);
    QFormLayout* engineLayout = new QFormLayout(engineGroup);
    
    m_engineTypeCombo = new QComboBox(engineGroup);
    m_engineTypeCombo->addItem(tr("PhysX 5.x"), "PhysX");
    m_engineTypeCombo->setEnabled(false); // Currently only PhysX is supported
    
    engineLayout->addRow(tr("Engine Type:"), m_engineTypeCombo);
    layout->addWidget(engineGroup);
    
    // Gravity group
    QGroupBox* gravityGroup = new QGroupBox(tr("Gravity"), m_generalTab);
    QFormLayout* gravityLayout = new QFormLayout(gravityGroup);
    
    m_gravityXSpin = new QDoubleSpinBox(gravityGroup);
    m_gravityXSpin->setRange(MIN_GRAVITY, MAX_GRAVITY);
    m_gravityXSpin->setSuffix(tr(" mm/s²"));
    m_gravityXSpin->setDecimals(2);
    
    m_gravityYSpin = new QDoubleSpinBox(gravityGroup);
    m_gravityYSpin->setRange(MIN_GRAVITY, MAX_GRAVITY);
    m_gravityYSpin->setSuffix(tr(" mm/s²"));
    m_gravityYSpin->setDecimals(2);
    
    m_gravityZSpin = new QDoubleSpinBox(gravityGroup);
    m_gravityZSpin->setRange(MIN_GRAVITY, MAX_GRAVITY);
    m_gravityZSpin->setSuffix(tr(" mm/s²"));
    m_gravityZSpin->setDecimals(2);
    
    gravityLayout->addRow(tr("X:"), m_gravityXSpin);
    gravityLayout->addRow(tr("Y:"), m_gravityYSpin);
    gravityLayout->addRow(tr("Z:"), m_gravityZSpin);
    layout->addWidget(gravityGroup);
    
    // Global material properties group
    QGroupBox* materialGroup = new QGroupBox(tr("Default Material Properties"), m_generalTab);
    QFormLayout* materialLayout = new QFormLayout(materialGroup);
    
    // Default material dropdown
    m_defaultMaterialCombo = new QComboBox(materialGroup);
    updateMaterialDropdown();
    
    // Custom material spinboxes (initially enabled)
    m_globalStaticFrictionSpin = new QDoubleSpinBox(materialGroup);
    m_globalStaticFrictionSpin->setRange(MIN_FRICTION, MAX_FRICTION);
    m_globalStaticFrictionSpin->setDecimals(3);
    m_globalStaticFrictionSpin->setSingleStep(0.1);
    
    m_globalDynamicFrictionSpin = new QDoubleSpinBox(materialGroup);
    m_globalDynamicFrictionSpin->setRange(MIN_FRICTION, MAX_FRICTION);
    m_globalDynamicFrictionSpin->setDecimals(3);
    m_globalDynamicFrictionSpin->setSingleStep(0.1);
    
    m_globalRestitutionSpin = new QDoubleSpinBox(materialGroup);
    m_globalRestitutionSpin->setRange(MIN_RESTITUTION, MAX_RESTITUTION);
    m_globalRestitutionSpin->setDecimals(3);
    m_globalRestitutionSpin->setSingleStep(0.1);
    
    materialLayout->addRow(tr("Default Material:"), m_defaultMaterialCombo);
    materialLayout->addRow(tr("Static Friction:"), m_globalStaticFrictionSpin);
    materialLayout->addRow(tr("Dynamic Friction:"), m_globalDynamicFrictionSpin);
    materialLayout->addRow(tr("Restitution:"), m_globalRestitutionSpin);
    layout->addWidget(materialGroup);
    
    layout->addStretch();
    m_tabWidget->addTab(m_generalTab, tr("General"));
}

void SceneConfigurationDialog::setupSolverTab()
{
    m_solverTab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(m_solverTab);
    
    // Solver iterations group
    QGroupBox* solverGroup = new QGroupBox(tr("Default Solver Settings"), m_solverTab);
    QFormLayout* solverLayout = new QFormLayout(solverGroup);
    
    m_solverIterationsSpin = new QSpinBox(solverGroup);
    m_solverIterationsSpin->setRange(MIN_SOLVER_ITERATIONS, MAX_SOLVER_ITERATIONS);
    m_solverIterationsSpin->setSuffix(tr(" iterations"));
    
    m_solverVelocityIterationsSpin = new QSpinBox(solverGroup);
    m_solverVelocityIterationsSpin->setRange(MIN_SOLVER_VELOCITY_ITERATIONS, MAX_SOLVER_VELOCITY_ITERATIONS);
    m_solverVelocityIterationsSpin->setSuffix(tr(" iterations"));
    
    solverLayout->addRow(tr("Position Iterations:"), m_solverIterationsSpin);
    solverLayout->addRow(tr("Velocity Iterations:"), m_solverVelocityIterationsSpin);
    layout->addWidget(solverGroup);
    
    // Contact settings group
    QGroupBox* contactGroup = new QGroupBox(tr("Default Contact Settings"), m_solverTab);
    QFormLayout* contactLayout = new QFormLayout(contactGroup);
    
    m_pcmEnabledCheck = new QCheckBox(tr("Enable PCM (Persistent Contact Manifold)"), contactGroup);
    m_stabilizationEnabledCheck = new QCheckBox(tr("Enable Stabilization"), contactGroup);
    
    m_contactOffsetSpin = new QDoubleSpinBox(contactGroup);
    m_contactOffsetSpin->setRange(MIN_OFFSET, MAX_OFFSET);
    m_contactOffsetSpin->setSuffix(tr(" mm"));
    m_contactOffsetSpin->setDecimals(2);
    
    m_restOffsetSpin = new QDoubleSpinBox(contactGroup);
    m_restOffsetSpin->setRange(MIN_OFFSET, MAX_OFFSET);
    m_restOffsetSpin->setSuffix(tr(" mm"));
    m_restOffsetSpin->setDecimals(2);
    
    contactLayout->addRow(m_pcmEnabledCheck);
    contactLayout->addRow(m_stabilizationEnabledCheck);
    contactLayout->addRow(tr("Contact Offset:"), m_contactOffsetSpin);
    contactLayout->addRow(tr("Rest Offset:"), m_restOffsetSpin);
    layout->addWidget(contactGroup);
    
    layout->addStretch();
    m_tabWidget->addTab(m_solverTab, tr("Solver"));
}

void SceneConfigurationDialog::setupAdvancedTab()
{
    m_advancedTab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(m_advancedTab);
    
    // Sleep settings group
    QGroupBox* sleepGroup = new QGroupBox(tr("Default Sleep Settings"), m_advancedTab);
    QFormLayout* sleepLayout = new QFormLayout(sleepGroup);
    
    m_sleepThresholdSpin = new QDoubleSpinBox(sleepGroup);
    m_sleepThresholdSpin->setRange(MIN_THRESHOLD, MAX_THRESHOLD);
    m_sleepThresholdSpin->setSuffix(tr(" mm/s"));
    m_sleepThresholdSpin->setDecimals(2);
    
    m_stabilizationThresholdSpin = new QDoubleSpinBox(sleepGroup);
    m_stabilizationThresholdSpin->setRange(MIN_THRESHOLD, MAX_THRESHOLD);
    m_stabilizationThresholdSpin->setSuffix(tr(" mm/s"));
    m_stabilizationThresholdSpin->setDecimals(2);
    
    m_wakeDistanceSpin = new QDoubleSpinBox(sleepGroup);
    m_wakeDistanceSpin->setRange(MIN_WAKE_DISTANCE, MAX_WAKE_DISTANCE);
    m_wakeDistanceSpin->setSuffix(tr(" mm"));
    m_wakeDistanceSpin->setDecimals(2);
    
    sleepLayout->addRow(tr("Sleep Threshold:"), m_sleepThresholdSpin);
    sleepLayout->addRow(tr("Stabilization Threshold:"), m_stabilizationThresholdSpin);
    sleepLayout->addRow(tr("Wake Distance:"), m_wakeDistanceSpin);
    layout->addWidget(sleepGroup);
    
    // Advanced features group
    QGroupBox* featuresGroup = new QGroupBox(tr("Default Advanced Features"), m_advancedTab);
    QFormLayout* featuresLayout = new QFormLayout(featuresGroup);
    
    m_ccdEnabledCheck = new QCheckBox(tr("Enable CCD (Continuous Collision Detection)"), featuresGroup);
    m_debugVisualizationCheck = new QCheckBox(tr("Enable Debug Visualization"), featuresGroup);
    
    featuresLayout->addRow(m_ccdEnabledCheck);
    featuresLayout->addRow(m_debugVisualizationCheck);
    layout->addWidget(featuresGroup);
    
    // Simulation settings group
    QGroupBox* simulationGroup = new QGroupBox(tr("Default Simulation Settings"), m_advancedTab);
    QFormLayout* simulationLayout = new QFormLayout(simulationGroup);
    
    m_maxSubStepsSpin = new QSpinBox(simulationGroup);
    m_maxSubStepsSpin->setRange(MIN_SUB_STEPS, MAX_SUB_STEPS);
    m_maxSubStepsSpin->setSuffix(tr(" steps"));
    
    m_fixedTimeStepSpin = new QDoubleSpinBox(simulationGroup);
    m_fixedTimeStepSpin->setRange(MIN_TIME_STEP, MAX_TIME_STEP);
    m_fixedTimeStepSpin->setSuffix(tr(" s"));
    m_fixedTimeStepSpin->setDecimals(4);
    
    simulationLayout->addRow(tr("Max Sub Steps:"), m_maxSubStepsSpin);
    simulationLayout->addRow(tr("Fixed Time Step:"), m_fixedTimeStepSpin);
    layout->addWidget(simulationGroup);
    
    layout->addStretch();
    m_tabWidget->addTab(m_advancedTab, tr("Advanced"));
}

void SceneConfigurationDialog::connectSignals()
{
    // Button connections
    connect(m_applyButton, &QPushButton::clicked, this, &SceneConfigurationDialog::onApplyClicked);
    connect(m_resetButton, &QPushButton::clicked, this, &SceneConfigurationDialog::onResetClicked);
    connect(m_cancelButton, &QPushButton::clicked, this, &SceneConfigurationDialog::onCancelClicked);
    
    // Value change connections
    connect(m_engineTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &SceneConfigurationDialog::onEngineTypeChanged);
    connect(m_gravityXSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onGravityChanged);
    connect(m_gravityYSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onGravityChanged);
    connect(m_gravityZSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onGravityChanged);
    connect(m_defaultMaterialCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &SceneConfigurationDialog::onDefaultMaterialChanged);
    connect(m_globalStaticFrictionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onGlobalMaterialChanged);
    connect(m_globalDynamicFrictionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onGlobalMaterialChanged);
    connect(m_globalRestitutionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onGlobalMaterialChanged);
    
    // Connect to MaterialManager signals if available
    if (m_materialManager) {
        connect(m_materialManager, &MaterialManager::materialAdded, 
                this, &SceneConfigurationDialog::onMaterialAdded);
        connect(m_materialManager, &MaterialManager::materialRemoved, 
                this, &SceneConfigurationDialog::onMaterialRemoved);
        connect(m_materialManager, &MaterialManager::materialUpdated, 
                this, &SceneConfigurationDialog::onMaterialUpdated);
    }
    connect(m_solverIterationsSpin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onSolverSettingsChanged);
    connect(m_solverVelocityIterationsSpin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onSolverSettingsChanged);
    connect(m_pcmEnabledCheck, &QCheckBox::toggled, this, &SceneConfigurationDialog::onSolverSettingsChanged);
    connect(m_stabilizationEnabledCheck, &QCheckBox::toggled, this, &SceneConfigurationDialog::onSolverSettingsChanged);
    connect(m_contactOffsetSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onSolverSettingsChanged);
    connect(m_restOffsetSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onSolverSettingsChanged);
    connect(m_sleepThresholdSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onAdvancedSettingsChanged);
    connect(m_stabilizationThresholdSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onAdvancedSettingsChanged);
    connect(m_wakeDistanceSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onAdvancedSettingsChanged);
    connect(m_ccdEnabledCheck, &QCheckBox::toggled, this, &SceneConfigurationDialog::onAdvancedSettingsChanged);
    connect(m_debugVisualizationCheck, &QCheckBox::toggled, this, &SceneConfigurationDialog::onDebugVisualizationChanged);
    connect(m_maxSubStepsSpin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onAdvancedSettingsChanged);
    connect(m_fixedTimeStepSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &SceneConfigurationDialog::onAdvancedSettingsChanged);
}

void SceneConfigurationDialog::loadCurrentConfiguration()
{
    if (!m_physicsEngine) {
        qDebug() << "No physics engine available for configuration loading";
        return;
    }
    
    // Load current gravity
    PxVec3 currentGravity = m_physicsEngine->getGravity();
    m_gravityXSpin->setValue(currentGravity.x);
    m_gravityYSpin->setValue(currentGravity.y);
    m_gravityZSpin->setValue(currentGravity.z);
    
    // Store original values for reset
    m_originalGravity = currentGravity;
    
    // Load current values from physics engine
    float staticFriction, dynamicFriction, restitution;
    m_physicsEngine->getGlobalMaterialProperties(staticFriction, dynamicFriction, restitution);
    m_globalStaticFrictionSpin->setValue(staticFriction);
    m_globalDynamicFrictionSpin->setValue(dynamicFriction);
    m_globalRestitutionSpin->setValue(restitution);
    
    int positionIterations, velocityIterations;
    m_physicsEngine->getSolverIterations(positionIterations, velocityIterations);
    m_solverIterationsSpin->setValue(positionIterations);
    m_solverVelocityIterationsSpin->setValue(velocityIterations);
    
    m_pcmEnabledCheck->setChecked(m_physicsEngine->isPCMEnabled());
    m_stabilizationEnabledCheck->setChecked(m_physicsEngine->isStabilizationEnabled());
    m_contactOffsetSpin->setValue(m_physicsEngine->getContactOffset());
    m_restOffsetSpin->setValue(m_physicsEngine->getRestOffset());
    m_sleepThresholdSpin->setValue(m_physicsEngine->getSleepThreshold());
    m_stabilizationThresholdSpin->setValue(m_physicsEngine->getStabilizationThreshold());
    m_wakeDistanceSpin->setValue(m_physicsEngine->getWakeDistance());
    m_ccdEnabledCheck->setChecked(m_physicsEngine->isCCDEnabled());
    m_debugVisualizationCheck->setChecked(m_physicsEngine->isDebugVisualizationEnabled());
    
    // Set default values for settings not yet implemented in PhysXEngine
    m_maxSubStepsSpin->setValue(1);
    m_fixedTimeStepSpin->setValue(1.0f / 60.0f);
    
    // Store original values
    m_originalGlobalStaticFriction = 0.95f;
    m_originalGlobalDynamicFriction = 0.90f;
    m_originalGlobalRestitution = 0.1f;
    m_originalSolverIterations = 255;
    m_originalSolverVelocityIterations = 255;
    m_originalPCMEnabled = true;
    m_originalStabilizationEnabled = true;
    m_originalContactOffset = 2.0f;
    m_originalRestOffset = 0.5f;
    m_originalSleepThreshold = 10.0f;
    m_originalStabilizationThreshold = 10.0f;
    m_originalWakeDistance = 100.0f;
    m_originalCCDEnabled = false;
    m_originalDebugVisualization = false;
    m_originalMaxSubSteps = 1;
    m_originalFixedTimeStep = 1.0f / 60.0f;
}

void SceneConfigurationDialog::onApplyClicked()
{
    if (!m_physicsEngine) {
        QMessageBox::warning(this, tr("Error"), tr("No physics engine available."));
        return;
    }
    
    validateInputs();
    applyConfiguration();
    
    QMessageBox::information(this, tr("Success"), tr("Default scene configuration applied successfully. New objects will use these settings."));
    emit configurationChanged();
    accept();
}

void SceneConfigurationDialog::onResetClicked()
{
    // Reset to original values
    m_gravityXSpin->setValue(m_originalGravity.x);
    m_gravityYSpin->setValue(m_originalGravity.y);
    m_gravityZSpin->setValue(m_originalGravity.z);
    m_globalStaticFrictionSpin->setValue(m_originalGlobalStaticFriction);
    m_globalDynamicFrictionSpin->setValue(m_originalGlobalDynamicFriction);
    m_globalRestitutionSpin->setValue(m_originalGlobalRestitution);
    m_solverIterationsSpin->setValue(m_originalSolverIterations);
    m_solverVelocityIterationsSpin->setValue(m_originalSolverVelocityIterations);
    m_pcmEnabledCheck->setChecked(m_originalPCMEnabled);
    m_stabilizationEnabledCheck->setChecked(m_originalStabilizationEnabled);
    m_contactOffsetSpin->setValue(m_originalContactOffset);
    m_restOffsetSpin->setValue(m_originalRestOffset);
    m_sleepThresholdSpin->setValue(m_originalSleepThreshold);
    m_stabilizationThresholdSpin->setValue(m_originalStabilizationThreshold);
    m_wakeDistanceSpin->setValue(m_originalWakeDistance);
    m_ccdEnabledCheck->setChecked(m_originalCCDEnabled);
    m_debugVisualizationCheck->setChecked(m_originalDebugVisualization);
    m_maxSubStepsSpin->setValue(m_originalMaxSubSteps);
    m_fixedTimeStepSpin->setValue(m_originalFixedTimeStep);
    
    QMessageBox::information(this, tr("Reset"), tr("Configuration reset to original values."));
}

void SceneConfigurationDialog::onCancelClicked()
{
    reject();
}

void SceneConfigurationDialog::onEngineTypeChanged(int index)
{
    // Currently only PhysX is supported
    Q_UNUSED(index)
}

void SceneConfigurationDialog::onGravityChanged()
{
    // Gravity changes are applied immediately
    PxVec3 newGravity(m_gravityXSpin->value(), m_gravityYSpin->value(), m_gravityZSpin->value());
    m_physicsEngine->setGravity(newGravity);
}

void SceneConfigurationDialog::onDefaultMaterialChanged(int index)
{
    if (!m_materialManager || index < 0) {
        return;
    }
    
    // Get the selected material name
    QString materialName = m_defaultMaterialCombo->itemData(index).toString();
    
    // Check if this is a custom material (not a default material)
    bool isCustomMaterial = !m_materialManager->isDefaultMaterial(materialName);
    
    MaterialProperties material = m_materialManager->getMaterial(materialName);
    
    // Update spinboxes with material properties
    m_globalStaticFrictionSpin->setValue(material.staticFriction);
    m_globalDynamicFrictionSpin->setValue(material.dynamicFriction);
    m_globalRestitutionSpin->setValue(material.restitution);
    
    // Enable/disable spinboxes based on material type
    // Custom materials can be edited, default materials are locked
    bool enableEditing = isCustomMaterial;
    m_globalStaticFrictionSpin->setEnabled(enableEditing);
    m_globalDynamicFrictionSpin->setEnabled(enableEditing);
    m_globalRestitutionSpin->setEnabled(enableEditing);
    
    qDebug() << "Selected default material:" << materialName 
             << "- Static:" << material.staticFriction 
             << "Dynamic:" << material.dynamicFriction 
             << "Restitution:" << material.restitution
             << "- Custom:" << isCustomMaterial;
}

void SceneConfigurationDialog::onGlobalMaterialChanged()
{
    // If a custom material is selected and values are changed, save the changes
    QString currentMaterial = getSelectedDefaultMaterial();
    if (m_materialManager && !m_materialManager->isDefaultMaterial(currentMaterial)) {
        // This is a custom material, changes should be saved back to the material
        saveCustomMaterial();
    }
    
    // Material changes would be applied to all objects
    // This is handled in applyConfiguration()
}

void SceneConfigurationDialog::onSolverSettingsChanged()
{
    // Solver settings changes are applied in applyConfiguration()
}

void SceneConfigurationDialog::onAdvancedSettingsChanged()
{
    // Advanced settings changes are applied in applyConfiguration()
}

void SceneConfigurationDialog::onDebugVisualizationChanged(bool enabled)
{
    if (m_physicsEngine) {
        m_physicsEngine->enableDebugVisualization(enabled);
        qDebug() << "Debug visualization" << (enabled ? "enabled" : "disabled");
    }
}

void SceneConfigurationDialog::validateInputs()
{
    // Validate gravity
    if (m_gravityXSpin->value() < MIN_GRAVITY || m_gravityXSpin->value() > MAX_GRAVITY ||
        m_gravityYSpin->value() < MIN_GRAVITY || m_gravityYSpin->value() > MAX_GRAVITY ||
        m_gravityZSpin->value() < MIN_GRAVITY || m_gravityZSpin->value() > MAX_GRAVITY) {
        QMessageBox::warning(this, tr("Invalid Input"), tr("Gravity values are out of range."));
        return;
    }
    
    // Validate friction values
    if (m_globalStaticFrictionSpin->value() < MIN_FRICTION || m_globalStaticFrictionSpin->value() > MAX_FRICTION ||
        m_globalDynamicFrictionSpin->value() < MIN_FRICTION || m_globalDynamicFrictionSpin->value() > MAX_FRICTION) {
        QMessageBox::warning(this, tr("Invalid Input"), tr("Friction values are out of range."));
        return;
    }
    
    // Validate restitution
    if (m_globalRestitutionSpin->value() < MIN_RESTITUTION || m_globalRestitutionSpin->value() > MAX_RESTITUTION) {
        QMessageBox::warning(this, tr("Invalid Input"), tr("Restitution value is out of range."));
        return;
    }
    
    // Validate solver iterations
    if (m_solverIterationsSpin->value() < MIN_SOLVER_ITERATIONS || m_solverIterationsSpin->value() > MAX_SOLVER_ITERATIONS ||
        m_solverVelocityIterationsSpin->value() < MIN_SOLVER_VELOCITY_ITERATIONS || m_solverVelocityIterationsSpin->value() > MAX_SOLVER_VELOCITY_ITERATIONS) {
        QMessageBox::warning(this, tr("Invalid Input"), tr("Solver iteration values are out of range."));
        return;
    }
}

void SceneConfigurationDialog::applyConfiguration()
{
    if (!m_physicsEngine) {
        return;
    }
    
    // Apply gravity
    PxVec3 newGravity(m_gravityXSpin->value(), m_gravityYSpin->value(), m_gravityZSpin->value());
    m_physicsEngine->setGravity(newGravity);
    
    // Apply solver iterations
    m_physicsEngine->setSolverIterations(m_solverIterationsSpin->value(), m_solverVelocityIterationsSpin->value());
    
    // Apply global material properties
    m_physicsEngine->setGlobalMaterialProperties(
        m_globalStaticFrictionSpin->value(),
        m_globalDynamicFrictionSpin->value(),
        m_globalRestitutionSpin->value()
    );
    
    // Apply PCM and stabilization settings
    m_physicsEngine->setPCMEnabled(m_pcmEnabledCheck->isChecked());
    m_physicsEngine->setStabilizationEnabled(m_stabilizationEnabledCheck->isChecked());
    
    // Apply contact and rest offsets
    m_physicsEngine->setContactOffset(m_contactOffsetSpin->value());
    m_physicsEngine->setRestOffset(m_restOffsetSpin->value());
    
    // Apply sleep and stabilization thresholds
    m_physicsEngine->setSleepThreshold(m_sleepThresholdSpin->value());
    m_physicsEngine->setStabilizationThreshold(m_stabilizationThresholdSpin->value());
    
    // Apply wake distance
    m_physicsEngine->setWakeDistance(m_wakeDistanceSpin->value());
    
    // Apply CCD setting
    m_physicsEngine->setCCDEnabled(m_ccdEnabledCheck->isChecked());
    
    // Apply debug visualization
    m_physicsEngine->enableDebugVisualization(m_debugVisualizationCheck->isChecked());
    
    qDebug() << "Applied default scene configuration:";
    qDebug() << "  Gravity:" << newGravity.x << newGravity.y << newGravity.z;
    qDebug() << "  Default solver iterations:" << m_solverIterationsSpin->value() << "position," << m_solverVelocityIterationsSpin->value() << "velocity";
    qDebug() << "  Default material properties:" << m_globalStaticFrictionSpin->value() << "static friction," << m_globalDynamicFrictionSpin->value() << "dynamic friction," << m_globalRestitutionSpin->value() << "restitution";
    qDebug() << "  PCM enabled:" << m_pcmEnabledCheck->isChecked();
    qDebug() << "  Stabilization enabled:" << m_stabilizationEnabledCheck->isChecked();
    qDebug() << "  Default contact offset:" << m_contactOffsetSpin->value() << "mm";
    qDebug() << "  Default rest offset:" << m_restOffsetSpin->value() << "mm";
    qDebug() << "  Default sleep threshold:" << m_sleepThresholdSpin->value() << "mm/s";
    qDebug() << "  Default stabilization threshold:" << m_stabilizationThresholdSpin->value() << "mm/s";
    qDebug() << "  Default wake distance:" << m_wakeDistanceSpin->value() << "mm";
    qDebug() << "  Default CCD enabled:" << m_ccdEnabledCheck->isChecked();
    qDebug() << "  Debug Visualization:" << m_debugVisualizationCheck->isChecked();
}

// Configuration getters
PxVec3 SceneConfigurationDialog::getGravity() const
{
    return PxVec3(m_gravityXSpin->value(), m_gravityYSpin->value(), m_gravityZSpin->value());
}

float SceneConfigurationDialog::getGlobalStaticFriction() const
{
    return m_globalStaticFrictionSpin->value();
}

float SceneConfigurationDialog::getGlobalDynamicFriction() const
{
    return m_globalDynamicFrictionSpin->value();
}

float SceneConfigurationDialog::getGlobalRestitution() const
{
    return m_globalRestitutionSpin->value();
}

QString SceneConfigurationDialog::getSelectedDefaultMaterial() const
{
    int currentIndex = m_defaultMaterialCombo->currentIndex();
    if (currentIndex < 0) {
        return "Wood"; // Default fallback
    }
    return m_defaultMaterialCombo->itemData(currentIndex).toString();
}

int SceneConfigurationDialog::getSolverIterations() const
{
    return m_solverIterationsSpin->value();
}

int SceneConfigurationDialog::getSolverVelocityIterations() const
{
    return m_solverVelocityIterationsSpin->value();
}

bool SceneConfigurationDialog::getPCMEnabled() const
{
    return m_pcmEnabledCheck->isChecked();
}

bool SceneConfigurationDialog::getStabilizationEnabled() const
{
    return m_stabilizationEnabledCheck->isChecked();
}

float SceneConfigurationDialog::getContactOffset() const
{
    return m_contactOffsetSpin->value();
}

float SceneConfigurationDialog::getRestOffset() const
{
    return m_restOffsetSpin->value();
}

float SceneConfigurationDialog::getWakeDistance() const
{
    return m_wakeDistanceSpin->value();
}

void SceneConfigurationDialog::updateMaterialDropdown()
{
    if (!m_defaultMaterialCombo) return;
    
    // Store current selection
    QString currentSelection = getSelectedDefaultMaterial();
    
    // Clear and repopulate dropdown
    m_defaultMaterialCombo->clear();
    
    // Add all available materials if MaterialManager is available
    if (m_materialManager) {
        QStringList allMaterials = m_materialManager->getAvailableMaterials();
        for (const QString& materialName : allMaterials) {
            m_defaultMaterialCombo->addItem(materialName, materialName);
        }
    }
    
    // Restore selection if it still exists, otherwise default to Wood
    bool selectionRestored = false;
    for (int i = 0; i < m_defaultMaterialCombo->count(); ++i) {
        if (m_defaultMaterialCombo->itemData(i).toString() == currentSelection) {
            m_defaultMaterialCombo->setCurrentIndex(i);
            selectionRestored = true;
            break;
        }
    }
    
    // If no selection was restored, default to Wood
    if (!selectionRestored && m_materialManager) {
        for (int i = 0; i < m_defaultMaterialCombo->count(); ++i) {
            if (m_defaultMaterialCombo->itemData(i).toString() == "Wood") {
                m_defaultMaterialCombo->setCurrentIndex(i);
                break;
            }
        }
    }
}

void SceneConfigurationDialog::onMaterialAdded(const QString& materialName)
{
    qDebug() << "SceneConfigurationDialog: Material added:" << materialName;
    updateMaterialDropdown();
}

void SceneConfigurationDialog::onMaterialRemoved(const QString& materialName)
{
    qDebug() << "SceneConfigurationDialog: Material removed:" << materialName;
    updateMaterialDropdown();
}

void SceneConfigurationDialog::onMaterialUpdated(const QString& materialName)
{
    qDebug() << "SceneConfigurationDialog: Material updated:" << materialName;
    // If the currently selected material was updated, refresh its properties
    QString currentMaterial = getSelectedDefaultMaterial();
    if (currentMaterial == materialName && m_materialManager) {
        MaterialProperties material = m_materialManager->getMaterial(materialName);
        m_globalStaticFrictionSpin->setValue(material.staticFriction);
        m_globalDynamicFrictionSpin->setValue(material.dynamicFriction);
        m_globalRestitutionSpin->setValue(material.restitution);
    }
}

void SceneConfigurationDialog::saveCustomMaterial()
{
    QString currentMaterial = getSelectedDefaultMaterial();
    if (currentMaterial == "Custom" || !m_materialManager || m_materialManager->isDefaultMaterial(currentMaterial)) {
        return; // Don't save for "Custom" or default materials
    }
    
    // Get current values from spinboxes
    MaterialProperties updatedMaterial(
        currentMaterial,
        m_globalStaticFrictionSpin->value(),
        m_globalDynamicFrictionSpin->value(),
        m_globalRestitutionSpin->value()
    );
    
    // Update the material in MaterialManager
    if (m_materialManager->updateMaterial(currentMaterial, updatedMaterial)) {
        qDebug() << "SceneConfigurationDialog: Updated custom material" << currentMaterial;
    } else {
        qWarning() << "SceneConfigurationDialog: Failed to update custom material" << currentMaterial;
    }
} 