#include "SoftBodyConfigDialog.h"
#include <QDebug>

SoftBodyConfigDialog::SoftBodyConfigDialog(QWidget* parent)
    : QDialog(parent)
    , m_materialGroup(nullptr)
    , m_densitySpinBox(nullptr)
    , m_youngsModulusSpinBox(nullptr)
    , m_poissonsRatioSpinBox(nullptr)
    , m_simulationGroup(nullptr)
    , m_dampingSpinBox(nullptr)
    , m_frictionSpinBox(nullptr)
    , m_okButton(nullptr)
    , m_cancelButton(nullptr)
    , m_applyButton(nullptr)
{
    setupUI();
    setWindowTitle("Soft Body Configuration");
    resize(400, 250);
}

void SoftBodyConfigDialog::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    
    // Material group
    m_materialGroup = new QGroupBox("Material Properties");
    QFormLayout* materialLayout = new QFormLayout(m_materialGroup);
    
    m_densitySpinBox = new QDoubleSpinBox();
    m_densitySpinBox->setRange(0.0, 100000.0);
    m_densitySpinBox->setValue(SoftBodyConfig().density * 1e6f);  // Convert g/mm³ to kg/m³ for display
    m_densitySpinBox->setSuffix(" kg/m³");
    materialLayout->addRow("Density:", m_densitySpinBox);
    
    m_youngsModulusSpinBox = new QDoubleSpinBox();
    m_youngsModulusSpinBox->setRange(0.0, 100000.0);
    m_youngsModulusSpinBox->setValue(SoftBodyConfig().youngsModulus);
    m_youngsModulusSpinBox->setSuffix(" MPa");
    materialLayout->addRow("Young's Modulus:", m_youngsModulusSpinBox);
    
    m_poissonsRatioSpinBox = new QDoubleSpinBox();
    m_poissonsRatioSpinBox->setRange(0.0, 0.5);
    m_poissonsRatioSpinBox->setValue(SoftBodyConfig().poissonsRatio);
    m_poissonsRatioSpinBox->setSingleStep(0.01);
    materialLayout->addRow("Poisson's Ratio:", m_poissonsRatioSpinBox);
    
    mainLayout->addWidget(m_materialGroup);
    
    // Simulation group
    m_simulationGroup = new QGroupBox("Simulation Parameters");
    QFormLayout* simulationLayout = new QFormLayout(m_simulationGroup);
    
    m_dampingSpinBox = new QDoubleSpinBox();
    m_dampingSpinBox->setRange(0.0, 1.0);
    m_dampingSpinBox->setValue(SoftBodyConfig().damping);
    m_dampingSpinBox->setSingleStep(0.1);
    simulationLayout->addRow("Damping:", m_dampingSpinBox);
    
    m_frictionSpinBox = new QDoubleSpinBox();
    m_frictionSpinBox->setRange(0.0, 2.0);
    m_frictionSpinBox->setValue(SoftBodyConfig().friction);
    m_frictionSpinBox->setSingleStep(0.1);
    simulationLayout->addRow("Friction:", m_frictionSpinBox);
    
    mainLayout->addWidget(m_simulationGroup);
    
    // Buttons
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_okButton = new QPushButton("OK");
    m_cancelButton = new QPushButton("Cancel");
    m_applyButton = new QPushButton("Apply");
    
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_applyButton);
    buttonLayout->addWidget(m_cancelButton);
    buttonLayout->addWidget(m_okButton);
    
    mainLayout->addLayout(buttonLayout);
    
    // Connect signals
    connect(m_okButton, &QPushButton::clicked, this, &SoftBodyConfigDialog::onAccepted);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    connect(m_applyButton, &QPushButton::clicked, this, &SoftBodyConfigDialog::onAccepted);
}

SoftBodyConfig SoftBodyConfigDialog::getConfig() const
{
    SoftBodyConfig config;
    
    // Always use existing geometry since that's effectively the only option
    config.useExistingGeometry = true;
    
    // Convert from UI units to simulation units
    // Density: kg/m³ -> g/mm³ (multiply by 1e-6)
    config.density = m_densitySpinBox->value() * 1e-6f;
    
    // Young's Modulus: MPa -> MPa (no conversion needed)
    config.youngsModulus = m_youngsModulusSpinBox->value();
    
    config.poissonsRatio = m_poissonsRatioSpinBox->value();
    config.damping = m_dampingSpinBox->value();
    config.friction = m_frictionSpinBox->value();
    
    return config;
}

void SoftBodyConfigDialog::setConfig(const SoftBodyConfig& config)
{
    // Convert from simulation units to UI units for display
    // Density: g/mm³ -> kg/m³ (multiply by 1e6)
    m_densitySpinBox->setValue(config.density * 1e6f);
    
    // Young's Modulus: MPa -> MPa (no conversion needed)
    m_youngsModulusSpinBox->setValue(config.youngsModulus);
    
    m_poissonsRatioSpinBox->setValue(config.poissonsRatio);
    m_dampingSpinBox->setValue(config.damping);
    m_frictionSpinBox->setValue(config.friction);
}

void SoftBodyConfigDialog::onAccepted()
{
    SoftBodyConfig config = getConfig();
    
    emit configAccepted(config);
    
    if (sender() == m_okButton) {
        accept();
    }
} 
