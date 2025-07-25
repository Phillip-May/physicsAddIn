#include "ObjectPropertiesDialog.h"
#include "MaterialManager.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QDebug>
#include <QGroupBox>

ObjectPropertiesDialog::ObjectPropertiesDialog(CadNode* item, MaterialManager* materialManager, QWidget* parent)
    : QDialog(parent)
    , m_item(item)
    , m_materialManager(materialManager)
{
    setupUI();
    connectSignals();
    updateUI();
    setWindowTitle(QString("Object Properties - %1").arg(item->name.c_str()));
}

void ObjectPropertiesDialog::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    
    // Custom properties checkbox
    m_useCustomPropertiesCheck = new QCheckBox("Use Custom Properties");
    m_useCustomPropertiesCheck->setToolTip("Enable to override default physics properties");
    mainLayout->addWidget(m_useCustomPropertiesCheck);
    
    // Add a small spacer
    mainLayout->addSpacing(5);
    
    // Custom center of mass checkbox
    m_useCustomCenterOfMassCheck = new QCheckBox("Use Custom Center of Mass");
    m_useCustomCenterOfMassCheck->setToolTip("Enable to override the default center of mass");
    m_useCustomCenterOfMassCheck->setChecked(false);
    mainLayout->addWidget(m_useCustomCenterOfMassCheck);
    
    // Physics properties group
    QGroupBox* physicsGroup = new QGroupBox("Physics Properties");
    QFormLayout* physicsLayout = new QFormLayout(physicsGroup);
    
    // Mass
    m_massSpin = new QDoubleSpinBox();
    m_massSpin->setRange(0.001, 10000.0);
    m_massSpin->setSingleStep(0.1);
    m_massSpin->setValue(100.0);
    m_massSpin->setSuffix(" kg");
    m_massSpin->setToolTip("Mass of the object in kilograms");
    physicsLayout->addRow("Mass:", m_massSpin);
    

    
    mainLayout->addWidget(physicsGroup);
    
    // Center of mass group
    QGroupBox* comGroup = new QGroupBox("Center of Mass (mm)");
    QFormLayout* comLayout = new QFormLayout(comGroup);
    
    // Center of mass X, Y, Z
    m_centerOfMassXSpin = new QDoubleSpinBox();
    m_centerOfMassXSpin->setRange(-1000.0, 1000.0);
    m_centerOfMassXSpin->setSingleStep(1.0);
    m_centerOfMassXSpin->setValue(0.0);
    m_centerOfMassXSpin->setSuffix(" mm");
    m_centerOfMassXSpin->setToolTip("X coordinate of center of mass");
    comLayout->addRow("X:", m_centerOfMassXSpin);
    
    m_centerOfMassYSpin = new QDoubleSpinBox();
    m_centerOfMassYSpin->setRange(-1000.0, 1000.0);
    m_centerOfMassYSpin->setSingleStep(1.0);
    m_centerOfMassYSpin->setValue(0.0);
    m_centerOfMassYSpin->setSuffix(" mm");
    m_centerOfMassYSpin->setToolTip("Y coordinate of center of mass");
    comLayout->addRow("Y:", m_centerOfMassYSpin);
    
    m_centerOfMassZSpin = new QDoubleSpinBox();
    m_centerOfMassZSpin->setRange(-1000.0, 1000.0);
    m_centerOfMassZSpin->setSingleStep(1.0);
    m_centerOfMassZSpin->setValue(0.0);
    m_centerOfMassZSpin->setSuffix(" mm");
    m_centerOfMassZSpin->setToolTip("Z coordinate of center of mass");
    comLayout->addRow("Z:", m_centerOfMassZSpin);
    
    mainLayout->addWidget(comGroup);
    
    // Material selection group
    QGroupBox* materialGroup = new QGroupBox("Material");
    QFormLayout* materialLayout = new QFormLayout(materialGroup);
    
    m_materialCombo = new QComboBox();
    m_materialCombo->setToolTip("Select material for this object");
    
    // Add available materials if material manager is available
    if (m_materialManager) {
        QStringList materials = m_materialManager->getAvailableMaterials();
        for (const QString& materialName : materials) {
            m_materialCombo->addItem(materialName);
        }
    } else {
        // Fallback to default materials
        QStringList defaultMaterials = {"Wood", "Metal", "Plastic", "Rubber", "Glass"};
        for (const QString& materialName : defaultMaterials) {
            m_materialCombo->addItem(materialName);
        }
    }
    
    materialLayout->addRow("Material:", m_materialCombo);
    mainLayout->addWidget(materialGroup);
    

    
    // Buttons
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    
    m_saveButton = new QPushButton("Save");
    m_cancelButton = new QPushButton("Cancel");
    
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_saveButton);
    buttonLayout->addWidget(m_cancelButton);
    
    mainLayout->addLayout(buttonLayout);
    
    setLayout(mainLayout);
    setFixedSize(450, 550);
}

void ObjectPropertiesDialog::connectSignals()
{
    connect(m_saveButton, &QPushButton::clicked, this, &ObjectPropertiesDialog::onSaveClicked);
    connect(m_cancelButton, &QPushButton::clicked, this, &ObjectPropertiesDialog::onCancelClicked);
    connect(m_useCustomPropertiesCheck, &QCheckBox::toggled, this, &ObjectPropertiesDialog::onUseCustomPropertiesToggled);
    connect(m_useCustomCenterOfMassCheck, &QCheckBox::toggled, this, &ObjectPropertiesDialog::onUseCustomCenterOfMassToggled);
    
    // Connect validation signals
    connect(m_massSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &ObjectPropertiesDialog::validateInputs);
    connect(m_centerOfMassXSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &ObjectPropertiesDialog::validateInputs);
    connect(m_centerOfMassYSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &ObjectPropertiesDialog::validateInputs);
    connect(m_centerOfMassZSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &ObjectPropertiesDialog::validateInputs);
}

void ObjectPropertiesDialog::updateUI()
{
    // Only enable if this is a Physics node
    bool isPhysics = m_item && m_item->type == CadNodeType::Physics && m_item->asPhysics();
    PhysicsNodeData* phys = isPhysics ? m_item->asPhysics() : nullptr;
    if (!isPhysics) {
        m_useCustomPropertiesCheck->setEnabled(false);
        m_useCustomCenterOfMassCheck->setEnabled(false);
        m_massSpin->setEnabled(false);
        m_centerOfMassXSpin->setEnabled(false);
        m_centerOfMassYSpin->setEnabled(false);
        m_centerOfMassZSpin->setEnabled(false);
        m_materialCombo->setEnabled(false);
        m_saveButton->setEnabled(false);
        return;
    }
    m_useCustomPropertiesCheck->setEnabled(true);
    m_useCustomCenterOfMassCheck->setEnabled(true);
    m_saveButton->setEnabled(true);
    // Set values from PhysicsNodeData
    m_useCustomPropertiesCheck->setChecked(phys->useCustomProperties);
    m_useCustomCenterOfMassCheck->setChecked(phys->useCustomCenterOfMass);
    m_massSpin->setValue(phys->mass);
    m_centerOfMassXSpin->setValue(phys->centerOfMass.X());
    m_centerOfMassYSpin->setValue(phys->centerOfMass.Y());
    m_centerOfMassZSpin->setValue(phys->centerOfMass.Z());
    // Set material combo
    int materialIndex = m_materialCombo->findText(QString::fromStdString(phys->materialName));
    if (materialIndex >= 0) {
        m_materialCombo->setCurrentIndex(materialIndex);
    } else if (m_materialCombo->count() > 0) {
        m_materialCombo->setCurrentIndex(0);
    }
    // Enable/disable controls based on checkboxes
    onUseCustomPropertiesToggled(phys->useCustomProperties);
    onUseCustomCenterOfMassToggled(phys->useCustomCenterOfMass);
}

void ObjectPropertiesDialog::onSaveClicked()
{
    // Only allow if Physics node
    bool isPhysics = m_item && m_item->type == CadNodeType::Physics && m_item->asPhysics();
    if (!isPhysics) { reject(); return; }
    PhysicsNodeData* phys = m_item->asPhysics();
    // Validate inputs
    if (m_massSpin->value() < 0.001 || m_massSpin->value() > 10000.0) {
        QMessageBox::warning(this, "Invalid Input", "Mass must be between 0.001 and 10000.0 kg.");
        m_massSpin->setFocus();
        return;
    }
    // Save values to PhysicsNodeData
    phys->useCustomProperties = m_useCustomPropertiesCheck->isChecked();
    phys->useCustomCenterOfMass = m_useCustomCenterOfMassCheck->isChecked();
    phys->mass = static_cast<float>(m_massSpin->value());
    phys->centerOfMass = gp_Vec(m_centerOfMassXSpin->value(), m_centerOfMassYSpin->value(), m_centerOfMassZSpin->value());
    // Save material
    QString materialName = m_materialCombo->currentText();
    if (!materialName.isEmpty()) {
        phys->materialName = materialName.toStdString();
        if (m_materialManager) {
            m_materialManager->setObjectMaterial(m_item, materialName);
        }
    }
    // Optionally, update friction/restitution from material if not using custom
    if (!phys->useCustomProperties && m_materialManager) {
        MaterialProperties mat = m_materialManager->getMaterial(QString::fromStdString(phys->materialName));
        phys->staticFriction = mat.staticFriction;
        phys->dynamicFriction = mat.dynamicFriction;
        phys->restitution = mat.restitution;
    }
    accept();
}

void ObjectPropertiesDialog::onCancelClicked()
{
    reject();
}

void ObjectPropertiesDialog::onUseCustomPropertiesToggled(bool checked)
{
    // Enable/disable property controls based on checkbox
    // Note: Friction and restitution are always read-only (controlled by material)
    m_massSpin->setEnabled(checked);
    m_materialCombo->setEnabled(checked);
    
    // Update validation
    validateInputs();
}

void ObjectPropertiesDialog::onUseCustomCenterOfMassToggled(bool checked)
{
    // Enable/disable center of mass controls based on checkbox
    m_centerOfMassXSpin->setEnabled(checked);
    m_centerOfMassYSpin->setEnabled(checked);
    m_centerOfMassZSpin->setEnabled(checked);
    
    // Update validation
    validateInputs();
}

void ObjectPropertiesDialog::validateInputs()
{
    bool isValid = true;
    
    if (m_useCustomPropertiesCheck->isChecked()) {
        isValid = m_massSpin->value() >= 0.001 && m_massSpin->value() <= 10000.0 &&
                  m_centerOfMassXSpin->value() >= -1000.0 && m_centerOfMassXSpin->value() <= 1000.0 &&
                  m_centerOfMassYSpin->value() >= -1000.0 && m_centerOfMassYSpin->value() <= 1000.0 &&
                  m_centerOfMassZSpin->value() >= -1000.0 && m_centerOfMassZSpin->value() <= 1000.0;
    }
    
    m_saveButton->setEnabled(isValid);
} 

