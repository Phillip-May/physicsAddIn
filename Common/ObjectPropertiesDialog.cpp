#include "ObjectPropertiesDialog.h"
#include "MaterialManager.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QDebug>
#include <QGroupBox>

ObjectPropertiesDialog::ObjectPropertiesDialog(Item item, ObjectPropertiesManager* manager, MaterialManager* materialManager, QWidget* parent)
    : QDialog(parent)
    , m_item(item)
    , m_manager(manager)
    , m_materialManager(materialManager)
{
    setupUI();
    connectSignals();
    
    // Load current properties from manager
    if (m_manager && m_manager->hasObjectProperties(item)) {
        m_currentProperties.mass = m_manager->getObjectMass(item);
        m_manager->getObjectFriction(item, m_currentProperties.staticFriction, m_currentProperties.dynamicFriction);
        m_currentProperties.restitution = m_manager->getObjectRestitution(item);
        m_currentProperties.centerOfMass = m_manager->getObjectCenterOfMass(item);
        m_currentProperties.useCustomProperties = m_manager->getUseCustomProperties(item);
        m_currentProperties.useCustomCenterOfMass = m_manager->getUseCustomCenterOfMass(item);
        
        // Get material from ObjectPropertiesManager first, then fallback to MaterialManager
        m_currentProperties.materialName = m_manager->getObjectMaterial(item);
        if (m_currentProperties.materialName.isEmpty() && m_materialManager) {
            m_currentProperties.materialName = m_materialManager->getObjectMaterial(item);
        }
    } else {
        m_currentProperties = m_manager ? m_manager->getDefaultProperties() : ObjectPropertiesManager::ObjectProperties();
        
        // If no properties exist, try to get material from MaterialManager
        if (m_materialManager && m_currentProperties.materialName.isEmpty()) {
            m_currentProperties.materialName = m_materialManager->getObjectMaterial(item);
        }
    }
    
    setObjectProperties(m_currentProperties);
    setWindowTitle(QString("Object Properties - %1").arg(item->Name()));
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

ObjectPropertiesManager::ObjectProperties ObjectPropertiesDialog::getObjectProperties() const
{
    ObjectPropertiesManager::ObjectProperties props;
    props.useCustomProperties = m_useCustomPropertiesCheck->isChecked();
    props.useCustomCenterOfMass = m_useCustomCenterOfMassCheck->isChecked();
    props.mass = m_massSpin->value();
    
    // Get friction and restitution from the selected material
    QString materialName = m_materialCombo->currentText();
    if (m_materialManager) {
        MaterialProperties material = m_materialManager->getMaterial(materialName);
        if (!material.name.isEmpty()) {
            props.staticFriction = material.staticFriction;
            props.dynamicFriction = material.dynamicFriction;
            props.restitution = material.restitution;
        } else {
            // Use default values if material not found
            props.staticFriction = 0.8f;
            props.dynamicFriction = 0.6f;
            props.restitution = 0.1f;
        }
    } else {
        // Use default values if no material manager
        props.staticFriction = 0.8f;
        props.dynamicFriction = 0.6f;
        props.restitution = 0.1f;
    }
    
    props.centerOfMass = PxVec3(
        m_centerOfMassXSpin->value(),
        m_centerOfMassYSpin->value(),
        m_centerOfMassZSpin->value()
    );
    props.materialName = materialName;
    
    return props;
}

void ObjectPropertiesDialog::setObjectProperties(const ObjectPropertiesManager::ObjectProperties& properties)
{
    m_currentProperties = properties;
    updateUI();
}

void ObjectPropertiesDialog::updateUI()
{
    m_useCustomPropertiesCheck->setChecked(m_currentProperties.useCustomProperties);
    m_useCustomCenterOfMassCheck->setChecked(m_currentProperties.useCustomCenterOfMass);
    m_massSpin->setValue(m_currentProperties.mass);
    m_centerOfMassXSpin->setValue(m_currentProperties.centerOfMass.x);
    m_centerOfMassYSpin->setValue(m_currentProperties.centerOfMass.y);
    m_centerOfMassZSpin->setValue(m_currentProperties.centerOfMass.z);
    
    // Set material combo
    int materialIndex = m_materialCombo->findText(m_currentProperties.materialName);
    if (materialIndex >= 0) {
        m_materialCombo->setCurrentIndex(materialIndex);
    }
    

    
    // Enable/disable controls based on checkboxes
    onUseCustomPropertiesToggled(m_currentProperties.useCustomProperties);
    onUseCustomCenterOfMassToggled(m_currentProperties.useCustomCenterOfMass);
}

void ObjectPropertiesDialog::onSaveClicked()
{
    // Validate inputs
    if (m_massSpin->value() < 0.001 || m_massSpin->value() > 10000.0) {
        QMessageBox::warning(this, "Invalid Input", "Mass must be between 0.001 and 10000.0 kg.");
        m_massSpin->setFocus();
        return;
    }
    

    
    // Apply properties to manager if available
    if (m_manager) {
        ObjectPropertiesManager::ObjectProperties props = getObjectProperties();
        
        m_manager->setObjectMass(m_item, props.mass);
        m_manager->setObjectFriction(m_item, props.staticFriction, props.dynamicFriction);
        m_manager->setObjectRestitution(m_item, props.restitution);
        m_manager->setObjectCenterOfMass(m_item, props.centerOfMass);
        m_manager->setObjectMaterial(m_item, props.materialName);
        m_manager->setUseCustomProperties(m_item, props.useCustomProperties);
        m_manager->setUseCustomCenterOfMass(m_item, props.useCustomCenterOfMass);
        
        // Apply to physics if object is in simulation
        m_manager->applyPropertiesToPhysics(m_item);
        
        qDebug() << "Saved object properties for" << m_item->Name();
    }
    
    // Also save material to MaterialManager
    if (m_materialManager) {
        QString materialName = m_materialCombo->currentText();
        if (!materialName.isEmpty()) {
            m_materialManager->setObjectMaterial(m_item, materialName);
            qDebug() << "Saved material" << materialName << "for object" << m_item->Name();
        }
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
