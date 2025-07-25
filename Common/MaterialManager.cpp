#include "MaterialManager.h"
#include <QDebug>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include "MaterialEditorDialog.h"
#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QMessageBox>

MaterialManager::MaterialManager(IPhysicsEngine* physicsEngine, QObject* parent)
    : QObject(parent)
    , m_physicsEngine(physicsEngine)
{
    initializeDefaultMaterials();
}

MaterialManager::~MaterialManager()
{
    // Cleanup handled by Qt's parent-child system
}

void MaterialManager::initializeDefaultMaterials()
{
    createDefaultMaterials();
    
    // Add default materials to the list
    m_defaultMaterialNames << "Metal" << "Plastic" << "Wood" << "Rubber" 
                          << "Ice" << "Glass" << "Stone" << "Fabric" << "Cardboard" << "Sheet Metal";
}

void MaterialManager::createDefaultMaterials()
{
    // Clear existing materials
    m_materials.clear();
    
    // Add default materials
    m_materials["Metal"] = createMetalMaterial();
    m_materials["Plastic"] = createPlasticMaterial();
    m_materials["Wood"] = createWoodMaterial();
    m_materials["Rubber"] = createRubberMaterial();
    m_materials["Ice"] = createIceMaterial();
    m_materials["Glass"] = createGlassMaterial();
    m_materials["Stone"] = createStoneMaterial();
    m_materials["Fabric"] = createFabricMaterial();
    m_materials["Cardboard"] = createCardboardMaterial();
    m_materials["Sheet Metal"] = createSheetMetalMaterial();
    
    qDebug() << "MaterialManager: Created" << m_materials.size() << "default materials";
}

QStringList MaterialManager::getAvailableMaterials() const
{
    return m_materials.keys();
}

MaterialProperties MaterialManager::getMaterial(const QString& materialName) const
{
    if (m_materials.contains(materialName)) {
        return m_materials[materialName];
    }
    
    // Return default material if not found
    qWarning() << "MaterialManager: Material not found:" << materialName << "- returning default";
    return MaterialProperties();
}

bool MaterialManager::addCustomMaterial(const MaterialProperties& material)
{
    if (!validateMaterialProperties(material)) {
        qWarning() << "MaterialManager: Invalid material properties for" << material.name;
        return false;
    }
    
    // Generate unique name if needed
    QString materialName = material.name;
    if (m_materials.contains(materialName)) {
        materialName = generateUniqueName(materialName);
    }
    
    MaterialProperties newMaterial = material;
    newMaterial.name = materialName;
    m_materials[materialName] = newMaterial;
    
    qDebug() << "MaterialManager: Added custom material:" << materialName;
    emit materialAdded(materialName);
    return true;
}

bool MaterialManager::removeCustomMaterial(const QString& materialName)
{
    if (isDefaultMaterial(materialName)) {
        qWarning() << "MaterialManager: Cannot remove default material:" << materialName;
        return false;
    }
    
    if (!m_materials.contains(materialName)) {
        qWarning() << "MaterialManager: Material not found:" << materialName;
        return false;
    }
    
    // Remove from all objects using this material
    QList<CadNode*> objectsToUpdate;
    for (auto it = m_objectMaterials.begin(); it != m_objectMaterials.end(); ++it) {
        if (it.value() == materialName) {
            objectsToUpdate.append(it.key());
        }
    }
    
    // Reset objects to default material
    for (CadNode* item : objectsToUpdate) {
        m_objectMaterials.remove(item);
    }
    
    m_materials.remove(materialName);
    
    qDebug() << "MaterialManager: Removed custom material:" << materialName;
    emit materialRemoved(materialName);
    return true;
}

bool MaterialManager::updateMaterial(const QString& materialName, const MaterialProperties& newProperties)
{
    if (!m_materials.contains(materialName)) {
        qWarning() << "MaterialManager: Material not found:" << materialName;
        return false;
    }
    
    if (!validateMaterialProperties(newProperties)) {
        qWarning() << "MaterialManager: Invalid material properties for" << materialName;
        return false;
    }
    
    MaterialProperties updatedMaterial = newProperties;
    updatedMaterial.name = materialName; // Keep original name
    m_materials[materialName] = updatedMaterial;
    
    // Update all objects using this material
    for (auto it = m_objectMaterials.begin(); it != m_objectMaterials.end(); ++it) {
        if (it.value() == materialName) {
            applyMaterialToObject(it.key(), materialName);
        }
    }
    
    qDebug() << "MaterialManager: Updated material:" << materialName;
    emit materialUpdated(materialName);
    return true;
}

bool MaterialManager::setObjectMaterial(CadNode* item, const QString& materialName)
{
    if (!m_materials.contains(materialName)) {
        qWarning() << "MaterialManager: Material not found:" << materialName;
        return false;
    }
    
    m_objectMaterials[item] = materialName;
    bool success = applyMaterialToObject(item, materialName);
    
    if (success) {
        emit objectMaterialChanged(item, materialName);
    }
    
    return success;
}

QString MaterialManager::getObjectMaterial(CadNode* item) const
{
    return m_objectMaterials.value(item, "");
}

bool MaterialManager::hasObjectMaterial(CadNode* item) const
{
    return m_objectMaterials.contains(item);
}

void MaterialManager::removeObjectMaterial(CadNode* item)
{
    if (m_objectMaterials.contains(item)) {
        QString oldMaterial = m_objectMaterials[item];
        m_objectMaterials.remove(item);
        emit objectMaterialChanged(item, "");
    }
}

bool MaterialManager::applyMaterialToObject(CadNode* item, const QString& materialName)
{
    if (!m_materials.contains(materialName)) {
        qWarning() << "MaterialManager: Material not found:" << materialName;
        return false;
    }
    
    return applyMaterialToObject(item, m_materials[materialName]);
}

bool MaterialManager::applyMaterialToObject(CadNode* item, const MaterialProperties& material)
{
    if (!m_physicsEngine) {
        qWarning() << "MaterialManager: No physics engine available";
        return false;
    }
    
    // Apply material properties to the physics object
    bool success = m_physicsEngine->setObjectMaterial(item, 
                                                     material.staticFriction,
                                                     material.dynamicFriction,
                                                     material.restitution);
    
    if (success) {
        qDebug() << "MaterialManager: Applied material" << material.name << "to object";
    } else {
        qWarning() << "MaterialManager: Failed to apply material" << material.name << "to object";
    }
    
    return success;
}

QStringList MaterialManager::getDefaultMaterialNames() const
{
    return m_defaultMaterialNames;
}

bool MaterialManager::isDefaultMaterial(const QString& materialName) const
{
    return m_defaultMaterialNames.contains(materialName);
}

void MaterialManager::onObjectAdded(CadNode* item)
{
    // Check if object has a previously assigned material
    if (m_objectMaterials.contains(item)) {
        QString materialName = m_objectMaterials[item];
        if (m_materials.contains(materialName)) {
            applyMaterialToObject(item, materialName);
        }
    } else {
        // Automatically assign Wood material to new objects
        if (m_materials.contains("Wood")) {
            setObjectMaterial(item, "Wood");
            qDebug() << "MaterialManager: Automatically assigned Wood material to new object";
        }
    }
}

void MaterialManager::onObjectRemoved(CadNode* item)
{
    // Remove object from material tracking
    m_objectMaterials.remove(item);
}

bool MaterialManager::validateMaterialProperties(const MaterialProperties& material) const
{
    if (material.name.isEmpty()) {
        return false;
    }
    
    if (material.staticFriction < 0.0f || material.staticFriction > 2.0f) {
        return false;
    }
    
    if (material.dynamicFriction < 0.0f || material.dynamicFriction > 2.0f) {
        return false;
    }
    
    if (material.restitution < 0.0f || material.restitution > 1.0f) {
        return false;
    }
    
    return true;
}

QString MaterialManager::generateUniqueName(const QString& baseName) const
{
    QString newName = baseName;
    int counter = 1;
    
    while (m_materials.contains(newName)) {
        newName = QString("%1_%2").arg(baseName).arg(counter);
        counter++;
    }
    
    return newName;
}

// Static material creation methods
MaterialProperties MaterialManager::createMetalMaterial()
{
    return MaterialProperties("Metal", 0.6f, 0.4f, 0.1f, QColor(192, 192, 192), 
                            "High density, conductive material with moderate friction");
}

MaterialProperties MaterialManager::createPlasticMaterial()
{
    return MaterialProperties("Plastic", 0.4f, 0.3f, 0.2f, QColor(255, 255, 0), 
                            "Lightweight, flexible material with low friction");
}

MaterialProperties MaterialManager::createWoodMaterial()
{
    return MaterialProperties("Wood", 0.5f, 0.4f, 0.3f, QColor(139, 69, 19), 
                            "Natural material with moderate friction and bounce");
}

MaterialProperties MaterialManager::createRubberMaterial()
{
    return MaterialProperties("Rubber", 0.8f, 0.6f, 0.8f, QColor(0, 0, 0), 
                            "Elastic material with high friction and bounce");
}

MaterialProperties MaterialManager::createIceMaterial()
{
    return MaterialProperties("Ice", 0.1f, 0.05f, 0.1f, QColor(240, 248, 255), 
                            "Slippery material with very low friction");
}

MaterialProperties MaterialManager::createGlassMaterial()
{
    return MaterialProperties("Glass", 0.3f, 0.2f, 0.9f, QColor(173, 216, 230), 
                            "Brittle material with low friction and high bounce");
}

MaterialProperties MaterialManager::createStoneMaterial()
{
    return MaterialProperties("Stone", 0.7f, 0.6f, 0.1f, QColor(128, 128, 128), 
                            "Dense material with high friction and low bounce");
}

MaterialProperties MaterialManager::createFabricMaterial()
{
    return MaterialProperties("Fabric", 0.6f, 0.5f, 0.0f, QColor(255, 182, 193), 
                            "Soft material with moderate friction and no bounce");
}

MaterialProperties MaterialManager::createCardboardMaterial()
{
    return MaterialProperties("Cardboard", 0.4f, 0.3f, 0.1f, QColor(210, 180, 140), 
                            "Lightweight paper-based material with low friction and minimal bounce");
}

MaterialProperties MaterialManager::createSheetMetalMaterial()
{
    return MaterialProperties("Sheet Metal", 0.15f, 0.12f, 0.05f, QColor(192, 192, 192), 
                            "Smooth metal surface with very low friction, default for kinematic actors like robots");
} 

void MaterialManager::showMaterialManagerDialog(QWidget* parent)
{
    // Create a simple dialog to show available materials and allow adding custom ones
    QDialog dialog(parent);
    dialog.setWindowTitle("Material Manager");
    dialog.setFixedSize(500, 400);

    QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);

    // Available materials list
    QLabel* materialsLabel = new QLabel("Available Materials:");
    mainLayout->addWidget(materialsLabel);

    QListWidget* materialsList = new QListWidget();
    QStringList materials = getAvailableMaterials();
    for (const QString& materialName : materials) {
        MaterialProperties material = getMaterial(materialName);
        QString itemText = QString("%1 (Static: %2, Dynamic: %3, Restitution: %4)")
                              .arg(materialName)
                              .arg(material.staticFriction)
                              .arg(material.dynamicFriction)
                              .arg(material.restitution);
        QListWidgetItem* item = new QListWidgetItem(itemText);
        if (isDefaultMaterial(materialName)) {
            item->setBackground(QColor(240, 240, 240)); // Light gray for default materials
        }
        materialsList->addItem(item);
    }
    mainLayout->addWidget(materialsList);

    // Buttons
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    QPushButton* addButton = new QPushButton("Add Custom Material");
    QPushButton* editButton = new QPushButton("Edit Material");
    QPushButton* removeButton = new QPushButton("Remove Material");
    QPushButton* closeButton = new QPushButton("Close");
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(editButton);
    buttonLayout->addWidget(removeButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(closeButton);
    mainLayout->addLayout(buttonLayout);

    // Connect signals
    QObject::connect(addButton, &QPushButton::clicked, &dialog, [&]() {
        MaterialEditorDialog editor(&dialog);
        if (editor.exec() == QDialog::Accepted) {
            MaterialProperties newMaterial = editor.getMaterialProperties();
            if (addCustomMaterial(newMaterial)) {
                // Refresh the list
                materialsList->clear();
                QStringList updatedMaterials = getAvailableMaterials();
                for (const QString& materialName : updatedMaterials) {
                    MaterialProperties material = getMaterial(materialName);
                    QString itemText = QString("%1 (Static: %2, Dynamic: %3, Restitution: %4)")
                                          .arg(materialName)
                                          .arg(material.staticFriction)
                                          .arg(material.dynamicFriction)
                                          .arg(material.restitution);
                    QListWidgetItem* item = new QListWidgetItem(itemText);
                    if (isDefaultMaterial(materialName)) {
                        item->setBackground(QColor(240, 240, 240));
                    }
                    materialsList->addItem(item);
                }
            }
        }
    }, Qt::QueuedConnection);

    QObject::connect(editButton, &QPushButton::clicked, &dialog, [&]() {
        QListWidgetItem* currentItem = materialsList->currentItem();
        if (!currentItem) {
            QMessageBox::warning(&dialog, "No Selection", "Please select a material to edit.");
            return;
        }
        QString materialName = currentItem->text().split(" ").first();
        if (isDefaultMaterial(materialName)) {
            QMessageBox::information(&dialog, "Default Material", 
                                   "Default materials cannot be edited. Create a custom material instead.");
            return;
        }
        MaterialProperties material = getMaterial(materialName);
        MaterialEditorDialog editor(material, &dialog);
        if (editor.exec() == QDialog::Accepted) {
            MaterialProperties updatedMaterial = editor.getMaterialProperties();
            if (updateMaterial(materialName, updatedMaterial)) {
                // Update the list item
                QString itemText = QString("%1 (Static: %2, Dynamic: %3, Restitution: %4)")
                                      .arg(materialName)
                                      .arg(updatedMaterial.staticFriction)
                                      .arg(updatedMaterial.dynamicFriction)
                                      .arg(updatedMaterial.restitution);
                currentItem->setText(itemText);
            }
        }
    }, Qt::QueuedConnection);

    QObject::connect(removeButton, &QPushButton::clicked, &dialog, [&]() {
        QListWidgetItem* currentItem = materialsList->currentItem();
        if (!currentItem) {
            QMessageBox::warning(&dialog, "No Selection", "Please select a material to remove.");
            return;
        }
        QString materialName = currentItem->text().split(" ").first();
        if (isDefaultMaterial(materialName)) {
            QMessageBox::information(&dialog, "Default Material", 
                                   "Default materials cannot be removed.");
            return;
        }
        if (QMessageBox::question(&dialog, "Confirm Removal", 
                                 QString("Are you sure you want to remove the material '%1'?").arg(materialName),
                                 QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
            if (removeCustomMaterial(materialName)) {
                materialsList->takeItem(materialsList->row(currentItem));
            }
        }
    }, Qt::QueuedConnection);

    QObject::connect(closeButton, &QPushButton::clicked, &dialog, [&dialog]() {
        dialog.accept();
    }, Qt::QueuedConnection);

    dialog.exec();
} 
