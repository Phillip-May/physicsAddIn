#include "MaterialManager.h"
#include "IPhysicsEngine.h"
#include <QDebug>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>

MaterialManager::MaterialManager(IPhysicsEngine* physicsEngine, QObject* parent)
    : QObject(parent)
    , m_physicsEngine(physicsEngine)
{
    initializeDefaultMaterials();
    
    // Connect to physics engine signals
    if (m_physicsEngine) {
        connect(m_physicsEngine, &IPhysicsEngine::objectAdded, 
                this, &MaterialManager::onObjectAdded);
        connect(m_physicsEngine, &IPhysicsEngine::objectRemoved, 
                this, &MaterialManager::onObjectRemoved);
        connect(m_physicsEngine, &IPhysicsEngine::robotAdded, 
                this, &MaterialManager::onObjectAdded);  // Treat robots as objects for material management
        connect(m_physicsEngine, &IPhysicsEngine::robotRemoved, 
                this, &MaterialManager::onObjectRemoved);  // Treat robots as objects for material management
    }
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
    QList<Item> objectsToUpdate;
    for (auto it = m_objectMaterials.begin(); it != m_objectMaterials.end(); ++it) {
        if (it.value() == materialName) {
            objectsToUpdate.append(it.key());
        }
    }
    
    // Reset objects to default material
    for (Item item : objectsToUpdate) {
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

bool MaterialManager::setObjectMaterial(Item item, const QString& materialName)
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

QString MaterialManager::getObjectMaterial(Item item) const
{
    return m_objectMaterials.value(item, "");
}

bool MaterialManager::hasObjectMaterial(Item item) const
{
    return m_objectMaterials.contains(item);
}

void MaterialManager::removeObjectMaterial(Item item)
{
    if (m_objectMaterials.contains(item)) {
        QString oldMaterial = m_objectMaterials[item];
        m_objectMaterials.remove(item);
        emit objectMaterialChanged(item, "");
    }
}

bool MaterialManager::applyMaterialToObject(Item item, const QString& materialName)
{
    if (!m_materials.contains(materialName)) {
        qWarning() << "MaterialManager: Material not found:" << materialName;
        return false;
    }
    
    return applyMaterialToObject(item, m_materials[materialName]);
}

bool MaterialManager::applyMaterialToObject(Item item, const MaterialProperties& material)
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

void MaterialManager::onObjectAdded(Item item)
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

void MaterialManager::onObjectRemoved(Item item)
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