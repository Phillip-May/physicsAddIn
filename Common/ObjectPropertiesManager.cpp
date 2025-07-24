#include "ObjectPropertiesManager.h"
#include "PhysXEngine.h"
#include "MaterialManager.h"
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QDir>

ObjectPropertiesManager::ObjectPropertiesManager(IPhysicsEngine* physicsEngine, 
                                             MaterialManager* materialManager, QObject* parent)
    : QObject(parent)
    , m_physicsEngine(physicsEngine)
    , m_materialManager(materialManager)
{
    // Connect to material manager signals if available
    if (m_materialManager) {
        connect(m_materialManager, &MaterialManager::materialUpdated, 
                this, &ObjectPropertiesManager::onMaterialChanged);
    }
}

ObjectPropertiesManager::~ObjectPropertiesManager()
{
    // Save all properties before destruction
    saveAllProperties();
}

bool ObjectPropertiesManager::setObjectMass(Item item, float mass)
{
    if (!item || !validateMass(mass)) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        m_objectProperties[item] = ObjectProperties();
    }
    
    m_objectProperties[item].mass = mass;
    emit propertiesChanged(item);
    return true;
}

float ObjectPropertiesManager::getObjectMass(Item item) const
{
    if (!item) {
        return getDefaultProperties().mass;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        return getDefaultProperties().mass;
    }
    
    return it->second.mass;
}

bool ObjectPropertiesManager::setObjectFriction(Item item, float staticFriction, float dynamicFriction)
{
    if (!item || !validateFriction(staticFriction) || !validateFriction(dynamicFriction)) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        m_objectProperties[item] = ObjectProperties();
    }
    
    m_objectProperties[item].staticFriction = staticFriction;
    m_objectProperties[item].dynamicFriction = dynamicFriction;
    emit propertiesChanged(item);
    return true;
}

void ObjectPropertiesManager::getObjectFriction(Item item, float& staticFriction, float& dynamicFriction) const
{
    if (!item) {
        ObjectProperties defaultProps = getDefaultProperties();
        staticFriction = defaultProps.staticFriction;
        dynamicFriction = defaultProps.dynamicFriction;
        return;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        ObjectProperties defaultProps = getDefaultProperties();
        staticFriction = defaultProps.staticFriction;
        dynamicFriction = defaultProps.dynamicFriction;
        return;
    }
    
    staticFriction = it->second.staticFriction;
    dynamicFriction = it->second.dynamicFriction;
}

bool ObjectPropertiesManager::setObjectRestitution(Item item, float restitution)
{
    if (!item || !validateRestitution(restitution)) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        m_objectProperties[item] = ObjectProperties();
    }
    
    m_objectProperties[item].restitution = restitution;
    emit propertiesChanged(item);
    return true;
}

float ObjectPropertiesManager::getObjectRestitution(Item item) const
{
    if (!item) {
        return getDefaultProperties().restitution;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        return getDefaultProperties().restitution;
    }
    
    return it->second.restitution;
}

bool ObjectPropertiesManager::setObjectCenterOfMass(Item item, const PxVec3& centerOfMass)
{
    if (!item || !validateCenterOfMass(centerOfMass)) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        m_objectProperties[item] = ObjectProperties();
    }
    
    m_objectProperties[item].centerOfMass = centerOfMass;
    emit propertiesChanged(item);
    return true;
}

PxVec3 ObjectPropertiesManager::getObjectCenterOfMass(Item item) const
{
    if (!item) {
        return getDefaultProperties().centerOfMass;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        return getDefaultProperties().centerOfMass;
    }
    
    return it->second.centerOfMass;
}

bool ObjectPropertiesManager::setObjectMaterial(Item item, const QString& materialName)
{
    if (!item) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        m_objectProperties[item] = ObjectProperties();
    }
    
    m_objectProperties[item].materialName = materialName;
    emit propertiesChanged(item);
    return true;
}

QString ObjectPropertiesManager::getObjectMaterial(Item item) const
{
    if (!item) {
        return getDefaultProperties().materialName;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        return getDefaultProperties().materialName;
    }
    
    return it->second.materialName;
}

bool ObjectPropertiesManager::setUseCustomProperties(Item item, bool useCustom)
{
    if (!item) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        m_objectProperties[item] = ObjectProperties();
    }
    
    m_objectProperties[item].useCustomProperties = useCustom;
    emit propertiesChanged(item);
    return true;
}

bool ObjectPropertiesManager::getUseCustomProperties(Item item) const
{
    if (!item) {
        return getDefaultProperties().useCustomProperties;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        return getDefaultProperties().useCustomProperties;
    }
    
    return it->second.useCustomProperties;
}

bool ObjectPropertiesManager::setUseCustomCenterOfMass(Item item, bool useCustom)
{
    if (!item) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        m_objectProperties[item] = ObjectProperties();
    }
    
    m_objectProperties[item].useCustomCenterOfMass = useCustom;
    emit propertiesChanged(item);
    return true;
}

bool ObjectPropertiesManager::getUseCustomCenterOfMass(Item item) const
{
    if (!item) {
        return getDefaultProperties().useCustomCenterOfMass;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        return getDefaultProperties().useCustomCenterOfMass;
    }
    
    return it->second.useCustomCenterOfMass;
}

bool ObjectPropertiesManager::applyPropertiesToPhysics(Item item)
{
    if (!item || !m_physicsEngine) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        return false;
    }
    
    const ObjectProperties& props = it->second;
    
    // Apply physics properties to the physics engine
    // This would typically involve updating the PhysX rigid body
    // For now, we'll just emit a signal that the physics engine can listen to
    emit propertiesChanged(item);
    
    return true;
}

bool ObjectPropertiesManager::loadPropertiesFromPhysics(Item item)
{
    if (!item || !m_physicsEngine) {
        return false;
    }
    
    // This would typically involve reading properties from the PhysX rigid body
    // For now, we'll just return true
    return true;
}

bool ObjectPropertiesManager::validateProperties(const ObjectProperties& properties) const
{
    return validateMass(properties.mass) &&
           validateFriction(properties.staticFriction) &&
           validateFriction(properties.dynamicFriction) &&
           validateRestitution(properties.restitution) &&
           validateCenterOfMass(properties.centerOfMass);
}

ObjectPropertiesManager::ObjectProperties ObjectPropertiesManager::getDefaultProperties() const
{
    return ObjectProperties();
}

bool ObjectPropertiesManager::saveObjectProperties(Item item)
{
    if (!item) {
        return false;
    }
    
    auto it = m_objectProperties.find(item);
    if (it == m_objectProperties.end()) {
        return false;
    }
    
    // Save to JSON file
    saveToJSON(QString("object_%1_properties.json").arg(item->Name()));
    emit propertiesSaved(item);
    return true;
}

bool ObjectPropertiesManager::loadObjectProperties(Item item)
{
    if (!item) {
        return false;
    }
    
    // Load from JSON file
    loadFromJSON(QString("object_%1_properties.json").arg(item->Name()));
    emit propertiesLoaded(item);
    return true;
}

bool ObjectPropertiesManager::saveAllProperties(const QString& filename)
{
    QString filepath = filename.isEmpty() ? PROPERTIES_FILE : filename;
    saveToJSON(filepath);
    return true;
}

bool ObjectPropertiesManager::loadAllProperties(const QString& filename)
{
    QString filepath = filename.isEmpty() ? PROPERTIES_FILE : filename;
    loadFromJSON(filepath);
    return true;
}

bool ObjectPropertiesManager::hasObjectProperties(Item item) const
{
    if (!item) {
        return false;
    }
    
    return m_objectProperties.find(item) != m_objectProperties.end();
}

void ObjectPropertiesManager::removeObjectProperties(Item item)
{
    if (!item) {
        return;
    }
    
    auto it = m_objectProperties.find(item);
    if (it != m_objectProperties.end()) {
        m_objectProperties.erase(it);
        emit propertiesChanged(item);
    }
}

QStringList ObjectPropertiesManager::getObjectsWithProperties() const
{
    QStringList objectNames;
    for (const auto& pair : m_objectProperties) {
        if (pair.first) {
            objectNames.append(pair.first->Name());
        }
    }
    return objectNames;
}

void ObjectPropertiesManager::onMaterialChanged(const QString& materialName)
{
    // Update all objects that use this material
    for (auto& pair : m_objectProperties) {
        if (pair.second.materialName == materialName) {
            emit propertiesChanged(pair.first);
        }
    }
}

void ObjectPropertiesManager::onPhysicsObjectAdded(Item item)
{
    // Load properties for newly added physics object
    if (item && hasObjectProperties(item)) {
        loadObjectProperties(item);
    }
}

void ObjectPropertiesManager::onPhysicsObjectRemoved(Item item)
{
    // Save properties before removing physics object
    if (item && hasObjectProperties(item)) {
        saveObjectProperties(item);
    }
}

void ObjectPropertiesManager::applyMaterialProperties(Item item, const ObjectProperties& props)
{
    if (!m_materialManager) {
        return;
    }
    
    // Apply material properties from the material manager
    MaterialProperties material = m_materialManager->getMaterial(props.materialName);
    if (material.name.isEmpty()) {
        return;
    }
    
    // Update object properties with material values
    auto it = m_objectProperties.find(item);
    if (it != m_objectProperties.end()) {
        it->second.staticFriction = material.staticFriction;
        it->second.dynamicFriction = material.dynamicFriction;
        it->second.restitution = material.restitution;
    }
}

void ObjectPropertiesManager::applyPhysicsProperties(Item item, const ObjectProperties& props)
{
    if (!m_physicsEngine) {
        return;
    }
    
    // Apply physics properties to the physics engine
    // This would involve updating the PhysX rigid body properties
    // For now, we'll just emit a signal
    emit propertiesChanged(item);
}

bool ObjectPropertiesManager::validateMass(float mass) const
{
    return mass >= MIN_MASS && mass <= MAX_MASS;
}

bool ObjectPropertiesManager::validateFriction(float friction) const
{
    return friction >= MIN_FRICTION && friction <= MAX_FRICTION;
}

bool ObjectPropertiesManager::validateRestitution(float restitution) const
{
    return restitution >= MIN_RESTITUTION && restitution <= MAX_RESTITUTION;
}

bool ObjectPropertiesManager::validateCenterOfMass(const PxVec3& centerOfMass) const
{
    return centerOfMass.x >= MIN_CENTER_OF_MASS && centerOfMass.x <= MAX_CENTER_OF_MASS &&
           centerOfMass.y >= MIN_CENTER_OF_MASS && centerOfMass.y <= MAX_CENTER_OF_MASS &&
           centerOfMass.z >= MIN_CENTER_OF_MASS && centerOfMass.z <= MAX_CENTER_OF_MASS;
}

void ObjectPropertiesManager::saveToJSON(const QString& filename)
{
    QJsonObject root;
    QJsonArray objectsArray;
    
    for (const auto& pair : m_objectProperties) {
        if (!pair.first) continue;
        
        QJsonObject objectObj;
        objectObj["name"] = pair.first->Name();
        objectObj["mass"] = pair.second.mass;
        objectObj["staticFriction"] = pair.second.staticFriction;
        objectObj["dynamicFriction"] = pair.second.dynamicFriction;
        objectObj["restitution"] = pair.second.restitution;
        objectObj["centerOfMassX"] = pair.second.centerOfMass.x;
        objectObj["centerOfMassY"] = pair.second.centerOfMass.y;
        objectObj["centerOfMassZ"] = pair.second.centerOfMass.z;
        objectObj["materialName"] = pair.second.materialName;
        objectObj["useCustomProperties"] = pair.second.useCustomProperties;
        
        objectsArray.append(objectObj);
    }
    
    root["objects"] = objectsArray;
    
    QJsonDocument doc(root);
    QFile file(filename);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
    }
}

void ObjectPropertiesManager::loadFromJSON(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        return;
    }
    
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) {
        return;
    }
    
    QJsonObject root = doc.object();
    QJsonArray objectsArray = root["objects"].toArray();
    
    for (const QJsonValue& value : objectsArray) {
        QJsonObject objectObj = value.toObject();
        
        // Note: We can't reconstruct Item pointers from JSON, so we'll just store the data
        // and let the physics engine handle the actual object references
        ObjectProperties props;
        props.mass = objectObj["mass"].toDouble();
        props.staticFriction = objectObj["staticFriction"].toDouble();
        props.dynamicFriction = objectObj["dynamicFriction"].toDouble();
        props.restitution = objectObj["restitution"].toDouble();
        props.centerOfMass = PxVec3(
            objectObj["centerOfMassX"].toDouble(),
            objectObj["centerOfMassY"].toDouble(),
            objectObj["centerOfMassZ"].toDouble()
        );
        props.materialName = objectObj["materialName"].toString();
        props.useCustomProperties = objectObj["useCustomProperties"].toBool();
        
        // Store in a way that can be retrieved later by object name
        // This is a simplified approach - in a real implementation you'd want
        // to maintain a mapping between object names and their properties
    }
} 