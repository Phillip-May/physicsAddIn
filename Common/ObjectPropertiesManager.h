#ifndef OBJECTPROPERTIESMANAGER_H
#define OBJECTPROPERTIESMANAGER_H

#include <QObject>
#include <map>
#include <memory>
#include "iitem.h"
#include "PxPhysicsAPI.h"

using namespace physx;

class IPhysicsEngine;
class MaterialManager;

/**
 * @brief Manages object-specific physics properties
 * 
 * This class handles mass, friction, restitution, center of mass,
 * and other object-specific physics properties. It stores these
 * properties persistently and can apply them to physics objects.
 */
class ObjectPropertiesManager : public QObject
{
    Q_OBJECT

public:
    // Object physics properties structure
    struct ObjectProperties {
        float mass;
        float staticFriction;
        float dynamicFriction;
        float restitution;
        PxVec3 centerOfMass;
        QString materialName;
        bool useCustomProperties;
        bool useCustomCenterOfMass;
        
        ObjectProperties() : mass(100.0f), staticFriction(0.8f), dynamicFriction(0.6f),
                           restitution(0.1f), centerOfMass(0,0,0), useCustomProperties(false), 
                           useCustomCenterOfMass(false) {}
    };

    explicit ObjectPropertiesManager(IPhysicsEngine* physicsEngine, 
                                  MaterialManager* materialManager, QObject* parent = nullptr);
    ~ObjectPropertiesManager();

    // Property management
    bool setObjectMass(Item item, float mass);
    float getObjectMass(Item item) const;
    
    bool setObjectFriction(Item item, float staticFriction, float dynamicFriction);
    void getObjectFriction(Item item, float& staticFriction, float& dynamicFriction) const;
    
    bool setObjectRestitution(Item item, float restitution);
    float getObjectRestitution(Item item) const;
    
    bool setObjectCenterOfMass(Item item, const PxVec3& centerOfMass);
    PxVec3 getObjectCenterOfMass(Item item) const;
    
    bool setObjectMaterial(Item item, const QString& materialName);
    QString getObjectMaterial(Item item) const;
    
    bool setUseCustomProperties(Item item, bool useCustom);
    bool getUseCustomProperties(Item item) const;
    
    bool setUseCustomCenterOfMass(Item item, bool useCustom);
    bool getUseCustomCenterOfMass(Item item) const;
    
    // Property application
    bool applyPropertiesToPhysics(Item item);
    bool loadPropertiesFromPhysics(Item item);
    
    // Property validation
    bool validateProperties(const ObjectProperties& properties) const;
    ObjectProperties getDefaultProperties() const;
    
    // Persistence
    bool saveObjectProperties(Item item);
    bool loadObjectProperties(Item item);
    bool saveAllProperties(const QString& filename = "");
    bool loadAllProperties(const QString& filename = "");
    
    // Utility
    bool hasObjectProperties(Item item) const;
    void removeObjectProperties(Item item);
    QStringList getObjectsWithProperties() const;

signals:
    void propertiesChanged(Item item);
    void propertiesSaved(Item item);
    void propertiesLoaded(Item item);
    void validationError(const QString& error);

private slots:
    void onMaterialChanged(const QString& materialName);
    void onPhysicsObjectAdded(Item item);
    void onPhysicsObjectRemoved(Item item);

private:
    IPhysicsEngine* m_physicsEngine;
    MaterialManager* m_materialManager;
    
    // Object properties storage
    std::map<Item, ObjectProperties> m_objectProperties;
    
    // Helper methods
    void applyMaterialProperties(Item item, const ObjectProperties& props);
    void applyPhysicsProperties(Item item, const ObjectProperties& props);
    bool validateMass(float mass) const;
    bool validateFriction(float friction) const;
    bool validateRestitution(float restitution) const;
    bool validateCenterOfMass(const PxVec3& centerOfMass) const;
    
    // Persistence helpers
    void saveToJSON(const QString& filename);
    void loadFromJSON(const QString& filename);
    
    // Constants
    static constexpr float MIN_MASS = 0.001f;
    static constexpr float MAX_MASS = 10000.0f;
    static constexpr float MIN_FRICTION = 0.0f;
    static constexpr float MAX_FRICTION = 2.0f;
    static constexpr float MIN_RESTITUTION = 0.0f;
    static constexpr float MAX_RESTITUTION = 1.0f;
    static constexpr float MIN_CENTER_OF_MASS = -1000.0f;
    static constexpr float MAX_CENTER_OF_MASS = 1000.0f;
    static constexpr const char* PROPERTIES_FILE = "object_properties.json";
};

#endif // OBJECTPROPERTIESMANAGER_H 
