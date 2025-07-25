#ifndef MATERIALMANAGER_H
#define MATERIALMANAGER_H

#include <QObject>
#include <QString>
#include <QMap>
#include <QList>
#include <QColor>
#include "IPhysicsEngine.h"
#include "PxPhysicsAPI.h"

using namespace physx;

/**
 * @brief Material properties structure
 */
struct MaterialProperties {
    QString name;
    float staticFriction;
    float dynamicFriction;
    float restitution;
    QColor color;  // For visualization
    QString description;
    
    MaterialProperties() : staticFriction(0.5f), dynamicFriction(0.3f), restitution(0.1f), color(Qt::gray) {}
    MaterialProperties(const QString& n, float sf, float df, float r, const QColor& c = Qt::gray, const QString& desc = "")
        : name(n), staticFriction(sf), dynamicFriction(df), restitution(r), color(c), description(desc) {}
};

/**
 * @brief Material manager for handling physics materials
 */
class MaterialManager : public QObject
{
    Q_OBJECT

public:
    explicit MaterialManager(IPhysicsEngine* physicsEngine, QObject* parent = nullptr);
    ~MaterialManager();

    // Material management
    QStringList getAvailableMaterials() const;
    MaterialProperties getMaterial(const QString& materialName) const;
    bool addCustomMaterial(const MaterialProperties& material);
    bool removeCustomMaterial(const QString& materialName);
    bool updateMaterial(const QString& materialName, const MaterialProperties& newProperties);
    
    // Object material assignment
    bool setObjectMaterial(CadNode* item, const QString& materialName);
    QString getObjectMaterial(CadNode* item) const;
    bool hasObjectMaterial(CadNode* item) const;
    void removeObjectMaterial(CadNode* item);
    
    // Material application
    bool applyMaterialToObject(CadNode* item, const QString& materialName);
    bool applyMaterialToObject(CadNode* item, const MaterialProperties& material);
    
    // Default materials
    void initializeDefaultMaterials();
    QStringList getDefaultMaterialNames() const;
    bool isDefaultMaterial(const QString& materialName) const;

    // Material manager dialog
    void showMaterialManagerDialog(QWidget* parent = nullptr);
    
    // Material presets
    static MaterialProperties createMetalMaterial();
    static MaterialProperties createPlasticMaterial();
    static MaterialProperties createWoodMaterial();
    static MaterialProperties createRubberMaterial();
    static MaterialProperties createIceMaterial();
    static MaterialProperties createGlassMaterial();
    static MaterialProperties createStoneMaterial();
    static MaterialProperties createFabricMaterial();
    static MaterialProperties createCardboardMaterial();
    static MaterialProperties createSheetMetalMaterial();

signals:
    void materialAdded(const QString& materialName);
    void materialRemoved(const QString& materialName);
    void materialUpdated(const QString& materialName);
    void objectMaterialChanged(CadNode* item, const QString& materialName);

private slots:
    void onObjectAdded(CadNode* item);
    void onObjectRemoved(CadNode* item);

private:
    IPhysicsEngine* m_physicsEngine;
    
    // Material storage
    QMap<QString, MaterialProperties> m_materials;
    QMap<CadNode*, QString> m_objectMaterials;  // Object -> Material mapping
    
    // Default material names
    QStringList m_defaultMaterialNames;
    
    // Helper methods
    void createDefaultMaterials();
    bool validateMaterialProperties(const MaterialProperties& material) const;
    QString generateUniqueName(const QString& baseName) const;
};

#endif // MATERIALMANAGER_H 
