#ifndef MATERIALMANAGER_H
#define MATERIALMANAGER_H

#include <QObject>
#include <QString>
#include <QMap>
#include <QList>
#include <QColor>

#include "CadNode.h"

struct MaterialProperties {
    QString name;
    float staticFriction;
    float dynamicFriction;
    float restitution;
    QColor color;
    QString description;

    MaterialProperties() : staticFriction(0.5f), dynamicFriction(0.3f), restitution(0.1f), color(Qt::gray) {}
    MaterialProperties(const QString& n, float sf, float df, float r, const QColor& c = Qt::gray, const QString& desc = "")
        : name(n), staticFriction(sf), dynamicFriction(df), restitution(r), color(c), description(desc) {}
};

class MaterialManager : public QObject
{
    Q_OBJECT

public:
    explicit MaterialManager(QObject* parent = nullptr);

    QStringList getAvailableMaterials() const;
    MaterialProperties getMaterial(const QString& materialName) const;
    bool addCustomMaterial(const MaterialProperties& material);
    bool removeCustomMaterial(const QString& materialName);
    bool updateMaterial(const QString& materialName, const MaterialProperties& newProperties);

    bool setObjectMaterial(CadNode* item, const QString& materialName);
    QString getObjectMaterial(CadNode* item) const;
    bool hasObjectMaterial(CadNode* item) const;
    void removeObjectMaterial(CadNode* item);

    bool applyMaterialToObject(CadNode* item, const QString& materialName);
    bool applyMaterialToObject(CadNode* item, const MaterialProperties& material);

    void initializeDefaultMaterials();
    QStringList getDefaultMaterialNames() const;
    bool isDefaultMaterial(const QString& materialName) const;

    void showMaterialManagerDialog(QWidget* parent = nullptr);

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

private:
    QMap<QString, MaterialProperties> m_materials;
    QMap<CadNode*, QString> m_objectMaterials;

    QStringList m_defaultMaterialNames;

    void createDefaultMaterials();
    bool validateMaterialProperties(const MaterialProperties& material) const;
    QString generateUniqueName(const QString& baseName) const;
};

#endif // MATERIALMANAGER_H
