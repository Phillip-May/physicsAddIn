#ifndef ROBODKCONVEYORHOST_H
#define ROBODKCONVEYORHOST_H

#include "ConveyorCore.h"
#include "PlacedItem.h"
#include "robodktypes.h"

#include <QByteArray>
#include <QString>
#include <QStringList>

#include <map>
#include <vector>

class IRoboDK;
class PhysicsWorld;

// Per-item persistent conveyor description. This is not the station-level live IO parameter.
extern const char* const kConveyorItemParam;

// Adapts RoboDK items to the shared conveyor runtime. See docs/conveyors.md.
class RoboDkConveyorHost final : public conveyorcore::Host {
public:
    struct Segment : PlacedItem {
        QString nextName;
        Item parent = nullptr;
        Item prototype = nullptr;
        TransformNodeData parameters;
        // Cached serialized form, used to avoid no-op station edits.
        QByteArray parametersBlob;
        conveyorcore::ConveyorSpec spec;
        // Derived from the exported prototype mesh.
        CadVec3 workpieceCentreOffsetMm;
        double spawnProgress = 0.0;
        double workpieceLengthAlongLaneMm = 0.0;
        CadTransform workpieceMeasuredAt;
        struct Stated {
            bool endInset = false;
            bool pitch = false;
            bool deckWidth = false;
            bool deckHeight = false;
        };
        Stated stated;
    };

    RoboDkConveyorHost(RoboDK* rdk, PhysicsWorld* world);

    void clear();
    conveyorcore::ConveyorId add(const Segment& segment);
    // Leaves a tombstone so existing ConveyorIds remain stable.
    void forget(conveyorcore::ConveyorId id);
    std::vector<conveyorcore::ConveyorId> invalidConveyors() const;
    conveyorcore::ConveyorId idNamed(const QString& name) const;
    conveyorcore::ConveyorId idOfItem(Item item) const;
    const std::vector<Segment>& segments() const { return m_segments; }
    Segment* mutableSegment(conveyorcore::ConveyorId id);
    void measureSegment(Segment& segment);
    bool workpieceHasTurned(const Segment& segment) const;
    bool remeasureMoved();
    // Leaves the existing configuration intact on failure.
    bool reconfigure(conveyorcore::ConveyorId id, const TransformNodeData& parameters,
                     QString* error);

    conveyorcore::ProductId productFor(Item item) const;
    Item itemFor(conveyorcore::ProductId id) const;
    void trackProduct(conveyorcore::ProductId id, Item item);
    void forgetProduct(conveyorcore::ProductId id);
    std::vector<conveyorcore::ProductId> invalidProducts() const;

    bool consumeStationChanged();

    bool conveyorSpec(conveyorcore::ConveyorId, conveyorcore::ConveyorSpec* out) const override;
    bool productHasBody(const conveyorcore::Product&) const override;
    bool productBodyPose(const conveyorcore::Product&, CadTransform* world) const override;
    double closestProgress(conveyorcore::ConveyorId, const CadVec3& world) const override;
    bool inDeleterVolume(const conveyorcore::Product&, const CadVec3& world) const override;
    bool reachedEnd(const conveyorcore::Product&, const CadVec3& world,
                    double allowanceMm) const override;
    void notePhysicalCenter(const conveyorcore::Product&, const CadVec3& world) override;
    conveyorcore::TransferTarget nextConveyor(conveyorcore::ConveyorId,
                                              bool leavingForward) const override;
    void reparentProduct(const conveyorcore::Product&, conveyorcore::ConveyorId) override;
    void placeProduct(const conveyorcore::Product&) override;
    bool spawnProduct(conveyorcore::ConveyorId, conveyorcore::Product* out) override;
    void destroyProduct(const conveyorcore::Product&) override;
    void sceneChanged() override {}

    bool placement(const Segment& segment, CadTransform* world) const;
    CadVec3 pathPoint(const Segment& segment, double progress) const;

private:
    bool lane(const Segment& segment, CadTransform* world) const;
    const Segment* originOf(const conveyorcore::Product& product) const;

    const Segment* segmentOf(conveyorcore::ConveyorId id) const;

    RoboDK* m_rdk = nullptr;
    PhysicsWorld* m_world = nullptr;
    std::vector<Segment> m_segments;
    std::map<conveyorcore::ProductId, Item> m_items;
    conveyorcore::ProductId m_nextProductId = 1;
    bool m_stationChanged = false;
};

bool conveyorSegmentFromItem(RoboDK* rdk, Item item, RoboDkConveyorHost::Segment* out,
                             QString* error);

Item createConveyorItem(RoboDK* rdk, const QString& name, QString* error);

// Returns true only when the per-item parameter changed.
bool writeConveyorParameters(RoboDK* rdk, RoboDkConveyorHost::Segment& segment);

// Parses the positional fields documented in docs/conveyors.md.
bool parseConveyorSegment(RoboDK* rdk, const QStringList& fields,
                          RoboDkConveyorHost::Segment* out, QString* error);

#endif // ROBODKCONVEYORHOST_H
