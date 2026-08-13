#include "RoboDkBridge.h"

#include "iitem.h"
#include "irobodk.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>

namespace rdkbridge {
namespace {

// RoboDK picks the exporter from the extension. STL is the one lossless-enough mesh format every
// RoboDK build writes without a licence prompt, and binary STL in particular needs no parser beyond
// a length check - which matters because this runs inside RoboDK's process.
constexpr const char* kExportExtension = ".stl";

QString uniqueTemporaryStlPath(Item item) {
    // The item pointer disambiguates two objects sharing a name, which RoboDK permits. It is only a
    // filename, so its being an address is unimportant beyond uniqueness within one export.
    const quintptr key = reinterpret_cast<quintptr>(item);
    return QDir(QDir::tempPath())
        .filePath(QString("robodk_physics_%1%2").arg(key, 0, 16).arg(kExportExtension));
}

void accumulateBounds(const CadVec3& vertex, CadVec3& minimum, CadVec3& maximum, bool& seeded) {
    if (!seeded) {
        minimum = vertex;
        maximum = vertex;
        seeded = true;
        return;
    }
    minimum.x = std::min(minimum.x, vertex.x);
    minimum.y = std::min(minimum.y, vertex.y);
    minimum.z = std::min(minimum.z, vertex.z);
    maximum.x = std::max(maximum.x, vertex.x);
    maximum.y = std::max(maximum.y, vertex.y);
    maximum.z = std::max(maximum.z, vertex.z);
}

struct VertexWelder {
    std::map<std::array<int64_t, 3>, uint32_t> seen;
    MeshData* mesh = nullptr;
    bool boundsSeeded = false;

    uint32_t add(double x, double y, double z) {
        constexpr double kQuantum = 1.0e-3; // millimetres
        const std::array<int64_t, 3> key{
            static_cast<int64_t>(std::llround(x / kQuantum)),
            static_cast<int64_t>(std::llround(y / kQuantum)),
            static_cast<int64_t>(std::llround(z / kQuantum))};
        const auto existing = seen.find(key);
        if (existing != seen.end()) return existing->second;
        const CadVec3 vertex{x, y, z};
        const uint32_t index = static_cast<uint32_t>(mesh->verticesMm.size());
        mesh->verticesMm.push_back(vertex);
        accumulateBounds(vertex, mesh->minimumMm, mesh->maximumMm, boundsSeeded);
        seen.emplace(key, index);
        return index;
    }
};

bool readBinaryStl(QFile& file, MeshData* out, QString* errorMessage) {
    // 80-byte header, a triangle count, then 50 bytes per facet.
    if (!file.seek(80)) {
        if (errorMessage) *errorMessage = "The STL file ended inside its header.";
        return false;
    }
    quint32 triangles = 0;
    if (file.read(reinterpret_cast<char*>(&triangles), 4) != 4) {
        if (errorMessage) *errorMessage = "The STL file has no triangle count.";
        return false;
    }
    const qint64 expected = 84 + static_cast<qint64>(triangles) * 50;
    if (triangles == 0 || file.size() < expected) {
        if (errorMessage) *errorMessage = "The STL file is shorter than its triangle count claims.";
        return false;
    }
    const QByteArray payload = file.read(static_cast<qint64>(triangles) * 50);
    if (payload.size() != static_cast<qint64>(triangles) * 50) {
        if (errorMessage) *errorMessage = "The STL file ended early.";
        return false;
    }
    VertexWelder welder;
    welder.mesh = out;
    out->indices.reserve(static_cast<size_t>(triangles) * 3);
    const char* cursor = payload.constData();
    for (quint32 triangle = 0; triangle < triangles; ++triangle) {
        const char* vertexData = cursor + 12;
        for (int corner = 0; corner < 3; ++corner) {
            float xyz[3];
            std::memcpy(xyz, vertexData + corner * 12, 12);
            out->indices.push_back(welder.add(xyz[0], xyz[1], xyz[2]));
        }
        cursor += 50;
    }
    return out->valid();
}

bool readAsciiStl(QFile& file, MeshData* out, QString* errorMessage) {
    if (!file.seek(0)) {
        if (errorMessage) *errorMessage = "The STL file could not be rewound.";
        return false;
    }
    QTextStream stream(&file);
    VertexWelder welder;
    welder.mesh = out;
    QString token;
    while (!stream.atEnd()) {
        stream >> token;
        if (token != QLatin1String("vertex")) continue;
        double xyz[3] = {0.0, 0.0, 0.0};
        stream >> xyz[0] >> xyz[1] >> xyz[2];
        out->indices.push_back(welder.add(xyz[0], xyz[1], xyz[2]));
    }
    // A facet loop that lost a corner would otherwise cook as a degenerate triangle.
    out->indices.resize(out->indices.size() - (out->indices.size() % 3));
    if (!out->valid()) {
        if (errorMessage) *errorMessage = "The STL file contained no usable triangles.";
        return false;
    }
    return true;
}

} // namespace

CadVec3 MeshData::halfExtentsMm() const {
    return CadVec3{std::max(0.5, (maximumMm.x - minimumMm.x) * 0.5),
                   std::max(0.5, (maximumMm.y - minimumMm.y) * 0.5),
                   std::max(0.5, (maximumMm.z - minimumMm.z) * 0.5)};
}

CadVec3 MeshData::centerMm() const {
    return CadVec3{(maximumMm.x + minimumMm.x) * 0.5,
                   (maximumMm.y + minimumMm.y) * 0.5,
                   (maximumMm.z + minimumMm.z) * 0.5};
}

CadTransform toCadTransform(const Mat& pose) {
    CadTransform transform;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
            transform.values[static_cast<size_t>(row * 4 + column)] = pose.Get(row, column);
        }
    }
    return transform;
}

Mat toRoboDkPose(const CadTransform& transform) {
    Mat pose;
    pose.setToIdentity();
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
            pose.Set(row, column, transform.values[static_cast<size_t>(row * 4 + column)]);
        }
    }
    return pose;
}

bool readStlFile(const QString& path, MeshData* out, QString* errorMessage) {
    if (!out) return false;
    *out = MeshData();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) *errorMessage = QString("Could not open %1.").arg(path);
        return false;
    }
    if (file.size() < 84) {
        if (errorMessage) *errorMessage = "The exported STL is too small to contain geometry.";
        return false;
    }
    // "solid" opens an ASCII file, but some binary writers put it in the header too, so the
    // declared triangle count deciding the file length is the test that settles it.
    quint32 declared = 0;
    if (file.seek(80) && file.read(reinterpret_cast<char*>(&declared), 4) == 4 &&
        file.size() == 84 + static_cast<qint64>(declared) * 50) {
        return readBinaryStl(file, out, errorMessage);
    }
    QString binaryError;
    if (readBinaryStl(file, out, &binaryError)) return true;
    *out = MeshData();
    return readAsciiStl(file, out, errorMessage);
}

bool exportItemMesh(RoboDK* rdk, Item item, MeshData* out, QString* errorMessage) {
    if (!rdk || !item || !out) {
        if (errorMessage) *errorMessage = "No item to export.";
        return false;
    }
    const QString path = uniqueTemporaryStlPath(item);
    QFile::remove(path);
    // Save writes the item in its own local frame, which is what the body wants: the item's pose is
    // applied to the actor and would otherwise be counted twice.
    rdk->Save(path, item);
    if (!QFileInfo::exists(path)) {
        if (errorMessage) {
            *errorMessage = "RoboDK exported no geometry for this item. Items without a mesh - "
                            "frames, targets and programs - cannot take part in the simulation.";
        }
        return false;
    }
    const bool read = readStlFile(path, out, errorMessage);
    QFile::remove(path);
    return read;
}

std::vector<CadVec3> reducedHullPoints(const std::vector<CadVec3>& verticesMm, size_t maximum) {
    if (verticesMm.size() <= maximum) return verticesMm;

    CadVec3 minimum{};
    CadVec3 maximum3{};
    bool seeded = false;
    for (const CadVec3& vertex : verticesMm) accumulateBounds(vertex, minimum, maximum3, seeded);

    const double spanX = std::max(1.0e-6, maximum3.x - minimum.x);
    const double spanY = std::max(1.0e-6, maximum3.y - minimum.y);
    const double spanZ = std::max(1.0e-6, maximum3.z - minimum.z);
    const CadVec3 center{(maximum3.x + minimum.x) * 0.5, (maximum3.y + minimum.y) * 0.5,
                         (maximum3.z + minimum.z) * 0.5};
    const int divisions = std::max(2, static_cast<int>(std::cbrt(static_cast<double>(maximum))));

    struct Candidate {
        CadVec3 point;
        double distanceSquared = -1.0;
    };
    std::map<int64_t, Candidate> cells;
    for (const CadVec3& vertex : verticesMm) {
        const auto bucket = [divisions](double value, double minimumValue, double span) {
            const int index = static_cast<int>((value - minimumValue) / span * divisions);
            return std::min(divisions - 1, std::max(0, index));
        };
        const int64_t key = (static_cast<int64_t>(bucket(vertex.x, minimum.x, spanX)) * divisions +
                             bucket(vertex.y, minimum.y, spanY)) * divisions +
                            bucket(vertex.z, minimum.z, spanZ);
        const double dx = vertex.x - center.x;
        const double dy = vertex.y - center.y;
        const double dz = vertex.z - center.z;
        const double distanceSquared = dx * dx + dy * dy + dz * dz;
        Candidate& candidate = cells[key];
        if (distanceSquared > candidate.distanceSquared) {
            candidate.point = vertex;
            candidate.distanceSquared = distanceSquared;
        }
    }

    std::vector<CadVec3> reduced;
    reduced.reserve(cells.size());
    for (const auto& cell : cells) reduced.push_back(cell.second.point);
    return reduced;
}

} // namespace rdkbridge
