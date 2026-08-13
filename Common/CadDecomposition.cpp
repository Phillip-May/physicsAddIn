// Convex decomposition. The one translation unit that defines the V-HACD
// implementation, so ENABLE_VHACD_IMPLEMENTATION belongs here and nowhere else.
#include "CadDecomposition.h"

#include "CadNodeOps.h"

#define ENABLE_VHACD_IMPLEMENTATION 1
#include <vhacd/VHACD.h>
#include "../external/CoACD/public/coacd.h"

#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <QDebug>
#include <TopExp_Explorer.hxx>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>

// Removes duplicate vertices and remaps indices, skipping degenerate triangles.
template <typename VertType, typename TriType>
void filterAndRemapVerticesAndIndices(
    const std::vector<VertType>& inputVerts,
    const std::vector<TriType>& inputTris,
    std::vector<VertType>& uniqueVerts,
    std::vector<std::array<uint32_t, 3>>& remappedTris,
    double epsilon = 1e-4)
{
    uniqueVerts.clear();
    remappedTris.clear();
    std::vector<int> oldToNew(inputVerts.size(), -1);
    for (size_t vi = 0; vi < inputVerts.size(); ++vi) {
        const auto& p = inputVerts[vi];
        int found = -1;
        for (size_t uj = 0; uj < uniqueVerts.size(); ++uj) {
            double dx = uniqueVerts[uj][0] - p[0], dy = uniqueVerts[uj][1] - p[1], dz = uniqueVerts[uj][2] - p[2];
            if (dx*dx + dy*dy + dz*dz < epsilon*epsilon) { found = (int)uj; break; }
        }
        if (found == -1) {
            oldToNew[vi] = (int)uniqueVerts.size();
            uniqueVerts.push_back(p);
        } else {
            oldToNew[vi] = found;
        }
    }
    for (const auto& tri : inputTris) {
        int i0 = oldToNew[tri[0]];
        int i1 = oldToNew[tri[1]];
        int i2 = oldToNew[tri[2]];
        if (i0 == i1 || i1 == i2 || i2 == i0) continue;
        remappedTris.push_back({(uint32_t)i0, (uint32_t)i1, (uint32_t)i2});
    }
}


// The decomposition input frame: faces are accumulated from the physics node itself (identity at
// its parent) and rebased by node->loc alone, so hull vertices land in the physics node's local
// frame regardless of its depth in the scene tree. Walking real ancestors here would have to
// change both sides of that pairing at once.
static void collectPhysicsMeshTriangles(CadNode* node, std::vector<double>& vertices,
                                        std::vector<uint32_t>& triangles, int& numInputFaces) {
    std::vector<FaceWithTransform> faces;
    qDebug() << "Node Name: " << node->name.c_str();
    collectFaceNodesWithTransform(node, TopLoc_Location(), faces);
    faces.erase(std::remove_if(faces.begin(), faces.end(),
        [](const FaceWithTransform& f) { return f.node->excludedFromDecomposition; }), faces.end());
    qDebug() << "Face count: " << faces.size();
    numInputFaces = static_cast<int>(faces.size());
    const TopLoc_Location physicsGlobalLoc = node->loc;
    uint32_t vertOffset = 0;
    for (const auto& faceInfo : faces) {
        TopLoc_Location relLoc = physicsGlobalLoc.Inverted() * faceInfo.accumulatedLoc;
        XCAFNodeData* xData = faceInfo.node->asXCAF();
        if (!xData || !xData->hasFace()) continue;
        TopoDS_Face face = xData->getFace();
        BRepMesh_IncrementalMesh mesher(face, 0.5);
        TopLoc_Location locCopy;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, locCopy);
        if (tri.IsNull() || tri->NbTriangles() == 0) continue;
        std::vector<uint32_t> localIndices(tri->NbNodes());
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i);
            p.Transform(relLoc.Transformation());
            vertices.push_back(p.X());
            vertices.push_back(p.Y());
            vertices.push_back(p.Z());
            localIndices[i-1] = vertOffset++;
        }
        for (int i = tri->Triangles().Lower(); i <= tri->Triangles().Upper(); ++i) {
            int n1, n2, n3;
            tri->Triangles()(i).Get(n1, n2, n3);
            triangles.push_back(localIndices[n1-1]);
            triangles.push_back(localIndices[n2-1]);
            triangles.push_back(localIndices[n3-1]);
        }
    }
}

// Total volume of distinct solids (deduplicated by TShape) under the node.
static double collectDistinctSolidVolume(CadNode* root, size_t& solidCount) {
    std::vector<TopoDS_Shape> solids;
    std::function<void(CadNode*)> collectSolids = [&](CadNode* node) {
        if (!node) return;
        XCAFNodeData* xData = node->asXCAF();
        if (xData && xData->type == TopAbs_SOLID && !xData->shape.IsNull()) {
            bool alreadyPresent = false;
            for (const auto& s : solids) {
                if (s.TShape().get() == xData->shape.TShape().get()) { alreadyPresent = true; break; }
            }
            if (!alreadyPresent) solids.push_back(xData->shape);
        }
        for (const auto& child : node->children) collectSolids(child.get());
    };
    collectSolids(root);
    double volume = 0.0;
    for (const auto& solid : solids) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(solid, props);
        volume += props.Mass();
    }
    solidCount = solids.size();
    return volume;
}

// Dedups/remaps one candidate hull; rejects degenerate, tiny, or nearly flat results.
static bool filterHullForStorage(const char* tag,
                                 const std::vector<std::array<double, 3>>& inputVerts,
                                 const std::vector<std::array<uint32_t, 3>>& inputTris,
                                 std::vector<std::array<double, 3>>& uniqueVerts,
                                 std::vector<std::array<uint32_t, 3>>& remappedTris) {
    const double epsilon = 1e-4; // 0.1mm for duplicate detection
    filterAndRemapVerticesAndIndices(inputVerts, inputTris, uniqueVerts, remappedTris, epsilon);
    if (uniqueVerts.size() < 4 || remappedTris.empty()) {
        qDebug() << tag << "Skipping degenerate hull (" << uniqueVerts.size() << " unique verts, " << remappedTris.size() << " triangles)";
        return false;
    }
    double minX=1e30,maxX=-1e30,minY=1e30,maxY=-1e30,minZ=1e30,maxZ=-1e30;
    for (const auto& p : uniqueVerts) {
        minX = std::min(minX, p[0]); maxX = std::max(maxX, p[0]);
        minY = std::min(minY, p[1]); maxY = std::max(maxY, p[1]);
        minZ = std::min(minZ, p[2]); maxZ = std::max(maxZ, p[2]);
    }
    double extentX = maxX - minX;
    double extentY = maxY - minY;
    double extentZ = maxZ - minZ;
    double diag = std::sqrt(extentX*extentX + extentY*extentY + extentZ*extentZ);
    if (diag < 1e-2) {
        qDebug() << tag << "Skipping very small hull (diag=" << diag << ")";
        return false;
    }
    int smallDims = 0;
    if (extentX < 1e-3) smallDims++;
    if (extentY < 1e-3) smallDims++;
    if (extentZ < 1e-3) smallDims++;
    if (smallDims >= 2) {
        qDebug() << tag << "Skipping nearly flat/coplanar/collinear hull (extents: " << extentX << extentY << extentZ << ")";
        return false;
    }
    return true;
}

static void reportDecompositionVolume(const char* tag, double originalVolume, double totalVolume) {
    if (originalVolume > 0.0 && totalVolume > 0.0) {
        double ratio = totalVolume / originalVolume;
        int percent = static_cast<int>(ratio * 100.0 + 0.5);
        QString biggerSmaller = (ratio > 1.0) ? "bigger" : ((ratio < 1.0) ? "smaller" : "equal");
        qDebug() << QString("%1 Decomposition is %2% of original (%3) (hull/original)")
                    .arg(tag)
                    .arg(percent)
                    .arg(biggerSmaller);
    }
}

void generateVHACDStub(const QString& nodeName, const QJsonObject& params, CadNode* node) {
    uint32_t maxConvexHulls = params.value("maxConvexHulls").toInt(8);
    uint32_t resolution = params.value("resolution").toInt(200000);
    double minimumVolumePercentErrorAllowed = params.value("minimumVolumePercentErrorAllowed").toDouble(1.0);
    uint32_t maxRecursionDepth = params.value("maxRecursionDepth").toInt(10);
    bool shrinkWrap = params.value("shrinkWrap").toBool(true);
    int fillMode = params.value("fillMode").toInt(0);
    uint32_t maxNumVerticesPerCH = params.value("maxNumVerticesPerCH").toInt(64);
    bool asyncACD = params.value("asyncACD").toBool(true);
    uint32_t minEdgeLength = params.value("minEdgeLength").toInt(2);
    bool findBestPlane = params.value("findBestPlane").toBool(false);
    int numInputFaces = 0;
    std::vector<double> vertices;
    std::vector<uint32_t> triangles;
    collectPhysicsMeshTriangles(node, vertices, triangles, numInputFaces);
    size_t solidCount = 0;
    double originalVolume = collectDistinctSolidVolume(node, solidCount);
    if (solidCount == 0) {
        qDebug() << "[VHACD] WARNING: No solids found under this Physics node. Cannot compute true volume comparison.";
    } else {
        qDebug() << "[VHACD] Found" << solidCount << "unique solids. Total original solid volume:" << originalVolume;
    }
    VHACD::IVHACD::Parameters vhacdParams;
    vhacdParams.m_maxConvexHulls = maxConvexHulls;
    vhacdParams.m_resolution = resolution;
    vhacdParams.m_minimumVolumePercentErrorAllowed = minimumVolumePercentErrorAllowed;
    vhacdParams.m_maxRecursionDepth = maxRecursionDepth;
    vhacdParams.m_shrinkWrap = shrinkWrap;
    vhacdParams.m_fillMode = static_cast<VHACD::FillMode>(fillMode);
    vhacdParams.m_maxNumVerticesPerCH = maxNumVerticesPerCH;
    vhacdParams.m_asyncACD = asyncACD;
    vhacdParams.m_minEdgeLength = minEdgeLength;
    vhacdParams.m_findBestPlane = findBestPlane;
    VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
    bool ok = interfaceVHACD->Compute(vertices.data(), (uint32_t)(vertices.size() / 3),
                                      triangles.data(), (uint32_t)(triangles.size() / 3), vhacdParams);
    PhysicsNodeData* physData = node->asPhysics();
    physData->hulls.clear();
    physData->convexHullGenerated = ok;
    int hullCount = 0;
    double totalVolume = 0.0;
    if (ok) {
        hullCount = interfaceVHACD->GetNConvexHulls();
        for (int i = 0; i < hullCount; ++i) {
            VHACD::IVHACD::ConvexHull ch;
            interfaceVHACD->GetConvexHull(i, ch);
            std::vector<std::array<double, 3>> inputVerts;
            std::vector<std::array<uint32_t, 3>> inputTris;
            for (const auto& v : ch.m_points) {
                inputVerts.push_back({v.mX, v.mY, v.mZ});
            }
            for (const auto& tri : ch.m_triangles) {
                inputTris.push_back({tri.mI0, tri.mI1, tri.mI2});
            }
            std::vector<std::array<double, 3>> uniqueVerts;
            std::vector<std::array<uint32_t, 3>> remappedTris;
            if (!filterHullForStorage("[VHACD]", inputVerts, inputTris, uniqueVerts, remappedTris)) continue;
            ConvexHullData hullData;
            hullData.vertices = std::move(uniqueVerts);
            hullData.indices = std::move(remappedTris);
            physData->hulls.push_back(std::move(hullData));
            totalVolume += ch.m_volume;
        }
    }
    if (interfaceVHACD) interfaceVHACD->Release();
    qDebug() << "[VHACD] Node:" << nodeName
             << "Input faces:" << numInputFaces
             << "Input vertices:" << vertices.size() / 3
             << "Input triangles:" << triangles.size() / 3
             << "Generated hulls:" << hullCount
             << "Total hull volume:" << totalVolume
             << "Success:" << ok;
    reportDecompositionVolume("[VHACD]", originalVolume, totalVolume);
}

void generateCoACDStub(const QString& nodeName, double concavity, double, double, CadNode* node,
    int maxConvexHull, std::string preprocess, int prepRes, int sampleRes, int mctsNodes, int mctsIter, int mctsDepth,
    bool pca, bool merge, bool decimate, int maxChVertex, bool extrude, double extrudeMargin, std::string apxMode, int seed) {
    int numInputFaces = 0;
    std::vector<double> vertices;
    std::vector<uint32_t> triangles;
    collectPhysicsMeshTriangles(node, vertices, triangles, numInputFaces);
    size_t solidCount = 0;
    double originalVolume = collectDistinctSolidVolume(node, solidCount);
    coacd::Mesh inputMesh;
    for (size_t i = 0; i < vertices.size(); i += 3) {
        inputMesh.vertices.push_back({vertices[i], vertices[i+1], vertices[i+2]});
    }
    for (size_t i = 0; i < triangles.size(); i += 3) {
        inputMesh.indices.push_back({static_cast<int>(triangles[i]), static_cast<int>(triangles[i+1]), static_cast<int>(triangles[i+2])});
    }
    std::vector<coacd::Mesh> resultMeshes = coacd::CoACD(
        inputMesh, concavity, maxConvexHull, preprocess, prepRes, sampleRes, mctsNodes, mctsIter, mctsDepth,
        pca, merge, decimate, maxChVertex, extrude, extrudeMargin, apxMode, static_cast<unsigned int>(seed)
    );
    bool ok = !resultMeshes.empty();
    PhysicsNodeData* physData = node->asPhysics();
    physData->hulls.clear();
    physData->convexHullGenerated = ok;
    int hullCount = 0;
    double totalVolume = 0.0;
    if (ok) {
        hullCount = static_cast<int>(resultMeshes.size());
        for (const auto& resultMesh : resultMeshes) {
            std::vector<std::array<double, 3>> inputVerts;
            std::vector<std::array<uint32_t, 3>> inputTris;
            for (const auto& v : resultMesh.vertices) {
                inputVerts.push_back({v[0], v[1], v[2]});
            }
            for (const auto& tri : resultMesh.indices) {
                inputTris.push_back({(uint32_t)tri[0], (uint32_t)tri[1], (uint32_t)tri[2]});
            }
            std::vector<std::array<double, 3>> uniqueVerts;
            std::vector<std::array<uint32_t, 3>> remappedTris;
            if (!filterHullForStorage("[CoACD]", inputVerts, inputTris, uniqueVerts, remappedTris)) continue;
            double hullVolume = 0.0;
            for (const auto& triangle : remappedTris) {
                const auto& v1 = uniqueVerts[triangle[0]];
                const auto& v2 = uniqueVerts[triangle[1]];
                const auto& v3 = uniqueVerts[triangle[2]];
                hullVolume += ((v1[0] * v2[1] * v3[2]) + (v1[1] * v2[2] * v3[0]) + (v1[2] * v2[0] * v3[1]) -
                              (v3[0] * v2[1] * v1[2]) - (v3[1] * v2[2] * v1[0]) - (v3[2] * v2[0] * v1[1])) / 6.0;
            }
            ConvexHullData hullData;
            hullData.vertices = std::move(uniqueVerts);
            hullData.indices = std::move(remappedTris);
            physData->hulls.push_back(std::move(hullData));
            totalVolume += std::abs(hullVolume);
        }
    }
    qDebug() << "[CoACD] Node:" << nodeName
             << "Input faces:" << numInputFaces
             << "Input vertices:" << vertices.size() / 3
             << "Input triangles:" << triangles.size() / 3
             << "Generated hulls:" << hullCount
             << "Total hull volume:" << totalVolume
             << "Success:" << ok;
    reportDecompositionVolume("[CoACD]", originalVolume, totalVolume);
}
