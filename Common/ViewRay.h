#ifndef VIEWRAY_H
#define VIEWRAY_H

#include "CadNode.h"

#include <vector>

namespace viewray {

struct Camera {
    CadTransform pose;
    bool poseIsWorldFromCamera = true;
    bool looksDownNegativeZ = true;

    int widthPx = 0;
    int heightPx = 0;

    // Host view angle in degrees.
    double viewAngleDeg = 30.0;
    bool angleIsVertical = true;
};

struct Ray {
    CadVec3 origin;
    CadVec3 direction;
};

double focalLengthPx(const Camera& camera);

CadVec3 cameraEye(const Camera& camera);

// Pixel coordinates use a top-left origin.
Ray rayThroughPixel(const Camera& camera, double pixelX, double pixelY);

// Returns false for points behind the camera.
bool projectPoint(const Camera& camera, const CadVec3& world, double* pixelX, double* pixelY);

// Finds the nearest hit in a triangle soup containing nine floats per triangle.
bool rayHitsTriangles(const Ray& ray, const std::vector<float>& verticesMm, int triangles,
                      double* distanceMm);

struct MeshEdges {
    std::vector<float> endpointsMm;
    // Boundary edges repeat their single face normal.
    std::vector<float> faceNormals;
    int edges = 0;
};

MeshEdges buildMeshEdges(const std::vector<float>& verticesMm, int triangles);

// Writes six floats per line segment.
void silhouetteLines(const MeshEdges& edges, const CadVec3& eyeMm, std::vector<float>* linesMm);

} // namespace viewray

#endif // VIEWRAY_H
