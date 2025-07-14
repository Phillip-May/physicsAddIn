#ifndef MESHLOADER_H
#define MESHLOADER_H

#include <vector>
#include <string>
#include <memory>
#include <QString>
#include "PxPhysicsAPI.h"

class Item;

/**
 * @brief Handles loading and processing of mesh files (STL, etc.)
 * 
 * This class provides functionality for loading mesh files from disk,
 * converting them to PhysX-compatible formats, and managing mesh data.
 */
class MeshLoader
{
public:
    // Mesh data structure
    struct MeshData {
        std::vector<PxVec3> vertices;
        std::vector<PxU32> indices;
        PxBounds3 bounds;
        bool isValid;
        
        MeshData() : isValid(false) {}
    };

    // Loading options
    struct LoadOptions {
        bool weldVertices = true;
        float weldTolerance = 0.05f;
        bool optimizeMesh = true;
        bool generateNormals = false;
        
        LoadOptions() = default;
    };

public:
    MeshLoader();
    ~MeshLoader();

    // Main loading interface
    MeshData loadFromFile(const QString& filePath, const LoadOptions& options = LoadOptions());
    MeshData loadFromRoboDKItem(Item item, const LoadOptions& options = LoadOptions());
    
    // Mesh processing
    MeshData processMesh(const MeshData& input, const LoadOptions& options);
    bool validateMesh(const MeshData& mesh);
    
    // Mesh optimization
    MeshData createConvexHull(const MeshData& input);
    MeshData simplifyMesh(const MeshData& input, float tolerance);
    MeshData weldVertices(const MeshData& input, float tolerance);
    
    // Debug and export
    bool saveMeshToFile(const MeshData& mesh, const QString& filePath);
    void exportDebugInfo(const MeshData& mesh, const QString& filePath);
    
    // Utility functions
    PxBounds3 calculateBounds(const std::vector<PxVec3>& vertices);
    float calculateVolume(const MeshData& mesh);
    bool isConvex(const MeshData& mesh);

private:
    // STL file loading
    bool loadBinarySTL(const QString& filePath, MeshData& mesh);
    bool loadAsciiSTL(const QString& filePath, MeshData& mesh);
    bool detectSTLFormat(const QString& filePath);
    
    // Mesh processing helpers
    void removeDuplicateVertices(MeshData& mesh);
    void optimizeIndices(MeshData& mesh);
    void calculateNormals(MeshData& mesh);
    
    // Validation helpers
    bool validateVertices(const std::vector<PxVec3>& vertices);
    bool validateIndices(const std::vector<PxU32>& indices, size_t vertexCount);
    bool checkMeshConsistency(const MeshData& mesh);
    
    // Debug helpers
    void logMeshInfo(const MeshData& mesh, const QString& context);
    void exportToOBJ(const MeshData& mesh, const QString& filePath);
    void exportToPLY(const MeshData& mesh, const QString& filePath);
    
    // Constants
    static constexpr float DEFAULT_WELD_TOLERANCE = 0.05f;
    static constexpr float MIN_VERTEX_DISTANCE = 0.001f;
    static constexpr size_t MAX_VERTICES = 1000000; // Safety limit
    static constexpr size_t MAX_INDICES = 3000000;  // Safety limit
};

#endif // MESHLOADER_H 