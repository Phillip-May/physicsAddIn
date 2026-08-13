#include "CadNodePackage.h"

#include "StringUtil.h"

#include <filesystem>
#include <fstream>


#include <miniz.h>

#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr const char kMeshMagic[8] = {'P', 'A', 'M', 'E', 'S', 'H', '0', '1'};

std::string normalizePackagePath(std::string path);

// Reads zip packages, replacing Qt's private QZipReader.
class ZipArchiveReader {
public:
    ZipArchiveReader() { std::memset(&m_zip, 0, sizeof(m_zip)); }

    ~ZipArchiveReader() {
        if (m_open) mz_zip_reader_end(&m_zip);
    }

    ZipArchiveReader(const ZipArchiveReader&) = delete;
    ZipArchiveReader& operator=(const ZipArchiveReader&) = delete;

    bool open(const std::string& path) {
        if (m_open) return true;
        if (!mz_zip_reader_init_file(&m_zip, path.c_str(), 0)) return false;
        m_open = true;

        const mz_uint count = mz_zip_reader_get_num_files(&m_zip);
        m_entries.reserve(count);
        for (mz_uint i = 0; i < count; ++i) {
            mz_zip_archive_file_stat stat;
            if (!mz_zip_reader_file_stat(&m_zip, i, &stat)) continue;
            Entry entry;
            entry.path = normalizePackagePath(stat.m_filename);
            entry.isFile = !mz_zip_reader_is_file_a_directory(&m_zip, i);
            entry.index = i;
            // Exact-name lookup, matching QZipReader::fileData. miniz's own
            // mz_zip_reader_locate_file defaults to case-insensitive and rescans every call.
            m_indexByPath[entry.path] = i;
            m_entries.push_back(std::move(entry));
        }
        return true;
    }

    bool isOpen() const { return m_open; }

    struct Entry {
        std::string path;   // normalised, forward slashes
        bool isFile = false;
        mz_uint index = 0;
    };

    const std::vector<Entry>& entries() const { return m_entries; }

    // Empty and missing entries both come back false, as QZipReader::fileData returning an
    // empty std::string did for callers that checked it.
    bool readEntry(const std::string& normalizedPath, std::string& out) {
        const auto it = m_indexByPath.find(normalizedPath);
        if (it == m_indexByPath.end()) return false;
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&m_zip, it->second, &stat)) return false;
        if (stat.m_uncomp_size == 0) return false;
        out.resize(static_cast<size_t>(stat.m_uncomp_size));
        return mz_zip_reader_extract_to_mem(&m_zip, it->second, out.data(),
                                            static_cast<size_t>(out.size()), 0) != MZ_FALSE;
    }

private:
    mz_zip_archive m_zip;
    bool m_open = false;
    std::vector<Entry> m_entries;
    std::unordered_map<std::string, mz_uint> m_indexByPath;
};

class ZipArchiveWriter {
public:
    ZipArchiveWriter() { std::memset(&m_zip, 0, sizeof(m_zip)); }

    ~ZipArchiveWriter() {
        if (m_open) mz_zip_writer_end(&m_zip);
    }

    ZipArchiveWriter(const ZipArchiveWriter&) = delete;
    ZipArchiveWriter& operator=(const ZipArchiveWriter&) = delete;

    bool open(const std::string& path) {
        if (!mz_zip_writer_init_file(&m_zip, path.c_str(), 0)) return false;
        m_open = true;
        return true;
    }

    bool addFile(const std::string& name, const std::string& data) {
        if (!m_open) return false;
        // MZ_DEFAULT_LEVEL is deflate level 6, which is what Qt's writer used.
        return mz_zip_writer_add_mem(&m_zip, name.c_str(), data.data(),
                                     static_cast<size_t>(data.size()), MZ_DEFAULT_LEVEL) != MZ_FALSE;
    }

    bool finish() {
        if (!m_open) return false;
        const bool finalized = mz_zip_writer_finalize_archive(&m_zip) != MZ_FALSE;
        const bool ended = mz_zip_writer_end(&m_zip) != MZ_FALSE;
        m_open = false;
        return finalized && ended;
    }

private:
    mz_zip_archive m_zip;
    bool m_open = false;
};

struct CadNodePackageContext {
    std::string packageFile;
    bool isZip = false;
    std::string jsonBasePath;
    // Held open across the whole load so mesh entries do not each reopen the archive.
    std::shared_ptr<ZipArchiveReader> archive;
};

// Was QDir::cleanPath plus QString methods. Collapses backslashes and repeated slashes and
// strips leading "./" and "/", so archive entry names and meshSource values normalise the same
// way they did before and still compare equal.
std::string normalizePackagePath(std::string path) {  // forward declared above for ZipArchiveReader
    for (char& c : path) {
        if (c == '\\') c = '/';
    }
    std::string collapsed;
    collapsed.reserve(path.size());
    for (char c : path) {
        if (c == '/' && !collapsed.empty() && collapsed.back() == '/') continue;
        collapsed.push_back(c);
    }
    path = std::move(collapsed);
    while (strutil::startsWith(path, "./")) path.erase(0, 2);
    while (!path.empty() && path.front() == '/') path.erase(0, 1);
    while (!path.empty() && path.back() == '/') path.pop_back();
    return path == "." ? std::string() : path;
}

std::string packageRelativePath(const std::string& basePath, const std::string& relativePath) {
    const std::string normalizedRelative = normalizePackagePath(relativePath);
    if (basePath.empty()) return normalizedRelative;
    return normalizePackagePath(basePath + "/" + normalizedRelative);
}

bool readPackageEntry(const CadNodePackageContext& context, const std::string& relativePath, std::string& outData, std::string* errorMessage) {
    const std::string entryPath = packageRelativePath(context.jsonBasePath, relativePath);
    if (context.isZip) {
        if (!context.archive || !context.archive->isOpen()) {
            if (errorMessage) *errorMessage = "Failed to open zip package: " + context.packageFile;
            return false;
        }
        if (!context.archive->readEntry(entryPath, outData) || outData.empty()) {
            if (errorMessage) *errorMessage = "Missing or empty package entry: " + entryPath;
            return false;
        }
        return true;
    }

    const std::filesystem::path assetPath =
        std::filesystem::path(context.jsonBasePath) / std::filesystem::path(relativePath);
    std::ifstream file(assetPath, std::ios::binary);
    if (!file) {
        if (errorMessage) *errorMessage = "Failed to open package asset: " + assetPath.string();
        return false;
    }
    outData.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return true;
}

bool loadMeshGeometryBlob(const std::string& bytes, MeshGeometryData& meshData, std::string* errorMessage) {
    if (bytes.size() < 8 + 8 + 24) {
        if (errorMessage) *errorMessage = "Mesh blob is too small.";
        return false;
    }

    const unsigned char* cursor = reinterpret_cast<const unsigned char*>(bytes.data());
    const unsigned char* end = cursor + bytes.size();

    auto readUInt32 = [&cursor, end](uint32_t& value) -> bool {
        if (end - cursor < 4) return false;
        value = static_cast<uint32_t>(cursor[0]) |
                (static_cast<uint32_t>(cursor[1]) << 8) |
                (static_cast<uint32_t>(cursor[2]) << 16) |
                (static_cast<uint32_t>(cursor[3]) << 24);
        cursor += 4;
        return true;
    };

    auto readFloat32 = [&cursor, end, &readUInt32](float& value) -> bool {
        uint32_t bits = 0;
        if (!readUInt32(bits)) return false;
        static_assert(sizeof(float) == sizeof(uint32_t), "Expected 32-bit float");
        std::memcpy(&value, &bits, sizeof(float));
        return true;
    };

    if (end - cursor < 8 || std::memcmp(cursor, kMeshMagic, 8) != 0) {
        if (errorMessage) *errorMessage = "Mesh blob has an unsupported format.";
        return false;
    }
    cursor += 8;

    uint32_t vertexCount = 0;
    uint32_t indexCount = 0;
    if (!readUInt32(vertexCount) || !readUInt32(indexCount)) {
        if (errorMessage) *errorMessage = "Mesh blob has a truncated header.";
        return false;
    }
    if (vertexCount == 0 || indexCount == 0 || (indexCount % 3) != 0) {
        if (errorMessage) *errorMessage = "Mesh blob has invalid vertex or index counts.";
        return false;
    }

    const uint64_t expectedSize = 8ull + 8ull + 24ull +
        static_cast<uint64_t>(vertexCount) * 3ull * sizeof(float) +
        static_cast<uint64_t>(indexCount) * sizeof(uint32_t);
    if (static_cast<uint64_t>(bytes.size()) != expectedSize) {
        if (errorMessage) *errorMessage = strutil::format("Mesh blob size mismatch. Expected %1 bytes, got %2 bytes.")
            .arg(expectedSize)
            .arg(bytes.size());
        return false;
    }

    for (float& value : meshData.bounds) {
        if (!readFloat32(value)) {
            if (errorMessage) *errorMessage = "Mesh blob has truncated bounds.";
            return false;
        }
    }

    meshData.vertices.resize(static_cast<size_t>(vertexCount) * 3);
    for (float& value : meshData.vertices) {
        if (!readFloat32(value)) {
            if (errorMessage) *errorMessage = "Mesh blob has truncated vertex data.";
            return false;
        }
    }

    meshData.indices.resize(static_cast<size_t>(indexCount));
    for (uint32_t& value : meshData.indices) {
        uint32_t index = 0;
        if (!readUInt32(index)) {
            if (errorMessage) *errorMessage = "Mesh blob has truncated index data.";
            return false;
        }
        if (index >= vertexCount) {
            if (errorMessage) *errorMessage = "Mesh blob contains an out-of-range index.";
            return false;
        }
        value = index;
    }

    meshData.loaded = true;
    return true;
}

bool loadMeshGeometryRecursive(CadNode* node, const CadNodePackageContext& context, std::string* errorMessage) {
    if (!node) return true;
    if (node->type == CadNodeType::MeshGeometry) {
        MeshGeometryData* meshData = node->asMeshGeometry();
        if (meshData && !meshData->meshSource.empty()) {
            std::string meshBytes;
            if (!readPackageEntry(context, meshData->meshSource, meshBytes, errorMessage)) return false;
            if (!loadMeshGeometryBlob(meshBytes, *meshData, errorMessage)) return false;
        }
    }
    for (auto& child : node->children) {
        if (!loadMeshGeometryRecursive(child.get(), context, errorMessage)) return false;
    }
    return true;
}

bool readPackageJson(const std::string& packageFile, std::string& jsonBytes, CadNodePackageContext& context, std::string* errorMessage) {
    const std::filesystem::path packagePath(packageFile);
    context.packageFile = packageFile;
    context.isZip = strutil::equalsCaseInsensitive(packagePath.extension().string(), ".zip");

    if (!context.isZip) {
        std::ifstream file(packagePath, std::ios::binary);
        if (!file) {
            if (errorMessage) *errorMessage = "Failed to open file: " + packageFile;
            return false;
        }
        jsonBytes.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        context.jsonBasePath = std::filesystem::absolute(packagePath).parent_path().string();
        return true;
    }

    // Opened once and kept on the context, so the mesh reads below reuse it.
    context.archive = std::make_shared<ZipArchiveReader>();
    if (!context.archive->open(packageFile)) {
        if (errorMessage) *errorMessage = "Failed to open zip package: " + packageFile;
        return false;
    }

    std::string jsonEntry;
    bool foundPreferredRobotJson = false;
    int jsonCount = 0;
    for (const ZipArchiveReader::Entry& entry : context.archive->entries()) {
        if (!entry.isFile) continue;
        if (strutil::equalsCaseInsensitive(std::filesystem::path(entry.path).filename().string(), "robot.json")) {
            jsonEntry = entry.path;
            foundPreferredRobotJson = true;
            break;
        }
        if (strutil::equalsCaseInsensitive(std::filesystem::path(entry.path).extension().string(), ".json")) {
            jsonEntry = entry.path;
            ++jsonCount;
        }
    }

    if (jsonEntry.empty() || (!foundPreferredRobotJson && jsonCount > 1)) {
        if (errorMessage) *errorMessage = "Zip package must contain robot.json or exactly one JSON file.";
        return false;
    }

    if (!context.archive->readEntry(jsonEntry, jsonBytes) || jsonBytes.empty()) {
        if (errorMessage) *errorMessage = "Package JSON is missing or empty: " + jsonEntry;
        return false;
    }

    const std::string jsonParent = std::filesystem::path(jsonEntry).parent_path().generic_string();
    context.jsonBasePath = jsonParent == "." ? std::string() : normalizePackagePath(jsonParent);
    return true;
}

void setParentPointersRecursive(CadNode* node, CadNode* parent = nullptr) {
    if (!node) return;
    node->parent = parent;
    for (auto& child : node->children) {
        setParentPointersRecursive(child.get(), node);
    }
}

} // namespace

std::shared_ptr<CadNode> loadCadNodePackage(const std::string& packageFile, std::string* errorMessage)
{
    std::string jsonBytes;
    CadNodePackageContext context;
    if (!readPackageJson(packageFile, jsonBytes, context, errorMessage)) return nullptr;

    // Parsing without exceptions: a malformed document comes back discarded, which mirrors how
    // QJsonDocument::fromJson reported failure through its error argument.
    const Json document = Json::parse(jsonBytes.data(),
                                      jsonBytes.data() + jsonBytes.size(),
                                      nullptr, /*allow_exceptions=*/false);
    if (document.is_discarded() || !document.is_object()) {
        if (errorMessage) *errorMessage = "Invalid package JSON.";
        return nullptr;
    }

    std::shared_ptr<CadNode> root = CadNode::fromJson(document);
    if (!root) {
        if (errorMessage) *errorMessage = "Package JSON did not produce a node tree.";
        return nullptr;
    }

    setParentPointersRecursive(root.get());
    if (!loadMeshGeometryRecursive(root.get(), context, errorMessage)) return nullptr;
    CadNode::resolveRobotReferences(root.get());
    return root;
}

bool cadPackageIsZip(const std::string& packageFile)
{
    return strutil::equalsCaseInsensitive(std::filesystem::path(packageFile).extension().string(), ".zip");
}

bool readCadPackageEntry(const std::string& packageFile,
                         const std::string& entryPath,
                         std::string* bytes,
                         std::string* errorMessage)
{
    if (!bytes) return false;
    bytes->clear();
    ZipArchiveReader archive;
    if (!archive.open(packageFile)) {
        if (errorMessage) *errorMessage = "Failed to open zip package: " + packageFile;
        return false;
    }
    if (!archive.readEntry(normalizePackagePath(entryPath), *bytes) || bytes->empty()) {
        if (errorMessage) *errorMessage = "Entry is missing or empty: " + entryPath;
        return false;
    }
    return true;
}

std::vector<std::string> listCadPackageEntries(const std::string& packageFile, std::string* errorMessage)
{
    std::vector<std::string> paths;
    ZipArchiveReader archive;
    if (!archive.open(packageFile)) {
        if (errorMessage) *errorMessage = "Failed to open zip package: " + packageFile;
        return paths;
    }
    for (const ZipArchiveReader::Entry& entry : archive.entries()) {
        if (entry.isFile) paths.push_back(entry.path);
    }
    return paths;
}

bool writeCadPackageArchive(const std::string& packageFile,
                            const std::vector<std::pair<std::string, std::string>>& entries,
                            std::string* errorMessage)
{
    const std::string tempPath = packageFile + ".tmp";
    std::error_code ignored;
    std::filesystem::remove(tempPath, ignored);
    {
        ZipArchiveWriter zipWriter;
        if (!zipWriter.open(tempPath)) {
            if (errorMessage) *errorMessage = "Failed to create archive: " + tempPath;
            return false;
        }
        for (const auto& entry : entries) {
            if (!zipWriter.addFile(normalizePackagePath(entry.first), entry.second)) {
                std::filesystem::remove(tempPath, ignored);
                if (errorMessage) *errorMessage = "Failed to write archive entry: " + entry.first;
                return false;
            }
        }
        // finish(), not the destructor: that only releases the writer, while this writes the
        // central directory. Without it the file is the right size and full of the right bytes, and
        // no zip reader on earth will open it.
        if (!zipWriter.finish()) {
            std::filesystem::remove(tempPath, ignored);
            if (errorMessage) *errorMessage = "Failed to finalise the archive: " + tempPath;
            return false;
        }
    }
    std::error_code renameError;
    std::filesystem::rename(tempPath, packageFile, renameError);
    if (renameError) {
        // Rename across devices fails; copying then removing is the fallback the temporary was
        // there to make safe in the first place.
        std::filesystem::copy_file(tempPath, packageFile,
                                   std::filesystem::copy_options::overwrite_existing, renameError);
        std::filesystem::remove(tempPath, ignored);
        if (renameError) {
            if (errorMessage) *errorMessage = "Failed to move the archive into place: " + packageFile;
            return false;
        }
    }
    return true;
}

bool saveCadNodePackage(const std::string& packageFile, const CadNode& root, const std::string& sourcePackageFile, std::string* errorMessage)
{
    // dump(4) matches the four-space indent QJsonDocument::Indented produced. Numbers are
    // spelled differently - nlohmann emits the shortest round-tripping form - so a re-saved
    // robot.json differs textually from one Qt wrote while parsing to the same values.
    const std::string jsonBytes = root.toJson().dump(4);
    const bool outputIsZip =
        strutil::equalsCaseInsensitive(std::filesystem::path(packageFile).extension().string(), ".zip");

    if (!outputIsZip) {
        std::ofstream file(packageFile, std::ios::binary | std::ios::trunc);
        if (!file) {
            if (errorMessage) *errorMessage = "Failed to write package JSON: " + packageFile;
            return false;
        }
        file.write(jsonBytes.data(), static_cast<std::streamsize>(jsonBytes.size()));
        return true;
    }

    const std::string tempPath = packageFile + ".tmp";
    std::filesystem::remove(tempPath);
    ZipArchiveWriter zipWriter;
    if (!zipWriter.open(tempPath)) {
        if (errorMessage) *errorMessage = "Failed to create zip package: " + tempPath;
        return false;
    }
    if (!zipWriter.addFile("robot.json", jsonBytes)) {
        std::filesystem::remove(tempPath);
        if (errorMessage) *errorMessage = "Failed to write package JSON into: " + tempPath;
        return false;
    }

    if (!sourcePackageFile.empty()) {
        if (strutil::equalsCaseInsensitive(
                std::filesystem::path(sourcePackageFile).extension().string(), ".zip")) {
            ZipArchiveReader zipReader;
            if (!zipReader.open(sourcePackageFile)) {
                if (errorMessage) *errorMessage = "Failed to open source zip package: " + sourcePackageFile;
                return false;
            }
            for (const ZipArchiveReader::Entry& entry : zipReader.entries()) {
                if (!entry.isFile) continue;
                if (strutil::equalsCaseInsensitive(std::filesystem::path(entry.path).extension().string(), ".json")) continue;
                std::string data;
                if (!zipReader.readEntry(entry.path, data)) continue;
                zipWriter.addFile(entry.path, data);
            }
        } else {
            // Directory-based source package: copy the sibling assets in, rebuilding their
            // relative paths as zip entry names. Was QDir::entryInfoList; the traversal order
            // and the two-level split of files then directories are kept as they were.
            const std::filesystem::path sourcePath(sourcePackageFile);
            const std::filesystem::path baseDir = std::filesystem::absolute(sourcePath).parent_path();
            const std::string jsonName = sourcePath.filename().string();

            const auto readWholeFile = [](const std::filesystem::path& path, std::string& out) {
                std::ifstream file(path, std::ios::binary);
                if (!file) return false;
                out.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
                return true;
            };

            std::error_code ec;
            for (const auto& entry : std::filesystem::directory_iterator(baseDir, ec)) {
                if (!entry.is_regular_file()) continue;
                const std::string name = entry.path().filename().string();
                if (strutil::equalsCaseInsensitive(name, jsonName)) continue;
                std::string data;
                if (readWholeFile(entry.path(), data)) zipWriter.addFile(name, data);
            }

            std::function<void(const std::filesystem::path&, const std::string&)> addDir =
                [&](const std::filesystem::path& dir, const std::string& relBase) {
                    std::error_code inner;
                    for (const auto& entry : std::filesystem::directory_iterator(dir, inner)) {
                        if (!entry.is_regular_file()) continue;
                        std::string data;
                        if (readWholeFile(entry.path(), data)) {
                            zipWriter.addFile(
                                normalizePackagePath(relBase + "/" + entry.path().filename().string()),
                                data);
                        }
                    }
                    for (const auto& entry : std::filesystem::directory_iterator(dir, inner)) {
                        if (!entry.is_directory()) continue;
                        addDir(entry.path(),
                               normalizePackagePath(relBase + "/" + entry.path().filename().string()));
                    }
                };
            for (const auto& entry : std::filesystem::directory_iterator(baseDir, ec)) {
                if (!entry.is_directory()) continue;
                addDir(entry.path(), entry.path().filename().string());
            }
        }
    }

    if (!zipWriter.finish()) {
        std::filesystem::remove(tempPath);
        if (errorMessage) *errorMessage = "Failed to write zip package: " + packageFile;
        return false;
    }

    std::error_code renameError;
    std::filesystem::remove(packageFile, renameError);
    std::filesystem::rename(tempPath, packageFile, renameError);
    if (renameError) {
        std::filesystem::remove(tempPath, renameError);
        if (errorMessage) *errorMessage = "Failed to replace output package: " + packageFile;
        return false;
    }
    return true;
}
