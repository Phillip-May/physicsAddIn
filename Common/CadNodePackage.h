#pragma once

#include "CadNode.h"

#include <memory>

#include <string>
#include <utility>
#include <vector>

std::shared_ptr<CadNode> loadCadNodePackage(const std::string& packageFile, std::string* errorMessage = nullptr);

bool cadPackageIsZip(const std::string& packageFile);
bool readCadPackageEntry(const std::string& packageFile,
                         const std::string& entryPath,
                         std::string* bytes,
                         std::string* errorMessage = nullptr);
// Entry paths in the archive, normalised the same way readCadPackageEntry expects them.
std::vector<std::string> listCadPackageEntries(const std::string& packageFile,
                                               std::string* errorMessage = nullptr);
bool writeCadPackageArchive(const std::string& packageFile,
                            const std::vector<std::pair<std::string, std::string>>& entries,
                            std::string* errorMessage = nullptr);
bool saveCadNodePackage(const std::string& packageFile,
                        const CadNode& root,
                        const std::string& sourcePackageFile = std::string(),
                        std::string* errorMessage = nullptr);
