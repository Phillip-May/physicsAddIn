#ifndef LIBRARYCATALOGUE_H
#define LIBRARYCATALOGUE_H

#include "CadNode.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace librarycatalogue {

enum class Category {
    Robot,
    LinearRail,
    Accessory,
    Tool
};

struct Entry {
    std::string id;
    std::string name;
    std::string path;
    Category category = Category::Robot;
    // Invalid packages remain visible with an error message.
    bool resolves = false;
    std::string error;
    std::shared_ptr<CadNode> root;
    std::string variantLabel;
    std::string variantId = "default";
    std::string assetKind;
    Json parameters = Json::object();
};

using Configure = std::function<void(CadNode* root, const Entry& entry)>;

// Presets are expanded into separate entries in stable order.
std::vector<Entry> scan(const std::string& catalogueRoot, const Configure& configure = {});

// Case-insensitive name/id search. Null only and empty search disable their filters.
bool matches(const Entry& entry, const Category* only, const std::string& search);

const char* categoryLabel(Category category);

} // namespace librarycatalogue

#endif // LIBRARYCATALOGUE_H
