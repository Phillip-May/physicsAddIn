#pragma once

#ifdef __EMSCRIPTEN__

#include <string>

// Browser file access for the WebAssembly build.
namespace WebFiles {

enum class Purpose {
    None = -1,
    Package = 0,
    Program = 1,
    Mastering = 2,
};

// Opens the browser's file picker. `accept` is an HTML accept list, e.g. ".zip,.json".
void requestOpen(Purpose purpose, const char* accept);

// Returns true once, when a requested file has finished loading, and fills in the path it
// was written to inside the Emscripten filesystem.
bool takeOpened(std::string* path, Purpose* purpose);

bool openPending();

// Writes contents out under a name the user chooses. Uses the File System Access API where
// it exists, which is a real native Save dialog, and falls back to an ordinary download
// elsewhere - Firefox and Safari do not implement it.
void saveText(const std::string& suggestedName, const std::string& contents);

} // namespace WebFiles

#endif // __EMSCRIPTEN__
