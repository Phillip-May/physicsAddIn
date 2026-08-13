#include "WebFiles.h"

#ifdef __EMSCRIPTEN__

#include <emscripten.h>

#include <cstring>

namespace {

char g_pendingPath[512] = {0};
int g_pendingPurpose = static_cast<int>(WebFiles::Purpose::None);
bool g_openPending = false;

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE char* robotSimFilePathBuffer() { return g_pendingPath; }

EMSCRIPTEN_KEEPALIVE int robotSimFilePathCapacity() { return static_cast<int>(sizeof(g_pendingPath)); }

// Called from JS once the bytes are in the Emscripten filesystem, or with Purpose::None if
// the user dismissed the picker.
EMSCRIPTEN_KEEPALIVE void robotSimFileOpened(int purpose) {
    g_pendingPurpose = purpose;
    g_openPending = false;
}

} // extern "C"

namespace WebFiles {

void requestOpen(Purpose purpose, const char* accept) {
    g_openPending = true;
    g_pendingPurpose = static_cast<int>(Purpose::None);

    EM_ASM({
        var purpose = $0;
        var accept = UTF8ToString($1);

        var input = document.createElement('input');
        input.type = 'file';
        input.accept = accept;
        input.style.display = 'none';

        var settled = false;
        // 'cancel' is not fired by every browser, so a window focus that arrives with no
        // selection is treated as a dismissal. Without this the UI would claim a picker was
        // still open forever.
        var finish = function (ok) {
            if (settled) return;
            settled = true;
            if (input.parentNode) input.parentNode.removeChild(input);
            window.removeEventListener('focus', onFocus, true);
            if (!ok) Module._robotSimFileOpened(-1);
        };
        var onFocus = function () {
            setTimeout(function () { if (!input.files || input.files.length === 0) finish(false); }, 800);
        };

        input.addEventListener('change', function () {
            if (!input.files || input.files.length === 0) { finish(false); return; }
            var file = input.files[0];
            var reader = new FileReader();
            reader.onload = function () {
                try { FS.mkdir('/uploads'); } catch (e) { /* already there */ }
                // The name is preserved because loaders dispatch on the extension.
                var path = '/uploads/' + file.name.replace(/[^A-Za-z0-9._-]/g, '_');
                FS.writeFile(path, new Uint8Array(reader.result));

                var buffer = Module._robotSimFilePathBuffer();
                var capacity = Module._robotSimFilePathCapacity();
                var bytes = new TextEncoder().encode(path);
                var count = Math.min(bytes.length, capacity - 1);
                for (var i = 0; i < count; ++i) Module.HEAPU8[buffer + i] = bytes[i];
                Module.HEAPU8[buffer + count] = 0;

                finish(true);
                Module._robotSimFileOpened(purpose);
            };
            reader.onerror = function () { finish(false); };
            reader.readAsArrayBuffer(file);
        });

        window.addEventListener('focus', onFocus, true);
        document.body.appendChild(input);
        input.click();
    }, static_cast<int>(purpose), accept);
    // clang-format on
}

bool takeOpened(std::string* path, Purpose* purpose) {
    if (g_pendingPurpose == static_cast<int>(Purpose::None)) return false;
    if (path) *path = g_pendingPath;
    if (purpose) *purpose = static_cast<Purpose>(g_pendingPurpose);
    g_pendingPurpose = static_cast<int>(Purpose::None);
    g_pendingPath[0] = '\0';
    return true;
}

bool openPending() { return g_openPending; }

void saveText(const std::string& suggestedName, const std::string& contents) {
    // clang-format off
    EM_ASM({
        var name = UTF8ToString($0);
        var text = UTF8ToString($1);
        var blob = new Blob([text], {type: 'text/plain'});

        var download = function () {
            var url = URL.createObjectURL(blob);
            var link = document.createElement('a');
            link.href = url;
            link.download = name;
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            setTimeout(function () { URL.revokeObjectURL(url); }, 1000);
        };

        // Chromium exposes a real Save As dialog; everything else gets the download.
        if (window.showSaveFilePicker) {
            window.showSaveFilePicker({
                suggestedName: name,
                types: [{description: 'Robot Program Text', accept: {'text/plain': ['.txt']}}]
            }).then(function (handle) {
                return handle.createWritable().then(function (writable) {
                    return writable.write(blob).then(function () { return writable.close(); });
                });
            }).catch(function (error) {
                // AbortError means the user cancelled, which is not a failure to fall back on.
                if (error && error.name !== 'AbortError') download();
            });
        } else {
            download();
        }
    }, suggestedName.c_str(), contents.c_str());
    // clang-format on
}

} // namespace WebFiles

#endif // __EMSCRIPTEN__
