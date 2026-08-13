# Vendored third-party sources

These are committed, unlike the dependencies under `external/` (imgui, implot, glfw, CoACD), which
are gitignored and cloned per BUILDING.md — `scripts/bootstrap_deps.ps1` fetches only CoACD.

The difference is deliberate. These are single-file libraries, so committing them costs little and
pins the exact version that was reviewed; the `external/` dependencies still need their one-time
clone step.

| Library | Version | Licence | Used for |
|---|---|---|---|
| [nlohmann/json](https://github.com/nlohmann/json) | 3.12.0 | MIT | replaces `QJsonObject` / `QJsonArray` / `QJsonDocument` |
| [miniz](https://github.com/richgel999/miniz) | 3.1.2 | MIT | replaces `QZipReader` / `QZipWriter` (Qt **private** headers) |
| [V-HACD](https://github.com/kmammou/v-hacd) | 4.1.0 | BSD-3-Clause | convex decomposition, `Common/CadDecomposition.cpp` |

nlohmann and miniz are both part of removing Qt from RobotSimulator.

## Provenance

```
third_party/nlohmann/json.hpp
    https://github.com/nlohmann/json/releases/download/v3.12.0/json.hpp
third_party/nlohmann/LICENSE.MIT
    https://raw.githubusercontent.com/nlohmann/json/v3.12.0/LICENSE.MIT

third_party/miniz/{miniz.c,miniz.h,LICENSE}
    https://github.com/richgel999/miniz/releases/download/3.1.2/miniz-3.1.2.zip

third_party/vhacd/{VHACD.h,LICENSE}
    https://github.com/kmammou/v-hacd/tree/v4.1.0
```

The miniz release ships pre-amalgamated `miniz.c` and `miniz.h`; only those and the licence are
kept. Its internal `MZ_VERSION` reads `"11.3.2"` even in the 3.1.2 release, which is an upstream
versioning quirk rather than a mismatched download.

V-HACD is header-only. `Common/CadDecomposition.cpp` is the one translation unit that defines
`ENABLE_VHACD_IMPLEMENTATION`. The upstream repository also ships a test app, demo meshes and MSVC
project files; none of it is used here and none of it is kept.

All three are unmodified. Configuration is applied through `build/thirdparty.pri` rather than by
editing the sources, so a version bump is a straight file replacement.

## Why miniz in particular

`Common/CadNodePackage.cpp` used `<private/qzipreader_p.h>`, a Qt **private** header. Private headers
tie the build to one exact Qt build, which is why qmake printed "This project is using private
headers and will therefore be tied to this specific Qt module build version" on every run. Dropping
it removes the `gui-private` module and that warning with it.
