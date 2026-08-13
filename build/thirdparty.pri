# nlohmann/json and miniz, the Qt-free replacements for QJson* and QZipReader/QZipWriter.
#
# Unlike build/imgui.pri and build/deps.pri, this file needs no existence checks and no
# clone step: third_party/ is committed. See third_party/README.md for provenance.

# Guarded: RobotSimulator includes this directly, the Qt targets get it through
# Common/Common.pri, and a target that ever pulled in both would otherwise compile miniz.c
# twice and fail to link on duplicate symbols.
!isEmpty(THIRDPARTY_PRI_INCLUDED): return()
THIRDPARTY_PRI_INCLUDED = 1

THIRDPARTY_ROOT = $$clean_path($$PWD/../third_party)

# Included as <nlohmann/json.hpp> and <miniz.h>.
INCLUDEPATH += $${THIRDPARTY_ROOT} $${THIRDPARTY_ROOT}/miniz

SOURCES += $${THIRDPARTY_ROOT}/miniz/miniz.c

# miniz exports zlib-compatible names - compress, uncompress, inflate, deflate and friends -
# unless this is set. Qt statically links its own zlib for libpng and QZip, so leaving them
# exported risks duplicate symbols at link time. Only the mz_zip_* archive API is used here,
# which this does not touch.
DEFINES += MINIZ_NO_ZLIB_COMPATIBLE_NAMES
