#pragma once

// WebGL lacks base-vertex draw calls, so large draw lists require 32-bit indices.
#define ImDrawIdx unsigned int
