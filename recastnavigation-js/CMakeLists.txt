cmake_minimum_required (VERSION 3.6)

project (recastnavigation-js)

option(BUILD_WASM "Build WASM." on)

set(CMAKE_VERBOSE_MAKEFILE on)

file(GLOB_RECURSE wrapperSourceFiles
    "src/wrapper/*.h"
    "src/wrapper/*.cpp"
)
file(GLOB_RECURSE recastNavigationSourceFiles
    "recastnavigation/Detour/*.h"
    "recastnavigation/Detour/*.cpp"
    "recastnavigation/Recast/*.h"
    "recastnavigation/Recast/*.cpp"
)

set (emccLinkerFlags "\
    -O3 \
    --memory-init-file 0 \
    --bind \
    -s MODULARIZE=1 \
    -s EXPORT_ES6=1 \
    --llvm-lto 1 \
    --g3 \
    ")
if (BUILD_WASM)
else ()
    set (emccLinkerFlags "${emccLinkerFlags} -s WASM=0")
endif ()

add_executable (recastnavigation-js ${wrapperSourceFiles} ${recastNavigationSourceFiles})
set_target_properties (recastnavigation-js PROPERTIES CXX_STANDARD 17)
set_target_properties (recastnavigation-js PROPERTIES SUFFIX ".mjs")
set_target_properties (recastnavigation-js PROPERTIES LINK_FLAGS ${emccLinkerFlags})
target_include_directories (recastnavigation-js PRIVATE
    "src"
    "recastnavigation/Detour/Include"
    "recastnavigation/DetourCrowd/Include"
    "recastnavigation/DetourTileCache/Include"
    "recastnavigation/Recast/Include"
)

