cmake_minimum_required (VERSION 3.6)

project (recastnavigation-js)

option(BUILD_WASM "Build WASM." on)

set(CMAKE_VERBOSE_MAKEFILE on)

file(GLOB_RECURSE wrapperSourceFiles
    "src/wrapper/Detour/*.h"
    "src/wrapper/Detour/*.cpp"
)
file(GLOB_RECURSE recastNavigationDetourSourceFiles
    "recastnavigation/Detour/*.h"
    "recastnavigation/Detour/*.cpp"

    "recastnavigation/DetourCrowd/*.h"
    "recastnavigation/DetourCrowd/*.cpp"
)

set (emccLinkerFlags "\
    -O0 \
    --memory-init-file 0 \
    --bind \
    -s MODULARIZE=1 \
    -s EXPORT_ES6=1 \
    --llvm-lto 1 \
    --g3 \
    -s ASSERTIONS=1 \
    ")
if (BUILD_WASM)
else ()
    set (emccLinkerFlags "${emccLinkerFlags} -s WASM=0")
endif ()

add_executable (recastnavigation-js ${wrapperSourceFiles} ${recastNavigationDetourSourceFiles})
set_target_properties (recastnavigation-js PROPERTIES CXX_STANDARD 17)
set_target_properties (recastnavigation-js PROPERTIES SUFFIX ".mjs")
set_target_properties (recastnavigation-js PROPERTIES LINK_FLAGS ${emccLinkerFlags})
target_include_directories (recastnavigation-js PRIVATE
    "src"
    "recastnavigation/Detour/Include"
    "recastnavigation/DetourCrowd/Include"
    "recastnavigation/DetourTileCache/Include"
)

# file(GLOB_RECURSE recastNavigationRecastSourceFiles
#     "recastnavigation/Recast/*.h"
#     "recastnavigation/Recast/*.cpp"
# )

# add_executable(recastnavigation-js.recast ${wrapperSourceFiles} ${recastNavigationRecastSourceFiles})
# set_target_properties (recastnavigation-js.recast PROPERTIES CXX_STANDARD 17)
# set_target_properties (recastnavigation-js.recast PROPERTIES SUFFIX ".mjs")
# set_target_properties (recastnavigation-js.recast PROPERTIES LINK_FLAGS ${emccLinkerFlags})
# target_include_directories (recastnavigation-js.recast PRIVATE
#     "src"
#     "recastnavigation/Recast/Include"
# )

