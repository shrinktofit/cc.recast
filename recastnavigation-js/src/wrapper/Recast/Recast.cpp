
#include "../../../recastnavigation/Recast/Include/Recast.h"
#include "../Common/Vec3.h"
#include <emscripten/bind.h>
#include <memory>

namespace wasm_port {
bool createHeightField(
    rcContext& ctx,
    rcHeightfield& hf,
    int width,
    int height,
    const Point3 &bmin,
    const Point3 &bmax,
    float cs,
    float ch
) {
    return rcCreateHeightfield(
        &ctx,
        hf,
        width,
        height,
        bmin.data(),
        bmax.data(),
        cs,
        ch
    );
}
}

EMSCRIPTEN_BINDINGS(recast) {
    emscripten::class_<rcContext>("rcContext")
        .constructor()
        ;

    emscripten::class_<rcHeightfield>("rcHeightfield")
        .constructor()
        ;

    emscripten::function("rcCreateHeightfield", wasm_port::createHeightField)
        ;
}

