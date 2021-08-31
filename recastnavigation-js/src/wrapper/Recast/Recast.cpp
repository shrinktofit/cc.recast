
#include "../../../recastnavigation/Recast/Include/Recast.h"
#include <emscripten/bind.h>
#include <memory>

using rcCreateHeightfield_t = bool(
    rcContext& ctx,
    rcHeightfield& hf,
    int width,
    int height,
    const float* bmin,
    const float* bmax,
    float cs,
    float ch
    );

EMSCRIPTEN_BINDINGS(recast) {
    emscripten::class_<rcContext>("rcContext")
        .constructor()
        ;

    emscripten::class_<rcHeightfield>("rcHeightfield")
        .constructor()
        ;

    emscripten::function("rcCreateHeightfield", emscripten::select_overload<rcCreateHeightfield_t>([](rcContext& ctx, rcHeightfield& hf, int width, int height,
						 const float* bmin, const float* bmax,
						 float cs, float ch) {
        return rcCreateHeightfield(&ctx, hf, width, height, bmin, bmax, cs, ch);
        }))
        ;
}

