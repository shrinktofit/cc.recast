
#include "../../../recastnavigation/Recast/Include/Recast.h"
#include "../Common/Vec3.h"
#include <emscripten/bind.h>
#include <memory>

namespace wasm_port {
    template <typename T>
    struct proxy_trait { };

    template <>
    struct proxy_trait<const float *> { };

    using const_float3_ptr_proxy = const Point3&;

    const float* unwrap(const_float3_ptr_proxy proxy) {
        return proxy.data();
    }

bool createHeightField(
    rcContext& ctx,
    rcHeightfield& hf,
    int width,
    int height,
    const_float3_ptr_proxy bmin,
    const_float3_ptr_proxy bmax,
    float cs,
    float ch
) {
    return rcCreateHeightfield(
        &ctx,
        hf,
        width,
        height,
        unwrap(bmin),
        unwrap(bmax),
        cs,
        ch
    );
}

bool rasterizeTriangle(
    rcContext& ctx,
    const_float3_ptr_proxy v0,
    const_float3_ptr_proxy v1,
    const_float3_ptr_proxy v2,
    const unsigned char area,
    rcHeightfield &solid,
    const int flagMergeThr=1
) {
    return rcRasterizeTriangle(
        &ctx,
        unwrap(v0),
        unwrap(v1),
        unwrap(v2),
        area,
        solid,
        flagMergeThr
    );
}

bool rasterizeTriangles(
    rcContext& ctx,
    std::uintptr_t p_verts_,
    std::uintptr_t p_tris_,
    const int nv,
    std::uintptr_t p_areas_,
    const int nt,
    rcHeightfield &solid,
    const int flagMergeThr=1
) {
    return rcRasterizeTriangles(
        &ctx,
        reinterpret_cast<const float*>(p_verts_),
        static_cast<const int>(nv),
        reinterpret_cast<const int*>(p_tris_),
        reinterpret_cast<const unsigned char*>(p_areas_),
        nt,
        solid,
        flagMergeThr
    );
}
}

EMSCRIPTEN_BINDINGS(recast) {
    emscripten::class_<rcContext>("Context")
        .constructor()
        ;

    emscripten::class_<rcConfig>("Config")
        .constructor()
        .property("width", &rcConfig::width)
        .property("height", &rcConfig::height)
        .property("tileSize", &rcConfig::tileSize)
        .property("borderSize", &rcConfig::borderSize)
        .property("cs", &rcConfig::cs)
        .property("ch", &rcConfig::ch)
        // .property("bmin", &rcConfig::bmin)
        // .property("bmax", &rcConfig::bmax)
        .property("walkableSlopeAngle", &rcConfig::walkableSlopeAngle)
        .property("walkableHeight", &rcConfig::walkableHeight)
        .property("walkableClimb", &rcConfig::walkableClimb)
        .property("walkableRadius", &rcConfig::walkableRadius)
        .property("maxEdgeLen", &rcConfig::maxEdgeLen)
        .property("maxSimplificationError", &rcConfig::maxSimplificationError)
        .property("minRegionArea", &rcConfig::minRegionArea)
        .property("mergeRegionArea", &rcConfig::mergeRegionArea)
        .property("maxVertsPerPoly", &rcConfig::maxVertsPerPoly)
        .property("detailSampleDist", &rcConfig::detailSampleDist)
        .property("detailSampleMaxError", &rcConfig::detailSampleMaxError)
        ;

    emscripten::class_<rcHeightfield>("Heightfield")
        .constructor()
        .property("width", &rcHeightfield::width)
        .property("height", &rcHeightfield::height)
        // .property("bmin", &rcHeightfield::bmin)
        // .property("bmax", &rcHeightfield::bmax)
        .property("cs", &rcHeightfield::cs)
        .property("ch", &rcHeightfield::ch)
        // .property("spans", &rcHeightfield::spans)
        // .property("pools", &rcHeightfield::pools)
        // .property("freelist", &rcHeightfield::freelist)
        ;

    emscripten::class_<rcCompactHeightfield>("CompactHeightfield")
        .constructor()
        .property("width", &rcCompactHeightfield::width)
        .property("height", &rcCompactHeightfield::height)
        .property("spanCount", &rcCompactHeightfield::spanCount)
        .property("walkableHeight", &rcCompactHeightfield::walkableHeight)
        .property("walkableClimb", &rcCompactHeightfield::walkableClimb)
        .property("borderSize", &rcCompactHeightfield::borderSize)
        .property("maxDistance", &rcCompactHeightfield::maxDistance)
        .property("maxRegions", &rcCompactHeightfield::maxRegions)
        // .property("bmin", &rcCompactHeightfield::bmin)
        // .property("bmax", &rcCompactHeightfield::bmax)
        .property("cs", &rcCompactHeightfield::cs)
        .property("ch", &rcCompactHeightfield::ch)
        // .property("cells", &rcCompactHeightfield::cells)
        // .property("spans", &rcCompactHeightfield::spans)
        // .property("dist", &rcCompactHeightfield::dist)
        // .property("areas", &rcCompactHeightfield::areas)
        ;

    emscripten::class_<rcHeightfieldLayerSet>("HeightfieldLayerSet")
        .constructor()
        .property("nlayers", &rcHeightfieldLayerSet::nlayers)
        ;

    emscripten::class_<rcContourSet>("ContourSet")
        .constructor()
        // .property("conts", &rcContourSet::conts)
        .property("nconts", &rcContourSet::nconts)
        // .property("bmin", &rcContourSet::bmin)
        // .property("bmax", &rcContourSet::bmax)
        .property("cs", &rcContourSet::cs)
        .property("ch", &rcContourSet::ch)
        .property("width", &rcContourSet::width)
        .property("height", &rcContourSet::height)
        .property("borderSize", &rcContourSet::borderSize)
        .property("maxError", &rcContourSet::maxError)
        ;

    emscripten::class_<rcPolyMesh>("PolyMesh")
        .constructor()
        // .property("verts", &rcPolyMesh::verts)
        // .property("polys", &rcPolyMesh::polys)
        // .property("regs", &rcPolyMesh::regs)
        // .property("flags", &rcPolyMesh::flags)
        // .property("areas", &rcPolyMesh::areas)
        .property("nverts", &rcPolyMesh::nverts)
        .property("npolys", &rcPolyMesh::npolys)
        .property("maxpolys", &rcPolyMesh::maxpolys)
        .property("nvp", &rcPolyMesh::nvp)
        // .property("bmin", &rcPolyMesh::bmin)
        // .property("bmax", &rcPolyMesh::bmax)
        .property("cs", &rcPolyMesh::cs)
        .property("ch", &rcPolyMesh::ch)
        .property("borderSize", &rcPolyMesh::borderSize)
        .property("maxEdgeError", &rcPolyMesh::maxEdgeError)
        ;

    emscripten::class_<rcPolyMeshDetail>("PolyMeshDetail")
        .constructor()
        // .property("meshes", &rcPolyMeshDetail::meshes)
        // .property("verts", &rcPolyMeshDetail::verts)
        // .property("tris", &rcPolyMeshDetail::tris)
        .property("nmeshes", &rcPolyMeshDetail::nmeshes)
        .property("nverts", &rcPolyMeshDetail::nverts)
        .property("ntris", &rcPolyMeshDetail::ntris)
        ;

    // Allocation Functions

    emscripten::function("allocHeightfield", rcAllocHeightfield, emscripten::allow_raw_pointers());
    emscripten::function("freeHeightField", rcFreeHeightField, emscripten::allow_raw_pointers());
    emscripten::function("allocCompactHeightfield", rcAllocCompactHeightfield, emscripten::allow_raw_pointers());
    emscripten::function("freeCompactHeightfield", rcFreeCompactHeightfield, emscripten::allow_raw_pointers());
    emscripten::function("allocHeightfieldLayerSet", rcAllocHeightfieldLayerSet, emscripten::allow_raw_pointers());
    emscripten::function("freeHeightfieldLayerSet", rcFreeHeightfieldLayerSet, emscripten::allow_raw_pointers());
    emscripten::function("allocContourSet", rcAllocContourSet, emscripten::allow_raw_pointers());
    emscripten::function("freeContourSet", rcFreeContourSet, emscripten::allow_raw_pointers());
    emscripten::function("allocPolyMesh", rcAllocPolyMesh, emscripten::allow_raw_pointers());
    emscripten::function("freePolyMesh", rcFreePolyMesh, emscripten::allow_raw_pointers());
    emscripten::function("allocPolyMeshDetail", rcAllocPolyMeshDetail, emscripten::allow_raw_pointers());
    emscripten::function("freePolyMeshDetail", rcFreePolyMeshDetail, emscripten::allow_raw_pointers());

    // Heightfield Functions

    emscripten::function("createHeightfield", wasm_port::createHeightField);
    emscripten::function("rasterizeTriangle", wasm_port::rasterizeTriangle);
    emscripten::function("rasterizeTriangles", wasm_port::rasterizeTriangles);

    // Compact Heightfield Functions

    emscripten::function("buildCompactHeightfield", rcBuildCompactHeightfield, emscripten::allow_raw_pointer<emscripten::arg<0>>());

    // Layer, Contour, Polymesh, and Detail Mesh Functions

    emscripten::function("buildContours", rcBuildContours, emscripten::allow_raw_pointer<emscripten::arg<0>>());
    emscripten::function("buildPolyMesh", rcBuildPolyMesh, emscripten::allow_raw_pointer<emscripten::arg<0>>());
    emscripten::function("buildPolyMeshDetail", rcBuildPolyMeshDetail, emscripten::allow_raw_pointer<emscripten::arg<0>>());
    emscripten::function("copyPolyMesh", rcCopyPolyMesh, emscripten::allow_raw_pointer<emscripten::arg<0>>());

    emscripten::value_object<wasm_port::Point3>("Point3")
        .field("x", &wasm_port::Point3::x)
        .field("y", &wasm_port::Point3::y)
        .field("z", &wasm_port::Point3::z)
        ;
}

