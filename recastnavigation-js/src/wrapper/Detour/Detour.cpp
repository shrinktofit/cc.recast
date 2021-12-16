
#include "Detour.h"
#include "DetourCrowd.h"
#include "DebugDraw.h"
#include "DetourDebugDraw.h"
#include "../Common/Vec3.h"
#include "../../../recastnavigation/Detour/Include/DetourNavMesh.h"
#include "../../../recastnavigation/Detour/Include/DetourNavMeshQuery.h"
#include <emscripten/bind.h>
#include <memory>
#include <iostream>

namespace wasm_port {
class DBuffer {
public:
    DBuffer (std::size_t size_): _size(size_) {
        _data = std::make_unique<unsigned char[]>(_size);
    }

    emscripten::val getBytes() {
        return emscripten::val(emscripten::typed_memory_view(_size, _data.get()));
    }

    unsigned char* data() {
        return _data.get();
    }

    std::size_t size() {
        return _size;
    }
private:
    std::unique_ptr<unsigned char[]> _data;
    std::size_t _size;
};

dtStatus initWithData(dtNavMesh &navMesh, DBuffer &data, int flags) {
    return navMesh.init(data.data(), data.size(), flags);
}

struct Point3Object: public Point3 {
public:
    float getX() const {
        return x;
    }

    void setX(float x_) {
        x = x_;
    }

    float getY() const {
        return y;
    }

    void setY(float y_) {
        y = y_;
    }

    float getZ() const {
        return z;
    }

    void setZ(float z_) {
        z = z_;
    }
};

struct FindNearestPolyResult {
    dtStatus status;
    dtPolyRef poly;
    Point3 point;
};

struct FindPathResult {
    dtStatus status;
    int size;
};

struct ClosestPointOnPolyResult {
    dtStatus status;
    bool posOverPoly;
};

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

class StraightPath {
public:
    StraightPath(std::size_t size_): _path(3 * size_), _flags(size_), _polys(size_) {
    }

    float getPath(std::size_t index_, unsigned component_) {
        assert(component_ < 3);
        return _path[3 * index_ + component_];
    }

    unsigned char getFlag(std::size_t index_) {
        return _flags[index_];
    }

    dtPolyRef getPoly(std::size_t index_) {
        return _polys[index_];
    }

    std::size_t size() const {
        return _flags.size();
    }

    float* _get_path_storage() {
        return _path.data();
    }

    unsigned char* _get_flag_storage() {
        return _flags.data();
    }

    dtPolyRef* _get_poly_storage() {
        return _polys.data();
    }

    void resize(std::size_t size_) {
        (*this) = StraightPath(size_);
    }

private:
    std::vector<float> _path;
    std::vector<unsigned char> _flags;
    std::vector<dtPolyRef> _polys;
};

struct FindStraightPathResult {
    dtStatus status;
    int size;
};

struct GetPolyHeightResult {
    dtStatus status;
    float height;
};

struct MoveAlongSurfaceResult {
    dtStatus status;
    int visitedSize;
};

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

dtStatus readRecastDemoSample(dtNavMesh &nav_mesh_, const unsigned char *data_begin_, const unsigned char *data_end_) {
    std::size_t pData = 0;
    if ((data_end_ - data_begin_) < sizeof(NavMeshSetHeader)) {
        return DT_FAILURE;
    }
    const NavMeshSetHeader &header = *reinterpret_cast<const NavMeshSetHeader*>(data_begin_);
    data_begin_ += sizeof(NavMeshSetHeader);

    if (header.magic != NAVMESHSET_MAGIC) {
        return DT_FAILURE | DT_WRONG_MAGIC;
    }

    if (header.version != NAVMESHSET_VERSION) {
        return DT_FAILURE | DT_WRONG_VERSION;
    }

    auto status = nav_mesh_.init(&header.params);
    if (!dtStatusSucceed(status)) {
        return status;
    }

    // Read tiles.
    std::cout << "Tiles: " << header.numTiles << std::endl;
	for (int i = 0; i < header.numTiles; ++i)
	{
        if ((data_end_ - data_begin_) <  sizeof(NavMeshTileHeader)) {
            return DT_FAILURE;
        }
		const NavMeshTileHeader &tileHeader = *reinterpret_cast<const NavMeshTileHeader*>(data_begin_);
        data_begin_ += sizeof(NavMeshTileHeader);

		if (!tileHeader.tileRef || !tileHeader.dataSize) {
			break;
        }

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) {
            return DT_FAILURE;
        }

        if ((data_end_ - data_begin_) < tileHeader.dataSize) {
            dtFree(data);
            return DT_FAILURE;
        }
        std::memcpy(data, data_begin_, tileHeader.dataSize);

		nav_mesh_.addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

    return DT_SUCCESS;
}

class JNavMeshQuery {
public:
    dtNavMeshQuery& impl() {
        return _impl;
    }

    const dtNavMeshQuery& impl() const {
        return _impl;
    }

    dtStatus init(dtNavMesh & nav_mesh_, const int max_nodes_) {
        return _impl.init(&nav_mesh_, max_nodes_);
    }

    FindNearestPolyResult findNearestPoly (
        const Point3 &center_,
        const Point3 &half_extents_,
        const dtQueryFilter &filter_
    ) {
        std::cout << "[findNearestPoly]" << center_.x << ", " << center_.y << ", " << center_.z << std::endl;
        FindNearestPolyResult result;
        result.status = _impl.findNearestPoly(center_.data(), half_extents_.data(), &filter_, &result.poly, result.point.data());
        std::cout << "[findNearestPoly]" <<
            "status: " << result.status <<
            "\n" <<
            "poly: " << result.poly <<
            "\n" <<
            "point: " << result.point.x << ", " << result.point.y << ", " << result.point.z <<
            std::endl;
        return result;
    }

    FindPathResult findPath(
        dtPolyRef start_poly_,
        dtPolyRef end_poly_,
        const Point3 &start_point_,
        const Point3 &end_point_,
        const dtQueryFilter &filter_,
        std::vector<dtPolyRef> &path_
    ) {
        FindPathResult result;
        result.status = _impl.findPath(
            start_poly_,
            end_poly_,
            start_point_.data(),
            end_point_.data(),
            &filter_,
            path_.data(),
            &result.size,
            path_.size());
        return result;
    }

    FindStraightPathResult findStraightPath(
        const Point3 & start_,
        const Point3 &end_,
        const std::vector<dtPolyRef>& path_,
        std::size_t path_size_, int options_,
        StraightPath &straight_path_
    ) {
        FindStraightPathResult result;
        result.status = _impl.findStraightPath(
            start_.data(),
            end_.data(),
            path_.data(),
            path_size_,
            straight_path_._get_path_storage(),
            straight_path_._get_flag_storage(),
            straight_path_._get_poly_storage(),
            &result.size,
            straight_path_.size(),
            options_
            );
        return result;
    }

    ClosestPointOnPolyResult closestPointOnPoly(
        dtPolyRef poly_,
        const Point3 &position_,
        Point3Object &closest_
    ) {
        ClosestPointOnPolyResult result;
        result.status = _impl.closestPointOnPoly(
            poly_,
            position_.data(),
            closest_.data(),
            &result.posOverPoly
            );
        return result;
    }

    GetPolyHeightResult getPolyHeight(
        dtPolyRef poly_,
        const Point3 &position_
    ) {
        GetPolyHeightResult result;
        result.status = _impl.getPolyHeight(poly_, position_.data(), &result.height);
        return result;
    }

    MoveAlongSurfaceResult moveAlongSurface(
        dtPolyRef start_poly_,
        const Point3 &start_position_,
        const Point3 &end_position_,
        dtQueryFilter &filter_,
        Point3Object &result_position_,
        std::vector<dtPolyRef> &visited_
    ) {
        MoveAlongSurfaceResult result;
        result.status = _impl.moveAlongSurface(
            start_poly_,
            start_position_.data(),
            end_position_.data(),
            &filter_,
            result_position_.data(),
            visited_.data(),
            &result.visitedSize,
            visited_.size()
        );
        std::cout << "[moveAlongSurface]" << "point: " << result_position_.x << ", " << result_position_.y << ", " << result_position_.z << std::endl;
        return result;
    }
private:
    dtNavMeshQuery _impl;
};

class Utils {
public:
    static int fixupCorridor(std::vector<dtPolyRef> &path_, const int path_count_, const std::vector<dtPolyRef> &visited_, const int visited_count_) {
        return _fixupCorridor(path_.data(), path_count_, path_.size(), visited_.data(), visited_count_);
    }

    static int fixupShortcuts(std::vector<dtPolyRef> &path_, const int path_count_, JNavMeshQuery &query_) {
        return _fixupShortcuts(path_.data(), path_count_, &query_.impl());
    }
private:
    static int _fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
                            const dtPolyRef* visited, const int nvisited)
    {
        int furthestPath = -1;
        int furthestVisited = -1;
        
        // Find furthest common polygon.
        for (int i = npath-1; i >= 0; --i)
        {
            bool found = false;
            for (int j = nvisited-1; j >= 0; --j)
            {
                if (path[i] == visited[j])
                {
                    furthestPath = i;
                    furthestVisited = j;
                    found = true;
                }
            }
            if (found)
                break;
        }

        // If no intersection found just return current path. 
        if (furthestPath == -1 || furthestVisited == -1)
            return npath;
        
        // Concatenate paths.	

        // Adjust beginning of the buffer to include the visited.
        const int req = nvisited - furthestVisited;
        const int orig = std::min(furthestPath+1, npath);
        int size = std::max(0, npath-orig);
        if (req+size > maxPath)
            size = maxPath-req;
        if (size)
            memmove(path+req, path+orig, size*sizeof(dtPolyRef));
        
        // Store visited
        for (int i = 0; i < req; ++i)
            path[i] = visited[(nvisited-1)-i];				
        
        return req+size;
    }

    // This function checks if the path has a small U-turn, that is,
    // a polygon further in the path is adjacent to the first polygon
    // in the path. If that happens, a shortcut is taken.
    // This can happen if the target (T) location is at tile boundary,
    // and we're (S) approaching it parallel to the tile edge.
    // The choice at the vertex can be arbitrary, 
    //  +---+---+
    //  |:::|:::|
    //  +-S-+-T-+
    //  |:::|   | <-- the step can end up in here, resulting U-turn path.
    //  +---+---+
    static int _fixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery)
    {
        if (npath < 3)
            return npath;

        // Get connected polygons
        static const int maxNeis = 16;
        dtPolyRef neis[maxNeis];
        int nneis = 0;

        const dtMeshTile* tile = 0;
        const dtPoly* poly = 0;
        if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
            return npath;
        
        for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
        {
            const dtLink* link = &tile->links[k];
            if (link->ref != 0)
            {
                if (nneis < maxNeis)
                    neis[nneis++] = link->ref;
            }
        }

        // If any of the neighbour polygons is within the next few polygons
        // in the path, short cut to that polygon directly.
        static const int maxLookAhead = 6;
        int cut = 0;
        for (int i = std::min(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--) {
            for (int j = 0; j < nneis; j++)
            {
                if (path[i] == neis[j]) {
                    cut = i;
                    break;
                }
            }
        }
        if (cut > 1)
        {
            int offset = cut-1;
            npath -= offset;
            for (int i = 1; i < npath; i++)
                path[i] = path[i+offset];
        }

        return npath;
    }
};
}

using namespace wasm_port;

template <typename PM>
struct pm_decomposer { };

template <typename ClassT, typename T, std::size_t N>
struct pm_decomposer<T (ClassT::*)[N]> {
    using class_type = ClassT;
    using element_type = T;
};

template <auto PM>
decltype(auto) array_buffer_view_field() {
    using Decomposer = pm_decomposer<decltype(PM)>;
    using Class = typename Decomposer::class_type;
    using Element = typename Decomposer::element_type;
    return emscripten::select_overload<emscripten::val(const Class &)>([](const Class &o_) -> emscripten::val {
        return emscripten::val(
            emscripten::typed_memory_view(std::size(o_.*PM), o_.*PM)
        );
    });
}

class DebugDrawWrapper: public emscripten::wrapper<duDebugDraw> {
public:
    EMSCRIPTEN_WRAPPER(DebugDrawWrapper);

    ~DebugDrawWrapper() {
        // call<void>("destructor");
    }

    void depthMask(bool state) {
        return call<void>("depthMask", state);
    }

    void texture(bool state) {
        return call<void>("texture", state);
    }

	void begin(duDebugDrawPrimitives prim, float size = 1.0f) {
        return call<void>("begin", prim, size);
    }

	void vertex(const float* pos, unsigned int color) {
        return this->vertex(pos[0], pos[1], pos[2], color);
    }

	void vertex(const float x, const float y, const float z, unsigned int color) {
        return call<void>("vertex", x, y, z, color);
    }

	void vertex(const float* pos, unsigned int color, const float* uv) {
        return this->vertex(pos[0], pos[1], pos[2], color, uv[0], uv[1]);
    }
	
	void vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v) {
        return call<void>("vertex", x, y, z, color, u, v);
    }
	
	void end() {
        return call<void>("end");
    }

    void vertexPositionColor(const float x, const float y, const float z, unsigned int color) {
        return this->vertex(x, y, z, color);
    }

    void vertexPositionColorUV(const float x, const float y, const float z, unsigned int color, const float u, const float v) {
        return this->vertex(x, y, z, color, u, v);
    }
};

using DebugDrawVertex = void(const float x, const float y, const float z, unsigned int color);

EMSCRIPTEN_BINDINGS(detour) {
    emscripten::register_vector<dtPolyRef>("PolyRefVector");

    emscripten::function("makePolyRefVector", emscripten::select_overload<std::vector<dtPolyRef>(std::size_t, dtPolyRef)>([](std::size_t size_, dtPolyRef value_) {
        return std::vector<dtPolyRef>(size_, value_);
    }));

    emscripten::value_object<Point3>("Point3")
        .field("x", &Point3::x)
        .field("y", &Point3::y)
        .field("z", &Point3::z)
        ;

    emscripten::class_<Point3Object>("Point3Object")
        .constructor()
        .property("x", &Point3Object::getX, &Point3Object::setX)
        .property("y", &Point3Object::getY, &Point3Object::setY)
        .property("z", &Point3Object::getZ, &Point3Object::setZ)
        ;

    emscripten::class_<DBuffer>("DBuffer")
        .constructor<std::size_t>()
        .function("getBytes", &DBuffer::getBytes)
        ;

    emscripten::constant("STRAIGHTPATH_START", DT_STRAIGHTPATH_START);
    emscripten::constant("STRAIGHTPATH_END", DT_STRAIGHTPATH_END);
    emscripten::constant("STRAIGHTPATH_OFFMESH_CONNECTION", DT_STRAIGHTPATH_OFFMESH_CONNECTION);

    emscripten::function("statusSucceed", &dtStatusSucceed);
    emscripten::function("statusFailed", &dtStatusFailed);
    emscripten::function("statusInProgress", &dtStatusInProgress);
    emscripten::function("statusDetail", &dtStatusDetail);

    emscripten::class_<dtPoly>("Poly")
        .property("verts", emscripten::select_overload<emscripten::val(const dtPoly &)>(
            [](const dtPoly &poly_) {
                return emscripten::val(
                    emscripten::typed_memory_view(DT_VERTS_PER_POLYGON, poly_.verts)
                );
            }))
        .property("vertCount", &dtPoly::vertCount)
        ;

    emscripten::class_<dtPolyDetail>("PolyDetail")
        .property("vertBase", &dtPolyDetail::vertBase)
        .property("triBase", &dtPolyDetail::triBase)
        .property("vertCount", &dtPolyDetail::vertCount)
        .property("triCount", &dtPolyDetail::triCount)
        ;
    
    emscripten::class_<dtMeshHeader>("MeshHeader")
        .property("version", &dtMeshHeader::version)
        .property("vertCount", &dtMeshHeader::vertCount)
        .property("polyCount", &dtMeshHeader::polyCount)
        .property("vertCount", &dtMeshHeader::detailVertCount)
        .property("detailTriCount", &dtMeshHeader::detailTriCount)
        ;

    emscripten::class_<dtMeshTile>("MeshTile")
        .property("header", emscripten::select_overload<const dtMeshHeader&(const dtMeshTile &)>(
            [](const dtMeshTile &tile_) -> const dtMeshHeader& {
                return *tile_.header;
            }))
        .property("verts", emscripten::select_overload<emscripten::val(const dtMeshTile &)>(
            [](const dtMeshTile &mesh_tile_) {
                std::cerr << "[Vertices]::" << mesh_tile_.header->vertCount << std::endl;
                return emscripten::val(
                    emscripten::typed_memory_view(3 * mesh_tile_.header->vertCount, mesh_tile_.verts)
                );
            }))
        .function("getPoly", emscripten::select_overload<const dtPoly*(const dtMeshTile &, int index)>(
            [](const dtMeshTile &mesh_tile_, int index) -> const dtPoly* {
                return mesh_tile_.polys + index;
            }), emscripten::allow_raw_pointers())
        .property("detailVerts", emscripten::select_overload<emscripten::val(const dtMeshTile &)>(
            [](const dtMeshTile &mesh_tile_) {
                std::cerr << "[DetailVertices]::" << mesh_tile_.header->detailVertCount << std::endl;
                return emscripten::val(
                    emscripten::typed_memory_view(3 * mesh_tile_.header->detailVertCount, mesh_tile_.detailVerts)
                );
            }))
        // .property("detailTris", emscripten::select_overload<emscripten::val(const dtMeshTile &)>(
        //     [](const dtMeshTile &mesh_tile_) {
        //         std::cerr << "[DetailTriangles]::" << mesh_tile_.header->detailVertCount << std::endl;
        //         return emscripten::val(
        //             emscripten::typed_memory_view(/* TODO, not 3 */3 * mesh_tile_.header->detailTriCount, mesh_tile_.detailTris)
        //         );
        //     }))
        .function("getDetailMesh", emscripten::select_overload<const dtPolyDetail*(const dtMeshTile &, int index)>(
            [](const dtMeshTile &mesh_tile_, int index) -> const dtPolyDetail* {
                return mesh_tile_.detailMeshes + index;
            }), emscripten::allow_raw_pointers())
        ;
        // .property("polys", emscripten::select_overload<emscripten::val(const dtMeshTile &)>(
        //     [](const dtMeshTile &mesh_tile_) {
        //         std::vector<dtPoly*> polys(mesh_tile_.header->polyCount);
        //         for (int i = 0; i < polys.size(); ++i) {
        //             polys[i] = mesh_tile_.polys + i;
        //         }
        //         return emscripten::val::array(polys);
        //     }))
        // ;
        
    emscripten::class_<dtNavMesh>("NavMesh")
        .constructor()
        .function("getMaxTiles", &dtNavMesh::getMaxTiles)
        .class_function("initWithData", &initWithData)
        // .function("initWithData", [](dtNavMesh &nav_mesh_, DBuffer &data_, int flags_) {
        //         return nav_mesh_.init(data_.data(), data_.size(), flags_);
        //     })
        // .function("initWithData", emscripten::select_overload<dtStatus(unsigned char*, int, int)>(&dtNavMesh::init), emscripten::allow_raw_pointers())
        .function("initWithParams", emscripten::select_overload<dtStatus(const dtNavMeshParams*)>(&dtNavMesh::init), emscripten::allow_raw_pointers())
        .function("getOffMeshConnectionPolyEndPoints", emscripten::select_overload<dtStatus(const dtNavMesh&, dtPolyRef, dtPolyRef, Point3&, Point3&)>(
            [](const dtNavMesh& nav_mesh_, dtPolyRef prev_poly_, dtPolyRef poly_, Point3 &start_position_, Point3 &end_position_) {
                return nav_mesh_.getOffMeshConnectionPolyEndPoints(
                    prev_poly_,
                    poly_,
                    start_position_.data(),
                    end_position_.data()
                );
            }))
        .function("getTile", emscripten::select_overload<const dtMeshTile*(const dtNavMesh &, int)>(
            [](const dtNavMesh &nav_mesh_, int index_) {
                return nav_mesh_.getTile(index_);
            }), emscripten::allow_raw_pointers())
        ;

    emscripten::function("readRecastDemoSample", emscripten::select_overload<dtStatus(dtNavMesh &nav_mesh_, DBuffer &data_)>(
        [](dtNavMesh &nav_mesh_, DBuffer &data_) {
            return readRecastDemoSample(nav_mesh_, data_.data(), data_.data() + data_.size());
        }));

    emscripten::class_<dtQueryFilter>("QueryFilter")
        .constructor()
        ;

    emscripten::value_object<FindNearestPolyResult>("FindNearestPolyResult")
        .field("status", &FindNearestPolyResult::status)
        .field("poly", &FindNearestPolyResult::poly)
        .field("point", &FindNearestPolyResult::point)
        ;

    emscripten::value_object<FindPathResult>("FindPathResult")
        .field("status", &FindPathResult::status)
        .field("size", &FindPathResult::size)
        ;

    emscripten::class_<StraightPath>("StraightPath")
        .constructor<std::size_t>()
        .function("getPath", &StraightPath::getPath)
        .function("getFlag", &StraightPath::getFlag)
        .function("getPoly", &StraightPath::getPoly)
        .function("resize", &StraightPath::resize)
        ;

    emscripten::value_object<FindStraightPathResult>("FindStraightPathResult")
        .field("status", &FindStraightPathResult::status)
        .field("size", &FindStraightPathResult::size)
        ;

    emscripten::value_object<ClosestPointOnPolyResult>("ClosestPointOnPolyResult")
        .field("status", &ClosestPointOnPolyResult::status)
        .field("posOverPoly", &ClosestPointOnPolyResult::posOverPoly)
        ;

    emscripten::value_object<GetPolyHeightResult>("GetPolyHeightResult")
        .field("status", &GetPolyHeightResult::status)
        .field("height", &GetPolyHeightResult::height)
        ;

    emscripten::value_object<MoveAlongSurfaceResult>("MoveAlongSurfaceResult")
        .field("status", &MoveAlongSurfaceResult::status)
        .field("visitedSize", &MoveAlongSurfaceResult::visitedSize)
        ;

    emscripten::class_<JNavMeshQuery>("NavMeshQuery")
        .constructor()
        .function("init", &JNavMeshQuery::init)
        .function("findNearestPoly", &JNavMeshQuery::findNearestPoly)
        .function("findPath", &JNavMeshQuery::findPath)
        .function("findStraightPath", &JNavMeshQuery::findStraightPath)
        .function("closestPointOnPoly", &JNavMeshQuery::closestPointOnPoly)
        .function("getPolyHeight", &JNavMeshQuery::getPolyHeight)
        .function("moveAlongSurface", &JNavMeshQuery::moveAlongSurface)
        ;

    emscripten::class_<Utils>("Utils")
        .class_function("fixupCorridor", &Utils::fixupCorridor)
        .class_function("fixupShortcuts", &Utils::fixupShortcuts)
        ;

    emscripten::class_<dtCrowdAgent>("CrowdAgent")
        .property("active", &dtCrowdAgent::active)
        .property("state", &dtCrowdAgent::state)
        .property("partial", &dtCrowdAgent::partial)
	// dtPathCorridor corridor;
	// dtLocalBoundary boundary;
        .property("topologyOptTime", &dtCrowdAgent::topologyOptTime)
        // neis
        .property("nneis", &dtCrowdAgent::nneis)
        .property("desiredSpeed", &dtCrowdAgent::desiredSpeed)
        .property("npos", emscripten::select_overload<emscripten::val(const dtCrowdAgent &)>([](const dtCrowdAgent &agent_) {
            return emscripten::val(emscripten::typed_memory_view(
                std::size(agent_.npos),
                agent_.npos
            ));
        }))
        .property("disp", array_buffer_view_field<&dtCrowdAgent::disp>())
        .property("dvel", array_buffer_view_field<&dtCrowdAgent::dvel>())
        .property("nvel", array_buffer_view_field<&dtCrowdAgent::nvel>())
        .property("vel", array_buffer_view_field<&dtCrowdAgent::vel>())
        .property("params", &dtCrowdAgent::params)
        // cornerVerts
        // cornerFlags
        // cornerPolys
        .property("ncorners", &dtCrowdAgent::ncorners)
        .property("targetState", &dtCrowdAgent::targetState)
        .property("targetRef", &dtCrowdAgent::targetRef)
        // .property("targetPos", &dtCrowdAgent::targetPos)
        // .property("active", &dtCrowdAgent::active) targetPathqRef
        .property("targetReplan", &dtCrowdAgent::targetReplan)
        .property("targetReplanTime", &dtCrowdAgent::targetReplanTime)
        ;

    emscripten::class_<dtCrowdAgentParams>("CrowdAgentParams")
        .constructor()

    /*
	/// User defined data attached to the agent.
	void* userData;
    */
        .property("radius", &dtCrowdAgentParams::radius)
        .property("height", &dtCrowdAgentParams::height)
        .property("maxAcceleration", &dtCrowdAgentParams::maxAcceleration)
        .property("maxSpeed", &dtCrowdAgentParams::maxSpeed)
        .property("collisionQueryRange", &dtCrowdAgentParams::collisionQueryRange)
        .property("pathOptimizationRange", &dtCrowdAgentParams::pathOptimizationRange)
        .property("separationWeight", &dtCrowdAgentParams::separationWeight)
        .property("updateFlags", &dtCrowdAgentParams::updateFlags)
        .property("obstacleAvoidanceType", &dtCrowdAgentParams::obstacleAvoidanceType)
        .property("queryFilterType", &dtCrowdAgentParams::queryFilterType)
        // .property("userData", [](const dtNavMeshParams &params_) -> emscripten::val {
        //     // TODO
        //     return emscripten::val::undefined();
        // }, [](dtNavMeshParams &params_, emscripten::val data_) -> void {
        //     // TODO
        // })
        ;

    emscripten::class_<dtCrowdAgentDebugInfo>("CrowdAgentDebugInfo")
        ;

    emscripten::class_<dtCrowd>("Crowd")
        .constructor()
        .function("init", &dtCrowd::init, emscripten::allow_raw_pointers())
        .function("getAgent", &dtCrowd::getAgent, emscripten::allow_raw_pointers())
        .function("getAgentCount", &dtCrowd::getAgentCount)
        .function("addAgent", emscripten::select_overload<int(dtCrowd &, const Point3 &, const dtCrowdAgentParams &)>(
            [](dtCrowd &crowd_, const Point3 &pos_, const dtCrowdAgentParams &params_) -> int {
                return crowd_.addAgent(pos_.data(), &params_);
            }))
        .function("updateAgentParameters", &dtCrowd::updateAgentParameters, emscripten::allow_raw_pointers())
        .function("removeAgent", &dtCrowd::removeAgent)
        .function("requestMoveTarget", emscripten::select_overload<bool(dtCrowd &, const int, dtPolyRef, const Point3&)>(
            [](dtCrowd &crowd_, const int idx_, dtPolyRef ref_, const Point3 &pos_) -> bool {
                return crowd_.requestMoveTarget(idx_, ref_, pos_.data());
            }))
        .function("requestMoveVelocity", emscripten::select_overload<bool(dtCrowd &, const int, const Point3&)>(
            [](dtCrowd &crowd_, const int idx_, const Point3 &vel_) -> bool {
                return crowd_.requestMoveVelocity(idx_, vel_.data());
            }))
        .function("resetMoveTarget", &dtCrowd::resetMoveTarget)
        .function("update", &dtCrowd::update, emscripten::allow_raw_pointers())
        .function("getFilter", &dtCrowd::getFilter, emscripten::allow_raw_pointers())
        .function("getQueryHalfExtents", emscripten::select_overload<Point3(dtCrowd &)>(
            [](dtCrowd &crowd_) -> Point3 {
                const auto extents = crowd_.getQueryHalfExtents();
                return Point3(extents[0], extents[1], extents[2]);
            }))
        .function("getQueryExtents", emscripten::select_overload<Point3(dtCrowd &)>(
            [](dtCrowd &crowd_) -> Point3 {
                const auto extents = crowd_.getQueryExtents();
                return Point3(extents[0], extents[1], extents[2]);
            }))
        ;

    emscripten::enum_<duDebugDrawPrimitives>("DebugDrawPrimitives")
        .value("DRAW_POINTS", duDebugDrawPrimitives::DU_DRAW_POINTS)
        .value("DRAW_LINES", duDebugDrawPrimitives::DU_DRAW_LINES)
        .value("DRAW_TRIS", duDebugDrawPrimitives::DU_DRAW_TRIS)
        .value("DRAW_QUADS", duDebugDrawPrimitives::DU_DRAW_QUADS)
        ;

    emscripten::class_<duDebugDraw>("DebugDraw")
        .allow_subclass<DebugDrawWrapper>("DebugDrawWrapper", emscripten::constructor<>())
        .function("depthMask", &DebugDrawWrapper::depthMask, emscripten::pure_virtual())
        .function("texture", &DebugDrawWrapper::texture, emscripten::pure_virtual())
        .function("begin", &DebugDrawWrapper::begin, emscripten::pure_virtual())
        .function("vertex", &DebugDrawWrapper::vertexPositionColor, emscripten::pure_virtual())
        .function("vertexUV", &DebugDrawWrapper::vertexPositionColorUV, emscripten::pure_virtual())
        .function("end", &DebugDrawWrapper::end, emscripten::pure_virtual())
        ;

    emscripten::function("debugDrawNavMesh", emscripten::select_overload<void(duDebugDraw& dd, const dtNavMesh& mesh, unsigned char flags)>([](
        duDebugDraw& dd, const dtNavMesh& mesh, unsigned char flags
    ) {
        return duDebugDrawNavMesh(&dd, mesh, flags);
    }));
}