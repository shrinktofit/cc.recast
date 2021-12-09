
#include "Detour.h"
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
        .property("verts", emscripten::select_overload<emscripten::memory_view<unsigned short>(const dtPoly &)>(
            [](const dtPoly &poly_) {
                return emscripten::typed_memory_view(DT_VERTS_PER_POLYGON, poly_.verts);
            }))
        .property("vertCount", &dtPoly::vertCount)
        ;

    emscripten::class_<dtMeshTile>("MeshTile")
        .property("verts", emscripten::select_overload<emscripten::memory_view<float>(const dtMeshTile &)>(
            [](const dtMeshTile &mesh_tile_) {
                return emscripten::typed_memory_view(mesh_tile_.header->vertCount, mesh_tile_.verts);
            }))
        .property("polys", emscripten::select_overload<emscripten::val(const dtMeshTile &)>(
            [](const dtMeshTile &mesh_tile_) {
                std::vector<dtPoly*> polys(mesh_tile_.header->polyCount);
                for (int i = 0; i < polys.size(); ++i) {
                    polys[i] = mesh_tile_.polys + i;
                }
                return emscripten::val::array(polys);
            }))
        ;
        
    emscripten::class_<dtNavMesh>("NavMesh")
        .constructor()
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
}