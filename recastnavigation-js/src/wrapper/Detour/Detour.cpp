
#include "Detour.h"
#include "../../../recastnavigation/Detour/Include/DetourNavMesh.h"
#include "../../../recastnavigation/Detour/Include/DetourNavMeshQuery.h"
#include <emscripten/bind.h>
#include <memory>

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

struct Point3 {
    Point3() = default;

    Point3(float x_, float y_, float z_): x(x_), y(y_), z(z_) { }

    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    float* data() {
        return &x;
    }

    const float* data() const {
        return &x;
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

		nav_mesh_.addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

    return DT_SUCCESS;
}

EMSCRIPTEN_BINDINGS(detour) {
    emscripten::register_vector<dtPolyRef>("PolyRefVector");

    emscripten::function("makePolyRefVector", emscripten::select_overload<std::vector<dtPolyRef>(std::size_t)>([](std::size_t size_) {
        return std::vector<dtPolyRef>(0, size_);
    }));

    emscripten::value_object<Point3>("Point3")
        .field("x", &Point3::x)
        .field("y", &Point3::y)
        .field("z", &Point3::z)
        ;

    emscripten::class_<DBuffer>("DBuffer")
        .constructor<std::size_t>()
        .function("getBytes", &DBuffer::getBytes)
        ;

    emscripten::function("statusSucceed", &dtStatusSucceed);
    emscripten::function("statusFailed", &dtStatusFailed);
    emscripten::function("statusInProgress", &dtStatusInProgress);
    emscripten::function("statusDetail", &dtStatusDetail);
        
    emscripten::class_<dtNavMesh>("NavMesh")
        .constructor()
        .class_function("initWithData", &initWithData)
        // .function("initWithData", [](dtNavMesh &nav_mesh_, DBuffer &data_, int flags_) {
        //         return nav_mesh_.init(data_.data(), data_.size(), flags_);
        //     })
        // .function("initWithData", emscripten::select_overload<dtStatus(unsigned char*, int, int)>(&dtNavMesh::init), emscripten::allow_raw_pointers())
        .function("initWithParams", emscripten::select_overload<dtStatus(const dtNavMeshParams*)>(&dtNavMesh::init), emscripten::allow_raw_pointers())
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

    emscripten::class_<dtNavMeshQuery>("NavMeshQuery")
        .constructor()
        .function("init", &dtNavMeshQuery::init, emscripten::allow_raw_pointers())
        .function("findNearestPoly", emscripten::select_overload<FindNearestPolyResult(const dtNavMeshQuery &, const Point3 &, const Point3 &, const dtQueryFilter &)>([](const dtNavMeshQuery &query_, const Point3 &center_, const Point3 &half_extents_, const dtQueryFilter &filter_) {
                FindNearestPolyResult result;
                result.status = query_.findNearestPoly(center_.data(), half_extents_.data(), &filter_, &result.poly, result.point.data());
                return result;
            }))
        .function("findPath", emscripten::select_overload<FindPathResult(const dtNavMeshQuery &, dtPolyRef, dtPolyRef, const Point3 &, const Point3 &, const dtQueryFilter &, std::vector<dtPolyRef> &)>(
            [](const dtNavMeshQuery &query_,
                dtPolyRef start_poly_,
                dtPolyRef end_poly_,
                const Point3 &start_point_,
                const Point3 &end_point_,
                const dtQueryFilter &filter_,
                std::vector<dtPolyRef> &path_) {
                FindPathResult result;
                result.status = query_.findPath(
                    start_poly_,
                    end_poly_,
                    start_point_.data(),
                    end_point_.data(),
                    &filter_,
                    path_.data(),
                    &result.size,
                    path_.size());
                return result;
            }))
        ;
}