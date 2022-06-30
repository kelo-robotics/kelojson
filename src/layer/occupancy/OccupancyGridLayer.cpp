#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/occupancy/OccupancyGridLayer.h>

namespace kelo {
namespace kelojson {

bool OccupancyGridLayer::initialise(const osm::Primitive::Store& store)
{
    std::vector<int> occ_grid_node_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::NODE),
            osm::Tags{{"layer", "occupancy_grid"}},
            "");

    for ( int node_id : occ_grid_node_ids )
    {
        OccupancyGrid occ_grid;
        if ( !occ_grid.initialise(node_id, store) )
        {
            return false;
        }
        occ_grids_[occ_grid.id] = occ_grid;
    }
    return true;
}

const std::map<int, OccupancyGrid>& OccupancyGridLayer::getAllOccupancyGrids() const
{
    return occ_grids_;
}

} // namespace kelojson
} // namespace kelo
