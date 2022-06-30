#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/occupancy/OccupancyGrid.h>

using kelo::geometry_common::Pose2D;

namespace kelo {
namespace kelojson {

bool OccupancyGrid::initialise(int node_id, const osm::Primitive::Store& store)
{
    const osm::NodePrimitive::Ptr node = osm::PrimitiveUtils::getNode(store, node_id);
    if ( node == nullptr )
    {
        std::cout << Print::Err << "[OccupancyGrid] Node id: " << node_id
                  << " does not exist." << Print::End << std::endl;
        return false;
    }

    id = node->getId();
    float theta;
    if ( !node->readTag<std::string>("filename", filename) ||
         !node->readTag<float>("free_thresh", free_threshold) ||
         !node->readTag<float>("occupied_thresh", occupied_threshold) ||
         !node->readTag<bool>("negate", negate) ||
         !node->readTag<std::string>("name", name) ||
         !node->readTag<float>("resolution", resolution) ||
         !node->readTag<float>("theta", theta) )
    {
        return false;
    }
    origin = Pose2D(node->getPosition(), theta);
    return true;
}

std::ostream& operator << (std::ostream& out, const OccupancyGrid& grid)
{
    out << "OccupancyGrid:" << std::endl
        << "    id: " << grid.id << std::endl
        << "    filename: " << grid.filename << std::endl
        << "    free_threshold: " << grid.free_threshold << std::endl
        << "    occupied_threshold: " << grid.occupied_threshold << std::endl
        << "    negate: " << grid.negate << std::endl
        << "    name: " << grid.name << std::endl
        << "    resolution: " << grid.resolution << std::endl
        << "    origin: " << grid.origin << std::endl;
    return out;
}

} // namespace kelojson
} // namespace kelo
