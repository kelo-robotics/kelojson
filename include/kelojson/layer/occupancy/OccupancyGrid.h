#ifndef KELO_KELOJSON_OCCUPANCY_GRID_H
#define KELO_KELOJSON_OCCUPANCY_GRID_H

#include <geometry_common/Pose2D.h>

#include <kelojson/osm/Primitive.h>

namespace kelo {
namespace kelojson {

class OccupancyGrid
{
    public:

        int id;
        std::string filename;
        float free_threshold;
        float occupied_threshold;
        bool negate;
        std::string name;
        float resolution;
        geometry_common::Pose2D origin;

        OccupancyGrid() = default;

        virtual ~OccupancyGrid() = default;

        bool initialise(int node_id, const osm::Primitive::Store& store);

        friend std::ostream& operator << (
                std::ostream& out,
                const OccupancyGrid& grid);

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OCCUPANCY_GRID_H
