#ifndef KELO_KELOJSON_OCCUPANCY_GRID_LAYER_H
#define KELO_KELOJSON_OCCUPANCY_GRID_LAYER_H

#include <kelojson/layer/Layer.h>
#include <kelojson/layer/occupancy/OccupancyGrid.h>

namespace kelo {
namespace kelojson {

class OccupancyGridLayer : public Layer
{
    public:

        using Ptr = std::shared_ptr<OccupancyGridLayer>;

        using ConstPtr = std::shared_ptr<const OccupancyGridLayer>;

        OccupancyGridLayer():
            Layer(LayerType::OCCUPANCY_GRID) {}

        virtual ~OccupancyGridLayer() = default;

        bool initialise(const osm::Primitive::Store& store) override;

        const std::map<int, OccupancyGrid>& getAllOccupancyGrids() const;

    private:

        std::map<int, OccupancyGrid> occ_grids_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OCCUPANCY_GRID_LAYER_H
