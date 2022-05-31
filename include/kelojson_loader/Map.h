#ifndef KELO_KELOJSON_MAP_H
#define KELO_KELOJSON_MAP_H

#include <memory>

#include <yaml-cpp/yaml.h>

#include <kelojson_loader/osm/Primitive.h>
#include <kelojson_loader/layer/Layer.h>
#include <kelojson_loader/layer/areas/AreasLayer.h>
#include <kelojson_loader/layer/zones/ZonesLayer.h>
#include <kelojson_loader/layer/topology/TopologyLayer.h>
#include <kelojson_loader/layer/occupancy/OccupancyGridLayer.h>

namespace kelo {
namespace kelojson {

class Map
{
    public:

        using Ptr = std::shared_ptr<Map>;

        using ConstPtr = std::shared_ptr<const Map>;

        Map() = default;

        virtual ~Map() = default;

        static Map::ConstPtr initialiseFromFile(const std::string& map_file);

        static Map::ConstPtr initialiseFromYAML(const YAML::Node& map_yaml);

        AreasLayer::ConstPtr getAreasLayer() const;

        ZonesLayer::ConstPtr getZonesLayer() const;

        TopologyLayer::ConstPtr getTopologyLayer() const;

        OccupancyGridLayer::ConstPtr getOccupancyGridLayer() const;

    protected:

        Layer::Map layers_;

        bool parseAllPrimitives(
                const YAML::Node& map_yaml,
                osm::Primitive::Store& osm_primitive_store);

        bool initialiseAllLayers(const osm::Primitive::Store& osm_primitive_store);

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_H
