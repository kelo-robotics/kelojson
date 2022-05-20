#ifndef KELO_KELOJSON_MAP_H
#define KELO_KELOJSON_MAP_H

#include <memory>

#include <yaml-cpp/yaml.h>

#include <kelojson_loader/osm/Primitive.h>

namespace kelo {
namespace kelojson {

class Map
{
    public:

        using Ptr = std::shared_ptr<Map>;

        using ConstPtr = std::shared_ptr<const Map>;

        Map() = default;

        virtual ~Map() = default;

        bool initialiseFromFile(const std::string& map_file);

        bool initialiseFromYAML(const YAML::Node& map_yaml);

        bool clear();

    protected:

        osm::Primitive::Store osm_primitive_store_;

        // OSM parsing methods 
        bool parseAllPrimitives(const YAML::Node& map_yaml);

        static osm::PrimitiveType inferPrimitiveType(const YAML::Node& feature);

        // bool isOsmGeometry(const YAML::Node& feature) const;
        // bool isOsmRelation(const YAML::Node& feature) const;
        // bool isOsmNode(const YAML::Node& feature) const;
        // bool isOsmWay(const YAML::Node& feature) const;

        bool initialiseAllLayers();


};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_H
