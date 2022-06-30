#ifndef KELO_KELOJSON_OSM_PRIMITIVE_FACTORY_H
#define KELO_KELOJSON_OSM_PRIMITIVE_FACTORY_H

#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

#include <kelojson/osm/Primitive.h>

namespace kelo {
namespace kelojson {
namespace osm {

class PrimitiveFactory
{
    public:

        static bool createPrimitive(
                const YAML::Node& feature,
                Primitive::Store& store);

    protected:

        static PrimitiveType inferPrimitiveType(const YAML::Node& feature);

};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_FACTORY_H
