#ifndef KELO_KELOJSON_OSM_PRIMITIVE_UTILS_H
#define KELO_KELOJSON_OSM_PRIMITIVE_UTILS_H

#include <geometry_common/Point2D.h>

#include <kelojson_loader/osm/Primitive.h>
#include <kelojson_loader/osm/NodePrimitive.h>
#include <kelojson_loader/osm/WayPrimitive.h>
#include <kelojson_loader/osm/RelationPrimitive.h>

namespace kelo {
namespace kelojson {
namespace osm {

class PrimitiveUtils
{
    public:

        static std::vector<int> filter(
                const Primitive::Map& primitives,
                const Tags& tags,
                const std::string& type);

        static const NodePrimitive::Ptr getNode(
                const Primitive::Store& store,
                int id);

        static const WayPrimitive::Ptr getWay(
                const Primitive::Store& store,
                int id);

        static const RelationPrimitive::Ptr getRelation(
                const Primitive::Store& store,
                int id);

        static geometry_common::PointVec2D getPoints(
                const Primitive::Store& store,
                const std::vector<int>& node_ids);

};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_UTILS_H
