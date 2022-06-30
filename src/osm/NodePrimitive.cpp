#include <yaml_common/Parser2.h>

#include <kelojson/osm/NodePrimitive.h>

using kelo::geometry_common::Point2D;
using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool NodePrimitive::initialise(const YAML::Node& feature)
{
    return ( Primitive::initialise(feature) &&
             Parser::hasKey(feature, "geometry") &&
             Parser::hasKey(feature["geometry"], "coordinates") &&
             parseCoordinates(feature["geometry"]["coordinates"]) );
}

bool NodePrimitive::initialise(int id, const YAML::Node& coordinates_yaml)
{
    setId(id);
    return parseCoordinates(coordinates_yaml);
}

bool NodePrimitive::parseCoordinates(const YAML::Node& coordinates_yaml)
{
    std::vector<float> coordinates;
    if ( !Parser::read<std::vector<float>>(coordinates_yaml, coordinates) ||
         coordinates.size() != 2 )
    {
        return false;
    }
    position_ = Point2D(coordinates[0], coordinates[1]);
    return true;
}

void NodePrimitive::setPosition(geometry_common::Point2D& position)
{
    position_ = position;
}

const Point2D& NodePrimitive::getPosition() const
{
    return position_;
}

void NodePrimitive::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "position: " << position_ << ">";
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
