#ifndef KELO_KELOJSON_OSM_NODE_PRIMITIVE_H
#define KELO_KELOJSON_OSM_NODE_PRIMITIVE_H

#include <geometry_common/Point2D.h>

#include <kelojson_loader/osm/Primitive.h>

namespace kelo {
namespace kelojson {
namespace osm {

class NodePrimitive : public Primitive
{
    public:

        using Ptr = std::shared_ptr<NodePrimitive>;

        using ConstPtr = std::shared_ptr<const NodePrimitive>;

        NodePrimitive():
            Primitive(PrimitiveType::NODE) {}

        NodePrimitive(const NodePrimitive& node):
            Primitive(node),
            position_(node.position_) {}

        virtual ~NodePrimitive () = default;

        bool initialise(const YAML::Node& feature);

        bool initialise(int id, const YAML::Node& coordinates);

        void setPosition(geometry_common::Point2D& position);

        const geometry_common::Point2D& getPosition() const;

        void write(std::ostream& out) const;

    private:

        geometry_common::Point2D position_;

        bool parseCoordinates(const YAML::Node& coordinates);

};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_NODE_PRIMITIVE_H
