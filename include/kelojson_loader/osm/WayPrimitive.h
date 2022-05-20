#ifndef KELO_KELOJSON_OSM_WAY_PRIMITIVE_H
#define KELO_KELOJSON_OSM_WAY_PRIMITIVE_H

#include <kelojson_loader/osm/NodePrimitive.h>

namespace kelo {
namespace kelojson {
namespace osm {

class WayPrimitive : public Primitive
{
    public:

        WayPrimitive():
            Primitive(PrimitiveType::WAY) {}

        WayPrimitive(const WayPrimitive& way):
            Primitive(way),
            node_ids_(way.node_ids_),
            way_type_(way.way_type_) {}

        virtual ~WayPrimitive () = default;

        bool initialise(const YAML::Node& feature);

        static bool extractNodes(
                const YAML::Node& feature,
                std::vector<NodePrimitive::Ptr>& nodes);

        const std::vector<int>& getNodeIds() const;

        const std::string& getWayType() const;

        void write(std::ostream& out) const;

    private:

        std::vector<int> node_ids_;
        std::string way_type_;

};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_WAY_PRIMITIVE_H
