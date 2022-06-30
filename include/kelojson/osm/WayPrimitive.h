#ifndef KELO_KELOJSON_OSM_WAY_PRIMITIVE_H
#define KELO_KELOJSON_OSM_WAY_PRIMITIVE_H

#include <kelojson/osm/NodePrimitive.h>

namespace kelo {
namespace kelojson {
namespace osm {

class WayPrimitive : public Primitive
{
    public:

        using Ptr = std::shared_ptr<WayPrimitive>;

        using ConstPtr = std::shared_ptr<const WayPrimitive>;

        WayPrimitive():
            Primitive(PrimitiveType::WAY) {}

        WayPrimitive(const WayPrimitive& way):
            Primitive(way),
            node_ids_(way.node_ids_) {}

        virtual ~WayPrimitive () = default;

        bool initialise(const YAML::Node& feature);

        static bool extractNodes(
                const YAML::Node& feature,
                std::vector<NodePrimitive::Ptr>& nodes);

        const std::vector<int>& getNodeIds() const;

        void write(std::ostream& out) const;

    private:

        std::vector<int> node_ids_;

};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_WAY_PRIMITIVE_H
