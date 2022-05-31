#ifndef KELO_KELOJSON_TOPOLOGY_EDGE_H
#define KELO_KELOJSON_TOPOLOGY_EDGE_H

#include <kelojson_loader/osm/Primitive.h>

namespace kelo {
namespace kelojson {

class TopologyEdge
{
    public:

        using Ptr = std::shared_ptr<TopologyEdge>;

        using ConstPtr = std::shared_ptr<const TopologyEdge>;

        using Vec = std::vector<TopologyEdge::Ptr>;

        using ConstVec = std::vector<TopologyEdge::ConstPtr>;

        using Matrix = std::vector<TopologyEdge::Vec>;

        TopologyEdge() = default;

        virtual ~TopologyEdge() = default;

        bool initialise(
                int way_id,
                const osm::Primitive::Store& store);

        int getPrimitiveId() const;

        const std::string& getName() const;

        friend std::ostream& operator << (
                std::ostream& out,
                const TopologyEdge& edge);

    protected:

        int primitive_id_;
        std::string name_;

        osm::Tags tags_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_EDGE_H
