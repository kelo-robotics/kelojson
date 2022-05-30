#ifndef KELO_KELOJSON_TOPOLOGY_NODE_H
#define KELO_KELOJSON_TOPOLOGY_NODE_H

#include <geometry_common/Point2D.h>

#include <kelojson_loader/osm/Primitive.h>
#include <kelojson_loader/layer/LayerType.h>
#include <kelojson_loader/layer/areas/Area.h>

namespace kelo {
namespace kelojson {

// forward declaration
class Layer;

class TopologyNode
{
    public:

        using Ptr = std::shared_ptr<TopologyNode>;

        using ConstPtr = std::shared_ptr<const TopologyNode>;

        using Vec = std::vector<TopologyNode::Ptr>;

        using ConstVec = std::vector<TopologyNode::ConstPtr>;

        TopologyNode() = default;

        virtual ~TopologyNode() = default;

        bool initialise(
                int node_id,
                size_t internal_id,
                const osm::Primitive::Store& store);

        bool initialiseInterLayerAssociation(
                const osm::RelationPrimitive::Ptr& relation,
                const std::map<LayerType, std::shared_ptr<Layer>>& layers);

        std::vector<int> getOverlappingAreaIds() const;

        bool isInArea(const Area& area) const;

        int getPrimitiveId() const;

        size_t getInternalId() const;

        const std::string& getName() const;

        const geometry_common::Point2D& getPosition() const;

        const std::map<LayerType, std::set<int>>& getInterlayerAssociations() const;

        friend std::ostream& operator << (std::ostream& out, const TopologyNode& node);

    protected:

        int primitive_id_;
        size_t internal_id_;
        std::string name_;
        std::map<LayerType, std::set<int>> inter_layer_associations_;
        geometry_common::Point2D position_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_NODE_H
