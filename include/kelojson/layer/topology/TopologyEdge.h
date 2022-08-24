#ifndef KELO_KELOJSON_TOPOLOGY_EDGE_H
#define KELO_KELOJSON_TOPOLOGY_EDGE_H

#include <geometry_common/LineSegment2D.h>

#include <kelojson/osm/WayPrimitive.h>
#include <kelojson/layer/topology/TopologyNode.h>
#include <kelojson/layer/areas/AreasLayer.h>

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
                const osm::WayPrimitive::ConstPtr& way,
                size_t internal_id,
                const TopologyNode::ConstPtr& start_node,
                const TopologyNode::ConstPtr& end_node);

        bool initialiseInterLayerAssociation(
                const std::map<LayerType, std::shared_ptr<Layer>>& layers);

        std::vector<int> getOverlappingAreaIds() const;

        bool isInArea(const Area& area) const;

        const geometry_common::LineSegment2D getLineSegment() const;

        int getPrimitiveId() const;

        size_t getInternalId() const;

        const std::string& getName() const;

        const TopologyNode::ConstPtr& getStartNode() const;

        const TopologyNode::ConstPtr& getEndNode() const;

        bool isOneWay() const;

        const std::map<LayerType, std::set<int>>& getInterlayerAssociations() const;

        const osm::Tags& getTags() const;

        friend std::ostream& operator << (
                std::ostream& out,
                const TopologyEdge& edge);

    protected:

        int primitive_id_;
        size_t internal_id_;
        std::string name_;

        TopologyNode::ConstPtr start_node_;
        TopologyNode::ConstPtr end_node_;
        bool is_oneway_{false};

        std::map<LayerType, std::set<int>> inter_layer_associations_;
        osm::Tags tags_;

        bool addAllOverlappingAreas(
                int start_node_area_id,
                int end_node_area_id,
                const AreasLayer& areas_layer);
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_EDGE_H
