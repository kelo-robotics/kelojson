#ifndef KELO_KELOJSON_TOPOLOGY_LAYER_H
#define KELO_KELOJSON_TOPOLOGY_LAYER_H

#include <kelojson_loader/layer/Layer.h>
#include <kelojson_loader/layer/areas/AreasLayer.h>
#include <kelojson_loader/layer/topology/TopologyNode.h>
#include <kelojson_loader/layer/topology/TopologyEdge.h>

namespace kelo {
namespace kelojson {

class TopologyLayer : public Layer
{
    public:

        using Ptr = std::shared_ptr<TopologyLayer>;

        using ConstPtr = std::shared_ptr<const TopologyLayer>;

        TopologyLayer():
            Layer(LayerType::TOPOLOGY) {}

        virtual ~TopologyLayer() = default;

        bool initialise(const osm::Primitive::Store& store) override;

        bool initialiseInterLayerAssociation(
                const Layer::Map& layers,
                const osm::Primitive::Store& store) override;

        const TopologyNode::ConstPtr getNodeWithInternalId(size_t internal_id) const;

        const TopologyNode::ConstPtr getNodeWithPrimitiveId(int primitive_id) const;

        const TopologyNode::ConstVec getNodesInArea(
                const Area& area) const;

        const TopologyNode::ConstPtr getNearestNodeInArea(
                const geometry_common::Point2D& point) const;

        const TopologyNode::ConstPtr getNearestNodeInArea(
                const geometry_common::Pose2D& pose,
                bool only_oneway = false,
                float theta_tolerance = M_PI/4) const;

        const TopologyNode::ConstVec getAdjacentNodes(const TopologyNode& node) const;

        const TopologyNode::ConstVec getAllNodes() const;

        const TopologyEdge::Matrix& getAdjacencyMatrix() const;

        friend std::ostream& operator << (
                std::ostream& out,
                const TopologyLayer& layer);

    private:

        TopologyNode::Vec nodes_;
        TopologyEdge::Matrix adjacency_matrix_;
        std::map<int, size_t> primitive_id_to_internal_id_;
        AreasLayer::ConstPtr areas_layer_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_LAYER_H
