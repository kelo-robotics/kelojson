#include <deque>

#include <geometry_common/Utils.h>

#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/topology/TopologyPlanner.h>
#include <kelojson/layer/topology/TopologyLayer.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::Pose2D;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool TopologyLayer::initialise(const osm::Primitive::Store& store)
{
    size_t internal_node_id_counter = 0;
    size_t internal_edge_id_counter = 0;

    std::vector<int> topology_way_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::WAY),
            osm::Tags{{"layer", "topology"}, {"highway", "robot_path"}},
            "LineString");

    /* create all nodes */
    for ( int way_id : topology_way_ids )
    {
        const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
        if ( way == nullptr )
        {
            std::cout << Print::Err << "[TopologyLayer] Way id: " << way_id
                      << " does not exist." << Print::End << std::endl;
            return false;
        }

        const std::vector<int>& node_ids = way->getNodeIds();
        if ( node_ids.size() < 2 )
        {
            std::cout << Print::Err << "[TopologyLayer] Way id: " << way_id
                      << " only contains " << node_ids.size() << " point."
                      << Print::End << std::endl;
            return false;
        }

        nodes_.reserve(nodes_.size() + node_ids.size());
        for ( int node_id : node_ids )
        {
            if ( primitive_id_to_internal_id_.find(node_id) !=
                 primitive_id_to_internal_id_.end() )
            {
                continue;
            }
            size_t internal_node_id = internal_node_id_counter++;
            TopologyNode::Ptr node = std::make_shared<TopologyNode>();
            if ( !node->initialise(node_id, internal_node_id, store) )
            {
                return false;
            }
            nodes_.push_back(node);
            primitive_id_to_internal_id_[node_id] = internal_node_id;
        }

    }

    /* fill adjacency matrix */
    adjacency_matrix_.clear();
    adjacency_matrix_.resize(nodes_.size(), TopologyEdge::Vec(nodes_.size(), nullptr));
    for ( int way_id : topology_way_ids )
    {
        const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
        if ( way == nullptr )
        {
            std::cout << Print::Err << "[TopologyLayer] Way id: " << way_id
                      << " does not exist." << Print::End << std::endl;
            return false;
        }

        bool is_oneway = way->getTag<bool>("oneway", false);

        const std::vector<int>& node_ids = way->getNodeIds();
        for ( size_t i = 0; i < node_ids.size(); i++ )
        {
            size_t curr_id = primitive_id_to_internal_id_.at(node_ids[i]);
            if ( i + 1 < node_ids.size() )
            {
                size_t next_id = primitive_id_to_internal_id_.at(node_ids[i+1]);
                size_t internal_edge_id = internal_edge_id_counter++;
                TopologyEdge::Ptr edge = std::make_shared<TopologyEdge>();
                if ( !edge->initialise(way, internal_edge_id, nodes_[curr_id], nodes_[next_id]) )
                {
                    return false;
                }
                edges_.push_back(edge);
                adjacency_matrix_[curr_id][next_id] = edge;
            }

            if ( !is_oneway && i > 0 )
            {
                size_t prev_id = primitive_id_to_internal_id_.at(node_ids[i-1]);
                adjacency_matrix_[curr_id][prev_id] = adjacency_matrix_[prev_id][curr_id];
            }
        }
    }

    std::cout << Print::Success << "[TopologyLayer] "
              << "Successfully initialised " << nodes_.size() << " nodes and "
              << edges_.size() << " edges."
              << Print::End << std::endl;
    return true;
}

bool TopologyLayer::initialiseInterLayerAssociation(
        const Layer::Map& layers,
        const osm::Primitive::Store& store)
{
    if ( nodes_.empty() )
    {
        return true;
    }

    std::vector<int> association_relation_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "topology"}},
            "association");

    if ( association_relation_ids.empty() )
    {
        return true;
    }

    for ( int relation_id : association_relation_ids )
    {
        const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
                store, relation_id);
        if ( relation == nullptr )
        {
            std::cout << Print::Err << "[TopologyLayer] Association relation id: "
                      << relation_id << " does not exist."<< Print::End << std::endl;
            return false;
        }

        const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
        if ( members.size() < 2 )
        {
            std::cout << Print::Err << "[TopologyLayer] Association relation id: "
                      << relation_id << " contains < 2 members. Every association "
                      << "should have at least two members."
                      << Print::End << std::endl;
            return false;
        }

        /* associated areas */
        const osm::RelationPrimitive::Member& parent_member = members[0];
        if ( parent_member.type != osm::PrimitiveType::NODE ||
             parent_member.role != "parent" )
        {
            std::cout << Print::Err << "[ZonesLayer] Association relation id: "
                      << relation_id << " contains first member which does not "
                      << "have a role as \"parent\" and/or is not of type NODE."
                      << Print::End << std::endl;
            return false;
        }

        if ( primitive_id_to_internal_id_.find(parent_member.id) ==
             primitive_id_to_internal_id_.end() )
        {
            std::cout << Print::Err << "[TopologyLayer] Association relation id: "
                      << relation_id << " contains first member with id: "
                      << parent_member.id << ", but could not find a "
                      << "TopologyNode with the same id."
                      << Print::End << std::endl;
            return false;
        }

        size_t internal_id = primitive_id_to_internal_id_.at(parent_member.id);
        if ( !nodes_[internal_id]->initialiseInterLayerAssociation(
                    relation, layers) )
        {
            std::cout << Print::Err << "[TopologyLayer] Association relation "
                      << "id: " << relation_id << " associates with "
                      << *(nodes_[internal_id]) << " but could not "
                      << "initialise inter-layer association."
                      << Print::End << std::endl;
            return false;
        }
    }

    for ( TopologyEdge::Ptr& edge : edges_ )
    {
        if ( !edge->initialiseInterLayerAssociation(layers) )
        {
            std::cout << Print::Err << "[TopologyLayer] Could not initialise "
                      << "inter-layer association for edge " << *edge
                      << Print::End << std::endl;
            return false;
        }
    }

    if ( layers.find(LayerType::AREAS) != layers.end() )
    {
        areas_layer_ = std::static_pointer_cast<AreasLayer>(layers.at(LayerType::AREAS));
    }
    return true;
}

const TopologyNode::ConstPtr TopologyLayer::getNodeWithInternalId(
        size_t internal_id) const
{
    return ( internal_id >= nodes_.size() )
           ? nullptr
           : nodes_[internal_id];
}

const TopologyNode::ConstPtr TopologyLayer::getNodeWithPrimitiveId(
        int primitive_id) const
{
    return ( primitive_id_to_internal_id_.find(primitive_id) ==
             primitive_id_to_internal_id_.end() )
           ? nullptr
           : nodes_[primitive_id_to_internal_id_.at(primitive_id)];
}

const TopologyNode::ConstVec TopologyLayer::getNodesInArea(
        const Area& area) const
{
    TopologyNode::ConstVec nodes_in_area;
    for ( const TopologyNode::Ptr& node : nodes_ )
    {
        if ( node->isInArea(area) )
        {
            nodes_in_area.push_back(node);
        }
    }
    return nodes_in_area;
}

const TopologyNode::ConstPtr TopologyLayer::getNearestNodeInArea(
        const Point2D& point) const
{
    if ( areas_layer_ == nullptr )
    {
        return nullptr;
    }
    const Area::ConstPtr area = areas_layer_->getAreaContaining(point);
    if ( area == nullptr )
    {
        return nullptr;
    }
    const TopologyNode::ConstVec nodes_in_area = getNodesInArea(*area);
    if ( nodes_in_area.empty() )
    {
        return nullptr;
    }
    float min_dist = std::numeric_limits<float>::max();
    size_t nearest_node_index = 0;
    for ( size_t i = 0; i < nodes_in_area.size(); i++ )
    {
        float dist = nodes_in_area[i]->getPosition().distTo(point);
        if ( dist < min_dist )
        {
            min_dist = dist;
            nearest_node_index = i;
        }
    }
    return nodes_in_area[nearest_node_index];
}

const TopologyNode::ConstVec TopologyLayer::getAdjacentNodes(
        const TopologyNode& node) const
{
    size_t internal_id = node.getInternalId();
    std::vector<size_t> adjacent_node_ids;
    for ( size_t i = 0; i < adjacency_matrix_[internal_id].size(); i++ )
    {
        if ( adjacency_matrix_[internal_id][i] != nullptr )
        {
            adjacent_node_ids.push_back(i);
        }
    }
    TopologyNode::ConstVec adjacent_nodes;
    adjacent_nodes.reserve(adjacent_node_ids.size());
    for ( size_t node_id : adjacent_node_ids )
    {
        adjacent_nodes.push_back(nodes_[node_id]);
    }
    return adjacent_nodes;
}

const TopologyNode::ConstVec TopologyLayer::getAllNodes() const
{
    TopologyNode::ConstVec nodes;
    nodes.reserve(nodes_.size());
    for ( size_t i = 0; i < nodes_.size(); i++ )
    {
        nodes.push_back(nodes_[i]);
    }
    return nodes;
}

const TopologyNode::ConstVec TopologyLayer::computePath(
        const TopologyNode& start,
        const TopologyNode& goal) const
{
    return TopologyPlanner::plan(nodes_, edges_, adjacency_matrix_, start, goal,
            TopologyPlanner::SearchType::BFS);
}

const TopologyEdge::ConstPtr TopologyLayer::getEdgeWithInternalId(
        size_t internal_id) const
{
    return ( internal_id >= edges_.size() )
           ? nullptr
           : edges_[internal_id];
}

const TopologyEdge::ConstVec TopologyLayer::getEdgesInArea(const Area& area) const
{
    TopologyEdge::ConstVec edges_in_area;
    for ( const TopologyEdge::Ptr& edge : edges_ )
    {
        if ( edge->isInArea(area) )
        {
            edges_in_area.push_back(edge);
        }
    }
    return edges_in_area;
}

const TopologyEdge::ConstPtr TopologyLayer::getNearestEdgeInArea(
        const geometry_common::Point2D& point) const
{
    if ( areas_layer_ == nullptr )
    {
        return nullptr;
    }
    const Area::ConstPtr area = areas_layer_->getAreaContaining(point);
    if ( area == nullptr )
    {
        return nullptr;
    }
    const TopologyEdge::ConstVec edges_in_area = getEdgesInArea(*area);
    if ( edges_in_area.empty() )
    {
        return nullptr;
    }
    float min_dist = std::numeric_limits<float>::max();
    size_t nearest_edge_index = 0;
    for ( size_t i = 0; i < edges_in_area.size(); i++ )
    {
        float dist = edges_in_area[i]->getLineSegment().minDistTo(point);
        if ( dist < min_dist )
        {
            min_dist = dist;
            nearest_edge_index = i;
        }
    }
    return edges_in_area[nearest_edge_index];
}

const TopologyEdge::ConstPtr TopologyLayer::getNearestEdgeInArea(
        const geometry_common::Pose2D& pose,
        bool only_oneway,
        float theta_tolerance) const
{
    if ( areas_layer_ == nullptr )
    {
        return nullptr;
    }
    const Area::ConstPtr area = areas_layer_->getAreaContaining(pose.position());
    if ( area == nullptr )
    {
        return nullptr;
    }
    const TopologyEdge::ConstVec edges_in_area = getEdgesInArea(*area);
    if ( edges_in_area.empty() )
    {
        return nullptr;
    }
    float min_dist = std::numeric_limits<float>::max();
    int nearest_edge_index = -1;
    for ( size_t i = 0; i < edges_in_area.size(); i++ )
    {
        const TopologyEdge::ConstPtr& edge = edges_in_area[i];
        const LineSegment2D segment = edge->getLineSegment();
        float theta = segment.angle();
        bool is_oneway = edge->isOneWay();
        if ( std::fabs(GCUtils::calcShortestAngle(theta, pose.theta)) < theta_tolerance &&
             ((only_oneway && is_oneway) || !only_oneway) )
        {
            float dist = segment.minDistTo(pose.position());
            if ( dist < min_dist )
            {
                min_dist = dist;
                nearest_edge_index = i;
            }
        }
    }
    if ( nearest_edge_index == -1 )
    {
        return nullptr;
    }
    return edges_in_area[nearest_edge_index];
}

const TopologyEdge::ConstVec TopologyLayer::getAllEdges() const
{
    TopologyEdge::ConstVec edges;
    edges.reserve(edges_.size());
    for ( size_t i = 0; i < edges_.size(); i++ )
    {
        edges.push_back(edges_[i]);
    }
    return edges;
}

const TopologyEdge::Matrix& TopologyLayer::getAdjacencyMatrix() const
{
    return adjacency_matrix_;
}

std::ostream& operator << (std::ostream& out, const TopologyLayer& layer)
{
    out << "TopologyLayer:" << std::endl
        << "  Nodes:" << std::endl;
    for ( size_t i = 0; i < layer.nodes_.size(); i++ )
    {
        out << "    " << *(layer.nodes_[i]) << std::endl;
    }

    out << "  Edges:" << std::endl;
    for ( size_t i = 0; i < layer.edges_.size(); i++ )
    {
        out << "    " << *(layer.edges_[i]) << std::endl;
    }

    out << "  Adjacency matrix:" << std::endl;
    /* print adjacency matrix */
    for ( size_t i = 0; i < layer.nodes_.size(); i++ )
    {
        out << "    ";
        for ( size_t j = 0; j < layer.nodes_.size(); j++ )
        {
            out << ( layer.adjacency_matrix_[i][j] != nullptr ) << " ";
        }
        out << std::endl;
    }
    out << std::endl;
    return out;
}

} // namespace kelojson
} // namespace kelo
