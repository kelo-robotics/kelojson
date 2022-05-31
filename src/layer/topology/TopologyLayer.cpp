#include <geometry_common/Utils.h>
#include <kelojson_loader/Print.h>
#include <kelojson_loader/osm/PrimitiveUtils.h>
#include <kelojson_loader/layer/topology/TopologyLayer.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Pose2D;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool TopologyLayer::initialise(const osm::Primitive::Store& store)
{
    size_t internal_node_id_counter = 0;

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
                TopologyEdge::Ptr edge = std::make_shared<TopologyEdge>();
                if ( !edge->initialise(way_id, store) )
                {
                    return false;
                }
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
              << "Successfully initialised " << nodes_.size() << " nodes."
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

const TopologyNode::ConstPtr TopologyLayer::getNearestNodeInArea(
        const Pose2D& pose,
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
    const TopologyNode::ConstVec nodes_in_area = getNodesInArea(*area);
    if ( nodes_in_area.empty() )
    {
        return nullptr;
    }
    float min_dist = std::numeric_limits<float>::max();
    int nearest_node_index = -1;
    for ( size_t i = 0; i < nodes_in_area.size(); i++ )
    {
        const TopologyNode::ConstPtr& node = nodes_in_area[i];
        const TopologyNode::ConstVec adjacent_nodes = getAdjacentNodes(*node);
        bool found = false;
        for ( const TopologyNode::ConstPtr& adj_node : adjacent_nodes )
        {
            float theta = (adj_node->getPosition() - node->getPosition()).angle();
            bool is_oneway = ( adjacency_matrix_.at(
                                    adj_node->getInternalId()).at(
                                        node->getInternalId()) == nullptr );
            if ( std::fabs(GCUtils::calcShortestAngle(theta, pose.theta)) < theta_tolerance &&
                 ((only_oneway && is_oneway) || !only_oneway) )
            {
                found = true;
                break;
            }
        }
        if ( !found )
        {
            continue;
        }
        float dist = pose.distTo(node->getPosition());
        if ( dist < min_dist )
        {
            min_dist = dist;
            nearest_node_index = i;
        }
    }
    if ( nearest_node_index == -1 )
    {
        return nullptr;
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
