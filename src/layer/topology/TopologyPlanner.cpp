#include <deque>

#include <kelojson/Print.h>
#include <kelojson/layer/topology/TopologyPlanner.h>

namespace kelo {
namespace kelojson {

const TopologyNode::ConstVec TopologyPlanner::plan(
        const TopologyNode::Vec& nodes,
        const TopologyEdge::Vec& edges,
        const TopologyEdge::Matrix& adjacency_matrix,
        const TopologyNode& start,
        const TopologyNode& goal,
        const SearchType& search_type)
{
    size_t start_id = start.getInternalId();
    size_t goal_id = goal.getInternalId();

    std::vector<bool> closed(nodes.size(), false);
    std::vector<size_t> parent_of(nodes.size(), nodes.size());
    closed[start_id] = true;

    if ( search_type == SearchType::BFS ) // Breadth first search
    {
        std::deque<size_t> fringe;

        fringe.push_back(start_id);
        bool goal_reached = false;

        while ( !fringe.empty() )
        {
            size_t curr_node_id = fringe.front();
            fringe.pop_front();
            closed[curr_node_id] = true;
            if ( curr_node_id == goal_id )
            {
                goal_reached = true;
                break;
            }

            for ( size_t i = 0; i < adjacency_matrix[curr_node_id].size(); i++ )
            {
                if ( adjacency_matrix[curr_node_id][i] != nullptr )
                {
                    if ( !closed[i] )
                    {
                        parent_of[i] = curr_node_id;
                        fringe.push_back(i);
                    }
                }
            }
        }

        return ( goal_reached )
               ? TopologyPlanner::backtrack(nodes, parent_of, start_id, goal_id)
               : TopologyNode::ConstVec();
    }
    return TopologyNode::ConstVec();
}

const TopologyNode::ConstVec TopologyPlanner::backtrack(
        const TopologyNode::Vec& nodes,
        const std::vector<size_t>& parent_of,
        size_t start_id,
        size_t goal_id)
{
    TopologyNode::ConstVec node_path;
    size_t curr_node_id = goal_id;

    while ( curr_node_id != start_id )
    {
        node_path.push_back(nodes[curr_node_id]);
        curr_node_id = parent_of[curr_node_id];
    }
    node_path.push_back(nodes[start_id]);
    std::reverse(node_path.begin(), node_path.end());

    return node_path;
}

} // namespace kelojson
} // namespace kelo
