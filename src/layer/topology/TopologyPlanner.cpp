#include <queue>

#include <kelojson/Print.h>
#include <kelojson/layer/topology/TopologyPlanner.h>

namespace kelo {
namespace kelojson {

const TopologyNode::ConstVec TopologyPlanner::plan(
        const TopologyNode::Vec& nodes,
        const TopologyEdge::Matrix& adjacency_matrix,
        const TopologyNode& start,
        const TopologyNode& goal,
        const SearchType& search_type)
{
    size_t start_id = start.getInternalId();
    size_t goal_id = goal.getInternalId();

    std::vector<bool> closed(nodes.size(), false);
    std::vector<float> f_values(nodes.size(), 0.0f);
    std::vector<size_t> parent_of(nodes.size(), nodes.size());

    std::priority_queue<Node, std::vector<Node>,
        std::function<bool(const Node&, const Node&)> > fringe(greaterNode);

    Node start_node;
    start_node.topology_node_id = start_id;
    fringe.push(start_node);
    bool goal_reached = false;
    Node current;

    while ( !fringe.empty() )
    {
        current = fringe.top();
        fringe.pop();
        size_t curr_node_id = current.topology_node_id;

        /* ignore already closed node */
        if ( closed[curr_node_id] )
        {
            continue;
        }

        closed[curr_node_id] = true;

        if ( curr_node_id == goal_id )
        {
            goal_reached = true;
            break;
        }

        /* add neighbours of current */
        for ( size_t i = 0; i < adjacency_matrix[curr_node_id].size(); i++ )
        {
            if ( adjacency_matrix[curr_node_id][i] == nullptr )
            {
                continue;
            }

            if ( closed[i] )
            {
                continue;
            }

            Node n;
            n.g = current.g;
            n.topology_node_id = i;
            /* update g, h and f values */
            switch ( search_type )
            {
                case SearchType::BFS :
                    n.g += 1.0f;
                    break;
                case SearchType::DIJKSTRA :
                    n.g += nodes[i]->getPosition().distTo(nodes[curr_node_id]->getPosition());
                    break;
            }
            n.f = n.g + n.h;

            /* ignore if n is already in fringe with lower cost */
            if ( parent_of[i] < nodes.size() && f_values[i] < n.f )
            {
                continue;
            }

            fringe.push(n);
            parent_of[i] = curr_node_id; // update parent
            f_values[i] = n.f; // update f value
        }
    }

    return ( goal_reached )
           ? TopologyPlanner::backtrack(nodes, parent_of, start_id, goal_id)
           : TopologyNode::ConstVec();
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
