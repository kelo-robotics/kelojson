#ifndef KELO_KELOJSON_TOPOLOGY_PLANNER_H
#define KELO_KELOJSON_TOPOLOGY_PLANNER_H

#include <kelojson/layer/topology/TopologyNode.h>
#include <kelojson/layer/topology/TopologyEdge.h>

namespace kelo {
namespace kelojson {

class TopologyPlanner
{
    public:

        enum class SearchType
        {
            BFS,
            DIJKSTRA
        };

        static const TopologyNode::ConstVec plan(
                const TopologyNode::Vec& nodes,
                const TopologyEdge::Vec& edges,
                const TopologyEdge::Matrix& adjacency_matrix,
                const TopologyNode& start,
                const TopologyNode& goal,
                const SearchType& search_type = SearchType::BFS);

    protected:

        struct Node
        {
            float g{0.0f}, h{0.0f}, f{0.0f};
            size_t topology_node_id;
        };

        static const TopologyNode::ConstVec backtrack(
                const TopologyNode::Vec& nodes,
                const std::vector<size_t>& parent_of,
                size_t start_id,
                size_t goal_id);

        static bool greaterNode(const Node& n1, const Node& n2)
        {
            return ( n1.f > n2.f );
        }

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_PLANNER_H
