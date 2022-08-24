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
            BFS
        };

        static const TopologyNode::ConstVec plan(
                const TopologyNode::Vec& nodes,
                const TopologyEdge::Vec& edges,
                const TopologyEdge::Matrix& adjacency_matrix,
                const TopologyNode& start,
                const TopologyNode& goal,
                const SearchType& search_type = SearchType::BFS);

    protected:

        static const TopologyNode::ConstVec backtrack(
                const TopologyNode::Vec& nodes,
                const std::vector<size_t>& parent_of,
                size_t start_id,
                size_t goal_id);

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_PLANNER_H
