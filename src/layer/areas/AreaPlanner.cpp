#include <kelojson/Print.h>
#include <kelojson/layer/areas/AreaPlanner.h>

namespace kelo {
namespace kelojson {

std::vector<int> AreaPlanner::plan(
        const Area::Map& areas,
        int start_area_id,
        int goal_area_id,
        const SearchType& search_type)
{
    if ( areas.find(start_area_id) == areas.end() )
    {
        std::cout << Print::Err << "[AreaPlanner] "
                  << "Cannot compute path between areas. Invalid start_area_id!"
                  << Print::End << std::endl;
        return std::vector<int>();
    }

    if ( areas.find(goal_area_id) == areas.end() )
    {
        std::cout << Print::Err << "[AreaPlanner] "
                  << "Cannot compute path between areas. Invalid goal_area_id!"
                  << Print::End << std::endl;
        return std::vector<int>();
    }

    if ( search_type == SearchType::BFS ) // Breadth first search
    {
        std::list<int> fringe;
        std::map<int, bool> closed;
        std::map<int, int> parent_of;
        for ( auto itr = areas.cbegin(); itr != areas.cend(); itr ++ )
        {
            closed[itr->first] = false;
        }

        closed.at(start_area_id) = true;
        fringe.push_back(start_area_id);
        bool goal_reached = false;

        while ( !fringe.empty() )
        {
            int curr_area_id = fringe.front();
            fringe.pop_front();
            closed[curr_area_id] = true;
            if ( curr_area_id == goal_area_id )
            {
                goal_reached = true;
                break;
            }

            if ( areas.find(curr_area_id) == areas.end() )
            {
                continue;
            }

            Area::ConstPtr area = areas.at(curr_area_id);
            std::vector<int> adjacent_area_ids = area->getAdjacentAreaIds();
            for ( int adj_area_id : adjacent_area_ids )
            {
                if ( !closed[adj_area_id] )
                {
                    parent_of[adj_area_id] = curr_area_id;
                    fringe.push_back(adj_area_id);
                }
            }
        }

        return ( goal_reached )
               ? AreaPlanner::backtrack(parent_of, start_area_id, goal_area_id)
               : std::vector<int>();
    }
    return std::vector<int>();
}

std::vector<int> AreaPlanner::backtrack(
        const std::map<int, int>& parent_of,
        int start_area_id,
        int goal_area_id)
{
    std::vector<int> area_path;
    int curr_area_id = goal_area_id;

    while ( curr_area_id != start_area_id )
    {
        area_path.push_back(curr_area_id);
        curr_area_id = parent_of.at(curr_area_id);
    }
    area_path.push_back(start_area_id);
    std::reverse(area_path.begin(), area_path.end());

    return area_path;
}

} // namespace kelojson
} // namespace kelo
