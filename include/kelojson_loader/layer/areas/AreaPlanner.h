#ifndef KELO_KELOJSON_AREA_PLANNER_H
#define KELO_KELOJSON_AREA_PLANNER_H

#include <kelojson_loader/layer/areas/Area.h>

namespace kelo {
namespace kelojson {

class AreaPlanner
{
    public:

        enum class SearchType
        {
            BFS
        };

        static std::vector<int> plan(
                const Area::Map& areas,
                int start_area_id,
                int goal_area_id,
                const SearchType& search_type = SearchType::BFS);

    protected:

        static std::vector<int> backtrack(
                const std::map<int, int>& parent_of,
                int start_area_id,
                int goal_area_id);

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_AREA_PLANNER_H
