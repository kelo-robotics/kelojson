#include <geometry_common/Utils.h>

#include <kelojson/Print.h>
#include <kelojson/osm/NodePrimitive.h>
#include <kelojson/osm/WayPrimitive.h>
#include <kelojson/osm/RelationPrimitive.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/areas/AreasLayer.h>
#include <kelojson/layer/areas/AreaPlanner.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::Polyline2D;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool AreasLayer::initialise(const osm::Primitive::Store& store)
{
    /* initialise all areas */
    std::vector<int> area_polygon_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::WAY),
            osm::Tags{{"layer", "areas"}},
            "Polygon");
    for ( int way_id : area_polygon_ids )
    {
        Area::Ptr area = std::make_shared<Area>();
        if ( !area->initialise(way_id, store) )
        {
            return false;
        }
        areas_[area->getId()] = area;
    }

    /* initialise all transitions */
    std::vector<int> area_transition_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "areas"}},
            "transition");
    for ( int relation_id : area_transition_ids )
    {
        Transition::Ptr transition = std::make_shared<Transition>();
        if ( !transition->initialise(relation_id, store, areas_) )
        {
            return false;
        }
        transitions_[transition->getId()] = transition;
    }

    /* set transitions const pointer to their specific area for ease of access */
    for ( auto area_itr = areas_.begin(); area_itr != areas_.end(); area_itr ++ )
    {
        int area_id = area_itr->first;
        Transition::ConstVec transitions_of_area;
        for ( auto itr = transitions_.cbegin(); itr != transitions_.cend(); itr ++ )
        {
            const std::pair<Area::ConstPtr, Area::ConstPtr>& associated_areas =
                itr->second->getAssociatedAreas();
            if ( associated_areas.first->getId() == area_id ||
                 associated_areas.second->getId() == area_id )
            {
                transitions_of_area.push_back(itr->second);
            }
        }
        area_itr->second->setTransitions(transitions_of_area);
    }

    std::cout << Print::Success << "[AreasLayer] Initialised "
              << areas_.size() << " areas and " 
              << transitions_.size() << " transitions." 
              << Print::End << std::endl;
    return true;
}

Area::ConstPtr AreasLayer::getArea(int id) const
{
    return ( areas_.find(id) == areas_.end() )
           ? nullptr
           : areas_.at(id);
}

Area::ConstPtr AreasLayer::getArea(const std::string& area_name) const
{
    int area_id;
    return ( !getAreaIdOf(area_name, area_id) )
           ? nullptr
           : areas_.at(area_id);
}

Area::ConstPtr AreasLayer::getAreaContaining(const Point2D& pt) const
{
    std::vector<int> candidate_area_ids;
    std::map<int, float> candidate_area_distances;
    for ( auto itr = areas_.cbegin(); itr != areas_.cend(); itr ++ )
    {
        if ( itr->second->isInsideBoundingBox(pt) )
        {
            candidate_area_ids.push_back(itr->first);
            candidate_area_distances[itr->first] = pt.distTo(itr->second->getMeanPoint());
        }
    }
    if ( candidate_area_ids.empty() )
    {
        return nullptr;
    }
    if ( candidate_area_ids.size() == 1 )
    {
        Area::ConstPtr area = getArea(candidate_area_ids.front());
        return ( area->contains(pt) ) ? area : nullptr;
    }

    std::sort(candidate_area_ids.begin(), candidate_area_ids.end(),
              [&candidate_area_distances](int& a, int& b)
              {
                  return candidate_area_distances[a] < candidate_area_distances[b];
              });

    for ( size_t i = 0; i < candidate_area_ids.size(); i++ )
    {
        Area::ConstPtr area = getArea(candidate_area_ids[i]);
        if ( area->contains(pt) )
        {
            return area;
        }
    }
    return nullptr;
}

bool AreasLayer::getAreaIdOf(const std::string& area_name, int& area_id) const
{
    for ( auto itr = areas_.cbegin(); itr != areas_.cend(); itr ++ )
    {
        if ( itr->second->getName() == area_name )
        {
            area_id = itr->first;
            return true;
        }
    }
    return false;
}

Transition::ConstPtr AreasLayer::getTransition(int id) const
{
    return ( transitions_.find(id) == transitions_.end() )
           ? nullptr
           : transitions_.at(id);
}

const Transition::ConstVec AreasLayer::getNearestTransitions(
        const geometry_common::Point2D& pt,
        float search_radius,
        size_t max_num_of_transitions) const
{
    if ( max_num_of_transitions == 0 )
    {
        return Transition::ConstVec();
    }

    // TODO: possible optimisation: only check transitions of area containing pt
    std::vector<int> transition_ids;
    std::map<int, float> transition_distances;
    for ( auto itr = transitions_.cbegin(); itr != transitions_.cend(); itr ++ )
    {
        const Point2D closest_pt = GCUtils::calcClosestPoint(
                itr->second->getPolyline().vertices, pt);
        float dist = closest_pt.distTo(pt);
        if ( dist < search_radius )
        {
            transition_ids.push_back(itr->first);
            transition_distances[itr->first] = dist;
        }
    }
    if ( transition_ids.empty() )
    {
        return Transition::ConstVec();
    }

    std::sort(transition_ids.begin(), transition_ids.end(),
              [&transition_distances](int& a, int& b)
              {
                  return transition_distances[a] < transition_distances[b];
              });


    Transition::ConstVec nearest_transitions;
    size_t N = std::min(transition_ids.size(), max_num_of_transitions);
    nearest_transitions.reserve(N);
    for ( size_t i = 0; i < N; i++ )
    {
        nearest_transitions.push_back(transitions_.at(transition_ids[i]));
    }
    return nearest_transitions;
}

std::vector<int> AreasLayer::computePath(int start_area_id, int goal_area_id) const
{
    return AreaPlanner::plan(areas_, start_area_id, goal_area_id);
}

std::vector<int> AreasLayer::computePath(
        const std::string& start_area_name,
        const std::string& goal_area_name) const
{
    int start_area_id, goal_area_id;
    if ( getAreaIdOf(start_area_name, start_area_id) &&
         getAreaIdOf(goal_area_name, goal_area_id) )
    {
        return computePath(start_area_id, goal_area_id);
    }

    std::cout << Print::Err << "[AreasLayer] "
              << "Could not get area id of start and/or goal area."
              << Print::End << std::endl;
    return std::vector<int>();
}

std::vector<int> AreasLayer::computePath(
        const geometry_common::Point2D& start_pt,
        const geometry_common::Point2D& goal_pt) const
{
    Area::ConstPtr start_area = getAreaContaining(start_pt);
    Area::ConstPtr goal_area = getAreaContaining(goal_pt);
    if ( start_area == nullptr || goal_area == nullptr )
    {
        std::cout << Print::Err << "[AreasLayer] "
                  << "Could not get area id of area containing start and/or goal point."
                  << Print::End << std::endl;
        return std::vector<int>();
    }
    return computePath(start_area->getId(), goal_area->getId());
}

std::string AreasLayer::getPrintablePath(const std::vector<int>& area_path) const
{
    std::string area_path_str;
    Area::ConstPtr area;
    for ( auto itr = area_path.begin(); itr != area_path.end(); itr ++ )
    {
        if ( !area_path_str.empty() )
        {
            area_path_str += " -> ";
        }
        area_path_str += getArea(*itr)->getName();
    }
    return area_path_str;
}

const Transition::ConstVec AreasLayer::getIntersectingTransitions(
        const LineSegment2D& line_segment) const
{
    Polyline2D polyline({line_segment.start, line_segment.end});
    return getIntersectingTransitions(polyline);
}

const Transition::ConstVec AreasLayer::getIntersectingTransitions(
        const Polyline2D& polyline) const
{
    if ( polyline.size() < 2 )
    {
        return Transition::ConstVec();
    }

    // TODO: possible optimisation: only check transitions of area where the
    // point is located
    Transition::ConstVec intersecting_area_transitions;
    for ( auto itr = transitions_.cbegin(); itr != transitions_.cend(); itr ++ )
    {
        if ( itr->second->getPolyline().intersects(polyline) )
        {
            intersecting_area_transitions.push_back(itr->second);
        }
    }
    return intersecting_area_transitions;
}

bool AreasLayer::contains(const Point2D& pt) const
{
    return ( getAreaContaining(pt) != nullptr );
}

const Area::Map& AreasLayer::getAllAreas() const
{
    return areas_;
}

const Transition::Map& AreasLayer::getAllTransitions() const
{
    return transitions_;
}

} // namespace kelojson
} // namespace kelo
