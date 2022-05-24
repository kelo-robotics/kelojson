#include <regex>

#include <geometry_common/Utils.h>

#include <kelojson_loader/Print.h>
#include <kelojson_loader/osm/PrimitiveUtils.h>
#include <kelojson_loader/layer/areas/Area.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Box;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool Area::initialise(int way_id, const osm::Primitive::Store& store)
{
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( way == nullptr )
    {
        return false;
    }

    id_ = way_id;
    std::string area_type_str = way->getTag<std::string>("indoor", "unknown");
    std::transform(area_type_str.begin(), area_type_str.end(),
                   area_type_str.begin(), ::toupper);
    type_ = asAreaType(area_type_str);
    name_ = way->getTag<std::string>("name", "");

    polygon_.vertices = osm::PrimitiveUtils::getPoints(store, way->getNodeIds());
    if ( polygon_.size() < 4 ) // last pt is repeated and triangle is smallest polygon
    {
        std::cout << Print::Err << "[Area] Area id: " << id_
                  << " only contains " << polygon_.size() << " points."
                  << Print::End << std::endl;
        return false;
    }
    polygon_.vertices.pop_back();
    mean_pt_ = GCUtils::calcMeanPoint(polygon_.vertices);
    bounding_box_ = Area::calcBoundingBox(polygon_);

    return initialiseTransitions(store);
}

bool Area::initialiseTransitions(const osm::Primitive::Store& store)
{
    std::vector<int> area_transition_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "areas"}},
            "transition");
    for ( int relation_id : area_transition_ids )
    {
        const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
                store, relation_id);
        const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();

        if ( members.size() < 4 ) // transition must have atleast two areas and two nodes as members
        {
            continue;
        }

        const osm::RelationPrimitive::Member& area_member_1 = members[0];
        const osm::RelationPrimitive::Member& area_member_2 = members[1];
        if ( area_member_1.id != id_ && area_member_2.id != id_ )
        {
            continue;
        }

        int adj_area_id = ( area_member_1.id == id_ ) ? area_member_2.id : area_member_1.id;
        std::string door_type_str = relation->getTag<std::string>("door", "");
        if ( door_type_str == "yes")
        {
            door_type_str = "generic";
        }
        std::transform(door_type_str.begin(), door_type_str.end(),
                       door_type_str.begin(), ::toupper);
        std::string name = relation->getTag<std::string>("name", "");

        Area::Transition::Ptr transition = std::make_shared<Area::Transition>();
        transition->door_type = asDoorType(door_type_str);
        transition->id = relation->getId();
        transition->associated_area_ids = std::make_pair(
                area_member_1.id, area_member_2.id);
        // if ( name.empty() )
        // {
        //     std::stringstream ss;
        //     ss << (transition.doorType == doorTypes::NONE ? "AreaTransition-" : "Door-");
        //     ss << getArea(transition.associatedAreaIds.first)->name << "-";
        //     ss << getArea(transition.associatedAreaIds.second)->name;
        //     name = ss.str();
        // }
        transition->name = name;
        std::vector<int> transition_coordinate_node_ids;
        transition_coordinate_node_ids.reserve(members.size() - 2);
        for ( size_t i = 2; i < members.size(); i++ )
        {
            transition_coordinate_node_ids.push_back(members[i].id);
        }
        transition->coordinates.vertices = osm::PrimitiveUtils::getPoints(
                store, transition_coordinate_node_ids);
        if ( transition->coordinates.size() < 2 )
        {
            std::cout << Print::Err << "[Area] Area id: " << id_
                      << " is associated with transition with < 2 vertices"
                      << Print::End << std::endl;
            return false;
        }
        transitions_[adj_area_id].push_back(transition);
    }
    return true;
}

std::vector<int> Area::adjacentAreaIds() const
{
    std::vector<int> adj_area_ids;
    adj_area_ids.reserve(transitions_.size());
    for ( auto itr = transitions_.cbegin(); itr != transitions_.cend(); itr ++ )
    {
        adj_area_ids.push_back(itr->first);
    }
    return adj_area_ids;
}

const std::map<int, Area::Transition::Vec>& Area::getAllTransitions() const
{
    return transitions_;
}

const Area::Transition::Vec Area::transitionsWithArea(int adjacent_area_id) const
{
    if ( transitions_.find(adjacent_area_id) == transitions_.end() )
    {
        return Area::Transition::Vec();
    }
    return transitions_.at(adjacent_area_id);
}

const std::map<int, Area::Transition::Vec> Area::allDoorTransitions() const
{
    std::map<int, Area::Transition::Vec> door_transitions;
    for ( auto itr = transitions_.cbegin(); itr != transitions_.cend(); itr ++ )
    {
        const Area::Transition::Vec door_transitions_with_area =
            doorTransitionsWithArea(itr->first);
        if ( !door_transitions_with_area.empty() )
        {
            door_transitions[itr->first] = door_transitions_with_area;
        }
    }
    return door_transitions;
}

const Area::Transition::Vec Area::doorTransitionsWithArea(int adjacent_area_id) const
{
    Area::Transition::Vec door_transitions;
    if ( transitions_.find(adjacent_area_id) != transitions_.end() )
    {
        for ( const Area::Transition::Ptr& transition : transitions_.at(adjacent_area_id) )
        {
            if ( transition->isDoor() )
            {
                door_transitions.push_back(transition);
            }
        }
    }
    return door_transitions;
}

bool Area::isInsideBoundingBox(const Point2D& point) const
{
    return ( point.x > bounding_box_.min_x &&
             point.x < bounding_box_.max_x &&
             point.y > bounding_box_.min_y &&
             point.y < bounding_box_.max_y );
}

float Area::boundingBoxArea() const
{
    return std::fabs(bounding_box_.max_x - bounding_box_.min_x) *
           std::fabs(bounding_box_.max_y - bounding_box_.min_y);
}

bool Area::contains(const Point2D& point) const
{
    return polygon_.containsPoint(point);
}

int Area::getId() const
{
    return id_;
}

AreaType Area::getType() const
{
    return type_;
}

const std::string& Area::getName() const
{
    return name_;
}

const geometry_common::Polygon2D& Area::getPolygon() const
{
    return polygon_;
}

const geometry_common::Point2D& Area::getMeanPoint() const
{
    return mean_pt_;
}

const geometry_common::Box Area::getBoundingBox() const
{
    return bounding_box_;
}

std::ostream& operator << (std::ostream& out, const Area& area)
{
    out << "Area:" << std::endl
        << "    id: " << area.id_ << std::endl
        << "    type: " << asString(area.type_) << std::endl
        << "    name: " << area.name_ << std::endl
        << "    polygon: " << area.polygon_ << std::endl
        << "    bounding_box: " << area.bounding_box_ << std::endl;

    std::ostringstream transitions_stream;
    transitions_stream << "    transitions: [" << std::endl;
    for ( auto itr = area.transitions_.cbegin(); itr != area.transitions_.cend(); itr ++ )
    {
        transitions_stream << "  - " << itr->first << ":" << std::endl;
        for ( const Area::Transition::Ptr& transition : itr->second )
        {
            transitions_stream << "    - " << *transition << std::endl;
        }
    }
    std::string transitions_string = transitions_stream.str();
    transitions_string = std::regex_replace(
            transitions_string, std::regex("\n"), "\n    ");
    out << transitions_string << "]" << std::endl;

    return out;
}

geometry_common::Box Area::calcBoundingBox(geometry_common::Polygon2D polygon)
{
    Box box(std::numeric_limits<float>::max(), std::numeric_limits<float>::min(),
            std::numeric_limits<float>::max(), std::numeric_limits<float>::min(),
            0.0f, 0.0f);
    for ( const Point2D pt : polygon.vertices )
    {
        if ( pt.x < box.min_x )
        {
            box.min_x = pt.x;
        }
        if ( pt.x > box.max_x )
        {
            box.max_x = pt.x;
        }
        if ( pt.y < box.min_y )
        {
            box.min_y = pt.y;
        }
        if ( pt.y > box.max_y )
        {
            box.max_y = pt.y;
        }
    }
    return box;
}

// bool Area::Transition::operator < (const Transition& other) const
// {
//     return false;
// }

bool Area::Transition::isDoor() const
{
    return ( door_type != DoorType::NONE );
}

float Area::Transition::width() const
{
    return ( coordinates.size() < 2 )
           ? 0.0
           : coordinates.vertices.front().distTo(coordinates.vertices.back());
}

std::ostream& operator << (std::ostream& out, const Area::Transition& transition)
{
    out << "Transition: " << std::endl
        << "        coordinates: " << transition.coordinates << std::endl
        << "        door_type: " << asString(transition.door_type) << std::endl
        << "        id: " << transition.id << std::endl
        << "        name: " << transition.name << std::endl
        << "        associated_area_ids: " << transition.associated_area_ids.first
        << " <--> " << transition.associated_area_ids.second;
    return out;
}

} // namespace kelojson
} // namespace kelo
