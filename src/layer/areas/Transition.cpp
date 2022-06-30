#include <kelojson/Print.h>
#include <kelojson/osm/RelationPrimitive.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/areas/Transition.h>
#include <kelojson/layer/areas/Area.h>

namespace kelo {
namespace kelojson {

bool Transition::initialise(
        int relation_id,
        const osm::Primitive::Store& store,
        const Area::Map& areas)
{
    const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
            store, relation_id);
    if ( relation == nullptr )
    {
        return false;
    }

    id_ = relation->getId();

    const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
    if ( members.size() < 4 )
    {
        std::cout << Print::Err << "[Transition] Transition id: " << id_
                  << " contains < 4 members. Every transition should have at "
                  << "least four members (two areas and at least two nodes)."
                  << Print::End << std::endl;
        return false;
    }

    /* associated areas */
    const osm::RelationPrimitive::Member& area_member_1 = members[0];
    const osm::RelationPrimitive::Member& area_member_2 = members[1];
    if ( area_member_1.type != osm::PrimitiveType::WAY ||
         area_member_1.role != "areas" ||
         area_member_2.type != osm::PrimitiveType::WAY ||
         area_member_2.role != "areas" )
    {
        std::cout << Print::Err << "[Transition] Transition id: " << id_
                  << " contains first and/or second member which is not an area. "
                  << "Relation member should have role as \"areas\" and "
                  << "it should be of WAY type"
                  << Print::End << std::endl;
        return false;
    }
    if ( areas.find(area_member_1.id) == areas.end() ||
         areas.find(area_member_2.id) == areas.end() )
    {
        std::cout << Print::Err << "[Transition] Transition id: " << id_
                  << " has associated area ids as " << area_member_1.id
                  << " and " << area_member_2.id << " but one/both "
                  << "areas do not exist"
                  << Print::End << std::endl;
        return false;
    }
    Area::ConstPtr area_1 = areas.at(area_member_1.id);
    Area::ConstPtr area_2 = areas.at(area_member_2.id);
    associated_areas_ = std::make_pair(area_1, area_2);

    /* polyline */
    std::vector<int> transition_coordinate_node_ids;
    transition_coordinate_node_ids.reserve(members.size() - 2);
    for ( size_t i = 2; i < members.size(); i++ )
    {
        if ( members[i].type != osm::PrimitiveType::NODE ||
             members[i].role != "node" )
        {
            std::cout << Print::Err << "[Transition] Transition id: " << id_
                      << " contains " << i << "th member which is not a node. "
                      << "Relation member should have role as \"node\" and "
                      << "it should be of NODE type"
                      << Print::End << std::endl;
            return false;
        }
        transition_coordinate_node_ids.push_back(members[i].id);
    }
    polyline_.vertices = osm::PrimitiveUtils::getPoints(
            store, transition_coordinate_node_ids);
    if ( polyline_.size() < 2 )
    {
        std::cout << Print::Err << "[Transition] Transition id: " << id_
                  << " has < 2 points in polyline."
                  << Print::End << std::endl;
        return false;
    }

    /* door type */
    door_type_ = asDoorType(relation->getTag<std::string>("door", ""));

    /* name */
    if ( !relation->readTag<std::string>("name", name_) )
    {
        name_ = ( isDoor() )
                ? "Door_" + area_1->getName() + "_" + area_2->getName()
                : "Transition_" + area_1->getName() + "_" + area_2->getName();
    }

    return true;
}

bool Transition::isDoor() const
{
    return ( door_type_ != DoorType::NONE );
}

float Transition::width() const
{
    return ( polyline_.size() < 2 )
           ? 0.0
           : polyline_.vertices.front().distTo(polyline_.vertices.back());
}

int Transition::getId() const
{
    return id_;
}

const std::string& Transition::getName() const
{
    return name_;
}

DoorType Transition::getDoorType() const
{
    return door_type_;
}

const std::pair<Area::ConstPtr, Area::ConstPtr> Transition::getAssociatedAreas() const
{
    return std::make_pair<Area::ConstPtr, Area::ConstPtr>(
            associated_areas_.first.lock(), associated_areas_.second.lock());
}

const geometry_common::Polyline2D& Transition::getPolyline() const
{
    return polyline_;
}

std::ostream& operator << (std::ostream& out, const Transition& transition)
{
    const std::pair<Area::ConstPtr, Area::ConstPtr> associated_areas =
        transition.getAssociatedAreas();
    out << "Transition: " << std::endl
        << "    id: " << transition.id_ << std::endl
        << "    name: " << transition.name_ << std::endl
        << "    door_type: " << asString(transition.door_type_) << std::endl
        << "    associated_areas: " << associated_areas.first->getName()
        << " <--> " << associated_areas.second->getName() << std::endl
        << "    polyline: " << transition.polyline_;
    return out;
}

} // namespace kelojson
} // namespace kelo
