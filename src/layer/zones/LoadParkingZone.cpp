#include <geometry_common/Utils.h>

#include <kelojson_loader/Print.h>
#include <kelojson_loader/osm/PrimitiveUtils.h>
#include <kelojson_loader/layer/zones/LoadParkingZone.h>

using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polyline2D;
using kelo::geometry_common::LineSegment2D;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool LoadParkingZone::initialise(int way_id, const osm::Primitive::Store& store)
{
    if ( !PolygonZone::initialise(way_id, store) )
    {
        return false;
    }

    /* primary_opening and secondary_opening edges */
    std::vector<int> all_load_parking_relation_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "zones"}},
            "load_parking_openings");
    int load_parking_relation_id;
    bool found = false;
    for ( int relation_id : all_load_parking_relation_ids )
    {
        const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
                store, relation_id);
        if ( relation == nullptr )
        {
            continue;
        }
        const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
        if ( members.size() >= 3 && // load_parking way polygon + at least 2 primary_opening nodes + (optional)secondary_opening nodes
             members[0].type == osm::PrimitiveType::WAY &&
             members[0].role == "load_parking_zone" &&
             members[0].id == id_ )
        {
            found = true;
            load_parking_relation_id = relation_id;
        }
    }
    if ( !found )
    {
        std::cout << Print::Err << "[LoadParkingZone] LoadParking id: " << id_
                  << " is not part of a \"load_parking_openings\" relation"
                  << Print::End << std::endl;
        return false;
    }

    const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
            store, load_parking_relation_id);
    const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
    /* polyline */
    std::vector<int> primary_opening_node_ids, secondary_opening_node_ids;
    for ( size_t i = 1; i < members.size(); i++ )
    {
        if ( members[i].type != osm::PrimitiveType::NODE ||
             (members[i].role != "primary_opening" && members[i].role != "secondary_opening")  )
        {
            std::cout << Print::Err << "[LoadParkingZone] LoadParking id: " << id_
                      << " contains " << i << "th member which is not a valid node. "
                      << "Relation member should have role as \"primary_opening\" or \"secondary_opening\" "
                      << "and it should be of NODE type"
                      << Print::End << std::endl;
            return false;
        }
        if ( members[i].role == "primary_opening" )
        {
            primary_opening_node_ids.push_back(members[i].id);
        }
        else 
        {
            secondary_opening_node_ids.push_back(members[i].id);
        }
    }
    primary_opening_.vertices = osm::PrimitiveUtils::getPoints(store, primary_opening_node_ids);
    secondary_opening_.vertices = osm::PrimitiveUtils::getPoints(store, secondary_opening_node_ids);
    if ( primary_opening_.size() < 2 )
    {
        std::cout << Print::Err << "[LoadParkingZone] LoadParking id: " << id_
                  << " has primary_opening with < 2 points in polyline."
                  << Print::End << std::endl;
        return false;
    }
    if ( secondary_opening_.size() == 1 )
    {
        std::cout << Print::Warn << "[LoadParkingZone] LoadParking id: " << id_
                  << " has secondary_opening with 1 point in polyline. Ignoring secondary opening"
                  << Print::End << std::endl;
        secondary_opening_.vertices.clear();
    }

    /* load orientation */
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( !way->readTag("theta", load_orientation_) )
    {
        /* Explicit Load Orientation is not defined,
         * calculate the orientation using the primary opening */
        LineSegment2D primary_opening_segment(primary_opening_.vertices.front(),
                                              primary_opening_.vertices.back());
        load_orientation_ = GCUtils::calcPerpendicularAngle(primary_opening_segment.angle());
        // Get a point along the perpendicular at a certain displacement from center
        Point2D test_pt(primary_opening_segment.center() +
                        Point2D::initFromRadialCoord(0.1f, load_orientation_));
        if ( polygon_.containsPoint(test_pt) )
        {
            load_orientation_ = GCUtils::calcReverseAngle(load_orientation_);
        }
    }

    /* load_parking_groups */
    std::vector<int> all_load_parking_group_relation_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "zones"}},
            "load_parking_group");
    for ( int relation_id : all_load_parking_group_relation_ids )
    {
        const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
                store, relation_id);
        if ( relation == nullptr )
        {
            continue;
        }
        for ( const osm::RelationPrimitive::Member& member : relation->getMembers() )
        {
            if ( member.type != osm::PrimitiveType::WAY ||
                 member.role != "parking" )
            {
                std::cout << Print::Err << "[LoadParkingZone] "
                          << "Found a \"load_parking_group\" relation containing "
                          << "a member which is not of WAY type"
                          << Print::End << std::endl;
                return false;
            }
            if ( member.id == id_ )
            {
                std::string name;
                if ( !relation->readTag<std::string>("name", name) )
                {
                    name = "load_parking_group_" + std::to_string(relation->getId());
                }
                load_parking_groups_.insert(name);
            }
        }
        if ( members.size() >= 3 && // load_parking way polygon + at least 2 primary_opening nodes + (optional)secondary_opening nodes
             members[0].type == osm::PrimitiveType::WAY &&
             members[0].role == "load_parking_zone" &&
             members[0].id == id_ )
        {
            found = true;
            load_parking_relation_id = relation_id;
        }
    }

    return true;
}

bool LoadParkingZone::belongsToGroup(const std::string& group_name) const
{
    return ( load_parking_groups_.find(group_name) != load_parking_groups_.end() );
}

const Pose2D LoadParkingZone::getPrimaryOpeningCenterPose() const
{
    LineSegment2D primary_opening_segment(primary_opening_.vertices.front(),
            primary_opening_.vertices.back());
    return Pose2D(primary_opening_segment.center(), load_orientation_);
}

float LoadParkingZone::getLoadOrientation() const
{
    return load_orientation_;
}

const std::set<std::string>& LoadParkingZone::getLoadParkingGroups() const
{
    return load_parking_groups_;
}

const Polyline2D& LoadParkingZone::getPrimaryOpening() const
{
    return primary_opening_;
}

const Polyline2D& LoadParkingZone::getSecondaryOpening() const
{
    return secondary_opening_;
}

void LoadParkingZone::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "polygon: " << polygon_
        << ", load_orientation: " << load_orientation_
        << ", primary_opening: " << primary_opening_
        << ", secondary_opening: " << secondary_opening_
        << ", load_parking_groups: [";
    for ( const std::string& group_name : load_parking_groups_ )
    {
        out << group_name << ", ";
    }
    out << "]>";
}

} // namespace kelojson
} // namespace kelo
