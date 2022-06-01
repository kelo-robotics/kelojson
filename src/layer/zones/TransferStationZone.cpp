#include <geometry_common/Utils.h>

#include <kelojson_loader/Print.h>
#include <kelojson_loader/osm/PrimitiveUtils.h>
#include <kelojson_loader/layer/zones/TransferStationZone.h>

using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polyline2D;
using kelo::geometry_common::LineSegment2D;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool TransferStationZone::initialise(int way_id, const osm::Primitive::Store& store)
{
    if ( !PolygonZone::initialise(way_id, store) )
    {
        return false;
    }

    /* opening edge */
    std::vector<int> all_transfer_station_opening_relation_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "zones"}},
            "transfer_station_opening");
    int transfer_station_opening_relation_id;
    bool found = false;
    for ( int relation_id : all_transfer_station_opening_relation_ids )
    {
        const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
                store, relation_id);
        if ( relation == nullptr )
        {
            continue;
        }
        const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
        if ( members.size() == 3 && // load_parking way polygon + 2 opening nodes
             members[0].type == osm::PrimitiveType::WAY &&
             members[0].role == "transfer_station_zone" &&
             members[0].id == id_ &&
             members[1].type == osm::PrimitiveType::NODE &&
             members[1].role == "opening" &&
             members[2].type == osm::PrimitiveType::NODE &&
             members[2].role == "opening" &&
             members[1].id != members[2].id )
        {
            found = true;
            transfer_station_opening_relation_id = relation_id;
        }
    }
    if ( !found )
    {
        std::cout << Print::Err << "[TransferStationZone] TransferStation id: "
                  << id_ << " is not part of a \"transfer_station_opening\" relation"
                  << Print::End << std::endl;
        return false;
    }
    const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
            store, transfer_station_opening_relation_id);
    const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
    const osm::NodePrimitive::Ptr opening_start_node = osm::PrimitiveUtils::getNode(
            store, members[1].id);
    const osm::NodePrimitive::Ptr opening_end_node = osm::PrimitiveUtils::getNode(
            store, members[2].id);
    if ( opening_start_node == nullptr || opening_end_node == nullptr )
    {
        std::cout << Print::Err << "[TransferStationZone] TransferStation id: "
                  << id_ << " is part of \"transfer_station_opening\" relation "
                  << "but \"opening\" node/s could not be found."
                  << Print::End << std::endl;
        return false;
    }
    opening_.start = opening_start_node->getPosition();
    opening_.end = opening_end_node->getPosition();

    /* orientation */
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( !way->readTag("theta", orientation_) )
    {
        /* Explicit Orientation is not defined,
         * calculate the orientation using the opening line segment */
        orientation_ = GCUtils::calcPerpendicularAngle(opening_.angle());
        // Get a point along the perpendicular at a certain displacement from center
        Point2D test_pt(opening_.center() +
                        Point2D::initFromRadialCoord(0.1f, orientation_));
        if ( polygon_.containsPoint(test_pt) )
        {
            orientation_ = GCUtils::calcReverseAngle(orientation_);
        }
    }

    /* transfer_station_groups */
    std::vector<int> all_transfer_station_group_relation_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "zones"}},
            "transfer_station_group");
    for ( int relation_id : all_transfer_station_group_relation_ids )
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
                 member.role != "station" )
            {
                std::cout << Print::Err << "[TransferStationZone] "
                          << "Found a \"transfer_station_group\" relation containing "
                          << "a member which is not of WAY type and/or does not "
                          << "have a role of type \"station\"."
                          << Print::End << std::endl;
                return false;
            }
            if ( member.id == id_ )
            {
                std::string name;
                if ( !relation->readTag<std::string>("name", name) )
                {
                    name = "transfer_station_group_" + std::to_string(relation->getId());
                }
                transfer_station_groups_.insert(name);
            }
        }
    }

    return true;
}

bool TransferStationZone::belongsToGroup(const std::string& group_name) const
{
    return ( transfer_station_groups_.find(group_name) != transfer_station_groups_.end() );
}

const geometry_common::Pose2D TransferStationZone::getOpeningCenterPose() const
{
    return Pose2D(opening_.center(), orientation_);
}

float TransferStationZone::getOrientation() const
{
    return orientation_;
}

const std::set<std::string>& TransferStationZone::getTransferStationGroups() const
{
    return transfer_station_groups_;
}

const geometry_common::LineSegment2D& TransferStationZone::getOpening() const
{
    return opening_;
}

void TransferStationZone::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "polygon: " << polygon_
        << ", orientation: " << orientation_
        << ", opening: " << opening_
        << ", transfer_station_groups: [";
    for ( const std::string& group_name : transfer_station_groups_ )
    {
        out << group_name << ", ";
    }
    out << "]>";
}

} // namespace kelojson
} // namespace kelo
