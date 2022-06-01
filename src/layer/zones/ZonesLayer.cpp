#include <geometry_common/Utils.h>

#include <kelojson_loader/Print.h>
#include <kelojson_loader/osm/PrimitiveUtils.h>
#include <kelojson_loader/layer/areas/DoorType.h>
#include <kelojson_loader/layer/zones/ZonesLayer.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::PointVec2D;
using kelo::geometry_common::Polygon2D;
using kelo::geometry_common::Polyline2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Path;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool ZonesLayer::initialise(const osm::Primitive::Store& store)
{
    if ( !initialiseZonesOfType<ChargingStationZone>(
             store, osm::PrimitiveType::NODE, ZoneType::CHARGING_STATION, "charging_station") ||
         !initialiseZonesOfType<WaitingLocationZone>(
             store, osm::PrimitiveType::NODE, ZoneType::WAITING_LOCATION, "waiting_location") ||
         !initialiseZonesOfType<ForbiddenZone>(
             store, osm::PrimitiveType::WAY, ZoneType::FORBIDDEN, "forbidden", "Polygon") ||
         !initialiseZonesOfType<RampZone>(
             store, osm::PrimitiveType::WAY, ZoneType::RAMP, "ramp", "Polygon") ||
         !initialiseZonesOfType<LoadParkingZone>(
             store, osm::PrimitiveType::WAY, ZoneType::LOAD_PARKING, "load_parking", "Polygon") ||
         !initialiseZonesOfType<TransferStationZone>(
             store, osm::PrimitiveType::WAY, ZoneType::TRANSFER_STATION, "transfer_station", "Polygon") ||
         !initialiseZonesOfType<OcclusionZone>(
             store, osm::PrimitiveType::WAY, ZoneType::OCCLUSION, "occlusion", "LineString") ||
         !autoGenerateOcclusionZones(store) )
    {
        return false;
    }

    std::cout << Print::Success << "[ZonesLayer] "
              << "Initialised " << zone_store_.size() << " types of zones"
              << Print::End << std::endl;
    return true;
}

bool ZonesLayer::initialiseInterLayerAssociation(
        const Layer::Map& layers,
        const osm::Primitive::Store& store)
{
    if ( zone_store_.empty() )
    {
        return true;
    }

    std::vector<int> association_relation_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "zones"}},
            "association");

    if ( association_relation_ids.empty() )
    {
        return true;
    }

    for ( int relation_id : association_relation_ids )
    {
        const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
                store, relation_id);
        if ( relation == nullptr )
        {
            return false;
        }

        const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
        if ( members.size() < 2 )
        {
            std::cout << Print::Err << "[ZonesLayer] Association relation id: "
                      << relation_id << " contains < 2 members. Every association "
                      << "should have at least two members."
                      << Print::End << std::endl;
            return false;
        }

        /* associated areas */
        const osm::RelationPrimitive::Member& parent_member = members[0];
        if ( parent_member.role != "parent" )
        {
            std::cout << Print::Err << "[ZonesLayer] Association relation id: "
                      << relation_id << " contains first member which does not "
                      << "have a role as \"parent\"."
                      << Print::End << std::endl;
            return false;
        }

        auto p2z_itr_pair = primitive_type_to_zone_type_map_.equal_range(parent_member.type);
        bool found = false;
        for ( auto itr = p2z_itr_pair.first; itr != p2z_itr_pair.second; itr ++ )
        {
            Zone::Map& zones_map = zone_store_[itr->second];
            if ( zones_map.find(parent_member.id) != zones_map.end() )
            {
                found = true;
                if ( !zones_map[parent_member.id]->initialiseInterLayerAssociation(
                            relation, layers) )
                {
                    std::cout << Print::Err << "[ZonesLayer] Association relation "
                              << "id: " << relation_id << " associates with "
                              << *(zones_map[parent_member.id]) << " but could not "
                              << "initialise inter-layer association."
                              << Print::End << std::endl;
                    return false;
                }
                break;
            }
        }
        if ( !found )
        {
            std::cout << Print::Err << "[ZonesLayer] Association relation id: "
                      << relation_id << " contains first member with id: "
                      << parent_member.id << ", but could not find a zone with "
                      << "the same id."
                      << Print::End << std::endl;
            return false;
        }
    }
    return true;
}

const ChargingStationZone::ConstVec ZonesLayer::getAllChargingStationZones() const
{
    return getAllZonesOfType<ChargingStationZone, ChargingStationZone::ConstVec>(
            ZoneType::CHARGING_STATION);
}

const ChargingStationZone::ConstPtr ZonesLayer::getChargingStationZone(int id) const
{
    return getZoneOfTypeWithId<ChargingStationZone, ChargingStationZone::ConstPtr>(
            ZoneType::CHARGING_STATION, id);
}

const ChargingStationZone::ConstPtr ZonesLayer::getChargingStationZone(
        const std::string& name) const
{
    return getZoneOfTypeWithName<ChargingStationZone, ChargingStationZone::ConstPtr>(
            ZoneType::CHARGING_STATION, name);
}

const WaitingLocationZone::ConstVec ZonesLayer::getAllWaitingLocationZones() const
{
    return getAllZonesOfType<WaitingLocationZone, WaitingLocationZone::ConstVec>(
            ZoneType::WAITING_LOCATION);
}

const WaitingLocationZone::ConstPtr ZonesLayer::getWaitingLocationZone(int id) const
{
    return getZoneOfTypeWithId<WaitingLocationZone, WaitingLocationZone::ConstPtr>(
            ZoneType::WAITING_LOCATION, id);
}

const WaitingLocationZone::ConstPtr ZonesLayer::getWaitingLocationZone(
        const std::string& name) const
{
    return getZoneOfTypeWithName<WaitingLocationZone, WaitingLocationZone::ConstPtr>(
            ZoneType::WAITING_LOCATION, name);
}

const ForbiddenZone::ConstVec ZonesLayer::getAllForbiddenZones() const
{
    return getAllZonesOfType<ForbiddenZone, ForbiddenZone::ConstVec>(ZoneType::FORBIDDEN);
}

bool ZonesLayer::isInsideForbiddenZone(const Point2D& pt) const
{
    if ( zone_store_.find(ZoneType::FORBIDDEN) == zone_store_.end() )
    {
        return false;
    }

    const Zone::Map& zones_map = zone_store_.at(ZoneType::FORBIDDEN);
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const ForbiddenZone::ConstPtr forbidden_zone =
            std::static_pointer_cast<const ForbiddenZone>(itr->second);
        if ( forbidden_zone->contains(pt) )
        {
            return true;
        }
    }
    return false;
}

const RampZone::ConstVec ZonesLayer::getAllRampZones() const
{
    return getAllZonesOfType<RampZone, RampZone::ConstVec>(ZoneType::RAMP);
}

const RampZone::ConstVec ZonesLayer::getIntersectingRampZones(
        const geometry_common::PointVec2D& pt_path) const
{
    if ( zone_store_.find(ZoneType::RAMP) == zone_store_.end() )
    {
        return RampZone::ConstVec();
    }

    const Zone::Map& zones_map = zone_store_.at(ZoneType::RAMP);
    Polyline2D path_polyline(pt_path);
    RampZone::ConstVec ramp_zones;
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const RampZone::ConstPtr ramp_zone =
            std::static_pointer_cast<const RampZone>(itr->second);
        const Polygon2D& polygon = ramp_zone->getPolygon();
        if ( polygon.intersects(path_polyline) )
        {
            ramp_zones.push_back(ramp_zone);
        }
    }
    return ramp_zones;
}

const RampZone::ConstVec ZonesLayer::getIntersectingRampZones(
        const geometry_common::Path& pose_path) const
{
    PointVec2D pt_path;
    pt_path.reserve(pose_path.size());
    for ( size_t i = 0; i < pose_path.size(); i++ )
    {
        pt_path.push_back(pose_path[i].position());
    }
    return getIntersectingRampZones(pt_path);
}

const LoadParkingZone::ConstVec ZonesLayer::getAllLoadParkingZones() const
{
    return getAllZonesOfType<LoadParkingZone, LoadParkingZone::ConstVec>(
            ZoneType::LOAD_PARKING);
}

const std::set<std::string> ZonesLayer::getAllLoadParkingGroupNames() const
{
    if ( zone_store_.find(ZoneType::LOAD_PARKING) == zone_store_.end() )
    {
        return std::set<std::string>();
    }

    std::set<std::string> all_group_names;
    const Zone::Map& zones_map = zone_store_.at(ZoneType::LOAD_PARKING);
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const LoadParkingZone::ConstPtr load_parking_zone =
            std::static_pointer_cast<const LoadParkingZone>(itr->second);
        const std::set<std::string>& group_names = load_parking_zone->getLoadParkingGroups();
        for ( const std::string& group_name : group_names )
        {
            all_group_names.insert(group_name);
        }
    }
    return all_group_names;
}

const LoadParkingZone::ConstVec ZonesLayer::getLoadParkingZonesInGroup(
        const std::string& group_name) const
{
    if ( zone_store_.find(ZoneType::LOAD_PARKING) == zone_store_.end() )
    {
        return LoadParkingZone::ConstVec();
    }

    LoadParkingZone::ConstVec load_parking_zones_in_group;
    const Zone::Map& zones_map = zone_store_.at(ZoneType::LOAD_PARKING);
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const LoadParkingZone::ConstPtr load_parking_zone =
            std::static_pointer_cast<const LoadParkingZone>(itr->second);

        if ( load_parking_zone->belongsToGroup(group_name) )
        {
            load_parking_zones_in_group.push_back(load_parking_zone);
        }
    }
    return load_parking_zones_in_group;
}

const LoadParkingZone::ConstPtr ZonesLayer::getLoadParkingZone(int id) const
{
    return getZoneOfTypeWithId<LoadParkingZone, LoadParkingZone::ConstPtr>(
            ZoneType::LOAD_PARKING, id);
}

const LoadParkingZone::ConstPtr ZonesLayer::getLoadParkingZone(
        const std::string& name) const
{
    return getZoneOfTypeWithName<LoadParkingZone, LoadParkingZone::ConstPtr>(
            ZoneType::LOAD_PARKING, name);
}

const TransferStationZone::ConstVec ZonesLayer::getAllTransferStationZones() const
{
    return getAllZonesOfType<TransferStationZone, TransferStationZone::ConstVec>(
            ZoneType::TRANSFER_STATION);
}

const TransferStationZone::ConstPtr ZonesLayer::getTransferStationZone(int id) const
{
    return getZoneOfTypeWithId<TransferStationZone, TransferStationZone::ConstPtr>(
            ZoneType::TRANSFER_STATION, id);
}

const TransferStationZone::ConstPtr ZonesLayer::getTransferStationZone(
        const std::string& name) const
{
    return getZoneOfTypeWithName<TransferStationZone, TransferStationZone::ConstPtr>(
            ZoneType::TRANSFER_STATION, name);
}

const std::set<std::string> ZonesLayer::getAllTransferStationGroupNames() const
{
    if ( zone_store_.find(ZoneType::TRANSFER_STATION) == zone_store_.end() )
    {
        return std::set<std::string>();
    }

    std::set<std::string> all_group_names;
    const Zone::Map& zones_map = zone_store_.at(ZoneType::TRANSFER_STATION);
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const TransferStationZone::ConstPtr transfer_station_zone =
            std::static_pointer_cast<const TransferStationZone>(itr->second);
        const std::set<std::string>& group_names =
            transfer_station_zone->getTransferStationGroups();
        for ( const std::string& group_name : group_names )
        {
            all_group_names.insert(group_name);
        }
    }
    return all_group_names;
}

const TransferStationZone::ConstVec ZonesLayer::getTransferStationZonesInGroup(
        const std::string& group_name) const
{
    if ( zone_store_.find(ZoneType::TRANSFER_STATION) == zone_store_.end() )
    {
        return TransferStationZone::ConstVec();
    }

    TransferStationZone::ConstVec transfer_station_zones_in_group;
    const Zone::Map& zones_map = zone_store_.at(ZoneType::TRANSFER_STATION);
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const TransferStationZone::ConstPtr transfer_station_zone =
            std::static_pointer_cast<const TransferStationZone>(itr->second);

        if ( transfer_station_zone->belongsToGroup(group_name) )
        {
            transfer_station_zones_in_group.push_back(transfer_station_zone);
        }
    }
    return transfer_station_zones_in_group;
}

const OcclusionZone::ConstVec ZonesLayer::getAllOcclusionZones() const
{
    return getAllZonesOfType<OcclusionZone, OcclusionZone::ConstVec>(ZoneType::OCCLUSION);
}

const OcclusionZone::ConstVec ZonesLayer::getIntersectingOcclusionZones(
        const PointVec2D& pt_path) const
{
    if ( zone_store_.find(ZoneType::OCCLUSION) == zone_store_.end() )
    {
        return OcclusionZone::ConstVec();
    }

    const Zone::Map& zones_map = zone_store_.at(ZoneType::OCCLUSION);
    Polyline2D path_polyline(pt_path);
    OcclusionZone::ConstVec occlusion_zones;
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const OcclusionZone::ConstPtr occlusion_zone =
            std::static_pointer_cast<const OcclusionZone>(itr->second);
        const Polyline2D& polyline = occlusion_zone->getPolyline();
        if ( polyline.intersects(path_polyline) )
        {
            occlusion_zones.push_back(occlusion_zone);
        }
    }
    return occlusion_zones;
}

const OcclusionZone::ConstVec ZonesLayer::getIntersectingOcclusionZones(
        const Path& pose_path) const
{
    PointVec2D pt_path;
    pt_path.reserve(pose_path.size());
    for ( size_t i = 0; i < pose_path.size(); i++ )
    {
        pt_path.push_back(pose_path[i].position());
    }
    return getIntersectingOcclusionZones(pt_path);
}

const OcclusionZone::ConstPtr ZonesLayer::getNearestOcclusionZone(
        const Point2D& pt) const
{
    if ( zone_store_.find(ZoneType::OCCLUSION) == zone_store_.end() )
    {
        return nullptr;
    }

    const Zone::Map& zones_map = zone_store_.at(ZoneType::OCCLUSION);
    OcclusionZone::ConstPtr nearest_occlusion_zone;
    float min_dist = std::numeric_limits<float>::max();
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const OcclusionZone::ConstPtr occlusion_zone =
            std::static_pointer_cast<const OcclusionZone>(itr->second);
        const Polyline2D& polyline = occlusion_zone->getPolyline();
        Point2D closest_pt = GCUtils::calcClosestPoint(polyline.vertices, pt);
        float dist = closest_pt.distTo(pt);
        if ( dist < min_dist )
        {
            min_dist = dist;
            nearest_occlusion_zone = occlusion_zone;
        }
    }
    return nearest_occlusion_zone;
}

const PointVec2D ZonesLayer::getOcclusionPointsAlong(const PointVec2D& pt_path) const
{
    if ( zone_store_.find(ZoneType::OCCLUSION) == zone_store_.end() )
    {
        return PointVec2D();
    }

    const Zone::Map& zones_map = zone_store_.at(ZoneType::OCCLUSION);
    Polyline2D path_polyline(pt_path);
    std::multimap<unsigned int, Point2D> intersection_pt_map;
    Pose2D intersection_pose;
    unsigned int segment_id;
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        const OcclusionZone::ConstPtr occlusion_zone =
            std::static_pointer_cast<const OcclusionZone>(itr->second);
        const Polyline2D& polyline = occlusion_zone->getPolyline();
        if ( polyline.calcClosestIntersectionPoseWith(
                    path_polyline, intersection_pose, segment_id) )
        {
            intersection_pt_map.insert(std::pair<unsigned int, Point2D>(
                        segment_id, intersection_pose.position()));
        }
    }
    PointVec2D intersection_pts;
    intersection_pts.reserve(intersection_pt_map.size());
    for ( auto itr = intersection_pt_map.cbegin(); itr != intersection_pt_map.cend(); itr ++ )
    {
        intersection_pts.push_back(itr->second);
    }
    return intersection_pts;
}

const PointVec2D ZonesLayer::getOcclusionPointsAlong(const Path& pose_path) const
{
    PointVec2D pt_path;
    pt_path.reserve(pose_path.size());
    for ( size_t i = 0; i < pose_path.size(); i++ )
    {
        pt_path.push_back(pose_path[i].position());
    }
    return getOcclusionPointsAlong(pt_path);
}

template <typename T>
bool ZonesLayer::initialiseZonesOfType(
        const osm::Primitive::Store& store,
        osm::PrimitiveType primitive_type,
        ZoneType zone_type,
        const std::string& characteristic,
        const std::string& type)
{
    osm::Tags tags{{"layer", "zones"}};
    if ( !characteristic.empty() )
    {
        tags["characteristic"] = characteristic;
    }
    std::vector<int> primitive_ids = osm::PrimitiveUtils::filter(
            store.at(primitive_type), tags, type);
    if ( primitive_ids.empty() )
    {
        return true;
    }

    Zone::Map zones;
    for ( int id : primitive_ids )
    {
        Zone::Ptr zone = std::make_shared<T>();
        if ( !zone->initialise(id, store) )
        {
            return false;
        }
        zones[zone->getId()] = zone;
    }
    zone_store_[zone_type] = zones;
    primitive_type_to_zone_type_map_.insert(
            std::pair<osm::PrimitiveType, ZoneType>(primitive_type, zone_type));
    return true;
}

bool ZonesLayer::autoGenerateOcclusionZones(const osm::Primitive::Store& store)
{
    /* since the auto generated occlusion zones are not actually mapped, they do
     * not have an ID assigned in kelojson file. This ID needs to also be
     * generated. It needs to be unique within the set of all occlusion zones.
     * Thus, the largest ID from the mapped occlusion zones is taken and the new
     * IDs are generated incrementally */
    int id = std::numeric_limits<int>::min();
    if ( zone_store_.find(ZoneType::OCCLUSION) == zone_store_.end() )
    {
        zone_store_[ZoneType::OCCLUSION] = Zone::Map();
        id = 0;
    }
    else
    {
        const Zone::Map& zone_map = zone_store_.at(ZoneType::OCCLUSION);
        for ( auto itr = zone_map.cbegin(); itr != zone_map.cend(); itr ++ )
        {
            if ( itr->second->getId() > id )
            {
                id = itr->second->getId();
            }
        }
    }
    id ++;

    Zone::Map& occlusion_zone_map = zone_store_[ZoneType::OCCLUSION];

    /* junction */
    std::vector<int> junction_way_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::WAY),
            osm::Tags{{"layer", "areas"}, {"indoor", "junction"}},
            "Polygon");
    for ( int way_id : junction_way_ids )
    {
        OcclusionZone::Ptr occlusion_zone = std::make_shared<OcclusionZone>();
        if ( !occlusion_zone->initialiseFromJunction(id++, way_id, store) )
        {
            return false;
        }
        occlusion_zone_map[occlusion_zone->getId()] = occlusion_zone;
    }

    /* door */
    std::vector<int> transition_relation_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "areas"}},
            "transition");
    for ( int relation_id : transition_relation_ids )
    {
        const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
                store, relation_id);
        if ( relation == nullptr )
        {
            continue;
        }

        /* door type */
        DoorType door_type = asDoorType(relation->getTag<std::string>("door", ""));
        if ( door_type == DoorType::NONE )
        {
            continue;
        }

        /* create two occlusion zones; one on each side of the door */
        OcclusionZone::Ptr occlusion_zone_first_side = std::make_shared<OcclusionZone>();
        OcclusionZone::Ptr occlusion_zone_second_side = std::make_shared<OcclusionZone>();
        if ( !occlusion_zone_first_side->initialiseFromDoor(
                 id++, relation_id, store, true) ||
             !occlusion_zone_second_side->initialiseFromDoor(
                 id++, relation_id, store, false) )
        {
            return false;
        }
        occlusion_zone_map[occlusion_zone_first_side->getId()] = occlusion_zone_first_side;
        occlusion_zone_map[occlusion_zone_second_side->getId()] = occlusion_zone_second_side;
    }

    return true;
}

template <typename T, typename TConstVec>
const TConstVec ZonesLayer::getAllZonesOfType(ZoneType zone_type) const
{
    if ( zone_store_.find(zone_type) == zone_store_.end() )
    {
        return TConstVec();
    }

    const Zone::Map& zones_map = zone_store_.at(zone_type);
    TConstVec zones;
    zones.reserve(zones_map.size());
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        zones.push_back(std::static_pointer_cast<const T>(itr->second));
    }
    return zones;
}

template <typename T, typename TConstPtr>
const TConstPtr ZonesLayer::getZoneOfTypeWithId(ZoneType zone_type, int id) const
{
    if ( zone_store_.find(zone_type) == zone_store_.end() )
    {
        return nullptr;
    }

    const Zone::Map& zones_map = zone_store_.at(zone_type);
    if ( zones_map.find(id) == zones_map.end() )
    {
        return nullptr;
    }

    return std::static_pointer_cast<const T>(zones_map.at(id));
}

template <typename T, typename TConstPtr>
const TConstPtr ZonesLayer::getZoneOfTypeWithName(
        ZoneType zone_type,
        const std::string& name) const
{
    if ( zone_store_.find(zone_type) == zone_store_.end() )
    {
        return nullptr;
    }

    const Zone::Map& zones_map = zone_store_.at(zone_type);
    for ( auto itr = zones_map.cbegin(); itr != zones_map.cend(); itr ++ )
    {
        if ( itr->second->getName() == name )
        {
            return std::static_pointer_cast<const T>(itr->second);
        }
    }
    return nullptr;
}

} // namespace kelojson
} // namespace kelo
