#include <geometry_common/Polygon2D.h>
#include <geometry_common/Utils.h>

#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/zones/OcclusionZone.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Vector2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::Polygon2D;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool OcclusionZone::initialiseFromJunction(
        int id,
        int way_id,
        const osm::Primitive::Store& store)
{
    if ( !PolylineZone::initialise(way_id, store) )
    {
        return false;
    }

    id_ = id;
    name_ = asString(getType()) + "_Junction_" + std::to_string(id_);
    return true;
}

bool OcclusionZone::initialiseFromDoor(
        int id,
        int relation_id,
        const osm::Primitive::Store& store,
        bool first_side)
{
    const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
            store, relation_id);
    if ( relation == nullptr )
    {
        return false;
    }

    id_ = id;
    primitive_type_ = osm::PrimitiveType::RELATION;
    name_ = asString(getType()) + "_Door_" + std::to_string(id_);

    const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();

    /* get node ids of transition */
    std::vector<int> door_node_ids;
    door_node_ids.reserve(members.size() - 2);
    for ( size_t i = 2; i < members.size(); i++ )
    {
        if ( members[i].type != osm::PrimitiveType::NODE ||
             members[i].role != "node" )
        {
            std::cout << Print::Err << "[OcclusionZone] Relation id: " << relation->getId()
                      << " contains " << i << "th member which is not a node. "
                      << "Relation member should have role as \"node\" and "
                      << "it should be of NODE type"
                      << Print::End << std::endl;
            return false;
        }
        door_node_ids.push_back(members[i].id);
    }

    polyline_.vertices = osm::PrimitiveUtils::getPoints(store, door_node_ids);

    if ( polyline_.size() < 2 )
    {
        std::cout << Print::Err << "[OcclusionZone] OcclusionZone id: " << id_
                  << " was being created from door transition which only contains "
                  << polyline_.size() << " points. Need at least two."
                  << Print::End << std::endl;
        return false;
    }

    /* extrude two perpendicular line segments on each side of the door transition */
    LineSegment2D door(polyline_.vertices.front(), polyline_.vertices.back());
    float perpendicular_angle = GCUtils::calcPerpendicularAngle(door.angle());
    if ( !first_side )
    {
        perpendicular_angle = GCUtils::calcReverseAngle(perpendicular_angle);
    }
    const Vector2D unit_vec = Point2D::initFromRadialCoord(1.0f, perpendicular_angle);
    const float perp_dist_from_door = 2 * door.length(); // TODO: find better heuristic
    Point2D start_perp = door.start + (unit_vec * perp_dist_from_door);
    Point2D end_perp = door.end + (unit_vec * perp_dist_from_door);
    /* add extruded points in order (start extruded -> actual transition -> end extruded) */
    polyline_.vertices.insert(polyline_.vertices.begin(), start_perp);
    polyline_.vertices.push_back(end_perp);
    return true;
}

bool OcclusionZone::contains(const geometry_common::Point2D& point) const
{
    Polygon2D polygon(polyline_);
    return polygon.containsPoint(point);
}

} // namespace kelojson
} // namespace kelo
