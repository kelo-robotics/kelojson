#include <kelojson_loader/Print.h>
#include <kelojson_loader/osm/PrimitiveUtils.h>
#include <kelojson_loader/layer/zones/RampZone.h>

using kelo::geometry_common::Polyline2D;

namespace kelo {
namespace kelojson {

bool RampZone::initialise(int way_id, const osm::Primitive::Store& store)
{
    if ( !PolygonZone::initialise(way_id, store) )
    {
        return false;
    }
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    inclination_ = way->getTag("inclination", 0.0f);

    /* top and bottom edges */
    std::vector<int> all_ramp_relation_ids = osm::PrimitiveUtils::filter(
            store.at(osm::PrimitiveType::RELATION),
            osm::Tags{{"layer", "zones"}},
            "ramp_edges");
    int ramp_relation_id;
    bool found = false;
    for ( int relation_id : all_ramp_relation_ids )
    {
        const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
                store, relation_id);
        if ( relation == nullptr )
        {
            continue;
        }
        const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
        if ( members.size() >= 5 && // ramp way polygon + at least 2 top nodes + at least 2 bottom nodes
             members[0].type == osm::PrimitiveType::WAY &&
             members[0].role == "ramp" &&
             members[0].id == id_ )
        {
            found = true;
            ramp_relation_id = relation_id;
        }
    }
    if ( !found )
    {
        std::cout << Print::Err << "[RampZone] Ramp id: " << id_
                  << " is not part of a \"ramp_edges\" relation"
                  << Print::End << std::endl;
        return false;
    }

    const osm::RelationPrimitive::Ptr relation = osm::PrimitiveUtils::getRelation(
            store, ramp_relation_id);
    const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
    /* polyline */
    std::vector<int> top_node_ids, bottom_node_ids;
    for ( size_t i = 1; i < members.size(); i++ )
    {
        if ( members[i].type != osm::PrimitiveType::NODE ||
             (members[i].role != "top" && members[i].role != "bottom")  )
        {
            std::cout << Print::Err << "[RampZone] Ramp id: " << id_
                      << " contains " << i << "th member which is not a valid node. "
                      << "Relation member should have role as \"top\" or \"bottom\" "
                      << "and it should be of NODE type"
                      << Print::End << std::endl;
            return false;
        }
        if ( members[i].role == "top" )
        {
            top_node_ids.push_back(members[i].id);
        }
        else 
        {
            bottom_node_ids.push_back(members[i].id);
        }
    }
    top_.vertices = osm::PrimitiveUtils::getPoints(store, top_node_ids);
    bottom_.vertices = osm::PrimitiveUtils::getPoints(store, bottom_node_ids);
    if ( top_.size() < 2 || bottom_.size() < 2 )
    {
        std::cout << Print::Err << "[RampZone] Ramp id: " << id_
                  << " has top and/or bottom with < 2 points in polyline."
                  << Print::End << std::endl;
        return false;
    }

    return true;
}

float RampZone::getInclination() const
{
    return inclination_;
}

const Polyline2D& RampZone::getTop() const
{
    return top_;
}

const Polyline2D& RampZone::getBottom() const
{
    return bottom_;
}

void RampZone::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "polygon: " << polygon_
        << ", inclination: " << inclination_
        << ", bottom: " << bottom_
        << ", top: " << top_ << ">";
}

} // namespace kelojson
} // namespace kelo
