#include <geometry_common/Utils.h>

#include <kelojson_loader/Print.h>
#include <kelojson_loader/osm/PrimitiveUtils.h>
#include <kelojson_loader/layer/zones/PolygonZone.h>

using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool PolygonZone::initialise(int way_id, const osm::Primitive::Store& store)
{
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( way == nullptr )
    {
        std::cout << Print::Err << "[PolygonZone] "
                  << "Could not find a way with id: " << way_id
                  << Print::End << std::endl;
        return false;
    }

    Zone::initialise(way);
    polygon_.vertices = osm::PrimitiveUtils::getPoints(store, way->getNodeIds());
    if ( polygon_.size() < 4 ) // last pt is repeated and triangle is smallest possible polygon
    {
        std::cout << Print::Err << "[PolygonZone] Zone id: " << id_
                  << " only contains " << polygon_.size() << " points."
                  << Print::End << std::endl;
        return false;
    }
    polygon_.vertices.pop_back();
    return true;
}

bool PolygonZone::contains(const geometry_common::Point2D& point) const
{
    return polygon_.containsPoint(point);
}

const geometry_common::Polygon2D& PolygonZone::getPolygon() const
{
    return polygon_;
}

const geometry_common::Point2D PolygonZone::meanPoint() const
{
    return GCUtils::calcMeanPoint(polygon_.vertices);
}

void PolygonZone::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "polygon: " << polygon_;
}

} // namespace kelojson
} // namespace kelo
