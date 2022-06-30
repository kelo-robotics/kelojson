#include <geometry_common/Utils.h>

#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/zones/PolylineZone.h>

using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool PolylineZone::initialise(int way_id, const osm::Primitive::Store& store)
{
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( way == nullptr )
    {
        std::cout << Print::Err << "[PolylineZone] "
                  << "Could not find a way with id: " << way_id
                  << Print::End << std::endl;
        return false;
    }

    Zone::initialise(way);
    polyline_.vertices = osm::PrimitiveUtils::getPoints(store, way->getNodeIds());
    if ( polyline_.size() < 2 )
    {
        std::cout << Print::Err << "[PolylineZone] Zone id: " << id_
                  << " only contains " << polyline_.size() << " points."
                  << Print::End << std::endl;
        return false;
    }
    return true;
}

const geometry_common::Polyline2D& PolylineZone::getPolyline() const
{
    return polyline_;
}

const geometry_common::Point2D PolylineZone::meanPoint() const
{
    return GCUtils::calcMeanPoint(polyline_.vertices);
}

void PolylineZone::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "polyline: " << polyline_;
}

} // namespace kelojson
} // namespace kelo
