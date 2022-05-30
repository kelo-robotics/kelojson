#ifndef KELO_KELOJSON_OCCLUSION_ZONE_H
#define KELO_KELOJSON_OCCLUSION_ZONE_H

#include <kelojson_loader/layer/zones/PolylineZone.h>

namespace kelo {
namespace kelojson {

class OcclusionZone : public PolylineZone
{
    public:

        using Ptr = std::shared_ptr<OcclusionZone>;

        using ConstPtr = std::shared_ptr<const OcclusionZone>;

        using Vec = std::vector<OcclusionZone::Ptr>;

        using ConstVec = std::vector<OcclusionZone::ConstPtr>;

        OcclusionZone():
            PolylineZone(ZoneType::OCCLUSION) {}

        virtual ~OcclusionZone() = default;

        bool initialiseFromJunction(
                int id,
                int way_id,
                const osm::Primitive::Store& store);

        bool initialiseFromDoor(
                int id,
                int relation_id,
                const osm::Primitive::Store& store,
                bool first_side);

        /**
         * @brief Assuming an imaginary edge from the first point of polyline to
         * the last point creates a polygon. The containment test is done on
         * that polygon.
         *
         * @param point to be tested in the created polygon
         *
         * @return true is point is inside the created polygon; false otherwise
         */
        bool contains(const geometry_common::Point2D& point) const;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OCCLUSION_ZONE_H
