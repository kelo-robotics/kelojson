#ifndef KELO_KELOJSON_POLYGON_ZONE_H
#define KELO_KELOJSON_POLYGON_ZONE_H

#include <geometry_common/Polygon2D.h>

#include <kelojson/layer/zones/Zone.h>

namespace kelo {
namespace kelojson {

class PolygonZone : public Zone
{
    public:

        using Ptr = std::shared_ptr<PolygonZone>;

        using ConstPtr = std::shared_ptr<const PolygonZone>;

        virtual ~PolygonZone() = default;

        virtual bool initialise(
                int way_id,
                const osm::Primitive::Store& store) override;

        bool contains(const geometry_common::Point2D& point) const;

        const geometry_common::Polygon2D& getPolygon() const;

        const geometry_common::Point2D meanPoint() const override;

        virtual void write(std::ostream& out) const override;

    protected:

        geometry_common::Polygon2D polygon_;

        PolygonZone(ZoneType type):
            Zone(type) {}

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_POLYGON_ZONE_H
