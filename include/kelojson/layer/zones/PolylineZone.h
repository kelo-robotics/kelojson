#ifndef KELO_KELOJSON_POLYLINE_ZONE_H
#define KELO_KELOJSON_POLYLINE_ZONE_H

#include <geometry_common/Polyline2D.h>

#include <kelojson/layer/zones/Zone.h>

namespace kelo {
namespace kelojson {

class PolylineZone : public Zone
{
    public:

        using Ptr = std::shared_ptr<PolylineZone>;

        using ConstPtr = std::shared_ptr<const PolylineZone>;

        virtual ~PolylineZone() = default;

        virtual bool initialise(
                int way_id,
                const osm::Primitive::Store& store) override;

        const geometry_common::Polyline2D& getPolyline() const;

        const geometry_common::Point2D meanPoint() const override;

        virtual void write(std::ostream& out) const override;

    protected:

        geometry_common::Polyline2D polyline_;

        PolylineZone(ZoneType type):
            Zone(type) {}

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_POLYLINE_ZONE_H
