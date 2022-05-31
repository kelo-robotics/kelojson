#ifndef KELO_KELOJSON_RAMP_ZONE_H
#define KELO_KELOJSON_RAMP_ZONE_H

#include <kelojson_loader/layer/zones/PolygonZone.h>

namespace kelo {
namespace kelojson {

class RampZone : public PolygonZone
{
    public:

        using Ptr = std::shared_ptr<RampZone>;

        using ConstPtr = std::shared_ptr<const RampZone>;

        using Vec = std::vector<RampZone::Ptr>;

        using ConstVec = std::vector<RampZone::ConstPtr>;

        RampZone():
            PolygonZone(ZoneType::RAMP) {}

        virtual ~RampZone() = default;

        bool initialise(int way_id, const osm::Primitive::Store& store) override;

        float getInclination() const;

        const geometry_common::Polyline2D& getTop() const;

        const geometry_common::Polyline2D& getBottom() const;

        void write(std::ostream& out) const override;

    protected:

        float inclination_;
        geometry_common::Polyline2D bottom_;
        geometry_common::Polyline2D top_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_RAMP_ZONE_H
