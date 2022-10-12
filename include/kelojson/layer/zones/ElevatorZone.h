#ifndef KELO_KELOJSON_ELEVATOR_ZONE_H
#define KELO_KELOJSON_ELEVATOR_ZONE_H

#include <kelojson/layer/zones/PolygonZone.h>

namespace kelo {
namespace kelojson {

class ElevatorZone : public PolygonZone
{
    public:

        using Ptr = std::shared_ptr<ElevatorZone>;

        using ConstPtr = std::shared_ptr<const ElevatorZone>;

        using Vec = std::vector<ElevatorZone::Ptr>;

        using ConstVec = std::vector<ElevatorZone::ConstPtr>;

        ElevatorZone():
            PolygonZone(ZoneType::ELEVATOR) {}

        virtual ~ElevatorZone() = default;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_ELEVATOR_ZONE_H
