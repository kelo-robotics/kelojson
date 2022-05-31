#ifndef KELO_KELOJSON_CHARGING_STATION_ZONE_H
#define KELO_KELOJSON_CHARGING_STATION_ZONE_H

#include <kelojson_loader/layer/zones/NodeZone.h>

namespace kelo {
namespace kelojson {

class ChargingStationZone : public NodeZone
{
    public:

        using Ptr = std::shared_ptr<ChargingStationZone>;

        using ConstPtr = std::shared_ptr<const ChargingStationZone>;

        using Vec = std::vector<ChargingStationZone::Ptr>;

        using ConstVec = std::vector<ChargingStationZone::ConstPtr>;

        ChargingStationZone():
            NodeZone(ZoneType::CHARGING_STATION) {}

        virtual ~ChargingStationZone() = default;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_CHARGING_STATION_ZONE_H
