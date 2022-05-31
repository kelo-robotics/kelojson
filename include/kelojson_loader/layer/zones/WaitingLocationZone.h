#ifndef KELO_KELOJSON_WAITING_LOCATION_ZONE_H
#define KELO_KELOJSON_WAITING_LOCATION_ZONE_H

#include <kelojson_loader/layer/zones/NodeZone.h>

namespace kelo {
namespace kelojson {

class WaitingLocationZone : public NodeZone
{
    public:

        using Ptr = std::shared_ptr<WaitingLocationZone>;

        using ConstPtr = std::shared_ptr<const WaitingLocationZone>;

        using Vec = std::vector<WaitingLocationZone::Ptr>;

        using ConstVec = std::vector<WaitingLocationZone::ConstPtr>;

        WaitingLocationZone():
            NodeZone(ZoneType::WAITING_LOCATION) {}

        virtual ~WaitingLocationZone() = default;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_WAITING_LOCATION_ZONE_H
