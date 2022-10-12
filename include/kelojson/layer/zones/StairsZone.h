#ifndef KELO_KELOJSON_STAIRS_ZONE_H
#define KELO_KELOJSON_STAIRS_ZONE_H

#include <kelojson/layer/zones/PolygonZone.h>

namespace kelo {
namespace kelojson {

class StairsZone : public PolygonZone
{
    public:

        using Ptr = std::shared_ptr<StairsZone>;

        using ConstPtr = std::shared_ptr<const StairsZone>;

        using Vec = std::vector<StairsZone::Ptr>;

        using ConstVec = std::vector<StairsZone::ConstPtr>;

        StairsZone():
            PolygonZone(ZoneType::STAIRS) {}

        virtual ~StairsZone() = default;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_STAIRS_ZONE_H
