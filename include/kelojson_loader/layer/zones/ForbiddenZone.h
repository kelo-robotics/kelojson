#ifndef KELO_KELOJSON_FORBIDDEN_ZONE_H
#define KELO_KELOJSON_FORBIDDEN_ZONE_H

#include <kelojson_loader/layer/zones/PolygonZone.h>

namespace kelo {
namespace kelojson {

class ForbiddenZone : public PolygonZone
{
    public:

        using Ptr = std::shared_ptr<ForbiddenZone>;

        using ConstPtr = std::shared_ptr<const ForbiddenZone>;

        using Vec = std::vector<ForbiddenZone::Ptr>;

        using ConstVec = std::vector<ForbiddenZone::ConstPtr>;

        ForbiddenZone():
            PolygonZone(ZoneType::FORBIDDEN) {}

        virtual ~ForbiddenZone() = default;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_FORBIDDEN_ZONE_H
