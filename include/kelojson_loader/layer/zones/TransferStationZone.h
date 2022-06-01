#ifndef KELO_KELOJSON_TRANSFER_STATION_ZONE_H
#define KELO_KELOJSON_TRANSFER_STATION_ZONE_H

#include <geometry_common/LineSegment2D.h>

#include <kelojson_loader/layer/zones/PolygonZone.h>

namespace kelo {
namespace kelojson {

class TransferStationZone : public PolygonZone
{
    public:

        using Ptr = std::shared_ptr<TransferStationZone>;

        using ConstPtr = std::shared_ptr<const TransferStationZone>;

        using Vec = std::vector<TransferStationZone::Ptr>;

        using ConstVec = std::vector<TransferStationZone::ConstPtr>;

        TransferStationZone():
            PolygonZone(ZoneType::TRANSFER_STATION) {}

        virtual ~TransferStationZone() = default;

        bool initialise(int way_id, const osm::Primitive::Store& store) override;

        bool belongsToGroup(const std::string& group_name) const;

        const geometry_common::Pose2D getOpeningCenterPose() const;

        float getOrientation() const;

        const std::set<std::string>& getTransferStationGroups() const;

        const geometry_common::LineSegment2D& getOpening() const;

        void write(std::ostream& out) const override;

    protected:

        float orientation_{0.0f};
        std::set<std::string> transfer_station_groups_; // groups to which this zone belongs
        geometry_common::LineSegment2D opening_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TRANSFER_STATION_ZONE_H
