#ifndef KELO_KELOJSON_LOAD_PARKING_ZONE_H
#define KELO_KELOJSON_LOAD_PARKING_ZONE_H

#include <kelojson_loader/layer/zones/PolygonZone.h>

namespace kelo {
namespace kelojson {

class LoadParkingZone : public PolygonZone
{
    public:

        using Ptr = std::shared_ptr<LoadParkingZone>;

        using ConstPtr = std::shared_ptr<const LoadParkingZone>;

        using Vec = std::vector<LoadParkingZone::Ptr>;

        using ConstVec = std::vector<LoadParkingZone::ConstPtr>;

        LoadParkingZone():
            PolygonZone(ZoneType::LOAD_PARKING) {}

        virtual ~LoadParkingZone() = default;

        bool initialise(int way_id, const osm::Primitive::Store& store) override;

        bool belongsToGroup(const std::string& group_name) const;

        const geometry_common::Pose2D getPrimaryOpeningCenterPose() const;

        float getLoadOrientation() const;

        const std::set<std::string>& getLoadParkingGroups() const;

        const geometry_common::Polyline2D& getPrimaryOpening() const;

        const geometry_common::Polyline2D& getSecondaryOpening() const;

        void write(std::ostream& out) const override;

    protected:

        float load_orientation_{0.0f};
        std::set<std::string> load_parking_groups_; // groups to which this zone belongs
        geometry_common::Polyline2D primary_opening_;
        geometry_common::Polyline2D secondary_opening_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_LOAD_PARKING_ZONE_H
