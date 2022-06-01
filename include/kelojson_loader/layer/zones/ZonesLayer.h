#ifndef KELO_KELOJSON_ZONES_LAYER_H
#define KELO_KELOJSON_ZONES_LAYER_H

#include <kelojson_loader/layer/Layer.h>

#include <kelojson_loader/layer/zones/Zone.h>
#include <kelojson_loader/layer/zones/ChargingStationZone.h>
#include <kelojson_loader/layer/zones/WaitingLocationZone.h>
#include <kelojson_loader/layer/zones/ForbiddenZone.h>
#include <kelojson_loader/layer/zones/RampZone.h>
#include <kelojson_loader/layer/zones/LoadParkingZone.h>
#include <kelojson_loader/layer/zones/TransferStationZone.h>
#include <kelojson_loader/layer/zones/OcclusionZone.h>

namespace kelo {
namespace kelojson {

class ZonesLayer : public Layer
{
    public:

        using Ptr = std::shared_ptr<ZonesLayer>;

        using ConstPtr = std::shared_ptr<const ZonesLayer>;

        ZonesLayer():
            Layer(LayerType::ZONES) {}

        virtual ~ZonesLayer() = default;

        bool initialise(const osm::Primitive::Store& store) override;

        bool initialiseInterLayerAssociation(
                const Layer::Map& layers,
                const osm::Primitive::Store& store) override;

        // ====================================================================
        // CHARGING_STATION
        // ====================================================================

        const ChargingStationZone::ConstVec getAllChargingStationZones() const;

        const ChargingStationZone::ConstPtr getChargingStationZone(int id) const;

        const ChargingStationZone::ConstPtr getChargingStationZone(
                const std::string& name) const;

        // ====================================================================
        // WAITING_LOCATION
        // ====================================================================

        const WaitingLocationZone::ConstVec getAllWaitingLocationZones() const;

        const WaitingLocationZone::ConstPtr getWaitingLocationZone(int id) const;

        const WaitingLocationZone::ConstPtr getWaitingLocationZone(
                const std::string& name) const;

        // ====================================================================
        // FORBIDDEN
        // ====================================================================

        const ForbiddenZone::ConstVec getAllForbiddenZones() const;

        bool isInsideForbiddenZone(const geometry_common::Point2D& pt) const;

        // ====================================================================
        // RAMP
        // ====================================================================

        const RampZone::ConstVec getAllRampZones() const;

        const RampZone::ConstVec getIntersectingRampZones(
                const geometry_common::PointVec2D& pt_path) const;

        const RampZone::ConstVec getIntersectingRampZones(
                const geometry_common::Path& pose_path) const;

        // ====================================================================
        // LOAD_PARKING
        // ====================================================================

        const LoadParkingZone::ConstVec getAllLoadParkingZones() const;

        const LoadParkingZone::ConstPtr getLoadParkingZone(int id) const;

        const LoadParkingZone::ConstPtr getLoadParkingZone(const std::string& name) const;

        const std::set<std::string> getAllLoadParkingGroupNames() const;

        const LoadParkingZone::ConstVec getLoadParkingZonesInGroup(
                const std::string& group_name) const;

        // ====================================================================
        // TRANSFER_STATION
        // ====================================================================

        const TransferStationZone::ConstVec getAllTransferStationZones() const;

        const TransferStationZone::ConstPtr getTransferStationZone(int id) const;

        const TransferStationZone::ConstPtr getTransferStationZone(
                const std::string& name) const;

        const std::set<std::string> getAllTransferStationGroupNames() const;

        const TransferStationZone::ConstVec getTransferStationZonesInGroup(
                const std::string& group_name) const;

        // ====================================================================
        // OCCLUSION
        // ====================================================================

        const OcclusionZone::ConstVec getAllOcclusionZones() const;

        const OcclusionZone::ConstVec getIntersectingOcclusionZones(
                const geometry_common::PointVec2D& pt_path) const;

        const OcclusionZone::ConstVec getIntersectingOcclusionZones(
                const geometry_common::Path& pose_path) const;

        const OcclusionZone::ConstPtr getNearestOcclusionZone(
                const geometry_common::Point2D& pt) const;

        const geometry_common::PointVec2D getOcclusionPointsAlong(
                const geometry_common::PointVec2D& pt_path) const;

        const geometry_common::PointVec2D getOcclusionPointsAlong(
                const geometry_common::Path& pose_path) const;

    private:

        Zone::Store zone_store_;
        std::multimap<osm::PrimitiveType, ZoneType> primitive_type_to_zone_type_map_;

        template <typename T>
        bool initialiseZonesOfType(
                const osm::Primitive::Store& store,
                osm::PrimitiveType primitive_type,
                ZoneType zone_type,
                const std::string& characteristic,
                const std::string& type = "");

        bool autoGenerateOcclusionZones(const osm::Primitive::Store& store);

        template <typename T, typename TConstVec>
        const TConstVec getAllZonesOfType(ZoneType zone_type) const;

        template <typename T, typename TConstPtr>
        const TConstPtr getZoneOfTypeWithId(ZoneType zone_type, int id) const;

        template <typename T, typename TConstPtr>
        const TConstPtr getZoneOfTypeWithName(
                ZoneType zone_type,
                const std::string& name) const;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_ZONES_LAYER_H
