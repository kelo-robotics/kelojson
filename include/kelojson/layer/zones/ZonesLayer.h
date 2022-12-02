/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Dharmin Bakaraniya
 * Sushant Chavan
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#ifndef KELO_KELOJSON_ZONES_LAYER_H
#define KELO_KELOJSON_ZONES_LAYER_H

#include <kelojson/layer/Layer.h>

#include <kelojson/layer/zones/Zone.h>
#include <kelojson/layer/zones/ChargingStationZone.h>
#include <kelojson/layer/zones/WaitingLocationZone.h>
#include <kelojson/layer/zones/ForbiddenZone.h>
#include <kelojson/layer/zones/RampZone.h>
#include <kelojson/layer/zones/LoadParkingZone.h>
#include <kelojson/layer/zones/TransferStationZone.h>
#include <kelojson/layer/zones/OcclusionZone.h>
#include <kelojson/layer/zones/StairsZone.h>
#include <kelojson/layer/zones/ElevatorZone.h>

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

        // ====================================================================
        // STAIRS
        // ====================================================================

        const StairsZone::ConstVec getAllStairsZones() const;

        const StairsZone::ConstPtr getStairsZone(int id) const;

        // ====================================================================
        // ELEVATOR
        // ====================================================================

        const ElevatorZone::ConstVec getAllElevatorZones() const;

        const ElevatorZone::ConstPtr getElevatorZone(int id) const;

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
