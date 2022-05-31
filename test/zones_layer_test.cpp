#include <gtest/gtest.h>

#include <kelojson_loader/Map.h>
#include <kelojson_loader/layer/zones/ZonesLayer.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using namespace kelo::kelojson;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polyline2D;
using kelo::geometry_common::PointVec2D;

class ZonesLayerFixture : public ::testing::Test
{
    public:
        void SetUp()
        {
            std::string kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
            kelojson_map = Map::initialiseFromFile(kelojson_map_file);
            ASSERT_NE(kelojson_map, nullptr);

            areas_layer = kelojson_map->getAreasLayer();
            ASSERT_NE(areas_layer, nullptr);

            area = areas_layer->getArea(-101782);
            ASSERT_NE(area, nullptr);
            EXPECT_EQ(area->getName(), "Room1");

            zones_layer = kelojson_map->getZonesLayer();
            ASSERT_NE(zones_layer, nullptr);
        }

    protected:
        Map::ConstPtr kelojson_map;
        AreasLayer::ConstPtr areas_layer;
        ZonesLayer::ConstPtr zones_layer;
        Area::ConstPtr area;
};

TEST_F(ZonesLayerFixture, simpleCreation)
{
}

TEST_F(ZonesLayerFixture, getAllChargingStationZones)
{
    const ChargingStationZone::ConstVec charging_stations =
        zones_layer->getAllChargingStationZones();
    ASSERT_EQ(charging_stations.size(), 1u);
    EXPECT_EQ(charging_stations.front()->getId(), -123214);
}

TEST_F(ZonesLayerFixture, getChargingStationZone)
{
    const ChargingStationZone::ConstPtr charging_station =
        zones_layer->getChargingStationZone(-123214);
    ASSERT_NE(charging_station, nullptr);
}

TEST_F(ZonesLayerFixture, getAllWaitingLocationZones)
{
    const WaitingLocationZone::ConstVec waiting_locations =
        zones_layer->getAllWaitingLocationZones();
    ASSERT_EQ(waiting_locations.size(), 1u);
    EXPECT_EQ(waiting_locations.front()->getId(), -123210);
}

TEST_F(ZonesLayerFixture, getAllForbiddenZones)
{
    const ForbiddenZone::ConstVec forbidden_zones = zones_layer->getAllForbiddenZones();
    ASSERT_EQ(forbidden_zones.size(), 1u);
    EXPECT_EQ(forbidden_zones.front()->getId(), -101954);
}

TEST_F(ZonesLayerFixture, isInsideForbiddenZone)
{
    EXPECT_TRUE(zones_layer->isInsideForbiddenZone(Point2D(5.4f, 1.5f)));
    EXPECT_FALSE(zones_layer->isInsideForbiddenZone(Point2D(0.0f, 0.0f)));
}

TEST_F(ZonesLayerFixture, getAllRampZones)
{
    const RampZone::ConstVec ramps = zones_layer->getAllRampZones();
    ASSERT_EQ(ramps.size(), 1u);
    EXPECT_EQ(ramps.front()->getId(), -101967);
}

TEST_F(ZonesLayerFixture, getIntersectingRampZones)
{
    PointVec2D pt_path{
        Point2D(3.5f, 1.5f),
        Point2D(2.5f, 1.5f),
        Point2D(2.0f, 1.5f)
    };
    const RampZone::ConstVec ramp_zones =
        zones_layer->getIntersectingRampZones(pt_path);
    EXPECT_EQ(ramp_zones.size(), 1u);
    EXPECT_EQ(ramp_zones.front()->getId(), -101967);
}

TEST_F(ZonesLayerFixture, getAllLoadParkingZones)
{
    const LoadParkingZone::ConstVec load_parking_zones = zones_layer->getAllLoadParkingZones();
    std::vector<int> true_load_parking_zone_ids{-101838, -101965};
    std::vector<int> load_parking_zone_ids;
    load_parking_zone_ids.reserve(load_parking_zones.size());
    for ( const LoadParkingZone::ConstPtr& load_parking_zone : load_parking_zones )
    {
        load_parking_zone_ids.push_back(load_parking_zone->getId());
    }
    EXPECT_EQ(load_parking_zone_ids.size(), true_load_parking_zone_ids.size());
    for ( size_t i = 0; i < true_load_parking_zone_ids.size(); i++ )
    {
        EXPECT_NE(std::find(load_parking_zone_ids.begin(),
                            load_parking_zone_ids.end(),
                            true_load_parking_zone_ids[i]),
                  load_parking_zone_ids.end());
    }
}

TEST_F(ZonesLayerFixture, getLoadParkingZone)
{
    const LoadParkingZone::ConstPtr load_parking_zone =
        zones_layer->getLoadParkingZone("room1_load_parking_1");
    EXPECT_NE(load_parking_zone, nullptr);
    EXPECT_EQ(load_parking_zone->getId(), -101838);
    EXPECT_NEAR(load_parking_zone->getLoadOrientation(), M_PI/2, 5e-2f);
    const Polyline2D& primary_opening = load_parking_zone->getPrimaryOpening();
    EXPECT_EQ(primary_opening.size(), 2u);
    const Polyline2D& secondary_opening = load_parking_zone->getSecondaryOpening();
    EXPECT_EQ(secondary_opening.size(), 2u);
    const std::set<std::string>& groups = load_parking_zone->getLoadParkingGroups();
    EXPECT_EQ(groups.size(), 1u);
    EXPECT_NE(groups.find("room1_load_parking"), groups.end());
    EXPECT_TRUE(load_parking_zone->belongsToGroup("room1_load_parking"));
    EXPECT_EQ(load_parking_zone->getPrimaryOpeningCenterPose(), Pose2D(0.924, -0.718, 1.582));

    const LoadParkingZone::ConstPtr load_parking_zone_2 =
        zones_layer->getLoadParkingZone(-101965);
    EXPECT_NE(load_parking_zone_2, nullptr);
    EXPECT_EQ(load_parking_zone_2->getName(), "room2_load_parking_1");
    EXPECT_NEAR(load_parking_zone_2->getLoadOrientation(), 1.6f, 1e-3f);
    const Polyline2D& primary_opening_2 = load_parking_zone_2->getPrimaryOpening();
    EXPECT_EQ(primary_opening_2.size(), 2u);
    const Polyline2D& secondary_opening_2 = load_parking_zone_2->getSecondaryOpening();
    EXPECT_EQ(secondary_opening_2.size(), 0u);
    const std::set<std::string>& groups_2 = load_parking_zone_2->getLoadParkingGroups();
    EXPECT_EQ(groups_2.size(), 0u);
    EXPECT_FALSE(load_parking_zone_2->belongsToGroup("room1_load_parking"));
    EXPECT_EQ(load_parking_zone_2->getPrimaryOpeningCenterPose(), Pose2D(3.968, -0.735, 1.6));
}

TEST_F(ZonesLayerFixture, loadParkingGroup)
{
    const std::set<std::string> all_group_names = zones_layer->getAllLoadParkingGroupNames();
    EXPECT_EQ(all_group_names.size(), 1u);
    EXPECT_NE(all_group_names.find("room1_load_parking"), all_group_names.end());

    const LoadParkingZone::ConstVec load_parking_zones_in_group =
        zones_layer->getLoadParkingZonesInGroup("room1_load_parking");
    EXPECT_EQ(load_parking_zones_in_group.size(), 1u);
    EXPECT_EQ(load_parking_zones_in_group.front()->getId(), -101838);
}

TEST_F(ZonesLayerFixture, getAllOcclusionZones)
{
    const OcclusionZone::ConstVec occlusion_zones = zones_layer->getAllOcclusionZones();
    EXPECT_EQ(occlusion_zones.size(), 6u);
}

TEST_F(ZonesLayerFixture, getIntersectingOcclusionZones)
{
    PointVec2D pt_path{
        Point2D(0.5f, 1.5f),
        Point2D(-0.5f, 1.5f),
        Point2D(-0.5f, 2.5f),
        Point2D(0.5f, 2.5f)
    };
    const OcclusionZone::ConstVec occlusion_zones =
        zones_layer->getIntersectingOcclusionZones(pt_path);
    EXPECT_EQ(occlusion_zones.size(), 2u);
}

TEST_F(ZonesLayerFixture, getNearestOcclusionZone)
{
    const OcclusionZone::ConstPtr occlusion_zone =
        zones_layer->getNearestOcclusionZone(Point2D(5.5f, -0.5f));
    EXPECT_EQ(occlusion_zone->getId(), -101969);
}

TEST_F(ZonesLayerFixture, getOcclusionPointsAlong)
{
    PointVec2D pt_path{
        Point2D(0.5f, 1.5f),
        Point2D(-0.5f, 1.5f),
        Point2D(-0.5f, 2.5f),
        Point2D(0.5f, 2.5f)
    };
    const PointVec2D occlusion_pts = zones_layer->getOcclusionPointsAlong(pt_path);
    EXPECT_EQ(occlusion_pts.size(), 2u);
    EXPECT_EQ(occlusion_pts.front(), Point2D(-0.017f, 1.5f));
    EXPECT_EQ(occlusion_pts.back(), Point2D(-0.5f, 1.954f));
}

TEST_F(ZonesLayerFixture, interLayerAssociation)
{
    const ChargingStationZone::ConstPtr charging_station =
        zones_layer->getChargingStationZone(-123214);
    ASSERT_NE(charging_station, nullptr);
    const std::map<LayerType, std::set<int>>& inter_layer_associations =
        charging_station->getInterlayerAssociations();
    EXPECT_NE(inter_layer_associations.find(LayerType::AREAS), inter_layer_associations.end());


    std::vector<int> area_ids = charging_station->getOverlappingAreaIds();
    EXPECT_EQ(area_ids.size(), 1u);
    EXPECT_EQ(area_ids.front(), -101782);
}

