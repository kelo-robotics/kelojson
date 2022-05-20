#include <gtest/gtest.h>

#include <kelojson_loader/KelojsonMap.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using namespace kelo::kelojson;
using kelo::geometry_common::Point2D;

class ZonesLayerFixture : public ::testing::Test
{
    public:
        void SetUp()
        {
            std::string kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
            EXPECT_TRUE(kelojson_map.loadFile(kelojson_map_file));

            areas_layer = kelojson_map.getAreasLayer();
            EXPECT_NE(areas_layer, nullptr);

            area = areas_layer->getArea(-101782);
            EXPECT_NE(area, nullptr);
            EXPECT_EQ(area->name, "Room1");

            zones_layer = kelojson_map.getZonesLayer();
            EXPECT_NE(zones_layer, nullptr);
        }

    protected:
        Map kelojson_map;
        const AreasLayer* areas_layer;
        const ZonesLayer* zones_layer;
        const Area* area;
};

TEST_F(ZonesLayerFixture, getAllChargingStations)
{
    std::vector<const ZoneNode*> charging_stations = zones_layer->getAllChargingStations();
    EXPECT_EQ(charging_stations.size(), 1u);
    EXPECT_EQ(charging_stations[0]->getFeatureId(), -123214);
}

TEST_F(ZonesLayerFixture, getAllWaitingLocations)
{
    std::vector<const ZoneNode*> waiting_locations = zones_layer->getAllWaitingLocations();
    EXPECT_EQ(waiting_locations.size(), 1u);
    EXPECT_EQ(waiting_locations[0]->getFeatureId(), -123210);
}

TEST_F(ZonesLayerFixture, getAllForbiddenAreas)
{
    std::vector<const ZonePolygon*> forbidden_areas = zones_layer->getAllForbiddenAreas();
    EXPECT_EQ(forbidden_areas.size(), 1u);
    EXPECT_EQ(forbidden_areas[0]->getFeatureId(), -101954);
}

// TEST_F(ZonesLayerFixture, getAllOcclusionRegions)
// {
//     const std::map<unsigned int, OcclusionRegion>& occlusion_regions = zones_layer->getAllOcclusionRegions();
//     EXPECT_EQ(occlusion_regions.size(), 4u);
// }

// TEST_F(ZonesLayerFixture, getNearestOcclusionRegions)
// {
//     std::vector<const OcclusionRegion*> occlusion_regions =
//         zones_layer->getNearestOcclusionRegions(Point2D(0, 0), 2.1);
//     for ( size_t i = 0; i < occlusion_regions.size(); i++ )
//     {
//         std::cout << occlusion_regions[i]->getFeatureId() << std::endl;
//         std::cout << occlusion_regions[i]->getInternalId() << std::endl;
//     }
//     // EXPECT_EQ(occlusion_regions.size(), 1u);
// }

TEST_F(ZonesLayerFixture, loadParking)
{
    EXPECT_TRUE(zones_layer->hasLoadParkings());
    std::vector<const LoadParking*> load_parking_zones = zones_layer->getAllLoadParkings();
    EXPECT_EQ(load_parking_zones.size(), 2u);

    const LoadParking* load_parking_zone = zones_layer->getLoadParking("room1_load_parking_1");
    EXPECT_NE(load_parking_zone, nullptr);
    EXPECT_TRUE(load_parking_zone->valid());
    const std::vector<unsigned int>& primary_opening_nodes = load_parking_zone->getPrimaryOpeningNodes();
    EXPECT_EQ(primary_opening_nodes.size(), 2u);
    const std::vector<unsigned int>& secondary_opening_nodes = load_parking_zone->getSecondaryOpeningNodes();
    EXPECT_EQ(secondary_opening_nodes.size(), 2u);

    const LoadParking* load_parking_zone_2 = zones_layer->getLoadParking("room2_load_parking_1");
    EXPECT_NE(load_parking_zone_2, nullptr);
    EXPECT_TRUE(load_parking_zone_2->valid());
    const std::vector<unsigned int>& primary_opening_nodes_2 = load_parking_zone_2->getPrimaryOpeningNodes();
    EXPECT_EQ(primary_opening_nodes_2.size(), 2u);
    const std::vector<unsigned int>& secondary_opening_nodes_2 = load_parking_zone_2->getSecondaryOpeningNodes();
    EXPECT_EQ(secondary_opening_nodes_2.size(), 0u);
}

TEST_F(ZonesLayerFixture, insideForbiddenArea)
{
    EXPECT_TRUE(zones_layer->insideForbiddenArea(Point2D(5.4f, 1.5f)));
    EXPECT_FALSE(zones_layer->insideForbiddenArea(Point2D(0.0f, 0.0f)));
}

