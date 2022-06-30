#include <gtest/gtest.h>

#include <kelojson/Map.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using namespace kelo::kelojson;
using kelo::geometry_common::Pose2D;

class OccupancyGridLayerFixture : public ::testing::Test
{
    public:
        void SetUp()
        {
            std::string kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
            kelojson_map = Map::initialiseFromFile(kelojson_map_file);
            ASSERT_NE(kelojson_map, nullptr);

            occ_grid_layer = kelojson_map->getOccupancyGridLayer();
            ASSERT_NE(occ_grid_layer, nullptr);
        }

    protected:
        Map::ConstPtr kelojson_map;
        OccupancyGridLayer::ConstPtr occ_grid_layer;
};

TEST_F(OccupancyGridLayerFixture, simpleCreation)
{
}

TEST_F(OccupancyGridLayerFixture, getAllOccupancyGrids)
{
    const std::map<int, OccupancyGrid>& occ_grids = occ_grid_layer->getAllOccupancyGrids();
    EXPECT_EQ(occ_grids.size(), 1u);
    ASSERT_NE(occ_grids.find(-116904), occ_grids.end());
    const OccupancyGrid& occ_grid = occ_grids.at(-116904);
    EXPECT_EQ(occ_grid.id, -116904);
    EXPECT_EQ(occ_grid.filename, "two_rooms.pgm");
    EXPECT_NEAR(occ_grid.free_threshold, 0.196f, 1e-3f);
    EXPECT_NEAR(occ_grid.occupied_threshold, 0.65f, 1e-3f);
    EXPECT_EQ(occ_grid.negate, false);
    EXPECT_EQ(occ_grid.name, "two_rooms");
    EXPECT_EQ(occ_grid.resolution, 0.02f);
    EXPECT_EQ(occ_grid.origin, Pose2D(-3.005f, -3.005f, 0.0f));
}
