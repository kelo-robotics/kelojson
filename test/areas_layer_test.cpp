#include <gtest/gtest.h>

#include <kelojson_loader/KelojsonMap.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using namespace kelo::kelojson;
using kelo::geometry_common::Point2D;

class AreasLayerFixture : public ::testing::Test
{
    public:
        void SetUp()
        {
            std::string kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
            EXPECT_TRUE(kelojson_map.loadFile(kelojson_map_file));

            areas_layer = kelojson_map.getAreasLayer();
            EXPECT_NE(areas_layer, nullptr);
        }

    protected:
        Map kelojson_map;
        const AreasLayer* areas_layer;
};

TEST_F(AreasLayerFixture, getArea)
{
    const Area* area = areas_layer->getArea(-101782);
    EXPECT_NE(area, nullptr);
    EXPECT_EQ(area->name, "Room1");

    const Area* area2 = areas_layer->getArea("Room1");
    EXPECT_NE(area2, nullptr);
    EXPECT_EQ(area2->featureId, -101782);

    const Area* area3 = areas_layer->getArea(Point2D(0, 0));
    EXPECT_NE(area3, nullptr);
    EXPECT_EQ(area3->name, "Room1");
    EXPECT_EQ(area3->featureId, -101782);
}

TEST_F(AreasLayerFixture, computePath)
{
    std::vector<int> area_path = areas_layer->computePath(-101782, -101789);
    EXPECT_EQ(area_path.size(), 5u);
    EXPECT_EQ(area_path[0], -101782);
    EXPECT_EQ(area_path[1], -101805);
    EXPECT_EQ(area_path[2], -101795);
    EXPECT_EQ(area_path[3], -101813);
    EXPECT_EQ(area_path[4], -101789);
}
