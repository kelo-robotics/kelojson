#include <gtest/gtest.h>

#include <kelojson_loader/KelojsonMap.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using namespace kelo::kelojson;

class MapFixture : public ::testing::Test
{
    public:
        void SetUp()
        {
            std::string kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
            EXPECT_TRUE(kelojson_map.loadFile(kelojson_map_file));
        }

    protected:
        Map kelojson_map;
};

TEST(KelojsonMap, incorrectCreation)
{
    std::string wrong_kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
    wrong_kelojson_map_file += ".wrong";
    Map wrong_kelojson_map;
    EXPECT_FALSE(wrong_kelojson_map.loadFile(wrong_kelojson_map_file));

    std::string empty_kelojson_map_string;
    Map empty_kelojson_map;
    EXPECT_FALSE(empty_kelojson_map.loadFromString(empty_kelojson_map_string));
}

TEST_F(MapFixture, getOsmPrimitives)
{
    const osm::Node* node = kelojson_map.getOsmNode(-104248);
    EXPECT_NE(node, nullptr);
    EXPECT_EQ(node->position, kelo::geometry_common::Point2D(-1.9369, -1.9592));

    const osm::Way* way = kelojson_map.getOsmWay(-101782);
    EXPECT_NE(way, nullptr);
    EXPECT_EQ(way->nodeIds.size(), 7u);
    EXPECT_EQ(way->wayType, "Polygon");

    const osm::Relation* relation = kelojson_map.getOsmRelation(-99769);
    EXPECT_NE(relation, nullptr);
    EXPECT_EQ(relation->relationType, "association");
}

TEST_F(MapFixture, layerGetters)
{
    std::vector<layerType::LayerType> layer_types = kelojson_map.getAvailableLayerTypes();
    EXPECT_EQ(layer_types.size(), 5u);
    EXPECT_EQ(layer_types[0], layerType::LayerType::UNDEFINED);
    EXPECT_EQ(layer_types[1], layerType::LayerType::AREAS);
    EXPECT_EQ(layer_types[2], layerType::LayerType::ZONES);
    EXPECT_EQ(layer_types[3], layerType::LayerType::TOPOLOGY);
    EXPECT_EQ(layer_types[4], layerType::LayerType::OCCUPANCY_GRID);
}
