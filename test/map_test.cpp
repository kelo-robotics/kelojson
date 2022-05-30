#include <gtest/gtest.h>

#include <kelojson_loader/Map.h>

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
            EXPECT_TRUE(kelojson_map.initialiseFromFile(kelojson_map_file));
        }

    protected:
        Map kelojson_map;
};

TEST_F(MapFixture, simpleCreation)
{
}

TEST(KelojsonMap, incorrectCreation)
{
    std::string wrong_kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
    wrong_kelojson_map_file += ".wrong";
    Map wrong_kelojson_map;
    EXPECT_FALSE(wrong_kelojson_map.initialiseFromFile(wrong_kelojson_map_file));

    std::string empty_kelojson_map_string;
    Map empty_kelojson_map;
    EXPECT_FALSE(empty_kelojson_map.initialiseFromFile(empty_kelojson_map_string));
}
