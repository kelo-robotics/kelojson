#include <gtest/gtest.h>

#include <kelojson_loader/KelojsonMap.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using kelo::kelojson::Map;

TEST(KeloJsonMap, simpleCreationFromFile)
{
    std::string kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
    Map kelojson_map;
    EXPECT_TRUE(kelojson_map.loadFile(kelojson_map_file));
}
