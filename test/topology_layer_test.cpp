#include <gtest/gtest.h>

#include <kelojson_loader/KelojsonMap.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using namespace kelo::kelojson;
using kelo::geometry_common::Point2D;

class TopologyLayerFixture : public ::testing::Test
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

            topology_layer = kelojson_map.getTopologyLayer();
            EXPECT_NE(topology_layer, nullptr);
        }

    protected:
        Map kelojson_map;
        const AreasLayer* areas_layer;
        const TopologyLayer* topology_layer;
        const Area* area;
};

TEST_F(TopologyLayerFixture, getNodesInArea)
{
    std::vector<int> node_ids = topology_layer->getNodesInArea(area);
    EXPECT_EQ(node_ids.size(), 1u);
    EXPECT_EQ(node_ids[0], -116953);
}

TEST_F(TopologyLayerFixture, getEdgesInArea)
{
    std::vector<unsigned int> edge_ids = topology_layer->getEdgesInArea(area, false);
    EXPECT_EQ(edge_ids.size(), 1u);
    EXPECT_EQ(edge_ids[0], 0u);
}

TEST_F(TopologyLayerFixture, getEdgeId)
{
    unsigned int edge_id;
    EXPECT_TRUE(topology_layer->getEdgeId(-116953, -116954, edge_id));
    EXPECT_EQ(edge_id, 0u);

    EXPECT_FALSE(topology_layer->getEdgeId(-116953, -116955, edge_id));
}

TEST_F(TopologyLayerFixture, getClosestNodeInArea)
{
    Point2D pt(0, 0);
    const TopologyNode* topology_node = topology_layer->getClosestNodeInArea(pt);
    EXPECT_NE(topology_node, nullptr);
    EXPECT_EQ(topology_node->featureId, -116953);
}

TEST_F(TopologyLayerFixture, getClosestEdgeInArea)
{
    Point2D pt(0, 0);
    const TopologyEdge* topology_edge = topology_layer->getClosestEdgeInArea(pt);
    EXPECT_NE(topology_edge, nullptr);
    EXPECT_EQ(topology_edge->featureId, -101829);
    EXPECT_EQ(topology_edge->edgeId, 0u);
}

TEST_F(TopologyLayerFixture, getConnectedEdges)
{
    Point2D pt(0, 0);
    const TopologyNode& topology_node = topology_layer->topologyNodes.at(-116955);
    std::vector<unsigned int> connected_edge_ids = topology_layer->getConnectedEdges(topology_node);
    EXPECT_EQ(connected_edge_ids.size(), 2u);
    EXPECT_EQ(connected_edge_ids[0], 1u);
    EXPECT_EQ(connected_edge_ids[1], 2u);
}

