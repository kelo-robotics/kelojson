#include <gtest/gtest.h>

#include <kelojson_loader/Map.h>
#include <kelojson_loader/layer/zones/ZonesLayer.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using namespace kelo::kelojson;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Pose2D;

class TopologyLayerFixture : public ::testing::Test
{
    public:
        void SetUp()
        {
            std::string kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
            ASSERT_TRUE(kelojson_map.initialiseFromFile(kelojson_map_file));

            areas_layer = kelojson_map.getAreasLayer();
            ASSERT_NE(areas_layer, nullptr);

            area = areas_layer->getArea(-101782);
            ASSERT_NE(area, nullptr);
            EXPECT_EQ(area->getName(), "Room1");

            topology_layer = kelojson_map.getTopologyLayer();
            ASSERT_NE(topology_layer, nullptr);
        }

    protected:
        Map kelojson_map;
        AreasLayer::ConstPtr areas_layer;
        TopologyLayer::ConstPtr topology_layer;
        Area::ConstPtr area;
};

TEST_F(TopologyLayerFixture, simpleCreation)
{
}

TEST_F(TopologyLayerFixture, getNodesInArea)
{
    const TopologyNode::ConstVec nodes = topology_layer->getNodesInArea(*area);
    EXPECT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes.front()->getPrimitiveId(), -116953);
}

TEST_F(TopologyLayerFixture, getNearestNodeInAreaPoint)
{
    Point2D pt(0, 0);
    const TopologyNode::ConstPtr node = topology_layer->getNearestNodeInArea(pt);
    EXPECT_NE(node, nullptr);
    EXPECT_EQ(node->getPrimitiveId(), -116953);
}

TEST_F(TopologyLayerFixture, getNearestNodeInAreaPose)
{
    Pose2D pose(-0.75f, 3.0f, 0.0f);
    const TopologyNode::ConstPtr node = topology_layer->getNearestNodeInArea(pose);
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->getPrimitiveId(), -116954);

    Pose2D pose_2(-0.75f, 3.0f, 3.0f);
    const TopologyNode::ConstPtr node_2 = topology_layer->getNearestNodeInArea(pose_2);
    ASSERT_EQ(node_2, nullptr);
}

TEST_F(TopologyLayerFixture, getNearestNodeInAreaPoseOneWay)
{
    Pose2D pose(-0.75f, 3.0f, 0.0f);
    const TopologyNode::ConstPtr node = topology_layer->getNearestNodeInArea(pose, true);
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->getPrimitiveId(), -123369);
}

TEST_F(TopologyLayerFixture, getNode)
{
    const TopologyNode::ConstPtr node = topology_layer->getNodeWithInternalId(0);
    EXPECT_NE(node, nullptr);

    const TopologyNode::ConstPtr node_2 = topology_layer->getNodeWithPrimitiveId(
            node->getPrimitiveId());
    EXPECT_NE(node_2, nullptr);

    EXPECT_EQ(node->getInternalId(), node_2->getInternalId());
    EXPECT_EQ(node->getPrimitiveId(), node_2->getPrimitiveId());
    EXPECT_EQ(node->getName(), node_2->getName());
    EXPECT_EQ(node->getPosition(), node_2->getPosition());
}

TEST_F(TopologyLayerFixture, getAdjacentNodes)
{
    const TopologyNode::ConstPtr node = topology_layer->getNodeWithPrimitiveId(-116953);
    const TopologyNode::ConstVec adjacent_nodes = topology_layer->getAdjacentNodes(*node);
    EXPECT_EQ(adjacent_nodes.size(), 1u);
    EXPECT_EQ(adjacent_nodes.front()->getPrimitiveId(), -116954);
}
