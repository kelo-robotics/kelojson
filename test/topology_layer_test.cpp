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
            kelojson_map = Map::initialiseFromFile(kelojson_map_file);
            ASSERT_NE(kelojson_map, nullptr);

            areas_layer = kelojson_map->getAreasLayer();
            ASSERT_NE(areas_layer, nullptr);

            area = areas_layer->getArea(-101782);
            ASSERT_NE(area, nullptr);
            EXPECT_EQ(area->getName(), "Room1");

            topology_layer = kelojson_map->getTopologyLayer();
            ASSERT_NE(topology_layer, nullptr);
        }

    protected:
        Map::ConstPtr kelojson_map;
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

TEST_F(TopologyLayerFixture, getOverlappingAreaId)
{
    const TopologyNode::ConstPtr node = topology_layer->getNodeWithPrimitiveId(-116953);
    EXPECT_NE(node, nullptr);

    int area_id;
    EXPECT_TRUE(node->getOverlappingAreaId(area_id));
    EXPECT_EQ(area_id, -101782);
}

TEST_F(TopologyLayerFixture, getAdjacentNodes)
{
    const TopologyNode::ConstPtr node = topology_layer->getNodeWithPrimitiveId(-116953);
    const TopologyNode::ConstVec adjacent_nodes = topology_layer->getAdjacentNodes(*node);
    EXPECT_EQ(adjacent_nodes.size(), 1u);
    EXPECT_EQ(adjacent_nodes.front()->getPrimitiveId(), -116954);
}

TEST_F(TopologyLayerFixture, getEdgesInArea)
{
    const TopologyEdge::ConstVec edges = topology_layer->getEdgesInArea(*area);
    EXPECT_EQ(edges.size(), 1u);
    EXPECT_EQ(edges.front()->getPrimitiveId(), -101829);
    EXPECT_EQ(edges.front()->getStartNode()->getPrimitiveId(), -116953);
    EXPECT_EQ(edges.front()->getEndNode()->getPrimitiveId(), -116954);
}

TEST_F(TopologyLayerFixture, getNearestEdgeInAreaPoint)
{
    Point2D pt(0, 0);
    const TopologyEdge::ConstPtr edge = topology_layer->getNearestEdgeInArea(pt);
    EXPECT_NE(edge, nullptr);
    EXPECT_EQ(edge->getPrimitiveId(), -101829);
    EXPECT_EQ(edge->getStartNode()->getPrimitiveId(), -116953);
    EXPECT_EQ(edge->getEndNode()->getPrimitiveId(), -116954);
}

TEST_F(TopologyLayerFixture, getNearestEdgeInAreaPose)
{
    Pose2D pose(-0.75f, 3.0f, 0.0f);
    const TopologyEdge::ConstPtr edge = topology_layer->getNearestEdgeInArea(pose);
    ASSERT_NE(edge, nullptr);
    EXPECT_EQ(edge->getPrimitiveId(), -101829);
    EXPECT_EQ(edge->getStartNode()->getPrimitiveId(), -116954);
    EXPECT_EQ(edge->getEndNode()->getPrimitiveId(), -116955);

    Pose2D pose_2(-0.75f, 3.0f, 0.8f);
    const TopologyEdge::ConstPtr edge_2 = topology_layer->getNearestEdgeInArea(pose_2, false, 0.1f);
    ASSERT_EQ(edge_2, nullptr);
}

TEST_F(TopologyLayerFixture, getNearestEdgeInAreaPoseOneWay)
{
    Pose2D pose(-0.75f, 3.0f, 0.0f);
    const TopologyEdge::ConstPtr edge = topology_layer->getNearestEdgeInArea(pose, true);
    ASSERT_NE(edge, nullptr);
    EXPECT_EQ(edge->getPrimitiveId(), -102098);
    EXPECT_EQ(edge->getStartNode()->getPrimitiveId(), -123369);
    EXPECT_EQ(edge->getEndNode()->getPrimitiveId(), -123370);
}

TEST_F(TopologyLayerFixture, getEdge)
{
    const TopologyEdge::ConstPtr edge = topology_layer->getEdgeWithInternalId(0);
    EXPECT_NE(edge, nullptr);
}
