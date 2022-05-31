#include <gtest/gtest.h>

#include <kelojson_loader/Map.h>
#include <kelojson_loader/layer/areas/AreasLayer.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

using namespace kelo::kelojson;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polyline2D;

class AreasLayerFixture : public ::testing::Test
{
    public:
        void SetUp()
        {
            std::string kelojson_map_file = mkstr(KELOJSON_TEST_MAP_FILE);
            kelojson_map = Map::initialiseFromFile(kelojson_map_file);
            ASSERT_NE(kelojson_map, nullptr);

            areas_layer = kelojson_map->getAreasLayer();
            ASSERT_NE(areas_layer, nullptr);
        }

    protected:
        Map::ConstPtr kelojson_map;
        AreasLayer::ConstPtr areas_layer;
};

TEST_F(AreasLayerFixture, simpleCreation)
{
}

TEST_F(AreasLayerFixture, getArea)
{
    Area::ConstPtr area = areas_layer->getArea(-101782);
    ASSERT_NE(area, nullptr);
    EXPECT_EQ(area->getName(), "Room1");

    Area::ConstPtr area2 = areas_layer->getArea("Room1");
    ASSERT_NE(area2, nullptr);
    EXPECT_EQ(area2->getId(), -101782);

    Area::ConstPtr area3 = areas_layer->getAreaContaining(Point2D(0, 0));
    ASSERT_NE(area3, nullptr);
    EXPECT_EQ(area3->getName(), "Room1");
    EXPECT_EQ(area3->getId(), -101782);
}

TEST_F(AreasLayerFixture, getAreaIdOf)
{
    int area_id;
    EXPECT_TRUE(areas_layer->getAreaIdOf("Room1", area_id));
    EXPECT_EQ(area_id, -101782);
}

TEST_F(AreasLayerFixture, getTransition)
{
    Transition::ConstPtr transition = areas_layer->getTransition(-99754);
    EXPECT_NE(transition, nullptr);
    EXPECT_EQ(transition->getDoorType(), DoorType::GENERIC);
    const std::pair<Area::ConstPtr, Area::ConstPtr> associated_areas = 
        transition->getAssociatedAreas();
    EXPECT_EQ(associated_areas.first->getId(), -101805);
    EXPECT_EQ(associated_areas.second->getId(), -101782);
}

TEST_F(AreasLayerFixture, getNearestTransitions)
{
    const Transition::ConstVec nearest_transitions =
        areas_layer->getNearestTransitions(Point2D(0.0f, 0.0f), 2.0f);
    EXPECT_EQ(nearest_transitions.size(), 1u);
    EXPECT_EQ(nearest_transitions.front()->getId(), -99754);

    const Transition::ConstVec nearest_transitions_2 =
        areas_layer->getNearestTransitions(Point2D(0.0f, 0.0f), 1.0f);
    EXPECT_EQ(nearest_transitions_2.size(), 0u);

    const Transition::ConstVec nearest_transitions_3 =
        areas_layer->getNearestTransitions(Point2D(0.0f, 0.0f), 4.0f, 2);
    EXPECT_EQ(nearest_transitions_3.size(), 2u);
    EXPECT_EQ(nearest_transitions_3.front()->getId(), -99754);
    EXPECT_EQ(nearest_transitions_3.back()->getId(), -99755);
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

TEST_F(AreasLayerFixture, getIntersectingTransitions)
{
    Polyline2D polyline({
            Point2D(-0.5f, 1.25f),
            Point2D(-0.5f, 3.0f),
            Point2D( 4.5f, 3.0f)
    });

    const Transition::ConstVec intersection_transitions =
        areas_layer->getIntersectingTransitions(polyline);
    std::vector<int> true_transition_ids{-99755, -99754, -99753};
    EXPECT_EQ(intersection_transitions.size(), true_transition_ids.size());
    for ( size_t i = 0; i < true_transition_ids.size(); i++ )
    {
        bool found = false;
        for ( size_t j = 0; j < intersection_transitions.size(); j++ )
        {
            if ( intersection_transitions[j]->getId() == true_transition_ids[i] )
            {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found);
    }
}

TEST_F(AreasLayerFixture, getAdjacentAreaIds)
{
    Area::ConstPtr area = areas_layer->getArea(-101795);
    std::vector<int> adjacent_area_ids = area->getAdjacentAreaIds();
    std::vector<int> true_adjacent_area_ids{-101805, -101813};
    EXPECT_EQ(adjacent_area_ids.size(), true_adjacent_area_ids.size());
    for ( size_t i = 0; i < true_adjacent_area_ids.size(); i++ )
    {
        EXPECT_NE(std::find(
                    adjacent_area_ids.begin(),
                    adjacent_area_ids.end(),
                    true_adjacent_area_ids[i]),
                  adjacent_area_ids.end());
    }
}

TEST_F(AreasLayerFixture, getTransitionsBetween)
{
    Area::ConstPtr area = areas_layer->getArea(-101795);
    Transition::ConstVec transitions =
        area->getTransitionsWith(-101805);
    EXPECT_EQ(transitions.size(), 1u);
    EXPECT_EQ(transitions.front()->getId(), -99755);

    Transition::ConstVec door_transitions =
        area->getTransitionsWith(-101805, true);
    EXPECT_EQ(door_transitions.size(), 0u);
}

TEST_F(AreasLayerFixture, contains)
{
    Point2D pt(0, 0);
    EXPECT_TRUE(areas_layer->contains(pt));

    Point2D pt2(10, 10);
    EXPECT_FALSE(areas_layer->contains(pt2));
}
