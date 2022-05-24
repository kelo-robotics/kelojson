#ifndef KELO_KELOJSON_AREA_H
#define KELO_KELOJSON_AREA_H

#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>
#include <geometry_common/Point2D.h>
#include <geometry_common/Polyline2D.h>
#include <geometry_common/Polygon2D.h>
#include <geometry_common/Box.h>

#include <kelojson_loader/osm/WayPrimitive.h>
#include <kelojson_loader/layer/areas/AreaType.h>
#include <kelojson_loader/layer/areas/DoorType.h>

namespace kelo {
namespace kelojson {

class Area
{
    public:

        using Ptr = std::shared_ptr<Area>;

        using ConstPtr = std::shared_ptr<const Area>;

        using Map = std::map<int, Area::Ptr>;

        struct Transition
        {
            using Ptr = std::shared_ptr<Transition>;
            using ConstPtr = std::shared_ptr<const Transition>;
            using Vec = std::vector<Transition::Ptr>;
            using ConstVec = std::vector<Transition::ConstPtr>;

            geometry_common::Polyline2D coordinates;
            DoorType door_type;
            int id;
            std::string name;
            std::pair<int, int> associated_area_ids;

            bool isDoor() const;

            float width() const;

            friend std::ostream& operator << (
                    std::ostream& out,
                    const Transition& transition);

        };

        Area() = default;

        virtual ~Area() = default;

        bool initialise(int way_id, const osm::Primitive::Store& store);

        std::vector<int> adjacentAreaIds() const;

        const std::map<int, Transition::Vec>& getAllTransitions() const;

        const Transition::Vec transitionsWithArea(int adjacent_area_id) const;

        const std::map<int, Transition::Vec> allDoorTransitions() const;

        const Transition::Vec doorTransitionsWithArea(int adjacent_area_id) const;

        bool isInsideBoundingBox(const geometry_common::Point2D& point) const;

        float boundingBoxArea() const;

        bool contains(const geometry_common::Point2D& point) const;

        int getId() const;

        AreaType getType() const;

        const std::string& getName() const;

        const geometry_common::Polygon2D& getPolygon() const;

        const geometry_common::Point2D& getMeanPoint() const;

        const geometry_common::Box getBoundingBox() const;

        friend std::ostream& operator << (std::ostream& out, const Area& area);

    private:

        int id_;
        AreaType type_;
        std::string name_;
        geometry_common::Polygon2D polygon_;
        geometry_common::Point2D mean_pt_;
        geometry_common::Box bounding_box_;
        std::map<int, Transition::Vec> transitions_;

        bool initialiseTransitions(const osm::Primitive::Store& store);

        static geometry_common::Box calcBoundingBox(
                geometry_common::Polygon2D polygon);

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_AREA_H
