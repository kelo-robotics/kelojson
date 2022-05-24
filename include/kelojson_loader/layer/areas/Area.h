#ifndef KELO_KELOJSON_AREA_H
#define KELO_KELOJSON_AREA_H

#include <iostream>
#include <string>

#include <geometry_common/Point2D.h>
#include <geometry_common/Polyline2D.h>
#include <geometry_common/Polygon2D.h>
#include <geometry_common/Box.h>

#include <kelojson_loader/osm/Primitive.h>
#include <kelojson_loader/layer/areas/AreaType.h>
#include <kelojson_loader/layer/areas/Transition.h>

namespace kelo {
namespace kelojson {

class Area
{
    public:

        using Ptr = std::shared_ptr<Area>;

        using ConstPtr = std::shared_ptr<const Area>;

        using Map = std::map<int, Area::Ptr>;

        Area() = default;

        virtual ~Area() = default;

        bool initialise(int way_id, const osm::Primitive::Store& store);

        void setTransitions(const Transition::ConstVec& transitions);

        const std::vector<int> getAdjacentAreaIds() const;

        const Transition::ConstVec getTransitionsWith(
                int adjacent_area_id,
                bool only_door = false) const;

        bool isInsideBoundingBox(const geometry_common::Point2D& point) const;

        float boundingBoxArea() const;

        bool contains(const geometry_common::Point2D& point) const;

        int getId() const;

        AreaType getType() const;

        const std::string& getName() const;

        const geometry_common::Polygon2D& getPolygon() const;

        const geometry_common::Point2D& getMeanPoint() const;

        const geometry_common::Box getBoundingBox() const;

        const Transition::ConstVec& getTransitions() const;

        friend std::ostream& operator << (std::ostream& out, const Area& area);

    private:

        int id_;
        AreaType type_;
        std::string name_;
        geometry_common::Polygon2D polygon_;
        geometry_common::Point2D mean_pt_;
        geometry_common::Box bounding_box_;
        Transition::ConstVec transitions_;

        static geometry_common::Box calcBoundingBox(
                geometry_common::Polygon2D polygon);

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_AREA_H
