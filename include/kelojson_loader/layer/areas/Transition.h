#ifndef KELO_KELOJSON_TRANSITION_H
#define KELO_KELOJSON_TRANSITION_H

#include <iostream>
#include <string>

#include <geometry_common/Point2D.h>
#include <geometry_common/Polyline2D.h>

#include <kelojson_loader/osm/Primitive.h>
#include <kelojson_loader/layer/areas/DoorType.h>

namespace kelo {
namespace kelojson {

// forward declaration
class Area;
using AreaConstPtr = std::shared_ptr<const Area>;

class Transition
{
    public:

        using Ptr = std::shared_ptr<Transition>;

        using ConstPtr = std::shared_ptr<const Transition>;

        using Map = std::map<int, Transition::Ptr>;

        using Vec = std::vector<Transition::Ptr>;

        using ConstVec = std::vector<Transition::ConstPtr>;

        Transition() = default;

        virtual ~Transition() = default;

        bool initialise(
                int relation_id,
                const osm::Primitive::Store& store,
                const std::map<int, std::shared_ptr<Area>>& areas);

        bool isDoor() const;

        float width() const;

        int getId() const;

        const std::string& getName() const;

        DoorType getDoorType() const;

        const std::pair<AreaConstPtr, AreaConstPtr>& getAssociatedAreas() const;

        const geometry_common::Polyline2D& getPolyline() const;

        friend std::ostream& operator << (
                std::ostream& out,
                const Transition& transition);

    private:

        int id_;
        std::string name_;
        DoorType door_type_;
        std::pair<AreaConstPtr, AreaConstPtr> associated_areas_;
        geometry_common::Polyline2D polyline_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TRANSITION_H
