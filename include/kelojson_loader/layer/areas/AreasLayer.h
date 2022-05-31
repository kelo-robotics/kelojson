#ifndef KELO_KELOJSON_AREAS_LAYER_H
#define KELO_KELOJSON_AREAS_LAYER_H

#include <kelojson_loader/layer/Layer.h>
#include <kelojson_loader/layer/areas/Area.h>
#include <kelojson_loader/layer/areas/Transition.h>

namespace kelo {
namespace kelojson {

class AreasLayer : public Layer
{
    public:

        using Ptr = std::shared_ptr<AreasLayer>;

        using ConstPtr = std::shared_ptr<const AreasLayer>;

        AreasLayer():
            Layer(LayerType::AREAS) {}

        virtual ~AreasLayer() = default;

        bool initialise(const osm::Primitive::Store& store) override;

        Area::ConstPtr getArea(int id) const;

        Area::ConstPtr getArea(const std::string& area_name) const;

        Area::ConstPtr getAreaContaining(const geometry_common::Point2D& point) const;

        bool getAreaIdOf(const std::string& area_name, int& area_id) const;

        Transition::ConstPtr getTransition(int id) const;

        const Transition::ConstVec getNearestTransitions(
                const geometry_common::Point2D& pt,
                float search_radius = std::numeric_limits<float>::max(),
                size_t max_num_of_transitions = std::numeric_limits<size_t>::max()) const;

        std::vector<int> computePath(int start_area_id, int goal_area_id) const;

        std::vector<int> computePath(
                const std::string& start_area_name,
                const std::string& goal_area_name) const;

        std::vector<int> computePath(
                const geometry_common::Point2D& start_pt,
                const geometry_common::Point2D& goal_pt) const;

        std::string getPrintablePath(const std::vector<int>& area_path) const;

        const Transition::ConstVec getIntersectingTransitions(
                const geometry_common::Point2D& start,
                const geometry_common::Point2D& end) const;

        const Transition::ConstVec getIntersectingTransitions(
                const geometry_common::Polyline2D& polyline) const;

        /**
         * @brief Returns true if the queried position overlaps with any of the
         * mapped areas
         *
         * @param pt queried point
         *
         * @return True if any area contains it; false otherwise
         */
        bool contains(const geometry_common::Point2D& pt) const;

        const Area::Map& getAllAreas() const;

        const Transition::Map& getAllTransitions() const;

    private:

        Area::Map areas_;

        Transition::Map transitions_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_AREAS_LAYER_H
