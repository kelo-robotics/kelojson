/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Dharmin Bakaraniya
 * Sushant Chavan
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#ifndef KELO_KELOJSON_AREAS_LAYER_H
#define KELO_KELOJSON_AREAS_LAYER_H

#include <kelojson/layer/Layer.h>
#include <kelojson/layer/areas/Area.h>
#include <kelojson/layer/areas/Transition.h>

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
                const geometry_common::LineSegment2D& line_segment) const;

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
