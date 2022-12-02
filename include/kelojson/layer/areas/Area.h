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

#ifndef KELO_KELOJSON_AREA_H
#define KELO_KELOJSON_AREA_H

#include <iostream>
#include <string>

#include <geometry_common/Point2D.h>
#include <geometry_common/Polyline2D.h>
#include <geometry_common/Polygon2D.h>
#include <geometry_common/Box2D.h>

#include <kelojson/osm/Primitive.h>
#include <kelojson/layer/areas/AreaType.h>
#include <kelojson/layer/areas/Transition.h>

namespace kelo {
namespace kelojson {

class Area
{
    public:

        using Ptr = std::shared_ptr<Area>;

        using ConstPtr = std::shared_ptr<const Area>;

        using WeakPtr = std::weak_ptr<const Area>;

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

        const geometry_common::Box2D getBoundingBox() const;

        const Transition::ConstVec getTransitions() const;

        friend std::ostream& operator << (std::ostream& out, const Area& area);

    private:

        int id_;
        AreaType type_;
        std::string name_;
        geometry_common::Polygon2D polygon_;
        geometry_common::Point2D mean_pt_;
        geometry_common::Box2D bounding_box_;
        Transition::WeakVec transitions_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_AREA_H
