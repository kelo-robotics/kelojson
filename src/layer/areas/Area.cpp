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

#include <regex>

#include <geometry_common/Utils.h>

#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/areas/Area.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Box2D;
using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool Area::initialise(int way_id, const osm::Primitive::Store& store)
{
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( way == nullptr )
    {
        return false;
    }

    id_ = way_id;
    type_ = asAreaType(way->getTag<std::string>("indoor", "unknown"));

    if ( !way->readTag<std::string>("name", name_) )
    {
        name_ = "Area_" + std::to_string(id_);
    }

    polygon_.vertices = osm::PrimitiveUtils::getPoints(store, way->getNodeIds());
    if ( polygon_.size() < 4 ) // last pt is repeated and triangle is smallest polygon
    {
        std::cout << Print::Err << "[Area] Area id: " << id_
                  << " only contains " << polygon_.size() << " points."
                  << Print::End << std::endl;
        return false;
    }
    polygon_.vertices.pop_back();
    mean_pt_ = GCUtils::calcMeanPoint(polygon_.vertices);
    bounding_box_ = Box2D(polygon_);

    return true;
}

void Area::setTransitions(const Transition::ConstVec& transitions)
{
    transitions_.clear();
    transitions_.reserve(transitions.size());
    for ( size_t i = 0; i < transitions.size(); i++ )
    {
        transitions_.push_back(transitions[i]);
    }
}

const std::vector<int> Area::getAdjacentAreaIds() const
{
    std::set<int> adjacent_area_ids;
    for ( const Transition::WeakPtr& transition_weak : transitions_ )
    {
        if ( const Transition::ConstPtr transition = transition_weak.lock() )
        {
            const std::pair<Area::ConstPtr, Area::ConstPtr>& associated_areas =
                transition->getAssociatedAreas();
            if ( associated_areas.first->getId() == id_ )
            {
                adjacent_area_ids.insert(associated_areas.second->getId());
            }
            else if ( associated_areas.second->getId() == id_ )
            {
                adjacent_area_ids.insert(associated_areas.first->getId());
            }
        }
    }
    std::vector<int> adjacent_area_ids_vec(adjacent_area_ids.begin(), adjacent_area_ids.end());
    return adjacent_area_ids_vec;
}

const Transition::ConstVec Area::getTransitionsWith(
        int adjacent_area_id,
        bool only_door) const
{
    Transition::ConstVec transitions_with_area;
    for ( const Transition::WeakPtr& transition_weak : transitions_ )
    {
        if ( const Transition::ConstPtr transition = transition_weak.lock() )
        {
            const std::pair<Area::ConstPtr, Area::ConstPtr>& associated_areas =
                transition->getAssociatedAreas();
            if ( (associated_areas.first->getId() == id_ &&
                  associated_areas.second->getId() == adjacent_area_id) ||
                 (associated_areas.first->getId() == adjacent_area_id &&
                  associated_areas.second->getId() == id_) )
            {
                if ( (only_door && transition->isDoor()) || !only_door )
                {
                    transitions_with_area.push_back(transition);
                }
            }
        }
    }
    return transitions_with_area;
}

bool Area::isInsideBoundingBox(const Point2D& point) const
{
    return ( point.x > bounding_box_.min_x &&
             point.x < bounding_box_.max_x &&
             point.y > bounding_box_.min_y &&
             point.y < bounding_box_.max_y );
}

float Area::boundingBoxArea() const
{
    return std::fabs(bounding_box_.max_x - bounding_box_.min_x) *
           std::fabs(bounding_box_.max_y - bounding_box_.min_y);
}

bool Area::contains(const Point2D& point) const
{
    return polygon_.containsPoint(point);
}

int Area::getId() const
{
    return id_;
}

AreaType Area::getType() const
{
    return type_;
}

const std::string& Area::getName() const
{
    return name_;
}

const geometry_common::Polygon2D& Area::getPolygon() const
{
    return polygon_;
}

const geometry_common::Point2D& Area::getMeanPoint() const
{
    return mean_pt_;
}

const geometry_common::Box2D Area::getBoundingBox() const
{
    return bounding_box_;
}

const Transition::ConstVec Area::getTransitions() const
{
    Transition::ConstVec transitions;
    transitions.reserve(transitions_.size());
    for ( size_t i = 0; i < transitions_.size(); i++ )
    {
        Transition::ConstPtr transition = transitions_[i].lock();
        if ( transition != nullptr )
        {
            transitions.push_back(transition);
        }
    }
    return transitions;
}

std::ostream& operator << (std::ostream& out, const Area& area)
{
    out << "Area:" << std::endl
        << "    id: " << area.id_ << std::endl
        << "    type: " << asString(area.type_) << std::endl
        << "    name: " << area.name_ << std::endl
        << "    polygon: " << area.polygon_ << std::endl
        << "    bounding_box: " << area.bounding_box_ << std::endl;
    return out;
}


} // namespace kelojson
} // namespace kelo
