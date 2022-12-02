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

#include <geometry_common/Utils.h>

#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/zones/PolygonZone.h>

using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool PolygonZone::initialise(int way_id, const osm::Primitive::Store& store)
{
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( way == nullptr )
    {
        std::cout << Print::Err << "[PolygonZone] "
                  << "Could not find a way with id: " << way_id
                  << Print::End << std::endl;
        return false;
    }

    Zone::initialise(way);
    polygon_.vertices = osm::PrimitiveUtils::getPoints(store, way->getNodeIds());
    if ( polygon_.size() < 4 ) // last pt is repeated and triangle is smallest possible polygon
    {
        std::cout << Print::Err << "[PolygonZone] Zone id: " << id_
                  << " only contains " << polygon_.size() << " points."
                  << Print::End << std::endl;
        return false;
    }
    polygon_.vertices.pop_back();
    return true;
}

bool PolygonZone::contains(const geometry_common::Point2D& point) const
{
    return polygon_.containsPoint(point);
}

const geometry_common::Polygon2D& PolygonZone::getPolygon() const
{
    return polygon_;
}

const geometry_common::Point2D PolygonZone::meanPoint() const
{
    return GCUtils::calcMeanPoint(polygon_.vertices);
}

void PolygonZone::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "polygon: " << polygon_;
}

} // namespace kelojson
} // namespace kelo
