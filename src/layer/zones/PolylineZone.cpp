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
#include <kelojson/layer/zones/PolylineZone.h>

using GCUtils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

bool PolylineZone::initialise(int way_id, const osm::Primitive::Store& store)
{
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( way == nullptr )
    {
        std::cout << Print::Err << "[PolylineZone] "
                  << "Could not find a way with id: " << way_id
                  << Print::End << std::endl;
        return false;
    }

    Zone::initialise(way);
    polyline_.vertices = osm::PrimitiveUtils::getPoints(store, way->getNodeIds());
    if ( polyline_.size() < 2 )
    {
        std::cout << Print::Err << "[PolylineZone] Zone id: " << id_
                  << " only contains " << polyline_.size() << " points."
                  << Print::End << std::endl;
        return false;
    }
    return true;
}

const geometry_common::Polyline2D& PolylineZone::getPolyline() const
{
    return polyline_;
}

const geometry_common::Point2D PolylineZone::meanPoint() const
{
    return GCUtils::calcMeanPoint(polyline_.vertices);
}

void PolylineZone::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "polyline: " << polyline_;
}

} // namespace kelojson
} // namespace kelo
