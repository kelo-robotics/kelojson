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

#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/occupancy/OccupancyGrid.h>

using kelo::geometry_common::Pose2D;

namespace kelo {
namespace kelojson {

bool OccupancyGrid::initialise(int node_id, const osm::Primitive::Store& store)
{
    const osm::NodePrimitive::Ptr node = osm::PrimitiveUtils::getNode(store, node_id);
    if ( node == nullptr )
    {
        std::cout << Print::Err << "[OccupancyGrid] Node id: " << node_id
                  << " does not exist." << Print::End << std::endl;
        return false;
    }

    id = node->getId();
    float theta;
    if ( !node->readTag<std::string>("filename", filename) ||
         !node->readTag<float>("free_thresh", free_threshold) ||
         !node->readTag<float>("occupied_thresh", occupied_threshold) ||
         !node->readTag<bool>("negate", negate) ||
         !node->readTag<std::string>("name", name) ||
         !node->readTag<float>("resolution", resolution) ||
         !node->readTag<float>("theta", theta) )
    {
        return false;
    }
    origin = Pose2D(node->getPosition(), theta);
    return true;
}

std::ostream& operator << (std::ostream& out, const OccupancyGrid& grid)
{
    out << "OccupancyGrid:" << std::endl
        << "    id: " << grid.id << std::endl
        << "    filename: " << grid.filename << std::endl
        << "    free_threshold: " << grid.free_threshold << std::endl
        << "    occupied_threshold: " << grid.occupied_threshold << std::endl
        << "    negate: " << grid.negate << std::endl
        << "    name: " << grid.name << std::endl
        << "    resolution: " << grid.resolution << std::endl
        << "    origin: " << grid.origin << std::endl;
    return out;
}

} // namespace kelojson
} // namespace kelo
