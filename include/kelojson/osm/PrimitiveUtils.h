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

#ifndef KELO_KELOJSON_OSM_PRIMITIVE_UTILS_H
#define KELO_KELOJSON_OSM_PRIMITIVE_UTILS_H

#include <geometry_common/Point2D.h>

#include <kelojson/osm/Primitive.h>
#include <kelojson/osm/NodePrimitive.h>
#include <kelojson/osm/WayPrimitive.h>
#include <kelojson/osm/RelationPrimitive.h>

namespace kelo {
namespace kelojson {
namespace osm {

class PrimitiveUtils
{
    public:

        static std::vector<int> filter(
                const Primitive::Map& primitives,
                const Tags& tags,
                const std::string& type);

        static const NodePrimitive::Ptr getNode(
                const Primitive::Store& store,
                int id);

        static const WayPrimitive::Ptr getWay(
                const Primitive::Store& store,
                int id);

        static const RelationPrimitive::Ptr getRelation(
                const Primitive::Store& store,
                int id);

        static geometry_common::PointVec2D getPoints(
                const Primitive::Store& store,
                const std::vector<int>& node_ids);

};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_UTILS_H
