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

#include <yaml_common/Parser2.h>

#include <kelojson/osm/NodePrimitive.h>

using kelo::geometry_common::Point2D;
using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool NodePrimitive::initialise(const YAML::Node& feature)
{
    return ( Primitive::initialise(feature) &&
             Parser::hasKey(feature, "geometry") &&
             Parser::hasKey(feature["geometry"], "coordinates") &&
             parseCoordinates(feature["geometry"]["coordinates"]) );
}

bool NodePrimitive::initialise(int id, const YAML::Node& coordinates_yaml)
{
    setId(id);
    return parseCoordinates(coordinates_yaml);
}

bool NodePrimitive::parseCoordinates(const YAML::Node& coordinates_yaml)
{
    std::vector<float> coordinates;
    if ( !Parser::read<std::vector<float>>(coordinates_yaml, coordinates) ||
         coordinates.size() != 2 )
    {
        return false;
    }
    position_ = Point2D(coordinates[0], coordinates[1]);
    return true;
}

void NodePrimitive::setPosition(geometry_common::Point2D& position)
{
    position_ = position;
}

const Point2D& NodePrimitive::getPosition() const
{
    return position_;
}

void NodePrimitive::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "position: " << position_ << ">";
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
