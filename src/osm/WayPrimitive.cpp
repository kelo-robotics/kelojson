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

#include <kelojson/osm/WayPrimitive.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool WayPrimitive::initialise(const YAML::Node& feature)
{
    return ( Primitive::initialise(feature) && 
             Parser::hasKey(feature, "geometry") &&
             Parser::read<std::string>(feature["geometry"], "type", type_) &&
             Parser::hasKey(feature["geometry"], "nodeIds") &&
             feature["geometry"]["nodeIds"].IsSequence() &&
             feature["geometry"]["nodeIds"].size() > 0 &&
             Parser::read<std::vector<int>>(feature["geometry"]["nodeIds"][0], node_ids_) );
}

bool WayPrimitive::extractNodes(const YAML::Node& feature, std::vector<NodePrimitive::Ptr>& nodes)
{
    if ( !Parser::hasKey(feature, "geometry") )
    {
        return false;
    }

    if ( !Parser::hasKey(feature["geometry"], "coordinates") ||
         !feature["geometry"]["coordinates"].IsSequence() ||
         !Parser::hasKey(feature["geometry"], "nodeIds") ||
         !feature["geometry"]["nodeIds"].IsSequence() )
    {
        return false;
    }

    const std::string type = Parser::get<std::string>(feature["geometry"], "type", "");
    bool is_polygon = ( type == "Polygon" );
    const YAML::Node& coords = ( is_polygon )
                               ? feature["geometry"]["coordinates"][0]
                               : feature["geometry"]["coordinates"];
    const YAML::Node& node_ids = feature["geometry"]["nodeIds"][0];
    if ( node_ids.size() != coords.size() )
    {
        return false;
    }

    nodes.clear();
    for ( size_t i = 0; i < coords.size(); i++ )
    {
        NodePrimitive::Ptr node = std::make_shared<NodePrimitive>();
        int node_id;
        if ( !Parser::read<int>(node_ids[i], node_id) ||
             !node->initialise(node_id, coords[i]) )
        {
            return false;
        }
        nodes.push_back(node);
    }
    return true;
}

const std::vector<int>& WayPrimitive::getNodeIds() const
{
    return node_ids_;
}

void WayPrimitive::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "node_ids: [";
    for ( size_t i = 0; i < node_ids_.size(); i++ )
    {
        out << node_ids_[i] << ", ";
    }
    out << "]>";
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
