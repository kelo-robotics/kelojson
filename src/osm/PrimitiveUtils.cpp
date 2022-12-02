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

#include <kelojson/osm/PrimitiveUtils.h>

using kelo::geometry_common::PointVec2D;

namespace kelo {
namespace kelojson {
namespace osm {

std::vector<int> PrimitiveUtils::filter(
        const Primitive::Map& primitives,
        const Tags& tags,
        const std::string& type)
{
    bool check_type = ( !type.empty() );
    std::vector<int> matching_ids;
    for ( auto itr = primitives.cbegin(); itr != primitives.cend(); itr ++ )
    {
        if ( check_type && itr->second->getType() != type )
        {
            continue;
        }
        const Tags& primitive_tags = itr->second->getTags();
        bool all_tags_match = true;
        for ( auto tag_itr = tags.cbegin(); tag_itr != tags.cend(); tag_itr ++ )
        {
            if ( primitive_tags.find(tag_itr->first) == primitive_tags.end() ||
                 primitive_tags.at(tag_itr->first) != tag_itr->second )
            {
                all_tags_match = false;
                break;
            }
        }
        if ( all_tags_match )
        {
            matching_ids.push_back(itr->first);
        }
    }
    return matching_ids;
}

const NodePrimitive::Ptr PrimitiveUtils::getNode(
        const Primitive::Store& store,
        int id)
{
    const Primitive::Map& nodes = store.at(PrimitiveType::NODE);
    if ( nodes.find(id) == nodes.end() )
    {
        return nullptr;
    }
    return std::static_pointer_cast<NodePrimitive>(nodes.at(id));
}

const WayPrimitive::Ptr PrimitiveUtils::getWay(
        const Primitive::Store& store,
        int id)
{
    const Primitive::Map& ways = store.at(PrimitiveType::WAY);
    if ( ways.find(id) == ways.end() )
    {
        return nullptr;
    }
    return std::static_pointer_cast<WayPrimitive>(ways.at(id));
}

const RelationPrimitive::Ptr PrimitiveUtils::getRelation(
        const Primitive::Store& store,
        int id)
{
    const Primitive::Map& relations = store.at(PrimitiveType::RELATION);
    if ( relations.find(id) == relations.end() )
    {
        return nullptr;
    }
    return std::static_pointer_cast<RelationPrimitive>(relations.at(id));
}

PointVec2D PrimitiveUtils::getPoints(
        const Primitive::Store& store,
        const std::vector<int>& node_ids)
{
    const Primitive::Map& nodes = store.at(PrimitiveType::NODE);

    PointVec2D pts;

    for ( size_t i = 0; i < node_ids.size(); i++ )
    {
        if ( nodes.find(node_ids[i]) == nodes.end() )
        {
            pts.clear();
            return pts;
        }
        const osm::NodePrimitive::Ptr node =
            std::static_pointer_cast<osm::NodePrimitive>(nodes.at(node_ids[i]));
        pts.push_back(node->getPosition());
    }
    return pts;
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
