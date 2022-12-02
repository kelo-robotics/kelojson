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

#include <kelojson/Print.h>
#include <kelojson/osm/NodePrimitive.h>
#include <kelojson/osm/WayPrimitive.h>
#include <kelojson/osm/RelationPrimitive.h>
#include <kelojson/osm/PrimitiveFactory.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool PrimitiveFactory::createPrimitive(const YAML::Node& feature, Primitive::Store& store)
{
    PrimitiveType type = PrimitiveFactory::inferPrimitiveType(feature);

    switch ( type )
    {
        case PrimitiveType::NODE:
        {
            NodePrimitive::Ptr node = std::make_shared<NodePrimitive>();
            if ( !node->initialise(feature) )
            {
                std::cout << Print::Err << "[PrimitiveFactory] "
                          << "NodePrimitive could not be initialised with "
                          << "following features " << feature
                          << Print::End << std::endl;
                return false;
            }
            store[PrimitiveType::NODE][node->getId()] = node;
            break;
        }

        case PrimitiveType::WAY:
        {
            std::vector<NodePrimitive::Ptr> nodes;
            if ( !WayPrimitive::extractNodes(feature, nodes) )
            {
                std::cout << Print::Err << "[PrimitiveFactory] "
                          << "WayPrimitive could not extract node with"
                          << "following features " << feature
                          << Print::End << std::endl;
                return false;
            }
            for ( NodePrimitive::Ptr node : nodes )
            {
                store[PrimitiveType::NODE][node->getId()] = node;
            }

            WayPrimitive::Ptr way = std::make_shared<WayPrimitive>();
            if ( !way->initialise(feature) )
            {
                std::cout << Print::Err << "[PrimitiveFactory] "
                          << "WayPrimitive could not be initialised with "
                          << "following features " << feature
                          << Print::End << std::endl;
                return false;
            }
            store[PrimitiveType::WAY][way->getId()] = way;
            break;
        }

        case PrimitiveType::RELATION:
        {
            RelationPrimitive::Ptr relation =
                std::make_shared<RelationPrimitive>();
            if ( !relation->initialise(feature) )
            {
                std::cout << Print::Err << "[PrimitiveFactory] "
                          << "RelationPrimitive could not be initialised with "
                          << "following features " << feature
                          << Print::End << std::endl;
                return false;
            }
            store[PrimitiveType::RELATION][relation->getId()] = relation;
            break;
        }

        case PrimitiveType::INVALID:
        default:
            std::cout << Print::Err << "[PrimitiveFactory] "
                      << "Unknown OSM primitive found!"
                      << Print::End << std::endl;
            return false;
    }
    return true;
}

PrimitiveType PrimitiveFactory::inferPrimitiveType(const YAML::Node& feature)
{
    if ( Parser::hasKey(feature, "relation", false) &&
         Parser::hasKey(feature["relation"], "members", false) )
    {
        return PrimitiveType::RELATION;
    }

    if ( Parser::hasKey(feature, "geometry", false) &&
         Parser::hasKey(feature["geometry"], "type", false) &&
         Parser::hasKey(feature["geometry"], "coordinates", false) )
    {
        std::string type;
        if ( Parser::read<std::string>(feature["geometry"], "type", type) )
        {
            return ( type == "Point" )
                   ? PrimitiveType::NODE
                   : PrimitiveType::WAY;
        }
    }

    return PrimitiveType::INVALID;
}

} // namespace osm
} // namespace kelojson
} // namespace kelo

