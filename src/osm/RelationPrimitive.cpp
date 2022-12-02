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

#include <kelojson/osm/RelationPrimitive.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool RelationPrimitive::initialise(const YAML::Node& feature)
{
    if ( !Primitive::initialise(feature) ||
         !Parser::hasKey(feature, "properties") ||
         !Parser::read<std::string>(feature["properties"], "type", type_ ) ||
         !Parser::hasKey(feature, "relation") ||
         !Parser::hasKey(feature["relation"], "members") ||
         !feature["relation"]["members"].IsSequence() )
    {
        return false;
    }

    for ( const YAML::Node& member_yaml : feature["relation"]["members"] )
    {
        Member member;
        if ( !member.initialise(member_yaml) )
        {
            return false;
        }
        members_.push_back(member);
    }
    return true;
}

bool RelationPrimitive::Member::initialise(const YAML::Node& member_yaml)
{
    std::string type_str;
    if ( !Parser::read<std::string>(member_yaml, "type", type_str) )
    {
        return false;
    }
    type = asPrimitiveType(type_str);
    if ( type == PrimitiveType::INVALID )
    {
        return false;
    }
    if ( !Parser::read<int>(member_yaml, "id", id) ||
         !Parser::read<std::string>(member_yaml, "role", role) )
    {
        return false;
    }
    return true;
}

void RelationPrimitive::Member::write(std::ostream& out) const
{
    out << "<Member "
        << "type: "<< asString(type)
        << ", id: " << id
        << ", role: " << role
        << ">";
}

const std::vector<RelationPrimitive::Member>& RelationPrimitive::getMembers() const
{
    return members_;
}

void RelationPrimitive::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "members: [" << std::endl;
    for ( size_t i = 0; i < members_.size(); i++ )
    {
        out << "    ";
        members_[i].write(out);
        out << std::endl;
    }
    out << "]>";
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
