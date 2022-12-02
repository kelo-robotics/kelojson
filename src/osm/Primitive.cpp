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

#include <kelojson/osm/Primitive.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool Primitive::initialise(const YAML::Node& feature)
{
    return Parser::read<int>(feature, "id", id_) && parseTags(feature);
}

bool Primitive::parseTags(const YAML::Node& feature)
{
    if ( !Parser::hasKey(feature, "properties", false) )
    {
        return true;
    }

    return Parser::read<std::map<std::string, std::string>>(
            feature, "properties", tags_, true);
}

void Primitive::setId(int id)
{
    id_ = id;
}

int Primitive::getId() const
{
    return id_;
}

PrimitiveType Primitive::getPrimitiveType() const
{
    return primitive_type_;
}

const std::string& Primitive::getType() const
{
    return type_;
}

const Tags& Primitive::getTags() const
{
    return tags_;
}

void Primitive::writeGeneric(std::ostream& out) const
{
    out << "<Primitive " << asString(getPrimitiveType())
        << ", id: " << getId()
        << ", type: " << getType();

    const Tags& tags = getTags();
    out << ", tags: [";
    for ( Tags::const_iterator itr = tags.cbegin(); itr != tags.cend(); itr ++ )
    {
        if ( itr != tags.cbegin() )
        {
            out << ", ";
        }
        out << itr->first << ": " << itr->second;
    }
    out << "], ";
}

template <>
std::string Primitive::convertStringTo<std::string>(const std::string& value)
{
    return value;
}

template <>
int Primitive::convertStringTo<int>(const std::string& value)
{
    return std::stoi(value);
}

template <>
size_t Primitive::convertStringTo<size_t>(const std::string& value)
{
    return std::stoul(value);
}

template <>
float Primitive::convertStringTo<float>(const std::string& value)
{
    return std::stof(value);
}

template <>
bool Primitive::convertStringTo<bool>(const std::string& value)
{
    if ( value == "yes" || value == "Yes" || value == "YES" ||
         value == "1" ||
         value == "true" || value == "True" || value == "TRUE" )
    {
        return true;
    }
    else if ( value == "no" || value == "No" || value == "NO" ||
              value == "0" ||
              value == "false" || value == "False" || value == "FALSE" )
    {
        return false;
    }

    throw std::invalid_argument("Could not convert to bool");
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
