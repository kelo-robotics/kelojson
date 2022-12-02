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

#ifndef KELO_KELOJSON_OSM_PRIMITIVE_TYPE_H
#define KELO_KELOJSON_OSM_PRIMITIVE_TYPE_H

#include <vector>
#include <string>
#include <algorithm>

namespace kelo {
namespace kelojson {
namespace osm {

enum class PrimitiveType
{
    INVALID = 0,
    NODE,
    WAY,
    RELATION,
};

const std::vector<std::string> primitive_type_strings = {
    "INVALID",
    "NODE",
    "WAY",
    "RELATION",
};

inline std::string asString(const PrimitiveType& primitive_type)
{
    size_t primitive_type_int = static_cast<size_t>(primitive_type);
    return ( primitive_type_int >= primitive_type_strings.size() )
           ? primitive_type_strings[0]
           : primitive_type_strings[primitive_type_int];
};

inline PrimitiveType asPrimitiveType(const std::string& primitive_type_string)
{
    std::string primitive_type_str = primitive_type_string;
    std::transform(primitive_type_str.begin(), primitive_type_str.end(),
                   primitive_type_str.begin(), ::toupper);
    PrimitiveType primitive_type = PrimitiveType::INVALID;
    for ( size_t i = 0; i < primitive_type_strings.size(); i++ )
    {
        if ( primitive_type_strings[i] == primitive_type_str )
        {
            primitive_type = static_cast<PrimitiveType>(i);
            break;
        }
    }
    return primitive_type;
};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_TYPE_H
