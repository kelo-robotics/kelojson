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

#ifndef KELO_KELOJSON_AREATYPE_H
#define KELO_KELOJSON_AREATYPE_H

namespace kelo {
namespace kelojson {

enum class AreaType
{
    UNKNOWN = 0,
    ROOM,
    CORRIDOR,
    JUNCTION,
    OPEN_AREA
};

const std::vector<std::string> area_type_strings = {
    "UNKNOWN",
    "ROOM",
    "CORRIDOR",
    "JUNCTION",
    "OPEN_AREA"
};

inline std::string asString(const AreaType& area_type)
{
    size_t area_type_int = static_cast<size_t>(area_type);
    return ( area_type_int >= area_type_strings.size() )
           ? area_type_strings[0]
           : area_type_strings[area_type_int];
};

inline AreaType asAreaType(const std::string& area_type_string)
{
    std::string area_type_str = area_type_string;
    if ( area_type_str == "area" )
    {
        area_type_str = "open_area";
    }
    std::transform(area_type_str.begin(), area_type_str.end(),
                   area_type_str.begin(), ::toupper);
    AreaType area_type = AreaType::UNKNOWN;
    for ( size_t i = 0; i < area_type_strings.size(); i++ )
    {
        if ( area_type_strings[i] == area_type_str )
        {
            area_type = static_cast<AreaType>(i);
            break;
        }
    }
    return area_type;
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_AREATYPE_H
