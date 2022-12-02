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

#ifndef KELO_KELOJSON_DOORTYPE_H
#define KELO_KELOJSON_DOORTYPE_H

#include <algorithm>

namespace kelo {
namespace kelojson {

enum class DoorType
{
    NONE = 0,
    HINGED,
    SLIDING,
    GENERIC
};

const std::vector<std::string> door_type_strings = {
    "NONE",
    "HINGED",
    "SLIDING",
    "GENERIC"
};

inline std::string asString(const DoorType& door_type)
{
    size_t door_type_int = static_cast<size_t>(door_type);
    return ( door_type_int >= door_type_strings.size() )
           ? door_type_strings[0]
           : door_type_strings[door_type_int];
};

inline DoorType asDoorType(const std::string& door_type_string)
{
    std::string door_type_str = door_type_string;
    if ( door_type_str == "yes" || door_type_str == "YES" )
    {
        door_type_str = "generic";
    }
    std::transform(door_type_str.begin(), door_type_str.end(),
                   door_type_str.begin(), ::toupper);
    DoorType door_type = DoorType::NONE;
    for ( size_t i = 0; i < door_type_strings.size(); i++ )
    {
        if ( door_type_strings[i] == door_type_str )
        {
            door_type = static_cast<DoorType>(i);
            break;
        }
    }
    return door_type;
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_DOORTYPE_H
