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

#ifndef KELO_KELOJSON_ZONETYPE_H
#define KELO_KELOJSON_ZONETYPE_H

namespace kelo {
namespace kelojson {

enum class ZoneType
{
    UNKNOWN = 0,
    FORBIDDEN,
    RAMP,
    LOAD_PARKING,
    TRANSFER_STATION,
    CHARGING_STATION,
    WAITING_LOCATION,
    OCCLUSION,
    STAIRS,
    ELEVATOR
};

const std::vector<std::string> zone_type_strings = {
    "UNKNOWN",
    "FORBIDDEN",
    "RAMP",
    "LOAD_PARKING",
    "TRANSFER_STATION",
    "CHARGING_STATION",
    "WAITING_LOCATION",
    "OCCLUSION",
    "STAIRS",
    "ELEVATOR"
};

inline std::string asString(const ZoneType& zone_type)
{
    size_t zone_type_int = static_cast<size_t>(zone_type);
    return ( zone_type_int >= zone_type_strings.size() )
           ? zone_type_strings[0]
           : zone_type_strings[zone_type_int];
};

inline ZoneType asZoneType(const std::string& zone_type_string)
{
    ZoneType zone_type = ZoneType::UNKNOWN;
    for ( size_t i = 0; i < zone_type_strings.size(); i++ )
    {
        if ( zone_type_strings[i] == zone_type_string )
        {
            zone_type = static_cast<ZoneType>(i);
            break;
        }
    }
    return zone_type;
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_ZONETYPE_H
