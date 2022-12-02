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

#ifndef KELO_KELOJSON_LAYERTYPE_H
#define KELO_KELOJSON_LAYERTYPE_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

namespace kelo {
namespace kelojson {

enum class LayerType
{
    UNDEFINED = 0,
    AREAS,
    SUBAREAS,
    ZONES,
    TOPOLOGY,
    OCCUPANCY_GRID
};

const std::vector<std::string> layer_type_strings = {
    "UNDEFINED",
    "AREAS",
    "SUBAREAS",
    "ZONES",
    "TOPOLOGY",
    "OCCUPANCY_GRID"
};

inline std::string asString(const LayerType& layer_type)
{
    size_t layer_type_int = static_cast<size_t>(layer_type);
    return ( layer_type_int >= layer_type_strings.size() )
           ? layer_type_strings[0]
           : layer_type_strings[layer_type_int];
};

inline LayerType asLayerType(const std::string& layer_type_string)
{
    std::string layer_type_str = layer_type_string;
    std::transform(layer_type_str.begin(), layer_type_str.end(),
                   layer_type_str.begin(), ::toupper);
    LayerType layer_type = LayerType::UNDEFINED;
    for ( size_t i = 0; i < layer_type_strings.size(); i++ )
    {
        if ( layer_type_strings[i] == layer_type_str )
        {
            layer_type = static_cast<LayerType>(i);
            break;
        }
    }
    return layer_type;
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_LAYERTYPE_H
