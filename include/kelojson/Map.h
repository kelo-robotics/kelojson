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

#ifndef KELO_KELOJSON_MAP_H
#define KELO_KELOJSON_MAP_H

#include <memory>

#include <yaml-cpp/yaml.h>

#include <kelojson/osm/Primitive.h>
#include <kelojson/layer/Layer.h>
#include <kelojson/layer/areas/AreasLayer.h>
#include <kelojson/layer/zones/ZonesLayer.h>
#include <kelojson/layer/topology/TopologyLayer.h>
#include <kelojson/layer/occupancy/OccupancyGridLayer.h>

namespace kelo {
namespace kelojson {

class Map
{
    public:

        using Ptr = std::shared_ptr<Map>;

        using ConstPtr = std::shared_ptr<const Map>;

        Map() = default;

        virtual ~Map() = default;

        static Map::ConstPtr initialiseFromFile(const std::string& map_file);

        static Map::ConstPtr initialiseFromYAML(const YAML::Node& map_yaml);

        AreasLayer::ConstPtr getAreasLayer() const;

        ZonesLayer::ConstPtr getZonesLayer() const;

        TopologyLayer::ConstPtr getTopologyLayer() const;

        OccupancyGridLayer::ConstPtr getOccupancyGridLayer() const;

    protected:

        Layer::Map layers_;

        bool parseAllPrimitives(
                const YAML::Node& map_yaml,
                osm::Primitive::Store& osm_primitive_store);

        bool initialiseAllLayers(const osm::Primitive::Store& osm_primitive_store);

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_H
