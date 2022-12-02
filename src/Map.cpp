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

#include <set>
#include <yaml_common/Parser2.h>

#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveFactory.h>
#include <kelojson/layer/LayerFactory.h>
#include <kelojson/Map.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {

Map::ConstPtr Map::initialiseFromFile(const std::string& map_file)
{
    YAML::Node map_yaml;
    if ( !Parser::loadFile(map_file, map_yaml) )
    {
        return nullptr;
    }

    return Map::initialiseFromYAML(map_yaml);
}

Map::ConstPtr Map::initialiseFromYAML(const YAML::Node& map_yaml)
{
    Map::Ptr kelojson_map = std::make_shared<Map>();
    osm::Primitive::Store osm_primitive_store;
    return ( kelojson_map->parseAllPrimitives(map_yaml, osm_primitive_store) &&
             kelojson_map->initialiseAllLayers(osm_primitive_store) )
           ? kelojson_map
           : nullptr;
}

bool Map::parseAllPrimitives(
        const YAML::Node& map_yaml,
        osm::Primitive::Store& osm_primitive_store)
{
    osm_primitive_store.emplace(osm::PrimitiveType::NODE, osm::Primitive::Map());
    osm_primitive_store.emplace(osm::PrimitiveType::WAY, osm::Primitive::Map());
    osm_primitive_store.emplace(osm::PrimitiveType::RELATION, osm::Primitive::Map());

    if ( !Parser::hasKey(map_yaml, "features") || !map_yaml["features"].IsSequence() )
    {
        std::cout << Print::Err << "[Map] "
                  << "map file does not have features"
                  << Print::End << std::endl;
        return false;
    }

    const YAML::Node& features = map_yaml["features"];
    bool success = true;
    for ( YAML::const_iterator feature = features.begin();
          feature != features.end(); feature++ )
    {
        if ( !osm::PrimitiveFactory::createPrimitive(*feature, osm_primitive_store) )
        {
            success = false;
            break;
        }
    }

    std::cout << Print::Success << "[Map] Parsed "
              << osm_primitive_store.at(osm::PrimitiveType::NODE).size() << " OSM Nodes, "
              << osm_primitive_store.at(osm::PrimitiveType::WAY).size() << " OSM Ways and "
              << osm_primitive_store.at(osm::PrimitiveType::RELATION).size() << " OSM Relations"
              << Print::End << std::endl;
    return success;
}

bool Map::initialiseAllLayers(const osm::Primitive::Store& osm_primitive_store)
{
    std::set<LayerType> layer_set;

    /* find all types of layers */
    std::vector<osm::PrimitiveType> primitive_types{
        osm::PrimitiveType::NODE,
        osm::PrimitiveType::WAY,
        osm::PrimitiveType::RELATION
    };
    for ( const osm::PrimitiveType primitive_type : primitive_types )
    {
        const osm::Primitive::Map& primitives = osm_primitive_store.at(primitive_type);
        for ( osm::Primitive::Map::const_iterator itr = primitives.cbegin();
              itr != primitives.cend();
              itr ++ )
        {
            layer_set.insert(asLayerType(itr->second->getTag<std::string>("layer", "")));
        }
    }

    for ( const LayerType layer_type : layer_set )
    {
        Layer::Ptr layer = LayerFactory::createLayer(layer_type, osm_primitive_store);
        if ( layer == nullptr && layer_type != LayerType::UNDEFINED )
        {
            return false;
        }
        layers_[layer_type] = layer;
    }

    for ( auto itr = layers_.begin(); itr != layers_.end(); itr ++ )
    {
        if ( itr->first == LayerType::UNDEFINED )
        {
            continue;
        }
        if ( !itr->second->initialiseInterLayerAssociation(layers_, osm_primitive_store) )
        {
            return false;
        }
    }

    return true;
}

AreasLayer::ConstPtr Map::getAreasLayer() const
{
    return ( layers_.find(LayerType::AREAS) == layers_.end() )
           ? nullptr
           : std::static_pointer_cast<AreasLayer>(layers_.at(LayerType::AREAS));
}

ZonesLayer::ConstPtr Map::getZonesLayer() const
{
    return ( layers_.find(LayerType::ZONES) == layers_.end() )
           ? nullptr
           : std::static_pointer_cast<ZonesLayer>(layers_.at(LayerType::ZONES));
}

TopologyLayer::ConstPtr Map::getTopologyLayer() const
{
    return ( layers_.find(LayerType::TOPOLOGY) == layers_.end() )
           ? nullptr
           : std::static_pointer_cast<TopologyLayer>(layers_.at(LayerType::TOPOLOGY));
}

OccupancyGridLayer::ConstPtr Map::getOccupancyGridLayer() const
{
    return ( layers_.find(LayerType::OCCUPANCY_GRID) == layers_.end() )
           ? nullptr
           : std::static_pointer_cast<OccupancyGridLayer>(layers_.at(LayerType::OCCUPANCY_GRID));
}

} // namespace kelojson
} // namespace kelo
