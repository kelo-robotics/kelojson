#include <yaml_common/Parser2.h>

#include <kelojson_loader/Map.h>
#include <kelojson_loader/osm/NodePrimitive.h>
#include <kelojson_loader/osm/WayPrimitive.h>
#include <kelojson_loader/osm/RelationPrimitive.h>
#include <kelojson_loader/Print.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {

bool Map::initialiseFromFile(const std::string& map_file)
{
    std::cout << "map_file: " << map_file << std::endl;
    YAML::Node map_yaml;
    if ( !Parser::loadFile(map_file, map_yaml) )
    {
        return false;
    }

    return initialiseFromYAML(map_yaml);
}

bool Map::initialiseFromYAML(const YAML::Node& map_yaml)
{
    if ( !clear() )
    {
        std::cout << Print::Err << "[Map] "
                  << "Could not clear existing content"
                  << Print::End << std::endl;
        return false;
    }

    return ( parseAllPrimitives(map_yaml) && initialiseAllLayers() );
}

bool Map::parseAllPrimitives(const YAML::Node& map_yaml)
{
    osm_primitive_store_.emplace(osm::PrimitiveType::NODE, osm::Primitive::Map());
    osm_primitive_store_.emplace(osm::PrimitiveType::WAY, osm::Primitive::Map());
    osm_primitive_store_.emplace(osm::PrimitiveType::RELATION, osm::Primitive::Map());

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
        osm::PrimitiveType type = Map::inferPrimitiveType(*feature);

        switch ( type )
        {
            case osm::PrimitiveType::NODE:
            {
                osm::NodePrimitive::Ptr node = std::make_shared<osm::NodePrimitive>();
                if ( node->initialise(*feature) )
                {
                    osm_primitive_store_[osm::PrimitiveType::NODE][node->getId()] = node;
                }
                else
                {
                    std::cout << Print::Err << "[Map] "
                              << "NodePrimitive could not be initialised with "
                              << "following features " << *feature
                              << Print::End << std::endl;
                    success = false;
                }
                break;
            }

            case osm::PrimitiveType::WAY:
            {
                std::vector<osm::NodePrimitive::Ptr> nodes;
                if ( !osm::WayPrimitive::extractNodes(*feature, nodes) )
                {
                    std::cout << Print::Err << "[Map] "
                              << "WayPrimitive could not extract node with"
                              << "following features " << *feature
                              << Print::End << std::endl;
                }
                for ( osm::NodePrimitive::Ptr node : nodes )
                {
                    osm_primitive_store_[osm::PrimitiveType::NODE][node->getId()] = node;
                }

                osm::WayPrimitive::Ptr way = std::make_shared<osm::WayPrimitive>();
                if ( way->initialise(*feature) )
                {
                    osm_primitive_store_[osm::PrimitiveType::WAY][way->getId()] = way;
                }
                else
                {
                    std::cout << Print::Err << "[Map] "
                              << "WayPrimitive could not be initialised with "
                              << "following features " << *feature
                              << Print::End << std::endl;
                    success = false;
                }
                break;
            }

            case osm::PrimitiveType::RELATION:
            {
                osm::RelationPrimitive::Ptr relation =
                    std::make_shared<osm::RelationPrimitive>();
                if ( relation->initialise(*feature) )
                {
                    osm_primitive_store_[osm::PrimitiveType::RELATION][relation->getId()] = relation;
                }
                else
                {
                    std::cout << Print::Err << "[Map] "
                              << "RelationPrimitive could not be initialised with "
                              << "following features " << *feature
                              << Print::End << std::endl;
                    success = false;
                }
                break;
            }

            case osm::PrimitiveType::INVALID:
            default:
                std::cout << Print::Err << "[Map] "
                          << "Unknown OSM primitive found!"
                          << Print::End << std::endl;
                success = false;
                break;
        }

        if ( !success )
        {
            break;
        }

    }
    std::cout << Print::Success << "[Map] Parsed "
              << osm_primitive_store_.at(osm::PrimitiveType::NODE).size() << " OSM Nodes, "
              << osm_primitive_store_.at(osm::PrimitiveType::WAY).size() << " OSM Ways and "
              << osm_primitive_store_.at(osm::PrimitiveType::RELATION).size() << " OSM Relations"
              << Print::End << std::endl;
    return success;
}

bool Map::initialiseAllLayers()
{
    return false;
}

bool Map::clear()
{
    // TODO:
    return true;
}

osm::PrimitiveType Map::inferPrimitiveType(const YAML::Node& feature)
{
    if ( Parser::hasKey(feature, "relation", false) &&
         Parser::hasKey(feature["relation"], "members", false) )
    {
        return osm::PrimitiveType::RELATION;
    }

    if ( Parser::hasKey(feature, "geometry", false) &&
         Parser::hasKey(feature["geometry"], "type", false) &&
         Parser::hasKey(feature["geometry"], "coordinates", false) )
    {
        std::string type;
        if ( Parser::read<std::string>(feature["geometry"], "type", type) )
        {
            return ( type == "Point" )
                   ? osm::PrimitiveType::NODE
                   : osm::PrimitiveType::WAY;
        }
    }

    return osm::PrimitiveType::INVALID;
}

} // namespace kelojson
} // namespace kelo
