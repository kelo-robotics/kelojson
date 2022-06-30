#include <yaml_common/Parser2.h>

#include <kelojson/Print.h>
#include <kelojson/osm/NodePrimitive.h>
#include <kelojson/osm/WayPrimitive.h>
#include <kelojson/osm/RelationPrimitive.h>
#include <kelojson/osm/PrimitiveFactory.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool PrimitiveFactory::createPrimitive(const YAML::Node& feature, Primitive::Store& store)
{
    PrimitiveType type = PrimitiveFactory::inferPrimitiveType(feature);

    switch ( type )
    {
        case PrimitiveType::NODE:
        {
            NodePrimitive::Ptr node = std::make_shared<NodePrimitive>();
            if ( !node->initialise(feature) )
            {
                std::cout << Print::Err << "[PrimitiveFactory] "
                          << "NodePrimitive could not be initialised with "
                          << "following features " << feature
                          << Print::End << std::endl;
                return false;
            }
            store[PrimitiveType::NODE][node->getId()] = node;
            break;
        }

        case PrimitiveType::WAY:
        {
            std::vector<NodePrimitive::Ptr> nodes;
            if ( !WayPrimitive::extractNodes(feature, nodes) )
            {
                std::cout << Print::Err << "[PrimitiveFactory] "
                          << "WayPrimitive could not extract node with"
                          << "following features " << feature
                          << Print::End << std::endl;
                return false;
            }
            for ( NodePrimitive::Ptr node : nodes )
            {
                store[PrimitiveType::NODE][node->getId()] = node;
            }

            WayPrimitive::Ptr way = std::make_shared<WayPrimitive>();
            if ( !way->initialise(feature) )
            {
                std::cout << Print::Err << "[PrimitiveFactory] "
                          << "WayPrimitive could not be initialised with "
                          << "following features " << feature
                          << Print::End << std::endl;
                return false;
            }
            store[PrimitiveType::WAY][way->getId()] = way;
            break;
        }

        case PrimitiveType::RELATION:
        {
            RelationPrimitive::Ptr relation =
                std::make_shared<RelationPrimitive>();
            if ( !relation->initialise(feature) )
            {
                std::cout << Print::Err << "[PrimitiveFactory] "
                          << "RelationPrimitive could not be initialised with "
                          << "following features " << feature
                          << Print::End << std::endl;
                return false;
            }
            store[PrimitiveType::RELATION][relation->getId()] = relation;
            break;
        }

        case PrimitiveType::INVALID:
        default:
            std::cout << Print::Err << "[PrimitiveFactory] "
                      << "Unknown OSM primitive found!"
                      << Print::End << std::endl;
            return false;
    }
    return true;
}

PrimitiveType PrimitiveFactory::inferPrimitiveType(const YAML::Node& feature)
{
    if ( Parser::hasKey(feature, "relation", false) &&
         Parser::hasKey(feature["relation"], "members", false) )
    {
        return PrimitiveType::RELATION;
    }

    if ( Parser::hasKey(feature, "geometry", false) &&
         Parser::hasKey(feature["geometry"], "type", false) &&
         Parser::hasKey(feature["geometry"], "coordinates", false) )
    {
        std::string type;
        if ( Parser::read<std::string>(feature["geometry"], "type", type) )
        {
            return ( type == "Point" )
                   ? PrimitiveType::NODE
                   : PrimitiveType::WAY;
        }
    }

    return PrimitiveType::INVALID;
}

} // namespace osm
} // namespace kelojson
} // namespace kelo

