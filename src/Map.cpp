#include <set>
#include <yaml_common/Parser2.h>

#include <kelojson_loader/Print.h>
#include <kelojson_loader/osm/PrimitiveFactory.h>
#include <kelojson_loader/layer/LayerFactory.h>
#include <kelojson_loader/Map.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {

bool Map::initialiseFromFile(const std::string& map_file)
{
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
        if ( !osm::PrimitiveFactory::createPrimitive(*feature, osm_primitive_store_) )
        {
            success = false;
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
    std::set<LayerType> layer_set;

    /* find all types of layers */
    std::vector<osm::PrimitiveType> primitive_types{
        osm::PrimitiveType::NODE,
        osm::PrimitiveType::WAY,
        osm::PrimitiveType::RELATION
    };
    for ( const osm::PrimitiveType primitive_type : primitive_types )
    {
        const osm::Primitive::Map& primitives = osm_primitive_store_.at(primitive_type);
        for ( osm::Primitive::Map::const_iterator itr = primitives.cbegin();
              itr != primitives.cend();
              itr ++ )
        {
            layer_set.insert(asLayerType(itr->second->getTag<std::string>("layer", "")));
        }
    }

    for ( const LayerType layer_type : layer_set )
    {
        Layer::Ptr layer = LayerFactory::createLayer(layer_type, osm_primitive_store_);
        if ( layer == nullptr && layer_type != LayerType::UNDEFINED )
        {
            return false;
        }
        layers_[layer_type] = layer;
    }

    return true;
}

bool Map::clear()
{
    // TODO:
    return true;
}

AreasLayer::ConstPtr Map::getAreasLayer() const
{
    return ( layers_.find(LayerType::AREAS) == layers_.end() )
           ? nullptr
           : std::static_pointer_cast<AreasLayer>(layers_.at(LayerType::AREAS));
}

} // namespace kelojson
} // namespace kelo
