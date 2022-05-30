#include <kelojson_loader/Print.h>

#include <kelojson_loader/layer/LayerFactory.h>
#include <kelojson_loader/layer/areas/AreasLayer.h>
#include <kelojson_loader/layer/zones/ZonesLayer.h>
#include <kelojson_loader/layer/topology/TopologyLayer.h>

namespace kelo {
namespace kelojson {

Layer::Ptr LayerFactory::createLayer(
        const LayerType& layer_type,
        const osm::Primitive::Store& store)
{
    Layer::Ptr layer;

    switch ( layer_type )
    {
        case LayerType::AREAS:
        {
            layer = std::make_shared<AreasLayer>();
            break;
        }
        case LayerType::ZONES:
        {
            layer = std::make_shared<ZonesLayer>();
            break;
        }
        case LayerType::TOPOLOGY:
        {
            layer = std::make_shared<TopologyLayer>();
            break;
        }
        case LayerType::UNDEFINED:
        {
            return nullptr;
        }
        default:
        {
            std::cout << Print::Err << "[LayerFactory] "
                      << "Layer of type " << asString(layer_type) << " is not supported."
                      << Print::End << std::endl;
            return nullptr;
        }
    }

    if ( !layer->initialise(store) )
    {
        std::cout << Print::Err << "[LayerFactory] "
                  << "Layer of type " << asString(layer_type) << " could not be initialised."
                  << Print::End << std::endl;
        return nullptr;
    }

    return layer;
}

} // namespace kelojson
} // namespace kelo
