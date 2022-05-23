#include <kelojson_loader/Print.h>

#include <kelojson_loader/layer/LayerFactory.h>
#include <kelojson_loader/layer/areas/AreasLayer.h>

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
        return nullptr;
    }

    return layer;
}

} // namespace kelojson
} // namespace kelo
