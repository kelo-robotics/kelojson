#ifndef KELO_KELOJSON_LAYER_FACTORY_H
#define KELO_KELOJSON_LAYER_FACTORY_H

#include <kelojson/layer/Layer.h>
#include <kelojson/osm/Primitive.h>

namespace kelo {
namespace kelojson {

class LayerFactory
{
    public:

        static Layer::Ptr createLayer(
                const LayerType& layer_type,
                const osm::Primitive::Store& store);

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_LAYER_FACTORY_H
