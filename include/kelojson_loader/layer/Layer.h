#ifndef KELO_KELOJSON_LAYER_H
#define KELO_KELOJSON_LAYER_H

#include <memory>
#include <map>

#include <kelojson_loader/layer/LayerType.h>
#include <kelojson_loader/osm/Primitive.h>

namespace kelo {
namespace kelojson {

class Layer
{
    public:

        using Ptr = std::shared_ptr<Layer>;

        using ConstPtr = std::shared_ptr<const Layer>;

        using Map = std::map<LayerType, Layer::Ptr>;

        virtual ~Layer() = default;

        virtual bool initialise(const osm::Primitive::Store& store) = 0;

        LayerType getType() const
        {
            return type_;
        }

    protected:

        Layer(const LayerType& type):
            type_(type) {}

    private:

        LayerType type_;

        Layer() = delete;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_LAYER_H
