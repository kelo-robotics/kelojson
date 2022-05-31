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
