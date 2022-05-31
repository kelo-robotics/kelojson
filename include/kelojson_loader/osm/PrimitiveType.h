#ifndef KELO_KELOJSON_OSM_PRIMITIVETYPE_H
#define KELO_KELOJSON_OSM_PRIMITIVETYPE_H

namespace kelo {
namespace kelojson {
namespace osm {

enum class PrimitiveType
{
    INVALID = 0,
    NODE,
    WAY,
    RELATION,
};

const std::vector<std::string> primitive_type_strings = {
    "INVALID",
    "NODE",
    "WAY",
    "RELATION",
};

inline std::string asString(const PrimitiveType& primitive_type)
{
    size_t primitive_type_int = static_cast<size_t>(primitive_type);
    return ( primitive_type_int >= primitive_type_strings.size() )
           ? primitive_type_strings[0]
           : primitive_type_strings[primitive_type_int];
};

inline PrimitiveType asPrimitiveType(const std::string& primitive_type_string)
{
    std::string primitive_type_str = primitive_type_string;
    std::transform(primitive_type_str.begin(), primitive_type_str.end(),
                   primitive_type_str.begin(), ::toupper);
    PrimitiveType primitive_type = PrimitiveType::INVALID;
    for ( size_t i = 0; i < primitive_type_strings.size(); i++ )
    {
        if ( primitive_type_strings[i] == primitive_type_str )
        {
            primitive_type = static_cast<PrimitiveType>(i);
            break;
        }
    }
    return primitive_type;
};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVETYPE_H
