#ifndef KELO_KELOJSON_AREATYPE_H
#define KELO_KELOJSON_AREATYPE_H

namespace kelo {
namespace kelojson {

enum class AreaType
{
    UNKNOWN = 0,
    ROOM,
    CORRIDOR,
    JUNCTION,
    OPEN_AREA
};

const std::vector<std::string> area_type_strings = {
    "UNKNOWN",
    "ROOM",
    "CORRIDOR",
    "JUNCTION",
    "OPEN_AREA"
};

inline std::string asString(const AreaType& area_type)
{
    size_t area_type_int = static_cast<size_t>(area_type);
    return ( area_type_int >= area_type_strings.size() )
           ? area_type_strings[0]
           : area_type_strings[area_type_int];
};

inline AreaType asAreaType(const std::string& area_type_string)
{
    std::string area_type_str = area_type_string;
    std::transform(area_type_str.begin(), area_type_str.end(),
                   area_type_str.begin(), ::toupper);
    AreaType area_type = AreaType::UNKNOWN;
    for ( size_t i = 0; i < area_type_strings.size(); i++ )
    {
        if ( area_type_strings[i] == area_type_str )
        {
            area_type = static_cast<AreaType>(i);
            break;
        }
    }
    return area_type;
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_AREATYPE_H
