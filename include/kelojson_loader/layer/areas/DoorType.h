#ifndef KELO_KELOJSON_DOORTYPE_H
#define KELO_KELOJSON_DOORTYPE_H

#include <algorithm>

namespace kelo {
namespace kelojson {

enum class DoorType
{
    NONE = 0,
    HINGED,
    SLIDING,
    GENERIC
};

const std::vector<std::string> door_type_strings = {
    "NONE",
    "HINGED",
    "SLIDING",
    "GENERIC"
};

inline std::string asString(const DoorType& door_type)
{
    size_t door_type_int = static_cast<size_t>(door_type);
    return ( door_type_int >= door_type_strings.size() )
           ? door_type_strings[0]
           : door_type_strings[door_type_int];
};

inline DoorType asDoorType(const std::string& door_type_string)
{
    std::string door_type_str = door_type_string;
    if ( door_type_str == "yes" || door_type_str == "YES" )
    {
        door_type_str = "generic";
    }
    std::transform(door_type_str.begin(), door_type_str.end(),
                   door_type_str.begin(), ::toupper);
    DoorType door_type = DoorType::NONE;
    for ( size_t i = 0; i < door_type_strings.size(); i++ )
    {
        if ( door_type_strings[i] == door_type_str )
        {
            door_type = static_cast<DoorType>(i);
            break;
        }
    }
    return door_type;
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_DOORTYPE_H
