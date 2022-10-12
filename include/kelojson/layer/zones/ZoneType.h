#ifndef KELO_KELOJSON_ZONETYPE_H
#define KELO_KELOJSON_ZONETYPE_H

namespace kelo {
namespace kelojson {

enum class ZoneType
{
    UNKNOWN = 0,
    FORBIDDEN,
    RAMP,
    LOAD_PARKING,
    TRANSFER_STATION,
    CHARGING_STATION,
    WAITING_LOCATION,
    OCCLUSION,
    STAIRS,
    ELEVATOR
};

const std::vector<std::string> zone_type_strings = {
    "UNKNOWN",
    "FORBIDDEN",
    "RAMP",
    "LOAD_PARKING",
    "TRANSFER_STATION",
    "CHARGING_STATION",
    "WAITING_LOCATION",
    "OCCLUSION",
    "STAIRS",
    "ELEVATOR"
};

inline std::string asString(const ZoneType& zone_type)
{
    size_t zone_type_int = static_cast<size_t>(zone_type);
    return ( zone_type_int >= zone_type_strings.size() )
           ? zone_type_strings[0]
           : zone_type_strings[zone_type_int];
};

inline ZoneType asZoneType(const std::string& zone_type_string)
{
    ZoneType zone_type = ZoneType::UNKNOWN;
    for ( size_t i = 0; i < zone_type_strings.size(); i++ )
    {
        if ( zone_type_strings[i] == zone_type_string )
        {
            zone_type = static_cast<ZoneType>(i);
            break;
        }
    }
    return zone_type;
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_ZONETYPE_H
