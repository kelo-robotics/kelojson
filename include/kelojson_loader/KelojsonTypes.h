#ifndef KELO_KELOJSON_TYPES_H
#define KELO_KELOJSON_TYPES_H

#include <string>
#include <vector>
#include <map>

// Layer tags
const std::string AREAS_LAYER = "areas";
const std::string SUBAREAS_LAYER = "subareas";
const std::string ZONES_LAYER = "zones";
const std::string TOPOLOGY_LAYER = "topology";
const std::string OCCUPANCY_GRID_LAYER = "occupancy-grid";

// Area type tags
const std::string AREA_TYPE_ROOM = "room";
const std::string AREA_TYPE_CORRIDOR = "corridor";
const std::string AREA_TYPE_OPEN_AREA = "area";

// Zone type tags
const std::string ZONE_TYPE_ELEVATOR = "elevator";
const std::string ZONE_TYPE_STAIRS = "stairs";
const std::string ZONE_TYPE_FORBIDDEN = "forbidden";
const std::string ZONE_TYPE_SLOWDOWN = "slowdown";
const std::string ZONE_TYPE_RAMP = "ramp";
const std::string ZONE_TYPE_CHARGING_STATION = "charging-station";
const std::string ZONE_TYPE_WAITING_LOCATION = "waiting-location";
const std::string ZONE_TYPE_OCCLUSION = "occlusion";

// Door type tags
const std::string DOOR_TYPE_HINGED = "hinged";
const std::string DOOR_TYPE_SLIDING = "sliding";

// Relation type tags
const std::string RELATION_TYPE_ASSOCIATION = "association";
const std::string RELATION_TYPE_TRANSITION = "transition";
const std::string RELATION_TYPE_RAMP_EDGES = "ramp-edges";
const std::string RELATION_TYPE_OCCLUSION_REGION = "occlusion_region";

namespace kelojson {

	namespace osm {
		namespace primitiveType {
			enum PrimitiveType {
				INVALID = 0,
				NODE,
				WAY,
				RELATION,
				COUNT
			};

			std::string getName(PrimitiveType type);
		}

		typedef std::map<std::string, std::string> Tags;
		typedef std::map<std::string, std::string>::iterator TagsItr;
		typedef std::map<std::string, std::string>::const_iterator TagsConstItr;

		// Forward declarations
		class Primitive;
	}

	// Forward declarations
	class Layer;
	class TopologyNode;
	class TopologyEdge;
	class Zone;

	namespace relationType {
		enum RelationType {
			UNKNOWN = 0,
			ADJACENT,
			OVERLAP
		};
	}

	namespace layerType {
		enum LayerType {
			UNDEFINED = 0,
			AREAS,
			SUBAREAS,
			ZONES,
			TOPOLOGY,
			OCCUPANCY_GRID
		};

		LayerType getType(std::string type);
		std::string getName(LayerType type);
	}

	typedef std::map<int, osm::Primitive*> OsmPrimitiveMap;
	typedef std::map<int, osm::Primitive*>::iterator OsmPrimitiveMapItr;
	typedef std::map<int, osm::Primitive*>::const_iterator OsmPrimitiveMapConstItr;

	typedef std::map<osm::primitiveType::PrimitiveType, OsmPrimitiveMap> OsmPrimitiveStore;
	typedef std::map<osm::primitiveType::PrimitiveType, OsmPrimitiveMap>::iterator OsmPrimitiveStoreItr;
	typedef std::map<osm::primitiveType::PrimitiveType, OsmPrimitiveMap>::const_iterator OsmPrimitiveStoreConstItr;

	typedef std::map<layerType::LayerType, Layer*> LayerMap;
	typedef std::map<layerType::LayerType, Layer*>::iterator LayerMapItr;
	typedef std::map<layerType::LayerType, Layer*>::const_iterator LayerMapConstItr;

	typedef std::map<int, Zone*> ZonesMap;
	typedef std::map<int, Zone*>::iterator ZonesMapItr;
	typedef std::map<int, Zone*>::const_iterator ZonesMapConstItr;

	typedef std::map<osm::primitiveType::PrimitiveType, ZonesMap> ZonesStore;
	typedef std::map<osm::primitiveType::PrimitiveType, ZonesMap>::iterator ZonesStoreItr;
	typedef std::map<osm::primitiveType::PrimitiveType, ZonesMap>::const_iterator ZonesStoreConstItr;

	typedef std::map<int, TopologyNode> TopologyNodeMap;
	typedef std::map<int, TopologyNode>::iterator TopologyNodeMapItr;
	typedef std::map<int, TopologyNode>::const_iterator TopologyNodeMapConstItr;

	typedef std::map<unsigned int, TopologyEdge> TopologyEdgeMap;
	typedef std::map<unsigned int, TopologyEdge>::iterator TopologyEdgeMapItr;
	typedef std::map<unsigned int, TopologyEdge>::const_iterator TopologyEdgeMapConstItr;
}

#endif //KELO_KELOJSON_TYPES_H
