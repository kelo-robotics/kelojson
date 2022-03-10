#include "kelojson_loader/KelojsonTypes.h"

namespace kelo {
namespace kelojson {
namespace osm {

std::string primitiveType::getName(PrimitiveType type) {
	return type == NODE ? "NODE" :
		   type == WAY ? "WAY" :
		   type == RELATION ? "RELATION" : "INVALID";
}

}

layerType::LayerType layerType::getType(std::string type) {
	return type == AREAS_LAYER ? layerType::AREAS :
		type == SUBAREAS_LAYER ? layerType::SUBAREAS :
		type == ZONES_LAYER ? layerType::ZONES :
		type == TOPOLOGY_LAYER ? layerType::TOPOLOGY :
		type == OCCUPANCY_GRID_LAYER ? layerType::OCCUPANCY_GRID :
		layerType::UNDEFINED;
}

std::string layerType::getName(layerType::LayerType type) {
	return type == layerType::AREAS ? "AREAS" :
		type == layerType::SUBAREAS ? "SUBAREAS" :
		type == layerType::ZONES ? "ZONES" :
		type == layerType::TOPOLOGY ? "TOPOLOGY" :
		type == layerType::OCCUPANCY_GRID ? "OCCUPANCY_GRID" : "UNDEFINED";
}

}
}
