#include "kelojson_loader/KelojsonLayer.h"
#include "kelojson_loader/KelojsonMap.h"

namespace kelo {
namespace kelojson {

Layer::Layer(layerType::LayerType type, const Map* map) {
	layerType = type;
	kelojsonMap = map;
}

void Layer::addOsmPrimitive(const osm::Primitive* prim) {
	osm::primitiveType::PrimitiveType type = prim->primitiveType;
	if (osmPrimitives.find(type) == osmPrimitives.end()) {
		osmPrimitives.insert(std::make_pair(type, std::vector<int>()));
	}
	osmPrimitives.at(type).push_back(prim->primitiveId);
}

void Layer::loadFeatures(const Map& map) {
	//std::cout << "[Kelojson Map] Loading features for Layer: " << layerType::getName(layerType) << std::endl;
	loadGeometries(map);
	loadRelations(map);
}

unsigned int Layer::getNumOsmPrimitives(osm::primitiveType::PrimitiveType primType) {
	if (osmPrimitives.find(primType) != osmPrimitives.end()) {
		return osmPrimitives.at(primType).size();
	}
	return 0;
}

OccupancyGridLayer::OccupancyGridLayer(const Map* map)
: Layer(layerType::OCCUPANCY_GRID, map) {
	
}

void OccupancyGridLayer::loadGeometries(const Map& map) {
	// Presently we only have OSM::Node geometries in this layer. Hence skip processing other geometric types
	const std::vector<int>& occGrids = osmPrimitives.at(osm::primitiveType::NODE);
	for (unsigned int i = 0; i < occGrids.size(); i++) {
		const osm::Node* node = map.getOsmNode(occGrids[i]);
		if (node != NULL) {
			OccupancyGrid og;
			og.featureId = occGrids[i];
			float theta;
			if (!node->getTagValue("filename", og.filename) ||
				!node->getTagValue("free_thresh", og.freeThreshold) ||
				!node->getTagValue("occupied_thresh", og.occupiedThreshold) ||
				!node->getTagValue("negate", og.negate) ||
				!node->getTagValue("name", og.name) ||
				!node->getTagValue("resolution", og.resolution) ||
				!node->getTagValue("theta", theta)) {
				continue;
			}
			og.origin = Pose2D(node->position.x, node->position.y, theta);
			occupancyGrids.insert(std::make_pair(og.featureId, og));
		}
	 }
	 //std::cout << "\tLoaded " << occupancyGrids.size() << "/" << occGrids.size() << " Occupancy grids." << std::endl;
}

void OccupancyGridLayer::loadRelations(const Map& /*map*/) {
	// pass
}

/*
SubAreasLayer::SubAreasLayer()
: Layer(LayerType::SUBAREAS) {
	
}

void SubAreasLayer::loadGeometries(const Map& map) {

}

void SubAreasLayer::loadRelations(const Map& map) {

}
*/

}
}
