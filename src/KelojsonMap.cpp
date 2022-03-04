
#include <fstream>
#include <iostream>
#include <yaml_common/Parser.h>

#include "kelojson_loader/KelojsonMap.h"

namespace kelojson {

using YAMLParser = kelo::yaml_common::Parser;

Map::Map() {
}

Map::~Map() {
	clear();
}

bool Map::loadFile(std::string filename) {
	std::ifstream stream(filename.c_str());
	if (stream.fail())
		return false;

	std::stringstream ss;
	ss << stream.rdbuf();
	std::string loadedFileContents = ss.str();
	return !loadedFileContents.empty() && loadFromString(loadedFileContents);
}

bool Map::loadFromString(std::string mapString) {
	bool success = clear();
	try {
		YAML::Node yaml = YAML::Load(mapString);
		success = success && loadOsmPrimitives(yaml);
		if (success)
			loadlayers();
	} catch (YAML::InvalidScalar&) {
		clear();
		return false;
	} catch (YAML::Exception&) {
		clear();
		return false;
	}
	return success;
}


bool Map::clear()
{
	clearOsmPrimitives();
	clearAllLayers();
	return osmPrimitiveStore.empty() && layers.empty();
}

void Map::clearOsmPrimitives() {
	for (OsmPrimitiveStoreItr storeItr = osmPrimitiveStore.begin(); storeItr != osmPrimitiveStore.end(); storeItr++) {
		for (OsmPrimitiveMapItr primItr = storeItr->second.begin(); primItr != storeItr->second.end(); primItr++) {
			delete primItr->second;
			primItr->second = NULL;
		}
		storeItr->second.clear();
	}
	osmPrimitiveStore.clear();
}

void Map::clearAllLayers() {
	for (LayerMapItr itr = layers.begin(); itr != layers.end(); itr++) {
		delete itr->second;
		itr->second = NULL;
	}
	layers.clear();
}

bool Map::loadOsmPrimitives(const YAML::Node& yaml) {
	if (!initializeOsmPrimitiveStore())
		return false;

	bool success = true;
	for (YAML::const_iterator feature = yaml["features"].begin(); feature != yaml["features"].end(); feature++) {
		if (isOsmNode(*feature)) {
			osm::Node* node = new osm::Node(*feature);
			osmPrimitiveStore.at(osm::primitiveType::NODE).insert(std::make_pair(node->primitiveId, node));
		}
		else if (isOsmWay(*feature)) {
			std::vector<osm::Node> nodes = osm::Way::extractNodes(*feature);
			for (unsigned int i = 0; i < nodes.size(); i++) {
				OsmPrimitiveMap& nodesMap = osmPrimitiveStore.at(osm::primitiveType::NODE);
				if (nodesMap.find(nodes[i].primitiveId) == nodesMap.end()) {
					osm::Node* n = new osm::Node(nodes[i]);
					nodesMap.insert(std::make_pair(n->primitiveId, n));
				}
			}
			osm::Way* way = new osm::Way(*feature);
			osmPrimitiveStore.at(osm::primitiveType::WAY).insert(std::make_pair(way->primitiveId, way));
		}
		else if (isOsmRelation(*feature)) {
			osm::Relation* relation = new osm::Relation(*feature);
			osmPrimitiveStore.at(osm::primitiveType::RELATION).insert(std::make_pair(relation->primitiveId, relation));
		}
		else {
			std::cout << "[Kelojson Map] Unknown OSM primitive found! " << std::endl;
			success = false;
		}
	}
	// std::cout << "[Kelojson Map] Loaded " << osmPrimitiveStore.at(osm::primitiveType::NODE).size() << " Osm Nodes, " << 
	// 										 osmPrimitiveStore.at(osm::primitiveType::WAY).size() << " Osm Ways and " << 
	// 										 osmPrimitiveStore.at(osm::primitiveType::RELATION).size() << " Osm Relations" << std::endl;
	return success;
}

bool Map::initializeOsmPrimitiveStore() {
	if (osmPrimitiveStore.empty()) {
		osmPrimitiveStore.insert(std::make_pair(osm::primitiveType::NODE, OsmPrimitiveMap()));
		osmPrimitiveStore.insert(std::make_pair(osm::primitiveType::WAY, OsmPrimitiveMap()));
		osmPrimitiveStore.insert(std::make_pair(osm::primitiveType::RELATION, OsmPrimitiveMap()));
		return true;
	}
	return false;
}

bool Map::isOsmGeometry(const YAML::Node& feature) const {
	return feature["geometry"] && 
		   feature["geometry"]["type"] && 
		   feature["geometry"]["coordinates"];
}

bool Map::isOsmRelation(const YAML::Node& feature) const {
	return feature["relation"] && 
		   feature["relation"]["members"];
}

bool Map::isOsmNode(const YAML::Node& feature) const {
	return isOsmGeometry(feature) &&
		   YAMLParser::getString(feature["geometry"], "type") == "Point";
}

bool Map::isOsmWay(const YAML::Node& feature) const {
	return isOsmGeometry(feature) &&
		   YAMLParser::getString(feature["geometry"], "type") != "Point";
}

const osm::Node* Map::getOsmNode(int id) const {
	const OsmPrimitiveMap& nodesMap = osmPrimitiveStore.at(osm::primitiveType::NODE);
	if (nodesMap.find(id) != nodesMap.end()) {
		return dynamic_cast<const osm::Node*>(nodesMap.at(id));
	}
	return NULL;
}

const osm::Way* Map::getOsmWay(int id) const {
	const OsmPrimitiveMap& waysMap = osmPrimitiveStore.at(osm::primitiveType::WAY);
	if (waysMap.find(id) != waysMap.end()) {
		return dynamic_cast<osm::Way*>(waysMap.at(id));
	}
	return NULL;
}

const osm::Relation* Map::getOsmRelation(int id) const {
	const OsmPrimitiveMap& relMap = osmPrimitiveStore.at(osm::primitiveType::RELATION);
	if (relMap.find(id) != relMap.end()) {
		return dynamic_cast<osm::Relation*>(relMap.at(id));
	}
	return NULL;
}

bool Map::loadlayers() {
	sortOsmPrimitivesIntoLayers();

	// Load features in each layer
	for (LayerMapItr itr = layers.begin(); itr != layers.end(); itr++) {
		itr->second->loadFeatures(*this);
	}
	return true;
}

void Map::sortOsmPrimitivesIntoLayers() {
	for (unsigned int primType = osm::primitiveType::NODE; primType <= osm::primitiveType::RELATION; primType++) {
		const OsmPrimitiveMap& primitives = osmPrimitiveStore.at(osm::primitiveType::PrimitiveType(primType));
		for (OsmPrimitiveMapConstItr itr = primitives.begin(); itr != primitives.end(); itr++) {
			layerType::LayerType type = itr->second->getLayerType();
			if (layers.find(type) == layers.end()) {
				Layer* layer = (type == layerType::AREAS) ? new AreasLayer(this) :
							   (type == layerType::ZONES) ? new ZonesLayer(this) :
							   (type == layerType::TOPOLOGY) ? new TopologyLayer(this) :
							   (type == layerType::OCCUPANCY_GRID) ? new OccupancyGridLayer(this) :
//  						   (type == LayerType::SUBAREAS) ? new SubAreasLayer(this) :
							   new Layer(layerType::UNDEFINED, this);
				layers.insert(std::make_pair(type, layer));
			}
			layers.at(type)->addOsmPrimitive(itr->second);
		}
	}

	// // Print debug info
	// std::cout << "[Kelojson Map] Created/Loaded " << layers.size() << " Layers" << std::endl;
	// for (LayerMapItr itr = layers.begin(); itr != layers.end(); itr++) {
	// 	std::cout << "\tLayer " << layerType::getName(itr->first).c_str() << " contains "
	// 			  << itr->second->getNumOsmPrimitives(osm::primitiveType::NODE) << " Nodes, "
	// 			  << itr->second->getNumOsmPrimitives(osm::primitiveType::WAY) << " Ways, and "
	// 			  << itr->second->getNumOsmPrimitives(osm::primitiveType::RELATION) << " Relations"
	// 			  << std::endl;
	// }
}

std::vector<layerType::LayerType> Map::getAvailableLayerTypes() const {
	std::vector<layerType::LayerType> types;
	for (LayerMap::const_iterator itr = layers.begin(); itr != layers.end(); itr++) {
		types.push_back(itr->first);
	}
	return types;
}

const AreasLayer* Map::getAreasLayer() const {
	if (layers.find(layerType::AREAS) != layers.end())
		return dynamic_cast<AreasLayer*>(layers.at(layerType::AREAS));
	return NULL;
}

const ZonesLayer* Map::getZonesLayer() const {
	if (layers.find(layerType::ZONES) != layers.end())
		return dynamic_cast<ZonesLayer*>(layers.at(layerType::ZONES));
	return NULL;
}

const TopologyLayer* Map::getTopologyLayer() const {
	if (layers.find(layerType::TOPOLOGY) != layers.end())
		return dynamic_cast<TopologyLayer*>(layers.at(layerType::TOPOLOGY));
	return NULL;
}

const OccupancyGridLayer* Map::getOccupancyLayer() const {
	if (layers.find(layerType::OCCUPANCY_GRID) != layers.end())
		return dynamic_cast<OccupancyGridLayer*>(layers.at(layerType::OCCUPANCY_GRID));
	return NULL;
}

// const SubAreasLayer* Map::getSubAreasLayer() const {
// 	if (layers.find(LayerType::SUBAREAS) != layers.end())
// 		return dynamic_cast<SubAreasLayer*>(layers.at(LayerType::SUBAREAS));
// 	return NULL;
// }

bool Map::insideMap(const Point2D& pos) const {
	const AreasLayer* areasLayer = getAreasLayer();
	return areasLayer && areasLayer->contains(pos);
}


}
