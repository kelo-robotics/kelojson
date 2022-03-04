#include <yaml_common/Parser.h>
#include "kelojson_loader/OsmPrimitives.h"

namespace kelojson {
namespace osm {

using YAMLParser = kelo::yaml_common::Parser;

Primitive::Primitive(const YAML::Node& feature) {
	primitiveId = YAMLParser::getInt(feature, "id");
	tags = getOsmTags(feature);
}

Tags Primitive::getOsmTags(const YAML::Node& feature) {
	Tags properties;
	if (feature["properties"]) {
		for (YAML::const_iterator it = feature["properties"].begin(); it != feature["properties"].end(); it++) {
			properties[it->first.as<std::string>()] = it->second.as<std::string>();
		}
	}
	return properties;
}

kelojson::layerType::LayerType Primitive::getLayerType() {
	if (tags.find("layer") != tags.end()) {
		return kelojson::layerType::getType(tags.at("layer"));
	}
	return kelojson::layerType::UNDEFINED;
}

Node::Node()
: Primitive() {
	primitiveType = osm::primitiveType::NODE;
}

Node::Node(const YAML::Node& feature)
: Primitive(feature) {
	primitiveType = osm::primitiveType::NODE;
	position = Point2D(feature["geometry"]["coordinates"][0].as<float>(),
					   feature["geometry"]["coordinates"][1].as<float>());
}

Node::Node(const Node& node)
: Primitive() {
	primitiveId = node.primitiveId;
	primitiveType = node.primitiveType;
	tags = node.tags;
	position = node.position;
}

Way::Way()
: Primitive() {
	primitiveType = osm::primitiveType::WAY;
}

Way::Way(const YAML::Node& feature)
: Primitive(feature) {
	primitiveType = osm::primitiveType::WAY;
	wayType = YAMLParser::getString(feature["geometry"], "type");
	YAML::Node nodes = feature["geometry"]["nodeIds"][0];
	for (unsigned int i = 0; i < nodes.size(); i++) {
		nodeIds.push_back(nodes[i].as<int>());
	}
}

std::vector<osm::Node> Way::extractNodes(const YAML::Node& feature) {
	std::vector<osm::Node> nodes;
	bool isPolygon = YAMLParser::getString(feature["geometry"], "type") == "Polygon";
	YAML::Node coords = isPolygon ? feature["geometry"]["coordinates"][0] : feature["geometry"]["coordinates"];
	YAML::Node nodeIds = feature["geometry"]["nodeIds"][0];

	for (unsigned int i = 0; i < coords.size(); i++) {
		osm::Node node;
		node.primitiveId = nodeIds[i].as<int>();
		node.position = Point2D(coords[i][0].as<float>(),
								coords[i][1].as<float>());
		nodes.push_back(node);
	}
	return nodes;
}

Relation::Relation()
: Primitive() {
	primitiveType = osm::primitiveType::RELATION;
}

Relation::Relation(const YAML::Node& feature)
: Primitive(feature) {
	primitiveType = osm::primitiveType::RELATION;
	relationType = YAMLParser::getString(feature["properties"], "type");
	YAML::Node rm = feature["relation"]["members"];
	for (unsigned int i = 0; i < rm.size(); i++) {
		members.push_back(RelationMember(rm[i]));
	}
}

RelationMember::RelationMember(const YAML::Node& feature) {
	std::string type = YAMLParser::getString(feature, "type");
	primitiveType = (type == "Node") ? osm::primitiveType::NODE : 
					(type == "Way") ? osm::primitiveType::WAY : osm::primitiveType::RELATION;
	primitiveId = YAMLParser::getInt(feature, "id");
	role = YAMLParser::getString(feature, "role");
}

}
}
