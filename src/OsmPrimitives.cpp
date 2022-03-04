#include "global_map/OsmPrimitives.h"
#include "yaml_common/Common.h"

namespace kelojson {
namespace osm {

Primitive::Primitive(const YAML::Node& feature) {
	primitiveId = yamlGetInt(feature, "id");
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
	position = Pos(feature["geometry"]["coordinates"][0].as<double>(),
				   feature["geometry"]["coordinates"][1].as<double>());
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
	wayType = yamlGetString(feature["geometry"], "type");
	YAML::Node nodes = feature["geometry"]["nodeIds"][0];
	for (unsigned int i = 0; i < nodes.size(); i++) {
		nodeIds.push_back(nodes[i].as<int>());
	}
}

std::vector<osm::Node> Way::extractNodes(const YAML::Node& feature) {
	std::vector<osm::Node> nodes;
	bool isPolygon = yamlGetString(feature["geometry"], "type") == "Polygon";
	YAML::Node coords = isPolygon ? feature["geometry"]["coordinates"][0] : feature["geometry"]["coordinates"];
	YAML::Node nodeIds = feature["geometry"]["nodeIds"][0];

	for (unsigned int i = 0; i < coords.size(); i++) {
		osm::Node node;
		node.primitiveId = nodeIds[i].as<int>();
		node.position = Pos(coords[i][0].as<double>(),
							coords[i][1].as<double>());
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
	relationType = yamlGetString(feature["properties"], "type");
	YAML::Node rm = feature["relation"]["members"];
	for (unsigned int i = 0; i < rm.size(); i++) {
		members.push_back(RelationMember(rm[i]));
	}
}

RelationMember::RelationMember(const YAML::Node& feature) {
	std::string type = yamlGetString(feature, "type");
	primitiveType = (type == "Node") ? osm::primitiveType::NODE : 
					(type == "Way") ? osm::primitiveType::WAY : osm::primitiveType::RELATION;
	primitiveId = yamlGetInt(feature, "id");
	role = yamlGetString(feature, "role");
}

}
}
