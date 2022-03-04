#include <iostream>
#include <geometry_common/LineSegment2D.h>

#include "kelojson_loader/KelojsonMap.h"
#include "kelojson_loader/KelojsonAreasLayer.h"
#include "kelojson_loader/KelojsonTopologyLayer.h"

using LineSegment2D = kelo::geometry_common::LineSegment2D;

namespace kelojson {

TopologyNode::TopologyNode()
: position(Point2D())
, featureId(0) {

}

TopologyNode::TopologyNode(int id, const Point2D& pos) 
: position(pos)
, featureId(id) {

}

std::pair<bool, int> TopologyNode::getOverlappingAreaId() const {
	if (interlayerAssociations.find(layerType::AREAS) != interlayerAssociations.end()) {
		std::set<int> areaIds = interlayerAssociations.at(layerType::AREAS);
		if (areaIds.size() == 1) { // Since areas are non-overlapping, a topological node belongs to only one area.
			return std::make_pair(true, *(areaIds.begin()));
		}
	}
	return std::make_pair(false, 0);
}

TopologyLayer::TopologyLayer(const Map* map)
: Layer(layerType::TOPOLOGY, map) {
	
}

void TopologyLayer::loadGeometries(const Map& map) {
	// Presently we only have LineString geometries in this layer. Hence skip processing other geometric types
	const std::vector<int>& ways = osmPrimitives.at(osm::primitiveType::WAY);
	int nSuccess = 0;
	for (unsigned int i = 0; i < ways.size(); i++) {
		const osm::Way* way = map.getOsmWay(ways[i]);
		if (way != NULL) {
			if (way->wayType != "LineString")
				continue;

			nSuccess++;
			for (unsigned int i = 0; i < way->nodeIds.size() - 1; i++) {
				int startId = way->nodeIds[i];
				int endId = way->nodeIds[i+1];

				// Check if the edge already exists in the topologyEdges map
				bool isNewEdge = true;
				for (TopologyEdgeMapConstItr itr = topologyEdges.begin(); itr != topologyEdges.end(); itr++) {
					TopologyEdge currEdge = itr->second;
					bool edgeExists = (currEdge.startNodeId == startId && currEdge.endNodeId == endId) ||
									  (currEdge.startNodeId == endId && currEdge.endNodeId == startId);
					if (edgeExists) {
						isNewEdge = false;
						break;
					}
				}
				if (!isNewEdge)
					continue;

				const osm::Node* startNode = map.getOsmNode(startId);
				const osm::Node* endNode = map.getOsmNode(endId);
				if (startNode == NULL || endNode == NULL) {
					nSuccess--;
					break;
				}

				// Create and insert the nodes if not already present in the topologyNodes map
				TopologyNode start(startId, startNode->position);
				TopologyNode end(endId, endNode->position);
				topologyNodes.insert(std::make_pair(start.featureId, start));
				topologyNodes.insert(std::make_pair(end.featureId, end));

				// Create and insert the topology edge
				TopologyEdge edge;
				edge.featureId = way->primitiveId;
				edge.edgeId = topologyEdges.size();
				edge.startNodeId = startId;
				edge.endNodeId = endId;
				topologyEdges.insert(std::make_pair(edge.edgeId, edge));
			}
		}
	}

	// Store the mapping between the internal and osm node ids.
	nodeInternalIdMap.clear();
	nodeFeatureIdMap.clear();
	unsigned int internalNodeId = 0;
	for (TopologyNodeMapConstItr itr = topologyNodes.begin(); itr != topologyNodes.end(); itr++) {
		int osmNodeId = itr->first;
		nodeInternalIdMap.insert(std::make_pair(osmNodeId, internalNodeId));
		nodeFeatureIdMap.insert(std::make_pair(internalNodeId, osmNodeId));
		internalNodeId++;
	}

	// std::cout << "\tSuccessfully processed " << nSuccess << "/" << ways.size() 
	// 		  << " topology Ways" << std::endl;
	// std::cout << "\tLoaded " << topologyNodes.size() << " topology nodes and " 
	// 		  << topologyEdges.size() << " topology edges." << std::endl;
}

void TopologyLayer::loadRelations(const Map& map) {
	loadInterlayerAssociations(map);
}

std::vector<int> TopologyLayer::getNodesInArea(const Area* area) const {
	std::vector<int> nodes;
	for (TopologyNodeMapConstItr itr = topologyNodes.begin(); itr != topologyNodes.end(); itr++) {
		std::set<int> overlappingAreas = itr->second.interlayerAssociations.at(layerType::AREAS);
		if (std::find(overlappingAreas.begin(), overlappingAreas.end(), area->featureId) != overlappingAreas.end()) {
			nodes.push_back(itr->second.featureId);
		}
	}
	return nodes;
}

std::vector<unsigned int> TopologyLayer::getEdgesInArea(const Area* area, bool fullyContained) const {
	std::set<unsigned int> edgeSet;
	std::vector<int> nodesInArea = getNodesInArea(area);
	if (!nodesInArea.empty()) {
		for (unsigned int i = 0; i < nodesInArea.size(); i++) {
			std::vector<unsigned int> connectedEdges = getConnectedEdges(topologyNodes.at(nodesInArea[i]));
			edgeSet.insert(connectedEdges.begin(), connectedEdges.end());
		}
	}

	if (fullyContained) {
		for (std::set<unsigned int>::iterator itr = edgeSet.begin(); itr != edgeSet.end(); ) {
			TopologyEdge edge = topologyEdges.at(*itr);
			if (std::find(nodesInArea.begin(), nodesInArea.end(), edge.startNodeId) == nodesInArea.end() ||
				std::find(nodesInArea.begin(), nodesInArea.end(), edge.endNodeId) == nodesInArea.end()) {
				// edge is not fully enclosed in this area. remove it
				edgeSet.erase(itr++);
			} else {
				++itr;
			}
		}
	}

	return std::vector<unsigned int>(edgeSet.begin(), edgeSet.end());
}

bool TopologyLayer::getInternalNodeId(int osmNodeId, unsigned int& internalNodeId) const {
	if (nodeInternalIdMap.find(osmNodeId) != nodeInternalIdMap.end()) {
		internalNodeId = nodeInternalIdMap.at(osmNodeId);
		return true;
	}
	return false;
}

bool TopologyLayer::getOsmNodeId(unsigned int internalNodeId, int& osmNodeId) const {
	if (nodeFeatureIdMap.find(internalNodeId) != nodeFeatureIdMap.end()) {
		osmNodeId = nodeFeatureIdMap.at(internalNodeId);
		return true;
	}
	return false;
}

bool TopologyLayer::getEdgeId(int startOsmNodeId, int endOsmNodeId, unsigned int& edgeId) const {
	for (TopologyEdgeMapConstItr itr = topologyEdges.begin(); itr != topologyEdges.end(); itr++) {
		const TopologyEdge& edge = itr->second;
		if ((edge.startNodeId == startOsmNodeId && edge.endNodeId == endOsmNodeId) ||
			(edge.startNodeId == endOsmNodeId && edge.endNodeId == startOsmNodeId)) {
			edgeId = edge.edgeId;
			return true;
		}
	}
	return false;
}

const TopologyNode* TopologyLayer::getClosestNodeInArea(const Point2D& point) const {
	const TopologyNode* closestNode = NULL;
	if (kelojsonMap != NULL && kelojsonMap->getAreasLayer() != NULL) {
		const Area* area = kelojsonMap->getAreasLayer()->getArea(point);
		if (area != NULL) {
			std::vector<int> nodeIds = getNodesInArea(area);
			if (!nodeIds.empty()) {
				double leastDistance = std::numeric_limits<double>::max();
				for (unsigned int i = 0; i < nodeIds.size(); i++) {
					const TopologyNode& node = topologyNodes.at(nodeIds[i]);
					double dist = node.position.getCartDist(point);
					if (dist < leastDistance) {
						leastDistance = dist;
						closestNode = &node;
					}
				}
			}
		}
	}
	return closestNode;
}

const TopologyEdge* TopologyLayer::getClosestEdgeInArea(const Point2D& point, const std::vector<int>& blockedEdges) const {
	const TopologyEdge* closestEdge = NULL;
	if (kelojsonMap != NULL && kelojsonMap->getAreasLayer() != NULL) {
		const Area* area = kelojsonMap->getAreasLayer()->getArea(point);
		if (area != NULL) {
			std::vector<unsigned int> edgeIds = getEdgesInArea(area);
			if (!edgeIds.empty()) {
				double leastDistance = std::numeric_limits<double>::max();
				for (unsigned int i = 0; i < edgeIds.size(); i++) {
					bool isBlocked = false;
					for (unsigned int bId = 0; bId < blockedEdges.size(); bId++) {
						if ((int)edgeIds[i] == blockedEdges[bId]) {
							isBlocked = true;
							break;
						}
					}
					if (!isBlocked) {
						double dist = minDistanceToEdge(point, edgeIds[i]);
						if (dist < leastDistance) {
							leastDistance = dist;
							closestEdge = &(topologyEdges.at(edgeIds[i]));
						}
					}
				}
			}
		}
	}
	return closestEdge;
}

std::vector<unsigned int> TopologyLayer::getConnectedEdges(const TopologyNode& node) const {
	std::vector<unsigned int> edges;
	for (TopologyEdgeMapConstItr itr = topologyEdges.begin(); itr != topologyEdges.end(); itr++) {
		if (itr->second.startNodeId == node.featureId || itr->second.endNodeId == node.featureId) {
			edges.push_back(itr->first);
		}
	}
	return edges;
}

double TopologyLayer::minDistanceToEdge(const Point2D& point, int edgeId) const {
	if (topologyEdges.find(edgeId) == topologyEdges.end())
		return std::numeric_limits<double>::max();

	const TopologyEdge& edge = topologyEdges.at(edgeId);
	const TopologyNode& start = topologyNodes.at(edge.startNodeId);
	const TopologyNode& end = topologyNodes.at(edge.endNodeId);
	return LineSegment2D(start.position, end.position).getMinDistFrom(point);
}

std::vector< std::vector<const kelojson::AreaTransition*> > TopologyLayer::getAreaTransitionsAlongPath(const std::vector<Point2D>& path) const {
	std::vector< std::vector<const kelojson::AreaTransition*> > transitionList;
	if (kelojsonMap != NULL) {
		for (unsigned int nodeId = 0; nodeId + 1 < path.size(); nodeId++) {
			Point2D start = path[nodeId];
			Point2D end = path[nodeId + 1];
			std::vector<const kelojson::AreaTransition*> transitionsOnEdge = kelojsonMap->getAreasLayer()->getIntersectingAreaTransitions(start, end);
			transitionList.push_back(transitionsOnEdge);
		}
	}
	return transitionList;
}

std::vector< std::vector<const Ramp*> > TopologyLayer::getRampsAlongPath(const std::vector<Point2D>& path) const {
	std::vector< std::vector<const Ramp*> > rampList;
	if (kelojsonMap != NULL && 
		kelojsonMap->getZonesLayer() != NULL && 
		kelojsonMap->getZonesLayer()->hasRamps()) {
		for (unsigned int nodeId = 0; nodeId + 1 < path.size(); nodeId++) {
			Point2D start = path[nodeId];
			Point2D end = path[nodeId + 1];
			std::vector<const Ramp*> rampsOnEdge = kelojsonMap->getZonesLayer()->getIntersectingRamps(start, end);
			rampList.push_back(rampsOnEdge);
		}
	}
	return rampList;
}

void TopologyLayer::loadInterlayerAssociations(const Map& map) {
	const std::vector<int>& relations = osmPrimitives.at(osm::primitiveType::RELATION);
	int nSuccess = 0;
	for (unsigned int i = 0; i < relations.size(); i++) {
		const osm::Relation* relation = map.getOsmRelation(relations[i]);
		if (relation != NULL && 
			relation->members.size() > 1 &&
			relation->relationType == RELATION_TYPE_ASSOCIATION) {
			const osm::RelationMember& parent = relation->members[0];
			if (parent.primitiveType == osm::primitiveType::NODE &&
				topologyNodes.find(parent.primitiveId) != topologyNodes.end()) {
				TopologyNode& node = topologyNodes[parent.primitiveId];
				for (unsigned int j = 1; j < relation->members.size(); j++) {
					const osm::RelationMember& child = relation->members[j];
					layerType::LayerType layerType = layerType::getType(child.role);
					switch (layerType) {
					case layerType::AREAS: {
						const AreasLayer* areasLayer = map.getAreasLayer();
						if (areasLayer != NULL && child.primitiveType == osm::primitiveType::WAY) {
							const Area* area = areasLayer->getArea(child.primitiveId);
							if (area != NULL) {
								if (node.interlayerAssociations.find(layerType::AREAS) == node.interlayerAssociations.end()) {
									node.interlayerAssociations.insert(std::make_pair(layerType::AREAS, std::set<int>()));
								}
								std::set<int>& overlappingAreas = node.interlayerAssociations[layerType::AREAS];
								overlappingAreas.insert(child.primitiveId);
								nSuccess++;
							}
						}
						break;
					}
					default:
						std::cout << "No implementation found to establish interlayer association from topology layer to "
								  << layerType::getName(layerType) << std::endl;
						break;
					}
				}
			}
			else {
				std::cout << "Failed to parse association relation " 
						  << relation->primitiveId 
						  << " in the Topology layer. Parent node not found!" 
						  << std::endl;
			}
		}
	}

	// std::cout << "\tSuccessfully loaded " << nSuccess << "/" << relations.size() 
	// 		  << " relations in the topology layer" << std::endl; 
}

}
