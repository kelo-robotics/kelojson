
#include <iostream>
#include <limits>
#include <geometry_common/LineSegment2D.h>
#include <geometry_common/Polygon2D.h>

#include "kelojson_loader/KelojsonMap.h"
#include "kelojson_loader/KelojsonAreasLayer.h"

using Polygon2D = kelo::geometry_common::Polygon2D;
using LineSegment2D = kelo::geometry_common::LineSegment2D;

namespace kelo {
namespace kelojson {

bool operator<(const AreaTransition& lhs, const AreaTransition& rhs) {
	if (lhs.coordinates.size() == rhs.coordinates.size()) {
		unsigned int nCoordinates = lhs.coordinates.size();
		for (unsigned int i = 0; i < nCoordinates; i++) {
			if (lhs.coordinates[i].x != rhs.coordinates[i].x)
				return lhs.coordinates[i].x < rhs.coordinates[i].x;
			else if (lhs.coordinates[i].y != rhs.coordinates[i].y)
				return lhs.coordinates[i].y < rhs.coordinates[i].y;
		}
		return false;
	} else {
		return lhs.coordinates.size() < rhs.coordinates.size();
	}
}

bool AreaTransition::isDoor() const {
	return doorType > doorTypes::NONE && doorType < doorTypes::COUNT;
}

double AreaTransition::width() const {
	return coordinates[0].distTo(coordinates[coordinates.size() - 1]);

}

areaTypes::AreaTypes areaTypes::getType(std::string type) {
	return type == AREA_TYPE_ROOM ? areaTypes::ROOM :
		   type == AREA_TYPE_CORRIDOR ? areaTypes::CORRIDOR :
		   type == AREA_TYPE_OPEN_AREA ? areaTypes::OPEN_AREA :
		   areaTypes::UNKNOWN;
}

std::string areaTypes::getName(AreaTypes type) {
	return type == areaTypes::ROOM ? "ROOM" :
		   type == areaTypes::CORRIDOR ? "CORRIDOR" :
		   type == areaTypes::OPEN_AREA ? "OPEN_AREA" : "UNKNOWN";
}

doorTypes::DoorTypes doorTypes::getType(std::string type) {
	return type == DOOR_TYPE_HINGED ? doorTypes::HINGED :
		   type == DOOR_TYPE_SLIDING ? doorTypes::SLIDING :
		   doorTypes::NONE;
}

std::string doorTypes::getName(DoorTypes type) {
	return type == doorTypes::HINGED ? "HINGED" :
		   type == doorTypes::SLIDING ? "SLIDING" : "NONE";
}

std::vector<int> Area::getAdjacentAreas() const {
	std::set<int> adjAreas;
	for (std::map<int, std::set<AreaTransition> >::const_iterator itr = transitions.begin(); itr != transitions.end(); itr++) {
		adjAreas.insert(itr->first);
	}
	return std::vector<int>(adjAreas.begin(), adjAreas.end());
}

std::vector<AreaTransition> Area::getTransitionsWithArea(int adjAreaId) const {
	if (transitions.find(adjAreaId) != transitions.end()) {
		const std::set<AreaTransition>& t = transitions.at(adjAreaId);
		return std::vector<AreaTransition>(t.begin(), t.end());
	}
	return std::vector<AreaTransition>();
}

std::map<int, std::vector<AreaTransition> > Area::getAllDoors() const {
	std::map<int, std::vector<AreaTransition> > doors;
	for (std::map<int, std::set<AreaTransition> >::const_iterator aItr = transitions.begin(); aItr != transitions.end(); aItr++) {
		for (std::set<AreaTransition>::const_iterator tItr = aItr->second.begin(); tItr != aItr->second.end(); tItr++) {
			if (tItr->doorType != doorTypes::NONE && tItr->doorType != doorTypes::COUNT) {
				if (doors.find(aItr->first) == doors.end()) {
					doors.insert(std::make_pair(aItr->first, std::vector<AreaTransition>()));
				}
				doors.at(aItr->first).push_back(*tItr);
			}
		}
	}
	return doors;
}

std::vector<AreaTransition> Area::getDoorsWithArea(int adjAreaId) const {
	std::map<int, std::vector<AreaTransition> > allDoors = getAllDoors();
	if (allDoors.find(adjAreaId) != allDoors.end())
		return allDoors.at(adjAreaId);
	return std::vector<AreaTransition>();
}

bool Area::contains(Point2D point) const {
	return Polygon2D(coordinates).containsPoint(point);
}

bool Area::insideBoundingBox(Point2D pt) const {
	return pt.x > boundingBox.first.x &&
		   pt.x < boundingBox.second.x &&
		   pt.y > boundingBox.first.y &&
		   pt.y < boundingBox.second.y;
}

double Area::getBoundingBoxArea() const {
	return abs((boundingBox.second.x - boundingBox.first.x) * 
			   (boundingBox.second.y - boundingBox.first.y));
}

AreasLayer::AreasLayer(const Map* map)
: Layer(layerType::AREAS, map) {

}

void AreasLayer::loadGeometries(const Map& map) {
	// Presently we only have Polygon geometries in this layer. Hence skip processing other geometric types
	const std::vector<int>& ways = osmPrimitives.at(osm::primitiveType::WAY);
	for (unsigned int i = 0; i < ways.size(); i++) {
		const osm::Way* way = map.getOsmWay(ways[i]);
		if (way != NULL) {
			if (way->wayType != "Polygon")
				continue;

			Area area;
			area.featureId = way->primitiveId;
			std::string indoorVal;
			way->getTagValue("indoor", indoorVal);
			area.areaType = areaTypes::getType(indoorVal);
			way->getTagValue("name", area.name);

			bool success = true;
			area.boundingBox.first = Point2D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
			area.boundingBox.second = Point2D(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
			for (unsigned int i = 0; i < way->nodeIds.size(); i++) {
				const osm::Node* node = map.getOsmNode(way->nodeIds[i]);
				if (node == NULL) {
					success = false;
					break;
				}
				Point2D pos = node->position;
				area.coordinates.push_back(pos);

				// Update bounding box
				if (pos.x < area.boundingBox.first.x)
					area.boundingBox.first.x = pos.x;
				if (pos.x > area.boundingBox.second.x)
					area.boundingBox.second.x = pos.x;

				if (pos.y < area.boundingBox.first.y)
					area.boundingBox.first.y = pos.y;
				if (pos.y > area.boundingBox.second.y)
					area.boundingBox.second.y = pos.y;
			}

			if (success) {
				areas.insert(std::make_pair(area.featureId, area));
			}
			else {
				std::cout << "Failed to load area: " << area.featureId << std::endl;
			}
		}
	}
	//std::cout << "\tLoaded " << areas.size() << " areas." << std::endl;
}

void AreasLayer::loadRelations(const Map& map) {
	if (osmPrimitives.find(osm::primitiveType::RELATION) != osmPrimitives.end()) {
		loadAreaTransitions(map);
	}
}

void AreasLayer::loadAreaTransitions(const Map& map) {
	const std::vector<int>& relations = osmPrimitives.at(osm::primitiveType::RELATION);
	int nSuccess = 0;
	for (unsigned int i = 0; i < relations.size(); i++) {
		const osm::Relation* relation = map.getOsmRelation(relations[i]);
		if (relation != NULL && 
			relation->members.size() >= 4 && // transition must have atleast two areas and two nodes as members
			relation->relationType == RELATION_TYPE_TRANSITION) {
			const osm::RelationMember& area1 = relation->members[0];
			const osm::RelationMember& area2 = relation->members[1];

			if (areas.find(area1.primitiveId) == areas.end() ||
				areas.find(area2.primitiveId) == areas.end()) {
				std::cout << "\tCould not find one or both parent areas of transition: " 
						  << relation->primitiveId << std::endl;
				continue;
			}

			std::string doorType;
			relation->getTagValue("door", doorType);

			std::string name;
			relation->getTagValue("name", name);

			AreaTransition transition;
			transition.doorType = doorTypes::getType(doorType);
			transition.featureId = relation->primitiveId;
			transition.associatedAreaIds = std::make_pair(area1.primitiveId, area2.primitiveId);
			if (name.empty())
			{
				// Construct a name for the transition
				std::stringstream ss;
				ss << (transition.doorType == doorTypes::NONE ? "AreaTransition-" : "Door-");
				ss << getArea(transition.associatedAreaIds.first)->name << "-";
				ss << getArea(transition.associatedAreaIds.second)->name;
				name = ss.str();
			}
			transition.name = name;

			bool success = true;
			for (unsigned int i = 2; i < relation->members.size(); i++) {
				const osm::RelationMember& node = relation->members[i];
				const osm::Node* n = map.getOsmNode(node.primitiveId);
				if (node.primitiveType != osm::primitiveType::NODE || n == NULL) {
					success = false;
					break;
				}
				transition.coordinates.push_back(n->position);
			}

			if (success) {
				std::map<int, std::set<AreaTransition> >& transitions1 = areas.at(area1.primitiveId).transitions;
				std::map<int, std::set<AreaTransition> >& transitions2 = areas.at(area2.primitiveId).transitions;
				if (transitions1.find(area2.primitiveId) == transitions1.end()) {
					transitions1.insert(std::make_pair(area2.primitiveId, std::set<AreaTransition>()));
				}
				if (transitions2.find(area1.primitiveId) == transitions2.end()) {
					transitions2.insert(std::make_pair(area1.primitiveId, std::set<AreaTransition>()));
				}

				transitions1.at(area2.primitiveId).insert(transition);
				transitions2.at(area1.primitiveId).insert(transition);
				nSuccess++;
			}
		}
	}

	// std::cout << "\tSuccessfully loaded " << nSuccess << "/" << relations.size() 
	// 		  << " relations in the Areas layer" << std::endl;
}

const Area* AreasLayer::getArea(int areaId) const {
	if (areas.find(areaId) != areas.end()) {
		return &(areas.at(areaId));
	}
	return NULL;
}


const Area* AreasLayer::getArea(const std::string& areaName) const {
	std::pair<bool, int> result = getAreaId(areaName);
	if (result.first)
		return getArea(result.second);
	return NULL;
}


const Area* AreasLayer::getArea(const Point2D& point) const {
	std::multiset<const Area*, AreaBBoxComparator> sortedAreas = getAreasByBBoxSize();
	for (std::multiset<const Area*, AreaBBoxComparator>::const_iterator itr = sortedAreas.begin(); itr != sortedAreas.end(); itr++) {
		if ((*itr)->contains(point))
			return getArea((*itr)->featureId);
	}
	return NULL;
}

const AreaTransition* AreasLayer::getAreaTransition(int featureId) const {
	std::map<int, const AreaTransition*> transitions = getAllAreaTransitions();
	for (std::map<int, const AreaTransition*>::const_iterator itr = transitions.begin(); itr != transitions.end(); itr++) {
		if (itr->second->featureId == featureId)
			return itr->second;
	}
	return NULL;
}

std::map<int, const AreaTransition*> AreasLayer::getAllAreaTransitions() const {
	std::map<int, const AreaTransition*> transitions;
	for (std::map<int, Area>::const_iterator areaItr = areas.begin(); areaItr != areas.end(); areaItr++) {
		const std::map<int, std::set<AreaTransition> >& areaTransitions = areaItr->second.transitions;
		for (std::map<int, std::set<AreaTransition> >::const_iterator transitionItr = areaTransitions.begin(); transitionItr != areaTransitions.end(); transitionItr++) {
			for (std::set<AreaTransition>::const_iterator itr = transitionItr->second.begin(); itr != transitionItr->second.end(); itr++) {
				if (transitions.find(itr->featureId) == transitions.end()) {
					transitions.insert(std::make_pair(itr->featureId, &(*itr)));
				}
			}
		}
	}
	return transitions;
}

bool AreasLayer::getDistanceToTransition(const Point2D& point, const AreaTransition* transition, double& distance) const {
	if (transition != NULL) {
		// Find the closest point on the transition
		Point2D closestPoint = transition->coordinates[0];
		double shortestDistance = closestPoint.distTo(point);
		for (unsigned int i = 1; i < transition->coordinates.size(); i++) {
			double dist = transition->coordinates[i].distTo(point);
			if (dist < shortestDistance) {
				shortestDistance = dist;
				closestPoint = transition->coordinates[i];
			}
		}
		distance = shortestDistance;
		return true;
	}
	distance = std::numeric_limits<double>::max();
	return false;
}

std::vector<const AreaTransition*> AreasLayer::getNearestTransitions(const Point2D& point, double searchRadius, unsigned int maxNumTransitions) const {
	if (maxNumTransitions <= 0)
		return std::vector<const AreaTransition*>();

	std::map<int, const AreaTransition*> allTransitions = getAllAreaTransitions();

	// Find distances to transitions and store them in a sorted ascending order (based on distance) using a multimap
	std::multimap<double, const AreaTransition*> distToTransitionsMap;
	for(std::map<int, const AreaTransition*>::const_iterator mItr = allTransitions.begin(); mItr != allTransitions.end(); mItr++) {
		double shortestDistance;
		if (getDistanceToTransition(point, mItr->second, shortestDistance) && shortestDistance <= searchRadius) {
			distToTransitionsMap.insert(std::make_pair(shortestDistance, mItr->second));
		}
	}

	if (distToTransitionsMap.empty()) {
		return std::vector<const AreaTransition*>();
	}

	// Create and return a vector of the nearest transitions upto a desired number of transtions
	std::vector<const AreaTransition*> nearestTransitions;
	std::multimap<double, const AreaTransition*>::const_iterator itr = distToTransitionsMap.begin();
	while (nearestTransitions.size() < maxNumTransitions && itr != distToTransitionsMap.end()) {
		nearestTransitions.push_back(itr->second);
		itr++;
	}

	return nearestTransitions;
}

std::string AreasLayer::getAreaName(int areaId) const {
	std::string name;
	const Area* area = getArea(areaId);
	if (area != NULL)
		name = area->name;
	return name;
}


std::pair<bool, int> AreasLayer::getAreaId(const std::string& areaName) const {
	bool found = false;
	int areaId = 0;
	for (std::map<int, Area>::const_iterator itr = areas.begin(); itr != areas.end(); itr++) {
		if (itr->second.name == areaName) {
			found = true;
			areaId = itr->first;
			break;
		}
	}
	return std::make_pair(found, areaId);
}

std::vector<int> AreasLayer::computePath(int startAreaId, int goalAreaId) const {
	if (areas.find(startAreaId) == areas.end()) {
		std::cout << "Cannot compute path between areas. INVALID START AREA ID!" << std::endl;
		return std::vector<int>();
	}

	if (areas.find(goalAreaId) == areas.end()) {
		std::cout << "Cannot compute path between areas. INVALID GOAL AREA ID!" << std::endl;
		return std::vector<int>();
	}

	// Breadth first search
	std::list<int> queue;
	std::map<int, bool> visited;
	std::map<int, int> parents;
	for (std::map<int, Area>::const_iterator itr = areas.begin(); itr != areas.end(); itr++) {
		visited.insert(std::make_pair(itr->first, false));
	}
	visited.at(startAreaId) = true;
	queue.push_back(startAreaId);

	while (!queue.empty()) {
		int currAreaId = queue.front();
		queue.pop_front();
		if (currAreaId == goalAreaId)
			break;

		// Check adjacent areas
		const Area* area = getArea(currAreaId);
		if (area != NULL) {
			std::vector<int> adjAreas = area->getAdjacentAreas();
			for (unsigned int i = 0; i < adjAreas.size(); i++) {
				int adjAreaId = adjAreas[i];
				if (!visited.at(adjAreaId)) {
					parents.insert(std::make_pair(adjAreaId, currAreaId));
					queue.push_back(adjAreaId);
					visited.at(adjAreaId) = true;
				}
			}
		}
	}

	// Reconstruct the path if goal reached
	std::list<int> path;
	if (parents.find(goalAreaId) != parents.end()) {
		int currAreaId = goalAreaId;

		while (currAreaId != startAreaId) {
			path.push_front(currAreaId);
			currAreaId = parents.at(currAreaId);
		}
		path.push_front(startAreaId);
	}

	return std::vector<int>(path.begin(), path.end());
}

std::vector<int> AreasLayer::computePath(const std::string& startAreaName, const std::string& goalAreaName) const {
	std::pair<bool, int> startRes = getAreaId(startAreaName);
	std::pair<bool, int> goalRes = getAreaId(goalAreaName);
	if (startRes.first && goalRes.first) {
		return computePath(startRes.second, goalRes.second);
	}
	return std::vector<int>();
}

std::vector<int> AreasLayer::computePath(const Point2D& startPos, const Point2D& goalPos) const {
	const Area* startArea = getArea(startPos);
	const Area* goalArea = getArea(goalPos);
	if (startArea!= NULL && goalArea != NULL) {
		// std::cout << "Computing path from " << startArea->name.c_str() << " to "
		// 		  << goalArea->name.c_str() << std::endl;
		return computePath(startArea->featureId, goalArea->featureId);
	}
	return std::vector<int>();
}

std::string AreasLayer::getPrintablePath(const std::vector<int>& path) const {
	std::string str;
	for (std::vector<int>::const_iterator itr = path.begin(); itr != path.end(); itr++) {
		if (!str.empty())
			str += " -> ";
		str += getAreaName(*itr);
	}
	return str;
}

std::vector<const AreaTransition*> AreasLayer::getIntersectingAreaTransitions(const std::vector<Point2D>& lineString) const {
	if (lineString.size() < 2)
		return std::vector<const AreaTransition*>();

	std::vector<const AreaTransition*> intersections;
	std::map<int, const AreaTransition*> transitions = getAllAreaTransitions();

	for(unsigned int edgeIdx = 0; edgeIdx < lineString.size() - 1; edgeIdx++) {
		LineSegment2D edge(lineString[edgeIdx], lineString[edgeIdx + 1]);
		for (std::map<int, const AreaTransition*>::const_iterator itr = transitions.begin(); itr != transitions.end(); itr++) {
			const AreaTransition* trans = itr->second;
			if (trans != NULL) {
				for (unsigned int transEdgeIdx = 0; transEdgeIdx < trans->coordinates.size() - 1; transEdgeIdx++) {
					Point2D transEdgeNode1 = trans->coordinates[transEdgeIdx];
					Point2D transEdgeNode2 = trans->coordinates[transEdgeIdx + 1];
					LineSegment2D transEdge(trans->coordinates[transEdgeIdx],
											trans->coordinates[transEdgeIdx + 1]);
					if (edge.intersects(transEdge)) {
						if (std::find(intersections.begin(), intersections.end(), trans) == intersections.end()) {
							intersections.push_back(trans);
							break;
						}
					}
				}
			}
		}
	}
	return intersections;
}

std::vector<const AreaTransition*> AreasLayer::getIntersectingAreaTransitions(const Point2D& start, const Point2D& end) const {
	std::vector<Point2D> lineString;
	lineString.push_back(start);
	lineString.push_back(end);
	return getIntersectingAreaTransitions(lineString);
}

bool AreasLayer::contains(const Point2D& pos) const {
	for (std::map<int, Area>::const_iterator itr = areas.begin(); itr != areas.end(); itr++) {
		if (itr->second.contains(pos)) {
			return true;
		}
	}
	return false;
}

std::multiset<const Area*, AreaBBoxComparator> AreasLayer::getAreasByBBoxSize() const {
	std::multiset<const Area*, AreaBBoxComparator> sortedAreas;
	for (std::map<int, Area>::const_iterator itr = areas.begin(); itr != areas.end(); itr++) {
		sortedAreas.insert(&(itr->second));
	}
	return sortedAreas;
}

}
}
