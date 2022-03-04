#include "global_map/KelojsonMap.h"
#include "global_map/KelojsonUtils.h"
#include "global_map/KelojsonAreasLayer.h"
#include "global_map/Utils.h"
#include "global_map/KelojsonZonesLayer.h"
#include <iostream>

namespace kelojson {

zoneTypes::ZoneTypes zoneTypes::getType(std::string type) {
	return type == ZONE_TYPE_ELEVATOR ? zoneTypes::FORBIDDEN :
		   type == ZONE_TYPE_STAIRS? zoneTypes::STAIRS :
		   type == ZONE_TYPE_FORBIDDEN ? zoneTypes::FORBIDDEN :
		   type == ZONE_TYPE_SLOWDOWN ? zoneTypes::SLOWDOWN :
		   type == ZONE_TYPE_RAMP ? zoneTypes::RAMP :
		   type == ZONE_TYPE_CHARGING_STATION ? zoneTypes::CHARGING_STATION  :
		   type == ZONE_TYPE_WAITING_LOCATION ? zoneTypes::WAITING_LOCATION  :
		   type == ZONE_TYPE_OCCLUSION ? zoneTypes::OCCLUSION  : zoneTypes::UNKNOWN;
}

std::string zoneTypes::getName(ZoneTypes type) {
	return type == zoneTypes::FORBIDDEN ? "FORBIDDEN" :
		   type == zoneTypes::STAIRS ? "STAIRS" :
		   type == zoneTypes::FORBIDDEN ? "FORBIDDEN" :
		   type == zoneTypes::SLOWDOWN ? "SLOWDOWN" :
		   type == zoneTypes::RAMP ? "RAMP" :
		   type == zoneTypes::CHARGING_STATION ? "CHARGING_STATION" :
		   type == zoneTypes::WAITING_LOCATION ? "WAITING_LOCATION" :
		   type == zoneTypes::OCCLUSION ? "OCCLUSION" : "UNKNOWN";
}

Zone::Zone(int featId, zoneTypes::ZoneTypes type, osm::primitiveType::PrimitiveType primType, const std::string& zoneName) 
: featureId(featId)
, zoneType(type)
, name(zoneName)
, primitiveType(primType)
{
}

bool Zone::hasName() const {
	return !name.empty();
}

std::string Zone::getName() const {
	if (hasName()) {
		return name;
	} else {
		std::stringstream ss;
		ss << featureId;
		return ss.str();
	}
}

std::vector<int> Zone::getOverlappingAreaIds() const {
	if (interlayerAssociations.find(layerType::AREAS) != interlayerAssociations.end()) {
		const std::set<int>& associations = interlayerAssociations.at(layerType::AREAS);
		return std::vector<int>(associations.begin(), associations.end());
	}
	return std::vector<int>();
}

ZoneNode::ZoneNode(int featId, zoneTypes::ZoneTypes type, osm::primitiveType::PrimitiveType primType, const Pose& p, const std::string& zoneName)
: Zone(featId, type, primType, zoneName)
, pose(p)
{
}

ZoneLine::ZoneLine(int featId, zoneTypes::ZoneTypes type, osm::primitiveType::PrimitiveType primType, const std::vector<Pos>& coords, const std::string& zoneName)
: Zone(featId, type, primType, zoneName)
, coordinates(coords)
{
}

ZonePolygon::ZonePolygon(int featId, zoneTypes::ZoneTypes type, osm::primitiveType::PrimitiveType primType, const std::vector<Pos>& coords, const std::string& zoneName)
: Zone(featId, type, primType, zoneName)
, coordinates(coords)
{
}

bool ZonePolygon::contains(Pos point) const {
	return kelojson::utils::pointInPolygon(point, coordinates);
}

Ramp::Ramp(int featureId, zoneTypes::ZoneTypes zoneType,
		   osm::primitiveType::PrimitiveType primitiveType,
		   const std::vector<Pos>& coordinates, double incline, const std::string& name) 
: ZonePolygon(featureId, zoneType, primitiveType, coordinates, name)
, inclination(incline) {
}

bool Ramp::valid() const {
	return coordinates.size() >= 4 &&	// Ramp is atleast a quad
		   topNodes.size() >= 2 &&		// Minimum two nodes marked as top nodes
		   bottomNodes.size() >= 2;		// Minimum two nodes marked as bottom nodes
}

std::vector<Pos> Ramp::getBottomNodePositions() const {
	std::vector<Pos> positions;
	for (unsigned int i = 0; i < bottomNodes.size(); i++) {
		positions.push_back(coordinates[bottomNodes[i]]);
	}
	return positions;
}

std::vector<Pos> Ramp::getTopNodePositions() const {
	std::vector<Pos> positions;
	for (unsigned int i = 0; i < topNodes.size(); i++) {
		positions.push_back(coordinates[topNodes[i]]);
	}
	return positions;
}

bool Ramp::intersectsEdge(const Pos& edgeStart, const Pos& edgeEnd) const {
	return kelojson::utils::edgeTouchesPolygon(edgeStart, edgeEnd, coordinates);
}

std::vector<int> Ramp::getOverlappingAreaTransitionIds(const Map* kelojsonMap) const {
	std::vector<int> overlappingTransitionIds;

	if (!kelojsonMap || !kelojsonMap->getAreasLayer()) {
		return overlappingTransitionIds;
	}

	std::vector<int> areaIds = getOverlappingAreaIds();
	if (areaIds.size() < 2) {
		// A zone should overlap with atleast two areas to have an overlapping transition
		return overlappingTransitionIds;
	}

	for (unsigned int i = 0; i < areaIds.size() - 1; i++) {
		for (unsigned int j = i+1; j < areaIds.size(); j++) {
			std::vector<AreaTransition> transitions = kelojsonMap->getAreasLayer()->getArea(areaIds[i])->getTransitionsWithArea(areaIds[j]);
			for (unsigned int k = 0; k < transitions.size(); k++) {
				Pos tStart = transitions[k].coordinates[0];
				Pos tEnd = transitions[k].coordinates[transitions[k].coordinates.size() - 1];
				if (intersectsEdge(tStart, tEnd)) {
					overlappingTransitionIds.push_back(transitions[k].featureId);
				}
			}
		}
	}

	return overlappingTransitionIds;
}

OcclusionRegion::OcclusionRegion(int id)
: featureId(id) {
}

OcclusionRegion::~OcclusionRegion() {
}

int OcclusionRegion::getFeatureId() const {
	return featureId;
}

void OcclusionRegion::addOcclusionLine(const ZoneLine* line) {
	if (!line || occlusionLines.find(line->getFeatureId()) != occlusionLines.end()) {
		return;
	}

	occlusionLines.insert(std::make_pair(line->getFeatureId(), line));
}

void OcclusionRegion::addAreaTransition(const AreaTransition* transition) {
	if (!transition || areaTransitions.find(transition->featureId) != areaTransitions.end()) {
		return;
	}

	areaTransitions.insert(std::make_pair(transition->featureId, transition));
}

bool OcclusionRegion::overlapsWithLineString(std::vector<Pos> lineString) const {
	bool overlaps = false;
	for (unsigned int i = 0; (i+1) < lineString.size() && !overlaps; i++) {
		overlaps = overlapsWithLineSegment(lineString[i], lineString[i + 1]);
	}

	return overlaps;
}

bool OcclusionRegion::overlapsWithLineSegment(Pos lineSegStart, Pos lineSegEnd) const {
	bool overlaps = false;

	for (std::map<int, const ZoneLine*>::const_iterator itr = occlusionLines.begin(); itr != occlusionLines.end() && !overlaps; itr++) {
		const ZoneLine* occlusionLine = itr->second;
		if (!occlusionLine || occlusionLine->getCoordinatesRef().size() < 2)
			continue;
		overlaps = utils::edgeTouchesLineString(lineSegStart, lineSegEnd, occlusionLine->getCoordinatesRef());
	}

	for (std::map<int, const AreaTransition*>::const_iterator itr = areaTransitions.begin(); itr != areaTransitions.end() && !overlaps; itr++) {
		const AreaTransition* transition = itr->second;
		if (!transition || transition->coordinates.size() < 2)
			continue;
		overlaps = utils::edgeTouchesLineString(lineSegStart, lineSegEnd, transition->coordinates);
	}

	return overlaps;
}

bool OcclusionRegion::getFirstPointOfContactWithLineString(std::vector<Pos> lineString, Pos& contactPoint) const {
	bool success = false;
	for (unsigned int i = 0; (i+1) < lineString.size(); i++) {
		Pos closestContactPoint;
		if (getFirstPointOfContactWithLineSegment(lineString[i], lineString[i + 1], closestContactPoint)) {
			contactPoint = closestContactPoint;
			success = true;
			break;
		}
	}
	return success;
}

bool OcclusionRegion::getFirstPointOfContactWithLineSegment(Pos lineSegStart, Pos lineSegEnd, Pos& contactPoint) const {
	std::map<int, Pos> contactPoints;

	for (std::map<int, const ZoneLine*>::const_iterator itr = occlusionLines.begin(); itr != occlusionLines.end(); itr++) {
		const ZoneLine* occlusionLine = itr->second;
		if (!occlusionLine || occlusionLine->getCoordinatesRef().size() < 2)
			continue;

		Pos closestContactPoint;
		if (getNearestIntersectionPoint(lineSegStart, lineSegEnd, occlusionLine->getCoordinatesRef(), closestContactPoint)) {
			contactPoints.insert(std::make_pair(lineSegStart.dist(closestContactPoint), closestContactPoint));
		}
	}

	for (std::map<int, const AreaTransition*>::const_iterator itr = areaTransitions.begin(); itr != areaTransitions.end(); itr++) {
		const AreaTransition* transition = itr->second;
		if (!transition || transition->coordinates.size() < 2)
			continue;

		Pos closestContactPoint;
		if (getNearestIntersectionPoint(lineSegStart, lineSegEnd, transition->coordinates, closestContactPoint)) {
			contactPoints.insert(std::make_pair(lineSegStart.dist(closestContactPoint), closestContactPoint));
		}
	}

	if (!contactPoints.empty()) {
		contactPoint = contactPoints.begin()->second;
	}

	return !contactPoints.empty();
}

double OcclusionRegion::dist(const Pos& queryPoint) const {
	double minDist = std::numeric_limits<double>::max();
	for (std::map<int, const ZoneLine*>::const_iterator itr = occlusionLines.begin(); itr != occlusionLines.end(); itr++) {
		const ZoneLine* occlusionLine = itr->second;
		if (!occlusionLine || occlusionLine->getCoordinatesRef().size() < 2)
			continue;

		for (unsigned int i = 0; i < occlusionLine->getCoordinatesRef().size(); i++) {
			double dist = queryPoint.dist(occlusionLine->getCoordinatesRef()[i]);
			if (dist < minDist) {
				minDist = dist;
			}
		}
	}

	for (std::map<int, const AreaTransition*>::const_iterator itr = areaTransitions.begin(); itr != areaTransitions.end(); itr++) {
		const AreaTransition* transition = itr->second;
		if (!transition || transition->coordinates.size() < 2)
			continue;

		for (unsigned int i = 0; i < transition->coordinates.size(); i++) {
			double dist = queryPoint.dist(transition->coordinates[i]);
			if (dist < minDist) {
				minDist = dist;
			}
		}
	}

	return minDist;
}

bool OcclusionRegion::contains(const Pos& point) const {
	if (!polyCoordinates.empty()) {
		return kelojson::utils::pointInPolygon(point, polyCoordinates);
	}
	return false;
}

bool OcclusionRegion::empty() const {
	return areaTransitions.empty() && occlusionLines.empty();
}

bool OcclusionRegion::generatePolygonCoords() {
	polyCoordinates.clear(); // clear previously generated poly coordinates

	std::vector< std::vector<Pos> > lineStrings;
	for (std::map<int, const kelojson::ZoneLine*>::const_iterator itr = occlusionLines.begin(); itr != occlusionLines.end(); itr++) {
		const kelojson::ZoneLine* line = itr->second;
		if (line && line->getCoordinatesRef().size() > 1) {
			lineStrings.push_back(line->getCoordinates());
		}
	}
	for (std::map<int, const kelojson::AreaTransition*>::const_iterator itr = areaTransitions.begin(); itr != areaTransitions.end(); itr++) {
		const kelojson::AreaTransition* transition = itr->second;
		if (transition && transition->coordinates.size() > 1) {
			lineStrings.push_back(transition->coordinates);
		}
	}

	if (lineStrings.size() < 2) {
		return false;
	}

	std::vector<unsigned int> lineStringOrder;
	std::vector<bool> reverse;
	unsigned int testLineId = 0;
	bool isConnected = false;

	lineStringOrder.push_back(testLineId);
	reverse.push_back(false);
	do {
		const std::vector<Pos>& lastLine = lineStrings[lineStringOrder.back()];
		isConnected = false;
		for (unsigned int i = 0; i < lineStrings.size(); i++) {
			if (std::find(lineStringOrder.begin(), lineStringOrder.end(), i) == lineStringOrder.end()) {
				const std::vector<Pos>& newTestLine = lineStrings[i];
				if (lineStringOrder.size() == 1) { // first line?
					isConnected = lastLine.front() == newTestLine.front() ||
								  lastLine.front() == newTestLine.back() ||
								  lastLine.back() == newTestLine.front() ||
								  lastLine.back() == newTestLine.back();
					if (isConnected) {
						// assign the correct direction for the first line
						reverse[0] = lastLine.front() == newTestLine.front() ||
									 lastLine.front() == newTestLine.back();

						// Insert the connectedLine
						lineStringOrder.push_back(i);
						reverse.push_back(lastLine.front() == newTestLine.back() ||
										  lastLine.back() == newTestLine.back());
					}
				} else {
					const Pos& lineEndPt = reverse.back() ? lastLine.front() : lastLine.back();

					isConnected = lineEndPt == newTestLine.front() ||
								  lineEndPt == newTestLine.back();
					if (isConnected) {
						// Insert the connectedLine
						lineStringOrder.push_back(i);
						reverse.push_back(lineEndPt == newTestLine.back());
					}
				}

				if (isConnected) {
					break;
				}
			}
		}
	} while(lineStringOrder.size() < lineStrings.size() && isConnected);

	bool success = lineStringOrder.size() == lineStrings.size() &&
				   lineStringOrder.size() == reverse.size();

	if (success) {
		for (unsigned int i = 0; i < lineStringOrder.size(); i++) {
			const std::vector<Pos>& line = lineStrings[lineStringOrder[i]];
			if (reverse[i]) {
				int j = polyCoordinates.empty() ? line.size() - 1 : line.size() - 2;
				for ( ; j >= 0; j--) {
					polyCoordinates.push_back(line[j]);
				}
			} else {
				int j = polyCoordinates.empty() ? 0 : 1;
				for ( ; j < (int)line.size(); j++) {
					polyCoordinates.push_back(line[j]);
				}
			}
		}
		polyCoordinates.push_back(polyCoordinates.front()); // Close the line loop
	}
	return success;
}

const std::vector<Pos>& OcclusionRegion::asPolygon() const {
	return polyCoordinates;
}

ZonesLayer::ZonesLayer(const Map* map)
: Layer(layerType::ZONES, map) {
	
}

ZonesLayer::~ZonesLayer() {
	for (ZonesStoreItr itr1 = zones.begin(); itr1 != zones.end(); itr1++) {
		for (ZonesMapItr itr2 = itr1->second.begin(); itr2 != itr1->second.end(); itr2++) {
			delete itr2->second;
			itr2->second = NULL;
		}
		itr1->second.clear();
	}
	zones.clear();
}

void ZonesLayer::loadGeometries(const Map& map) {
	loadOsmWays(map);
	loadOsmNodes(map);
}

void ZonesLayer::loadOsmNodes(const Map& map) {
	if (osmPrimitives.find(osm::primitiveType::NODE) == osmPrimitives.end())
		return;

	const std::vector<int>& nodes = osmPrimitives.at(osm::primitiveType::NODE);
	if (nodes.empty())
		return;

	if (zones.find(osm::primitiveType::NODE) == zones.end()) {
		zones.insert(std::make_pair(osm::primitiveType::NODE, ZonesMap()));
	}
	ZonesMap& nodeZones = zones.at(osm::primitiveType::NODE);
	unsigned int nSuccess = 0;
	for (unsigned int i = 0; i < nodes.size(); i++) {
		const osm::Node* node = map.getOsmNode(nodes[i]);
		if (node != NULL) {
			double theta = 0.0;
			node->getTagValue("theta", theta);
			nodeZones.insert(std::make_pair(node->primitiveId, 
											new ZoneNode(node->primitiveId,
														 getZoneType(node),
														 osm::primitiveType::NODE,
														 Pose(node->position, theta),
														 getZoneName(node))));
			nSuccess++;
		}
	}
	//std::cout << "\tSuccessfully Loaded " << nSuccess << "/" << nodes.size() << " zone nodes." << std::endl;
}

void ZonesLayer::loadOsmWays(const Map& map) {
	if (osmPrimitives.find(osm::primitiveType::WAY) == osmPrimitives.end())
		return;

	const std::vector<int>& ways = osmPrimitives.at(osm::primitiveType::WAY);
	if (ways.empty())
		return;

	if (zones.find(osm::primitiveType::WAY) == zones.end()) {
		zones.insert(std::make_pair(osm::primitiveType::WAY, ZonesMap()));
	}

	ZonesMap& wayZones = zones.at(osm::primitiveType::WAY);
	unsigned int nSuccess = 0;
	for (unsigned int i = 0; i < ways.size(); i++) {
		const osm::Way* way = map.getOsmWay(ways[i]);
		if (!way || (way->wayType != "Polygon" && way->wayType != "LineString"))
			continue;

		std::vector<Pos> coordinates;
		bool success = true;
		for (unsigned int i = 0; i < way->nodeIds.size(); i++) {
			const osm::Node* node = map.getOsmNode(way->nodeIds[i]);
			if (node == NULL) {
				success = false;
				break;
			}
			coordinates.push_back(node->position);
		}

		if (success) {
			zoneTypes::ZoneTypes zoneType = getZoneType(way);
			if (zoneType == zoneTypes::OCCLUSION) {
				wayZones.insert(std::make_pair(way->primitiveId,
											   new ZoneLine(way->primitiveId,
															zoneType,
															osm::primitiveType::WAY,
															coordinates,
															getZoneName(way))));
			} else if (zoneType == zoneTypes::RAMP) {
				wayZones.insert(std::make_pair(way->primitiveId, 
											   new Ramp(way->primitiveId,
														zoneType,
														osm::primitiveType::WAY,
														coordinates,
														getRampInclination(way),
														getZoneName(way))));
			} else {
				wayZones.insert(std::make_pair(way->primitiveId,
											   new ZonePolygon(way->primitiveId,
															   zoneType,
															   osm::primitiveType::WAY,
															   coordinates,
															   getZoneName(way))));
			}
			nSuccess++;
		}
		else {
			std::cout << "Failed to load zone: " << way->primitiveId << std::endl;
		}
	}
	//std::cout << "\tSuccessfully Loaded " << nSuccess << "/" << ways.size() << " zone polygons." << std::endl;
}

void ZonesLayer::loadRelations(const Map& map) {
	loadOcclusionRegions(map);
	loadInterlayerAssociations(map);
	loadIntralayerAssociations(map);
}

std::vector<const ZoneNode*> ZonesLayer::getAllChargingStations() const {
	return getNodes(zoneTypes::CHARGING_STATION);
}

std::vector<const ZoneNode*> ZonesLayer::getAllWaitingLocations() const {
	return getNodes(zoneTypes::WAITING_LOCATION);
}

std::vector<const ZonePolygon*> ZonesLayer::getAllForbiddenAreas() const {
	return getPolygons(zoneTypes::FORBIDDEN);
}

const ZoneLine* ZonesLayer::getOcclusionLine(int featureId) const {
	const ZoneLine* line = NULL;
	std::vector<const ZoneLine*> occlusionLines = getAllOcclusionLines();
	for (unsigned int i = 0; i < occlusionLines.size(); i++) {
		if (occlusionLines[i] && occlusionLines[i]->getFeatureId() == featureId) {
			line = occlusionLines[i];
			break;
		}
	}
	return line;
}

std::vector<const ZoneLine*> ZonesLayer::getAllOcclusionLines() const {
	return getLines(zoneTypes::OCCLUSION);
}

std::vector< std::vector<const ZoneLine*> > ZonesLayer::getIntersectingOcclusionLines(const std::vector<Pos>& path) const {
	std::vector< std::vector<const ZoneLine*> > lines;

	if (path.size() < 2)
		return lines;

	std::vector<const ZoneLine*> occlusions = getAllOcclusionLines();
	for (unsigned int i = 0; (i + 1) < path.size(); i++) {
		std::vector<const ZoneLine*> segIntersections;
		Pos segStart = path[i];
		Pos segEnd = path[i + 1];

		for (unsigned int j = 0; j < occlusions.size(); j++) {
			if (!occlusions[j] || occlusions[j]->getCoordinatesRef().size() < 2)
				continue;

			if (utils::edgeTouchesLineString(segStart, segEnd, occlusions[j]->getCoordinatesRef())) {
				segIntersections.push_back(occlusions[j]);
			}
		}
		lines.push_back(segIntersections);
	}

	return lines;
}

std::vector< std::vector<const ZoneLine*> > ZonesLayer::getIntersectingOcclusionLines(const std::vector<Pose>& path) const {
	std::vector<Pos> positionList;
	positionList.reserve(path.size());
	for (unsigned int i = 0; i < path.size(); i++) {
		positionList.push_back(path[i].p);
	}

	return getIntersectingOcclusionLines(positionList);
}

std::vector<const ZoneLine*> ZonesLayer::getNearestOcclusionLines(const Pos& queryPosition, double searchRadius) const {
	std::vector<const ZoneLine*> lines;
	std::multimap<double, const ZoneLine*> nearestLineMap; // sort and store the nearest lines

	std::vector<const ZoneLine*> occlusions = getAllOcclusionLines();
	for (unsigned int i = 0; i < occlusions.size(); i++) {
		if (!occlusions[i] || occlusions[i]->getCoordinatesRef().size() < 2)
			continue;

		for (unsigned int j = 0; j < occlusions[i]->getCoordinatesRef().size(); j++) {
			double dist = queryPosition.dist(occlusions[i]->getCoordinatesRef()[j]);
			if (dist <= searchRadius) {
				nearestLineMap.insert(std::make_pair(dist, occlusions[i]));
				break;
			}
		}
	}

	// Get the sorted list of occlusion lines
	for (std::multimap<double, const ZoneLine*>::const_iterator itr = nearestLineMap.begin(); itr != nearestLineMap.end(); itr++) {
		lines.push_back(itr->second);
	}
	return lines;
}

const OcclusionRegion* ZonesLayer::getOcclusionRegion(int featureId) const {
	const OcclusionRegion* region = NULL;
	const std::map<int, OcclusionRegion>& occRegions = getAllOcclusionRegions();

	for (std::map<int, OcclusionRegion>::const_iterator itr = occRegions.begin(); itr != occRegions.end(); itr++) {
		if (itr->second.getFeatureId() == featureId) {
			region = &(itr->second);
			break;
		}
	}
	return region;
}

const std::map<int, OcclusionRegion>& ZonesLayer::getAllOcclusionRegions() const {
	return occlusionRegions;
}

std::vector< std::vector<const OcclusionRegion*> > ZonesLayer::getIntersectingOcclusionRegions(const std::vector<Pos>& path) const {
	std::vector< std::vector<const OcclusionRegion*> > regions;

	if (path.size() < 2)
		return regions;

	for (unsigned int i = 0; (i + 1) < path.size(); i++) {
		std::multimap<double, const OcclusionRegion*> intersectingOcclusions;
		Pos segStart = path[i];
		Pos segEnd = path[i + 1];
		for (std::map<int, OcclusionRegion>::const_iterator itr = occlusionRegions.begin(); itr != occlusionRegions.end(); itr++) {
			Pos contactPoint;
			if (itr->second.getFirstPointOfContactWithLineSegment(segStart, segEnd, contactPoint)) {
				intersectingOcclusions.insert(std::make_pair(segStart.dist(contactPoint), &(itr->second)));
			}
		}
		std::vector<const OcclusionRegion*> sortedIntersectingOcclusions;
		for (std::multimap<double, const OcclusionRegion*>::const_iterator itr = intersectingOcclusions.begin(); itr != intersectingOcclusions.end(); itr++) {
			sortedIntersectingOcclusions.push_back(itr->second);
		}
		regions.push_back(sortedIntersectingOcclusions);
	}

	return regions;
}

std::vector< std::vector<const OcclusionRegion*> > ZonesLayer::getIntersectingOcclusionRegions(const std::vector<Pose>& path) const {
	std::vector<Pos> positionList;
	positionList.reserve(path.size());
	for (unsigned int i = 0; i < path.size(); i++) {
		positionList.push_back(path[i].p);
	}

	return getIntersectingOcclusionRegions(positionList);
}

std::vector<const OcclusionRegion*> ZonesLayer::getNearestOcclusionRegions(const Pos& queryPosition, double searchRadius) const {
	std::vector<const OcclusionRegion*> regions;
	std::multimap<double, const OcclusionRegion*> nearestRegionsMap; // sort and store the nearest lines

	for (std::map<int, OcclusionRegion>::const_iterator itr = occlusionRegions.begin(); itr != occlusionRegions.end(); itr++) {
		double distToOcclusionRegion = itr->second.dist(queryPosition);
		if (distToOcclusionRegion < searchRadius) {
			nearestRegionsMap.insert(std::make_pair(distToOcclusionRegion, &(itr->second)));
		}
	}

	// Get the sorted list of occlusion regions
	for (std::multimap<double, const OcclusionRegion*>::const_iterator itr = nearestRegionsMap.begin(); itr != nearestRegionsMap.end(); itr++) {
		regions.push_back(itr->second);
	}
	return regions;
}

std::vector< std::vector<Pos> > ZonesLayer::getOcclusionPointAlongPath(const std::vector<Pos>& path) const {
	std::vector< std::vector<Pos> > edgeOcclusions(path.size() - 1);

	std::vector < std::vector<const OcclusionRegion*> > occRegions = getIntersectingOcclusionRegions(path);
	if (occRegions.size() != path.size() - 1)
		return edgeOcclusions;

	std::vector<int> processedOccRegions;
	for (unsigned int edgeIdx = 0; (edgeIdx + 1) < path.size(); edgeIdx++) {
		std::vector<const OcclusionRegion*> intersectingOcclusions = occRegions[edgeIdx];
		if (!intersectingOcclusions.empty()) {
			std::vector<Pos>& intersectionPts = edgeOcclusions[edgeIdx];
			for (unsigned int occIdx = 0; occIdx < intersectingOcclusions.size(); occIdx++) {
				const OcclusionRegion* occlusion = intersectingOcclusions[occIdx];
				if (!occlusion || std::find(processedOccRegions.begin(),
											processedOccRegions.end(),
											occlusion->getFeatureId()) != processedOccRegions.end())
					continue;

				Pos intPoint;
				if (occlusion->getFirstPointOfContactWithLineSegment(path[edgeIdx],
										 							 path[edgeIdx + 1],
																	 intPoint)) {
					Pos revDirection = (path[edgeIdx] - intPoint);
					revDirection.normalize();
					Pos pointBeforeIntersection = intPoint + revDirection * 0.05; // 5 cm before intersection
					if (!occlusion->contains(pointBeforeIntersection)) {
						// Add the intersection point only when entering an occlusion region for first time
						intersectionPts.push_back(intPoint);
						processedOccRegions.push_back(occlusion->getFeatureId());
					}
				}
			}
		}
	}
	return edgeOcclusions;
}

std::vector< std::vector<Pos> > ZonesLayer::getOcclusionPointAlongPath(const std::vector<Pose>& path) const {
	std::vector<Pos> positionList;
	positionList.reserve(path.size());
	for (unsigned int i = 0; i < path.size(); i++) {
		positionList.push_back(path[i].p);
	}

	return getOcclusionPointAlongPath(positionList);
}

bool ZonesLayer::getFirstOcclusionPointAlongPath(const std::vector<Pos>& path, Pos& occlusionPoint) const {
	std::vector < std::vector<const OcclusionRegion*> > occRegions = getIntersectingOcclusionRegions(path);
	if (occRegions.size() != path.size() - 1)
		return false;
	
	for (unsigned int edgeIdx = 0; (edgeIdx + 1) < path.size(); edgeIdx++) {
		std::vector<const OcclusionRegion*> intersectingOcclusions = occRegions[edgeIdx];
		for (unsigned int occIdx = 0; occIdx < intersectingOcclusions.size(); occIdx++) {
			const OcclusionRegion* occlusion = intersectingOcclusions[occIdx];
			if (!occlusion)
				continue;

			Pos intPoint;
			if (occlusion->getFirstPointOfContactWithLineSegment(path[edgeIdx],
									 							 path[edgeIdx + 1],
																 intPoint)) {
				Pos revDirection = (path[edgeIdx] - intPoint);
				revDirection.normalize();
				Pos pointBeforeIntersection = intPoint + revDirection * 0.05; // 5 cm before intersection
				if (!occlusion->contains(pointBeforeIntersection)) {
					occlusionPoint = intPoint;
					return true;
				}
			}
		}
	}
	return false;
}

bool ZonesLayer::getFirstOcclusionPointAlongPath(const std::vector<Pose>& path, Pos& occlusionPoint) const {
	std::vector<Pos> positionList;
	positionList.reserve(path.size());
	for (unsigned int i = 0; i < path.size(); i++) {
		positionList.push_back(path[i].p);
	}

	return getFirstOcclusionPointAlongPath(positionList, occlusionPoint);
}

bool ZonesLayer::hasRamps() const {
	return !getAllRamps().empty();
}

std::vector<const Ramp*> ZonesLayer::getAllRamps() const {
	std::vector<const Ramp*> ramps;
	std::vector<const ZonePolygon*> polygons = getPolygons(zoneTypes::RAMP);
	for (unsigned int i = 0; i < polygons.size(); i++) {
		const Ramp* ramp = dynamic_cast<const Ramp*>(polygons[i]);
		if (ramp != NULL) {
			ramps.push_back(ramp);
		}
	}
	return ramps;
}

std::vector<const Ramp*> ZonesLayer::getIntersectingRamps(const Pos& start, const Pos& end) const {
	std::vector<const Ramp*> ramps = getAllRamps();
	for (std::vector<const Ramp*>::iterator itr = ramps.begin(); itr != ramps.end(); ) {
		const Ramp* ramp = *itr;
		if (ramp == NULL || !ramp->intersectsEdge(start, end)) {
			itr = ramps.erase(itr);
		} else {
			++itr;
		}
	}
	return ramps;
}

std::vector<const Ramp*> ZonesLayer::getIntersectingRamps(const std::vector<Pos>& lineString) const {
	std::vector<const Ramp*> intersectingRamps;
	if (lineString.size() < 2)
		return intersectingRamps;

	for(unsigned int i = 0; (i + 1) < lineString.size(); i++) {
		std::vector<const Ramp*> ramps = getIntersectingRamps(lineString[i], lineString[i+1]);
		if (!ramps.empty())
			intersectingRamps.insert(intersectingRamps.end(), ramps.begin(), ramps.end());
	}

	return intersectingRamps;
}

bool ZonesLayer::insideForbiddenArea(const Pos& pos) const {
	std::vector<const ZonePolygon*> forbiddenAreas = getAllForbiddenAreas();
	for (unsigned int i = 0; i < forbiddenAreas.size(); i++) {
		if (forbiddenAreas[i] && forbiddenAreas[i]->contains(pos)) {
			return true;
		}
	}
	return false;
}

void ZonesLayer::loadOcclusionRegions(const Map& map) {
	if (osmPrimitives.find(osm::primitiveType::RELATION) == osmPrimitives.end())
		return;

	const std::vector<int>& relations = osmPrimitives.at(osm::primitiveType::RELATION);
	int nSuccess = 0;
	const AreasLayer* areasLayer = map.getAreasLayer();
	for (unsigned int i = 0; i < relations.size(); i++) {
		const osm::Relation* relation = map.getOsmRelation(relations[i]);
		if (relation != NULL && 
			relation->members.size() > 0 &&
			relation->relationType == RELATION_TYPE_OCCLUSION_REGION) {
			OcclusionRegion occlusionRegion(relation->primitiveId);
			for (unsigned int mId = 0; mId < relation->members.size(); mId++) {
				const osm::RelationMember& member = relation->members[mId];
				if (member.role == "transition") {
					if (areasLayer) {
						const AreaTransition* transition = areasLayer->getAreaTransition(member.primitiveId);
						if (transition) {
							occlusionRegion.addAreaTransition(transition);
						} else {
							std::cout << "Failed to load area transition associated with Occlusion region: "
									  << relation->primitiveId << ". AreaTransition not found in Areas Layer!" 
									  << std::endl;
						}
					} else {
						std::cout << "Failed to load area transition associated with Occlusion region: "
								  << relation->primitiveId << ". Invalid Areas Layer!" << std::endl;
						break;
					}

				} else if (member.role == "line") {
					const ZoneLine* occlusionLine = getOcclusionLine(member.primitiveId);
					if (occlusionLine) {
						occlusionRegion.addOcclusionLine(occlusionLine);
					} else {
						std::cout << "Failed to occlusion line associated with Occlusion region: "
								  << relation->primitiveId << ". Occlusion line not found in Zones Layer!" << std::endl;
						break;
					}
				} else {
					std::cout << "Invalid role '" << member.role.c_str() 
							  << "' defined for a member of OcclusionRegion relation with feature id: " 
							  << relation->primitiveId << std::endl;
					break;
				}
			}
			if (!occlusionRegion.empty()) {
				occlusionRegion.generatePolygonCoords();
				occlusionRegions.insert(std::make_pair(occlusionRegion.getFeatureId(), occlusionRegion));
				nSuccess++;
			}
		}
	}

	// std::cout << "\tSuccessfully loaded " << nSuccess << "/" << relations.size() 
	// 		  << " occlusion relations in the Zones layer" << std::endl;
}

void ZonesLayer::loadInterlayerAssociations(const Map& map) {
	if (osmPrimitives.find(osm::primitiveType::RELATION) == osmPrimitives.end())
		return;

	const std::vector<int>& relations = osmPrimitives.at(osm::primitiveType::RELATION);
	int nSuccess = 0;
	for (unsigned int i = 0; i < relations.size(); i++) {
		const osm::Relation* relation = map.getOsmRelation(relations[i]);
		if (relation != NULL && 
			relation->members.size() > 1 &&
			relation->relationType == RELATION_TYPE_ASSOCIATION) {
			const osm::RelationMember& parent = relation->members[0];
			if (zones.find(parent.primitiveType) != zones.end() &&
				zones.at(parent.primitiveType).find(parent.primitiveId) != zones.at(parent.primitiveType).end()) {
				Zone* zone = zones.at(parent.primitiveType).at(parent.primitiveId);
				for (unsigned int j = 1; j < relation->members.size(); j++) {
					const osm::RelationMember& child = relation->members[j];
					layerType::LayerType layerType = layerType::getType(child.role);
					switch (layerType) {
					case layerType::AREAS: {
						const AreasLayer* areasLayer = map.getAreasLayer();
						if (areasLayer != NULL && child.primitiveType == osm::primitiveType::WAY) {
							const Area* area = areasLayer->getArea(child.primitiveId);
							if (area != NULL) {
								if (zone->getInterlayerAssociations().find(layerType::AREAS) == zone->getInterlayerAssociations().end()) {
									zone->getInterlayerAssociations().insert(std::make_pair(layerType::AREAS, std::set<int>()));
								}
								std::set<int>& overlappingAreas = zone->getInterlayerAssociations()[layerType::AREAS];
								overlappingAreas.insert(child.primitiveId);
								nSuccess++;
							}
						}
						break;
					}
					default:
						std::cout << "No implementation found to establish interlayer association from zones layer to "
								  << layerType::getName(layerType) << std::endl;
						break;
					}
				}
			}
			else {
				std::cout << "Failed to parse association relation " 
						  << relation->primitiveId 
						  << " in the Zones layer!" 
						  << std::endl;
			}
		}
	}

	// std::cout << "\tSuccessfully loaded " << nSuccess << "/" << relations.size() 
	// 		  << " inter-layer relations in the Zones layer" << std::endl; 
}

void ZonesLayer::loadIntralayerAssociations(const Map& map) {
	if (osmPrimitives.find(osm::primitiveType::RELATION) == osmPrimitives.end())
		return;

	const std::vector<int>& relations = osmPrimitives.at(osm::primitiveType::RELATION);
	int nSuccess = 0;
	for (unsigned int i = 0; i < relations.size(); i++) {
		const osm::Relation* relation = map.getOsmRelation(relations[i]);
		unsigned int minMembers = 5; // The ramp polygon plus min 2 nodes each for bottom and top edge
		if (relation != NULL && 
			relation->members.size() >= minMembers &&
			relation->relationType == RELATION_TYPE_RAMP_EDGES) {
			const osm::RelationMember& rampMember = relation->members[0];
			if (zones.find(rampMember.primitiveType) != zones.end() &&
				zones.at(rampMember.primitiveType).find(rampMember.primitiveId) != zones.at(rampMember.primitiveType).end()) {
				Ramp* ramp = dynamic_cast<Ramp*>(zones.at(rampMember.primitiveType).at(rampMember.primitiveId));
				const osm::Way* way = map.getOsmWay(rampMember.primitiveId);
				if (ramp != NULL && way != NULL) {
					std::vector<unsigned int> bottomNodeIds, topNodeIds;
					bool failure = false;
					for (unsigned int j = 1; !failure && j < relation->members.size(); j++) {
						const osm::RelationMember& m = relation->members[j];
						if (m.primitiveType == osm::primitiveType::NODE && m.role == "bottom") {
							std::vector<int>::const_iterator it = std::find(way->nodeIds.begin(), way->nodeIds.end(), m.primitiveId);
							if (it != way->nodeIds.end()) {
								bottomNodeIds.push_back((unsigned int)std::distance(way->nodeIds.begin(), it));
							} else {
								failure = true;
								std::cout << "Failed to find member node " 
										  << m.primitiveId 
										  << " marked as a bottom node of ramp "
										  << rampMember.primitiveId
										  << std::endl;
							}
						}
						else if (m.primitiveType == osm::primitiveType::NODE && m.role == "top") {
							std::vector<int>::const_iterator it = std::find(way->nodeIds.begin(), way->nodeIds.end(), m.primitiveId);
							if (it != way->nodeIds.end()) {
								topNodeIds.push_back((unsigned int)std::distance(way->nodeIds.begin(), it));
							} else {
								failure = true;
								std::cout << "Failed to find member node " 
										  << m.primitiveId 
										  << " marked as a top node of ramp "
										  << rampMember.primitiveId
										  << std::endl;
							}
						}
					}
					if (!failure) {
						nSuccess++;
						ramp->setBottomNodes(bottomNodeIds);
						ramp->setTopNodes(topNodeIds);
					}
				}
			}
			else {
				std::cout << "Failed to parse association relation " 
						  << relation->primitiveId 
						  << " in the Zones layer!" 
						  << std::endl;
			}
		}
	}

	// std::cout << "\tSuccessfully loaded " << nSuccess << "/" << relations.size() 
	// 		  << " intra-layer relations in the Zones layer" << std::endl; 
}

std::vector<const ZoneNode*> ZonesLayer::getNodes(zoneTypes::ZoneTypes type) const {
	std::vector<const ZoneNode*> nodes;
	if (zones.find(osm::primitiveType::NODE) == zones.end())
		return nodes;

	const ZonesMap& nodeZones = zones.at(osm::primitiveType::NODE);
	for (ZonesMapConstItr itr = nodeZones.begin(); itr != nodeZones.end(); itr++) {
		if (itr->second &&
			itr->second->getZoneType() == type &&
			itr->second->getPrimitiveType() == osm::primitiveType::NODE) {
			nodes.push_back(dynamic_cast<const ZoneNode*>(itr->second));
		}
	}
	return nodes;
}

std::vector<const ZoneLine*> ZonesLayer::getLines(zoneTypes::ZoneTypes type) const {
	std::vector<const ZoneLine*> lines;
	if (zones.find(osm::primitiveType::WAY) == zones.end())
		return lines;

	const ZonesMap& wayZones = zones.at(osm::primitiveType::WAY);
	for (ZonesMapConstItr itr = wayZones.begin(); itr != wayZones.end(); itr++) {
		if (itr->second &&
			itr->second->getZoneType() == type &&
			itr->second->getPrimitiveType() == osm::primitiveType::WAY) {
			lines.push_back(dynamic_cast<const ZoneLine*>(itr->second));
		}
	}
	return lines;
}

std::vector<const ZonePolygon*> ZonesLayer::getPolygons(zoneTypes::ZoneTypes type) const {
	std::vector<const ZonePolygon*> polygons;
	if (zones.find(osm::primitiveType::WAY) == zones.end())
		return polygons;

	const ZonesMap& wayZones = zones.at(osm::primitiveType::WAY);
	for (ZonesMapConstItr itr = wayZones.begin(); itr != wayZones.end(); itr++) {
		if (itr->second &&
			itr->second->getZoneType() == type &&
			itr->second->getPrimitiveType() == osm::primitiveType::WAY) {
			polygons.push_back(dynamic_cast<const ZonePolygon*>(itr->second));
		}
	}
	return polygons;
}

std::string ZonesLayer::getZoneName(const osm::Primitive* primitive) const {
	std::string name;
	primitive->getTagValue("name", name);
	return name;
}

zoneTypes::ZoneTypes ZonesLayer::getZoneType(const osm::Primitive* primitive) const {
	std::string characteristic;
	primitive->getTagValue("characteristic", characteristic);
	return zoneTypes::getType(characteristic);
}

double ZonesLayer::getRampInclination(const osm::Primitive* primitive) const {
	double inclination;
	primitive->getTagValue("inclination", inclination);
	return inclination;
}

}
