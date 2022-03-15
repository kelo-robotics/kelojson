
#include <iostream>
#include <geometry_common/Polygon2D.h>
#include <geometry_common/TransformMatrix2D.h>
#include <geometry_common/Utils.h>

#include "kelojson_loader/KelojsonMap.h"
#include "kelojson_loader/KelojsonAreasLayer.h"
#include "kelojson_loader/KelojsonZonesLayer.h"

using Polyline2D = kelo::geometry_common::Polyline2D;
using Polygon2D = kelo::geometry_common::Polygon2D;
using LineSegment2D = kelo::geometry_common::LineSegment2D;
using TransformMatrix2D = kelo::geometry_common::TransformMatrix2D;
using Utils = kelo::geometry_common::Utils;

namespace kelo {
namespace kelojson {

zoneTypes::ZoneTypes zoneTypes::getType(std::string type) {
	return type == ZONE_TYPE_ELEVATOR ? zoneTypes::FORBIDDEN :
		   type == ZONE_TYPE_STAIRS? zoneTypes::STAIRS :
		   type == ZONE_TYPE_FORBIDDEN ? zoneTypes::FORBIDDEN :
		   type == ZONE_TYPE_SLOWDOWN ? zoneTypes::SLOWDOWN :
		   type == ZONE_TYPE_RAMP ? zoneTypes::RAMP :
		   type == ZONE_TYPE_LOAD_PARKING ? zoneTypes::LOAD_PARKING :
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
		   type == zoneTypes::LOAD_PARKING ? "LOAD_PARKING" :
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

ZoneNode::ZoneNode(int featId, zoneTypes::ZoneTypes type, osm::primitiveType::PrimitiveType primType, const Pose2D& p, const std::string& zoneName)
: Zone(featId, type, primType, zoneName)
, pose(p)
{
}

ZoneLine::ZoneLine(int featId, zoneTypes::ZoneTypes type, osm::primitiveType::PrimitiveType primType, const std::vector<Point2D>& coords, const std::string& zoneName)
: Zone(featId, type, primType, zoneName)
, coordinates(coords)
{
}

ZonePolygon::ZonePolygon(int featId, zoneTypes::ZoneTypes type, osm::primitiveType::PrimitiveType primType, const std::vector<Point2D>& coords, const std::string& zoneName)
: Zone(featId, type, primType, zoneName)
, coordinates(coords)
{
}

bool ZonePolygon::contains(Point2D point) const {
	return Polygon2D(coordinates).containsPoint(point);
}

Ramp::Ramp(int featureId, zoneTypes::ZoneTypes zoneType,
		   osm::primitiveType::PrimitiveType primitiveType,
		   const std::vector<Point2D>& coordinates, double incline, const std::string& name) 
: ZonePolygon(featureId, zoneType, primitiveType, coordinates, name)
, inclination(incline) {
}

bool Ramp::valid() const {
	return coordinates.size() >= 4 &&	// Ramp is atleast a quad
		   topNodes.size() >= 2 &&		// Minimum two nodes marked as top nodes
		   bottomNodes.size() >= 2;		// Minimum two nodes marked as bottom nodes
}

std::vector<Point2D> Ramp::getBottomNodePositions() const {
	std::vector<Point2D> positions;
	for (unsigned int i = 0; i < bottomNodes.size(); i++) {
		positions.push_back(coordinates[bottomNodes[i]]);
	}
	return positions;
}

std::vector<Point2D> Ramp::getTopNodePositions() const {
	std::vector<Point2D> positions;
	for (unsigned int i = 0; i < topNodes.size(); i++) {
		positions.push_back(coordinates[topNodes[i]]);
	}
	return positions;
}

bool Ramp::intersectsEdge(const Point2D& edgeStart, const Point2D& edgeEnd) const {
	return Polygon2D(coordinates).intersects(LineSegment2D(edgeStart, edgeEnd));
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
				Point2D tStart = transitions[k].coordinates[0];
				Point2D tEnd = transitions[k].coordinates[transitions[k].coordinates.size() - 1];
				if (intersectsEdge(tStart, tEnd)) {
					overlappingTransitionIds.push_back(transitions[k].featureId);
				}
			}
		}
	}

	return overlappingTransitionIds;
}

LoadParking::LoadParking(int featureId,
						 const std::vector<Point2D>& coordinates,
						 const std::string& name) 
: ZonePolygon(featureId, zoneTypes::LOAD_PARKING, osm::primitiveType::WAY, coordinates, name)
{
}

bool LoadParking::valid() const {
	return coordinates.size() >= 3 &&		// Load parking must atleast be a triangle
		   primaryOpeningNodes.size() >= 2;	// Minimum two nodes marked as top nodes
}

std::vector<Point2D> LoadParking::getPrimaryOpeningPositions() const
{
	std::vector<Point2D> positions;
	for (unsigned int nodeOffset: primaryOpeningNodes) {
		positions.push_back(coordinates[nodeOffset]);
	}
	return positions;
}

std::vector<Point2D> LoadParking::getSecondaryOpeningPositions() const
{
	std::vector<Point2D> positions;
	for (unsigned int nodeOffset: secondaryOpeningNodes) {
		positions.push_back(coordinates[nodeOffset]);
	}
	return positions;
}

void LoadParking::setBelongsToLoadParkingGroup(const std::string& groupName)
{
	if (!belongsToGroup(groupName)) {
		loadParkingGroups.push_back(groupName);
	}
}

bool LoadParking::belongsToGroup(const std::string& groupName) const
{
	return std::find(loadParkingGroups.begin(), loadParkingGroups.end(), groupName) != loadParkingGroups.end();
}

Pose2D LoadParking::getPrimaryOpeningCenterPose() const
{
	const auto primaryOpening = getPrimaryOpeningPositions();
	kelo::geometry_common::LineSegment2D seg(primaryOpening.front(), primaryOpening.back());
	return Pose2D(seg.center(), getLoadOrientation());
}

OcclusionRegion::OcclusionRegion(unsigned int id)
: internalId(id) {
}

OcclusionRegion::~OcclusionRegion() {
}

int OcclusionRegion::getFeatureId() const {
	return featureId;
}

void OcclusionRegion::setName(const std::string& name) {
	this->name = name;
}

const std::string& OcclusionRegion::getName() const{
	return name;
}

void OcclusionRegion::addOcclusionLine(const kelo::geometry_common::Polyline2D& line) {
	for (const auto& l : occlusionLines) {
		if (l == line) {
			return;
		}
	}

	occlusionLines.push_back(line);
}

void OcclusionRegion::addAreaTransition(const AreaTransition* transition) {
	if (!transition) {
		return;
	}
	addOcclusionLine(kelo::geometry_common::Polyline2D(transition->coordinates));
}

bool OcclusionRegion::overlapsWithLineString(std::vector<Point2D> lineString) const {
	bool overlaps = false;
	for (unsigned int i = 0; (i+1) < lineString.size() && !overlaps; i++) {
		overlaps = overlapsWithLineSegment(lineString[i], lineString[i + 1]);
	}

	return overlaps;
}

bool OcclusionRegion::overlapsWithLineSegment(Point2D lineSegStart, Point2D lineSegEnd) const {
	bool overlaps = false;

	for (auto itr = occlusionLines.begin(); itr != occlusionLines.end() && !overlaps; itr++) {
		const Polyline2D& occlusionLine = *itr;
		if (occlusionLine.size() < 2)
			continue;
		overlaps = occlusionLine.intersects(LineSegment2D(lineSegStart, lineSegEnd));
	}

	return overlaps;
}

bool OcclusionRegion::getFirstPointOfContactWithLineString(std::vector<Point2D> lineString, Point2D& contactPoint) const {
	bool success = false;
	for (unsigned int i = 0; (i+1) < lineString.size(); i++) {
		Point2D closestContactPoint;
		if (getFirstPointOfContactWithLineSegment(lineString[i], lineString[i + 1], closestContactPoint)) {
			contactPoint = closestContactPoint;
			success = true;
			break;
		}
	}
	return success;
}

bool OcclusionRegion::getFirstPointOfContactWithLineSegment(Point2D lineSegStart, Point2D lineSegEnd, Point2D& contactPoint) const {
	std::map<int, Point2D> contactPoints;
	LineSegment2D segment(lineSegStart, lineSegEnd);

	for (auto itr = occlusionLines.begin(); itr != occlusionLines.end(); itr++) {
		const Polyline2D& occlusionLine = *itr;
		if (occlusionLine.size() < 2)
			continue;

		Point2D closestContactPoint;
		if (occlusionLine.calcClosestIntersectionPointWith(segment, closestContactPoint)) {
			contactPoints.insert(std::make_pair(lineSegStart.distTo(closestContactPoint), closestContactPoint));
		}
	}

	if (!contactPoints.empty()) {
		contactPoint = contactPoints.begin()->second;
	}

	return !contactPoints.empty();
}

double OcclusionRegion::dist(const Point2D& queryPoint) const {
	double minDist = std::numeric_limits<double>::max();
	for (auto itr = occlusionLines.begin(); itr != occlusionLines.end(); itr++) {
		const Polyline2D& occlusionLine = *itr;
		if (occlusionLine.size() < 2)
			continue;

		for (unsigned int i = 0; i < occlusionLine.size(); i++) {
			double dist = queryPoint.distTo(occlusionLine[i]);
			if (dist < minDist) {
				minDist = dist;
			}
		}
	}

	return minDist;
}

bool OcclusionRegion::contains(const Point2D& point) const {
	if (!polyCoordinates.empty()) {
		return Polygon2D(polyCoordinates).containsPoint(point);
	}
	return false;
}

bool OcclusionRegion::empty() const {
	return occlusionLines.empty();
}

bool OcclusionRegion::generatePolygonCoords() {
	polyCoordinates.clear(); // clear previously generated poly coordinates

	std::vector< std::vector<Point2D> > lineStrings;
	for (auto itr = occlusionLines.begin(); itr != occlusionLines.end(); itr++) {
		const Polyline2D& line = *itr;
		if (line.size() > 1) {
			lineStrings.push_back(line.vertices);
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
		const std::vector<Point2D>& lastLine = lineStrings[lineStringOrder.back()];
		isConnected = false;
		for (unsigned int i = 0; i < lineStrings.size(); i++) {
			if (std::find(lineStringOrder.begin(), lineStringOrder.end(), i) == lineStringOrder.end()) {
				const std::vector<Point2D>& newTestLine = lineStrings[i];
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
					const Point2D& lineEndPt = reverse.back() ? lastLine.front() : lastLine.back();

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
			const std::vector<Point2D>& line = lineStrings[lineStringOrder[i]];
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

const std::vector<Point2D>& OcclusionRegion::asPolygon() const {
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

void ZonesLayer::loadFeatures(const Map& map) {
	Layer::loadFeatures(map);
	autoGenerateOcclusionRegions(map);
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
														 Pose2D(node->position.x, node->position.y, theta),
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

		std::vector<Point2D> coordinates;
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
			} else if (zoneType == zoneTypes::LOAD_PARKING) {
				wayZones.insert(std::make_pair(way->primitiveId, 
											   new LoadParking(way->primitiveId,
														coordinates,
														getZoneName(way))));
			}  else {
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

std::vector< std::vector<const ZoneLine*> > ZonesLayer::getIntersectingOcclusionLines(const std::vector<Point2D>& path) const {
	std::vector< std::vector<const ZoneLine*> > lines;

	if (path.size() < 2)
		return lines;

	std::vector<const ZoneLine*> occlusions = getAllOcclusionLines();
	for (unsigned int i = 0; (i + 1) < path.size(); i++) {
		std::vector<const ZoneLine*> segIntersections;
		Point2D segStart = path[i];
		Point2D segEnd = path[i + 1];

		for (unsigned int j = 0; j < occlusions.size(); j++) {
			if (!occlusions[j] || occlusions[j]->getCoordinatesRef().size() < 2)
				continue;

			if (Polygon2D(occlusions[j]->getCoordinatesRef()).intersects(LineSegment2D(segStart, segEnd))) {
				segIntersections.push_back(occlusions[j]);
			}
		}
		lines.push_back(segIntersections);
	}

	return lines;
}

std::vector< std::vector<const ZoneLine*> > ZonesLayer::getIntersectingOcclusionLines(const std::vector<Pose2D>& path) const {
	std::vector<Point2D> positionList;
	positionList.reserve(path.size());
	for (unsigned int i = 0; i < path.size(); i++) {
		positionList.push_back(Point2D(path[i].x, path[i].y));
	}

	return getIntersectingOcclusionLines(positionList);
}

std::vector<const ZoneLine*> ZonesLayer::getNearestOcclusionLines(const Point2D& queryPosition, double searchRadius) const {
	std::vector<const ZoneLine*> lines;
	std::multimap<double, const ZoneLine*> nearestLineMap; // sort and store the nearest lines

	std::vector<const ZoneLine*> occlusions = getAllOcclusionLines();
	for (unsigned int i = 0; i < occlusions.size(); i++) {
		if (!occlusions[i] || occlusions[i]->getCoordinatesRef().size() < 2)
			continue;

		for (unsigned int j = 0; j < occlusions[i]->getCoordinatesRef().size(); j++) {
			double dist = queryPosition.distTo(occlusions[i]->getCoordinatesRef()[j]);
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

const std::map<unsigned int, OcclusionRegion>& ZonesLayer::getAllOcclusionRegions() const {
	return occlusionRegions;
}

std::vector< std::vector<const OcclusionRegion*> > ZonesLayer::getIntersectingOcclusionRegions(const std::vector<Point2D>& path) const {
	std::vector< std::vector<const OcclusionRegion*> > regions;

	if (path.size() < 2)
		return regions;

	for (unsigned int i = 0; (i + 1) < path.size(); i++) {
		std::multimap<double, const OcclusionRegion*> intersectingOcclusions;
		Point2D segStart = path[i];
		Point2D segEnd = path[i + 1];
		for (std::map<unsigned int, OcclusionRegion>::const_iterator itr = occlusionRegions.begin(); itr != occlusionRegions.end(); itr++) {
			Point2D contactPoint;
			if (itr->second.getFirstPointOfContactWithLineSegment(segStart, segEnd, contactPoint)) {
				intersectingOcclusions.insert(std::make_pair(segStart.distTo(contactPoint), &(itr->second)));
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

std::vector< std::vector<const OcclusionRegion*> > ZonesLayer::getIntersectingOcclusionRegions(const std::vector<Pose2D>& path) const {
	std::vector<Point2D> positionList;
	positionList.reserve(path.size());
	for (unsigned int i = 0; i < path.size(); i++) {
		positionList.push_back(Point2D(path[i].x, path[i].y));
	}

	return getIntersectingOcclusionRegions(positionList);
}

std::vector<const OcclusionRegion*> ZonesLayer::getNearestOcclusionRegions(const Point2D& queryPosition, double searchRadius) const {
	std::vector<const OcclusionRegion*> regions;
	std::multimap<double, const OcclusionRegion*> nearestRegionsMap; // sort and store the nearest lines

	for (std::map<unsigned int, OcclusionRegion>::const_iterator itr = occlusionRegions.begin(); itr != occlusionRegions.end(); itr++) {
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

std::vector< std::vector<Point2D> > ZonesLayer::getOcclusionPointAlongPath(const std::vector<Point2D>& path) const {
	std::vector< std::vector<Point2D> > edgeOcclusions(path.size() - 1);

	std::vector < std::vector<const OcclusionRegion*> > occRegions = getIntersectingOcclusionRegions(path);
	if (occRegions.size() != path.size() - 1)
		return edgeOcclusions;

	std::vector<unsigned int> processedOccRegions;
	for (unsigned int edgeIdx = 0; (edgeIdx + 1) < path.size(); edgeIdx++) {
		std::vector<const OcclusionRegion*> intersectingOcclusions = occRegions[edgeIdx];
		if (!intersectingOcclusions.empty()) {
			std::vector<Point2D>& intersectionPts = edgeOcclusions[edgeIdx];
			for (unsigned int occIdx = 0; occIdx < intersectingOcclusions.size(); occIdx++) {
				const OcclusionRegion* occlusion = intersectingOcclusions[occIdx];
				if (!occlusion || std::find(processedOccRegions.begin(),
											processedOccRegions.end(),
											occlusion->getInternalId()) != processedOccRegions.end())
					continue;

				Point2D intPoint;
				if (occlusion->getFirstPointOfContactWithLineSegment(path[edgeIdx],
										 							 path[edgeIdx + 1],
																	 intPoint)) {
					Point2D revDirection = (path[edgeIdx] - intPoint);
					revDirection.normalise();
					Point2D pointBeforeIntersection = intPoint + revDirection * 0.05; // 5 cm before intersection
					if (!occlusion->contains(pointBeforeIntersection)) {
						// Add the intersection point only when entering an occlusion region for first time
						intersectionPts.push_back(intPoint);
						processedOccRegions.push_back(occlusion->getInternalId());
					}
				}
			}
		}
	}
	return edgeOcclusions;
}

std::vector< std::vector<Point2D> > ZonesLayer::getOcclusionPointAlongPath(const std::vector<Pose2D>& path) const {
	std::vector<Point2D> positionList;
	positionList.reserve(path.size());
	for (unsigned int i = 0; i < path.size(); i++) {
		positionList.push_back(Point2D(path[i].x, path[i].y));
	}

	return getOcclusionPointAlongPath(positionList);
}

bool ZonesLayer::getFirstOcclusionPointAlongPath(const std::vector<Point2D>& path, Point2D& occlusionPoint) const {
	std::vector < std::vector<const OcclusionRegion*> > occRegions = getIntersectingOcclusionRegions(path);
	if (occRegions.size() != path.size() - 1)
		return false;
	
	for (unsigned int edgeIdx = 0; (edgeIdx + 1) < path.size(); edgeIdx++) {
		std::vector<const OcclusionRegion*> intersectingOcclusions = occRegions[edgeIdx];
		for (unsigned int occIdx = 0; occIdx < intersectingOcclusions.size(); occIdx++) {
			const OcclusionRegion* occlusion = intersectingOcclusions[occIdx];
			if (!occlusion)
				continue;

			Point2D intPoint;
			if (occlusion->getFirstPointOfContactWithLineSegment(path[edgeIdx],
									 							 path[edgeIdx + 1],
																 intPoint)) {
				Point2D revDirection = (path[edgeIdx] - intPoint);
				revDirection.normalise();
				Point2D pointBeforeIntersection = intPoint + revDirection * 0.05; // 5 cm before intersection
				if (!occlusion->contains(pointBeforeIntersection)) {
					occlusionPoint = intPoint;
					return true;
				}
			}
		}
	}
	return false;
}

bool ZonesLayer::getFirstOcclusionPointAlongPath(const std::vector<Pose2D>& path, Point2D& occlusionPoint) const {
	std::vector<Point2D> positionList;
	positionList.reserve(path.size());
	for (unsigned int i = 0; i < path.size(); i++) {
		positionList.push_back(Point2D(path[i].x, path[i].y));
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

std::vector<const Ramp*> ZonesLayer::getIntersectingRamps(const Point2D& start, const Point2D& end) const {
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

std::vector<const Ramp*> ZonesLayer::getIntersectingRamps(const std::vector<Point2D>& lineString) const {
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

bool ZonesLayer::hasLoadParkings() const {
	return !getAllLoadParkings().empty();
}

std::vector<const LoadParking*> ZonesLayer::getAllLoadParkings() const {
	std::vector<const LoadParking*> loadParkings;
	std::vector<const ZonePolygon*> polygons = getPolygons(zoneTypes::LOAD_PARKING);
	for (unsigned int i = 0; i < polygons.size(); i++) {
		const LoadParking* loadParking = dynamic_cast<const LoadParking*>(polygons[i]);
		if (loadParking != NULL) {
			loadParkings.push_back(loadParking);
		}
	}
	return loadParkings;
}

std::vector<std::string> ZonesLayer::getAllLoadParkingGroupNames() const {
	std::set<std::string> groupNames;
	for (const LoadParking* loadParking : getAllLoadParkings()) {
		if (!loadParking)
			continue;
		for (const auto& name : loadParking->getAllGroupNames()) {
			groupNames.insert(name);
		}
	}
	return std::vector<std::string>(groupNames.begin(), groupNames.end());
}

std::vector<const LoadParking*> ZonesLayer::getAllLoadParkingsInGroup(const std::string& loadParkingGroupName) const {
	std::vector<const LoadParking*> loadParkings;
	for (const LoadParking* loadParking : getAllLoadParkings()) {
		if (loadParking && loadParking->belongsToGroup(loadParkingGroupName)) {
			loadParkings.push_back(loadParking);
		}
	}
	return loadParkings;
}

const LoadParking* ZonesLayer::getLoadParking(const std::string& name) const {
	for (const LoadParking* loadParking : getAllLoadParkings()) {
		if (loadParking && loadParking->getName() == name) {
			return loadParking;
		}
	}
	return NULL;
}

bool ZonesLayer::insideForbiddenArea(const Point2D& pos) const {
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
			OcclusionRegion occlusionRegion(occlusionRegions.size());
			occlusionRegion.setFeatureId(relation->primitiveId);
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
						occlusionRegion.addOcclusionLine(Polyline2D(occlusionLine->getCoordinatesRef()));
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
				std::string name;
				relation->getTagValue("name", name);
				if (name.empty())
				{
					// Construct a name for the occlusion region
					std::stringstream ss;
					ss << "OcclusionRegion-" << std::abs(relation->primitiveId);
					name = ss.str();
				}
				occlusionRegion.setName(name);
				occlusionRegion.generatePolygonCoords();
				occlusionRegions.insert(std::make_pair(occlusionRegion.getInternalId(), occlusionRegion));
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
		if (relation == NULL)
			continue;
		if (relation->relationType == RELATION_TYPE_RAMP_EDGES) {
			if (loadRampEdges(map, relation)) {
				nSuccess++;
			}
			else {
				std::cout << "Failed to parse load ramp edges relation " 
						  << relation->primitiveId 
						  << " in the Zones layer!" 
						  << std::endl;
			}
		}
		else if (relation->relationType == RELATION_TYPE_LOAD_PARKING_OPENINGS) {
			if (loadLoadParkingOpenings(map, relation)) {
				nSuccess++;
			} else {
				std::cout << "Failed to parse load parking opening relation " 
						  << relation->primitiveId 
						  << " in the Zones layer!" 
						  << std::endl;
			}
		}
		else if (relation->relationType == RELATION_TYPE_LOAD_PARKING_GROUP) {
			if (loadLoadParkingGroup(relation)) {
				nSuccess++;
			} else {
				std::cout << "Failed to parse load parking group relation " 
						  << relation->primitiveId 
						  << " in the Zones layer!" 
						  << std::endl;
			}
		}
	}

	std::cout << "\tSuccessfully loaded " << nSuccess << "/" << relations.size() 
			  << " intra-layer relations in the Zones layer" << std::endl;
}

bool ZonesLayer::loadRampEdges(const Map& map, const osm::Relation* relation)
{
	if (relation->members.size() < 5)// The ramp polygon plus min 2 nodes each for bottom and top edge
		return false;

	ZonesMap& zonePolygons = zones.at(osm::primitiveType::WAY);
	const osm::RelationMember& rampMember = relation->members[0];

	if (zonePolygons.find(rampMember.primitiveId) == zonePolygons.end()) {
		return false;
	}

	Ramp* ramp = dynamic_cast<Ramp*>(zonePolygons.at(rampMember.primitiveId));
	const osm::Way* way = map.getOsmWay(rampMember.primitiveId);
	if (ramp == NULL || way == NULL) {
		return false;
	}

	std::vector<unsigned int> bottomNodeIds, topNodeIds;
	for (unsigned int j = 1; j < relation->members.size(); j++) {
		const osm::RelationMember& m = relation->members[j];
		if (m.primitiveType != osm::primitiveType::NODE ||
			(m.role != "bottom" && m.role != "top"))
			continue;

		std::vector<int>::const_iterator it = std::find(way->nodeIds.begin(), way->nodeIds.end(), m.primitiveId);
		if (it == way->nodeIds.end())
			return false;

		unsigned int relativeNodeOffset = (unsigned int)std::distance(way->nodeIds.begin(), it);
		if (m.role == "bottom") {
			bottomNodeIds.push_back(relativeNodeOffset);
		} else {
			topNodeIds.push_back(relativeNodeOffset);
		}
	}

	if (bottomNodeIds.size() < 2 || topNodeIds.size() < 2) {
		return false;
	}

	ramp->setBottomNodes(bottomNodeIds);
	ramp->setTopNodes(topNodeIds);
	return true;
}

bool ZonesLayer::loadLoadParkingOpenings(const Map& map, const osm::Relation* relation)
{
	if (relation->members.size() < 3 || // The load parking polygon plus min 2 nodes for primary opening
		zones.find(osm::primitiveType::WAY) == zones.end()) { // There must be a polygon for load parking
		return false;
	}

	ZonesMap& zonePolygons = zones.at(osm::primitiveType::WAY);
	const osm::RelationMember& loadParkingMember = relation->members[0];

	if (zonePolygons.find(loadParkingMember.primitiveId) == zonePolygons.end()) {
		return false;
	}

	LoadParking* loadParking = dynamic_cast<LoadParking*>(zonePolygons.at(loadParkingMember.primitiveId));
	const osm::Way* way = map.getOsmWay(loadParkingMember.primitiveId);
	if (loadParking == NULL || way == NULL) {
		return false;
	}

	std::vector<unsigned int> primaryOpeningNodes, secondaryOpeningNodes;
	for (unsigned int j = 1; j < relation->members.size(); j++) {
		const osm::RelationMember& m = relation->members[j];
		if (m.primitiveType != osm::primitiveType::NODE ||
			(m.role != "primary_opening" && m.role != "secondary_opening"))
			continue;

		std::vector<int>::const_iterator it = std::find(way->nodeIds.begin(), way->nodeIds.end(), m.primitiveId);
		if (it == way->nodeIds.end())
			return false;

		unsigned int relativeNodeOffset = (unsigned int)std::distance(way->nodeIds.begin(), it);
		if (m.role == "primary_opening") {
			primaryOpeningNodes.push_back(relativeNodeOffset);
		} else {
			secondaryOpeningNodes.push_back(relativeNodeOffset);
		}
	}

	if (primaryOpeningNodes.size() < 2)
		return false;

	if (secondaryOpeningNodes.size() == 1) {
		// Secondary opening should either have no nodes OR should have more than 1 node
		return false;
	}

	loadParking->setPrimaryOpeningNodes(primaryOpeningNodes);
	if (secondaryOpeningNodes.size() > 1) {
		loadParking->setSecondaryOpeningNodes(secondaryOpeningNodes);
	}

	float orientation = 0.0F;
	if (way->getTagValue("theta", orientation)) {
		loadParking->setLoadOrientation(orientation);
	}
	else {
		// Explicit Load Orientation is not defined,
		// calculate the orientation using the primary opening
		std::vector<Point2D> primaryOpeningPos = loadParking->getPrimaryOpeningPositions();
		LineSegment2D primaryOpeningSeg(primaryOpeningPos[0],
										primaryOpeningPos[primaryOpeningPos.size() - 1]);
		orientation = Utils::calcPerpendicularAngle(primaryOpeningSeg.angle());
		Point2D center = primaryOpeningSeg.center();
		const float displacement = 0.1; // 10 cm
		// Get a point along the perpendicular at a certain displacement from center
		Point2D testPt(center.x + (std::cos(orientation) * displacement),
					   center.y + (std::sin(orientation) * displacement));
		if (Polygon2D(loadParking->getCoordinatesRef()).containsPoint(testPt))
		{
			orientation = Utils::calcReverseAngle(orientation);
		}
	}
	loadParking->setLoadOrientation(orientation);

	return true;
}

bool ZonesLayer::loadLoadParkingGroup(const osm::Relation* relation) {
	if (zones.find(osm::primitiveType::WAY) == zones.end())
		return false;
	ZonesMap& zonePolygons = zones.at(osm::primitiveType::WAY);

	std::string groupName;
	relation->getTagValue("name", groupName);
	if (groupName.empty())
		return false;

	std::vector<LoadParking*> loadParkingMembers;
	for (const osm::RelationMember& m : relation->members) {
		if (m.primitiveType != osm::primitiveType::WAY)
			return false;

		LoadParking* loadParking = dynamic_cast<LoadParking*>(zonePolygons.at(m.primitiveId));
		if (!loadParking)
			return false;

		loadParkingMembers.push_back(loadParking);
	}

	for (LoadParking* parking : loadParkingMembers) {
		parking->setBelongsToLoadParkingGroup(groupName);
	}
	return true;
}

void ZonesLayer::autoGenerateOcclusionRegions(const Map& map) {
	const AreasLayer* areasLayer = map.getAreasLayer();
	if (!areasLayer || areasLayer->areas.empty())
		return;

	// Generate occlusion regions at junctions
	for (const auto& a : areasLayer->areas) {
		const Area& area = a.second;
		if (area.areaType != areaTypes::JUNCTION)
			continue;

		OcclusionRegion occRegion(occlusionRegions.size());
		for (const auto& ta : area.transitions) {
			const std::set<AreaTransition>& transitionWithArea = ta.second;
			for (const AreaTransition& transition : transitionWithArea) {
				occRegion.addAreaTransition(&transition);
			}
		}
		if (!occRegion.empty() && occRegion.generatePolygonCoords()) {
			// Construct a name for the occlusion region
			std::stringstream ss;
			ss << "Junction-" << area.name << "-OcclusionRegion";
			occRegion.setName(ss.str());
			occlusionRegions.insert(std::make_pair(occRegion.getInternalId(), occRegion));
		}
	}

	// Generate occlusion regions at doors
	for (const auto& t : areasLayer->getAllAreaTransitions()) {
		const AreaTransition* transition = t.second;
		if (transition && transition->isDoor()) {
			const std::vector<Point2D>& coords =  transition->coordinates;
			if (coords.size() < 2)
				continue;

			LineSegment2D doorSeg(coords.front(), coords.back());
			float perpAngle = Utils::calcPerpendicularAngle(doorSeg.angle());
			Point2D unitVec(std::cos(perpAngle), std::sin(perpAngle));
			const float maxPerpDistFromDoor = 2.5F; // meters

			std::vector<Point2D> firstEdge{coords.front(),
										   coords.front() + (unitVec * maxPerpDistFromDoor)};
			std::vector<Point2D> secondEdge{coords.back(),
											coords.back() + (unitVec * maxPerpDistFromDoor)};
			std::vector<Point2D> thirdEdge{coords.front(),
										   coords.front() - (unitVec * maxPerpDistFromDoor)};
			std::vector<Point2D> fourthEdge{coords.back(),
											coords.back() - (unitVec * maxPerpDistFromDoor)};

			OcclusionRegion occRegion1(occlusionRegions.size());
			// The order of adding the below 3 elements to occ region is crucial.
			// The transition must be the second element among the three elements
			occRegion1.addOcclusionLine(firstEdge);
			occRegion1.addAreaTransition(transition);
			occRegion1.addOcclusionLine(secondEdge);
			if (!occRegion1.empty() && occRegion1.generatePolygonCoords()) {
				// Construct a name for the occlusion region
				std::stringstream ss;
				ss << "Door-" << transition->name << "-OcclusionRegion-1";
				occRegion1.setName(ss.str());
				occlusionRegions.insert(std::make_pair(occRegion1.getInternalId(), occRegion1));
			}

			OcclusionRegion occRegion2(occlusionRegions.size());
			occRegion2.addAreaTransition(transition);
			occRegion2.addOcclusionLine(thirdEdge);
			occRegion2.addOcclusionLine(fourthEdge);
			if (!occRegion2.empty() && occRegion2.generatePolygonCoords()) {
				// Construct a name for the occlusion region
				std::stringstream ss;
				ss << "Door-" << transition->name << "-OcclusionRegion-2";
				occRegion2.setName(ss.str());
				occlusionRegions.insert(std::make_pair(occRegion2.getInternalId(), occRegion2));
			}
		}
	}
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
}
