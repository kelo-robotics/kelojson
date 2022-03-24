#ifndef KELO_KELOJSON_ZONES_LAYER_H
#define KELO_KELOJSON_ZONES_LAYER_H

#include <geometry_common/Polygon2D.h>
#include "kelojson_loader/KelojsonLayer.h"

namespace kelo {
namespace kelojson {

	// Forward declarations
	class AreaTransition;

	namespace zoneTypes {
		enum ZoneTypes {
			UNKNOWN = 0,
			ELEVATOR,
			STAIRS,
			FORBIDDEN,
			SLOWDOWN,
			RAMP,
			LOAD_PARKING,
			CHARGING_STATION,
			WAITING_LOCATION,
			OCCLUSION,
			COUNT
		};

		ZoneTypes getType(std::string type);
		std::string getName(ZoneTypes type);
	}

	class Zone {
	public:
		Zone(int featureId, zoneTypes::ZoneTypes zoneType,
			 osm::primitiveType::PrimitiveType primitiveType,const std::string& name = "");
		virtual ~Zone() {}
		inline int getFeatureId() const { return featureId; }
		inline zoneTypes::ZoneTypes getZoneType() const { return zoneType; }
		inline osm::primitiveType::PrimitiveType getPrimitiveType() const { return primitiveType; }
		bool hasName() const;
		std::string getName() const;
		std::vector<int> getOverlappingAreaIds() const;
		virtual Point2D meanPoint() const = 0;

		std::map<layerType::LayerType, std::set<int> >& getInterlayerAssociations() { return interlayerAssociations; }

	protected:
		int featureId;
		zoneTypes::ZoneTypes zoneType;
		std::string name;
		std::map<layerType::LayerType, std::set<int> > interlayerAssociations;
		osm::primitiveType::PrimitiveType primitiveType;
	};

	class ZoneNode : public Zone {
	public:
		ZoneNode(int featureId, zoneTypes::ZoneTypes zoneType,
				osm::primitiveType::PrimitiveType primitiveType, const Pose2D& pose,
				const std::string& name = "");
		virtual ~ZoneNode() {}
		inline Pose2D getPose() const { return pose; }

		Point2D meanPoint() const override { return Point2D(pose.x, pose.x); }

	protected:
		Pose2D pose;
	};

	class ZoneLine : public Zone {
	public:
		ZoneLine(int featureId, zoneTypes::ZoneTypes zoneType,
				osm::primitiveType::PrimitiveType primitiveType,
				const std::vector<Point2D>& coordinates, const std::string& name = "");
		virtual ~ZoneLine() {}
		inline std::vector<Point2D> getCoordinates() const { return coordinates; }
		inline const std::vector<Point2D>& getCoordinatesRef() const { return coordinates; }

		Point2D meanPoint() const override;

	protected:
		std::vector<Point2D> coordinates;
	};

	class ZonePolygon : public Zone {
	public:
		ZonePolygon(int featureId, zoneTypes::ZoneTypes zoneType,
					osm::primitiveType::PrimitiveType primitiveType,
					const std::vector<Point2D>& coordinates, const std::string& name = "");
		virtual ~ZonePolygon() {}
		inline std::vector<Point2D> getCoordinates() const { return coordinates; }
		inline const std::vector<Point2D>& getCoordinatesRef() const { return coordinates; }

		bool contains(Point2D point) const;

		Point2D meanPoint() const override { return geometry_common::Polygon2D(coordinates).meanPoint(); }

	protected:
		std::vector<Point2D> coordinates;
	};

	class Ramp : public ZonePolygon {
	public:
		Ramp(int featureId, zoneTypes::ZoneTypes zoneType,
			 osm::primitiveType::PrimitiveType primitiveType,
			 const std::vector<Point2D>& coordinates, double incline, const std::string& name = "");
		virtual ~Ramp() {}

		bool valid() const;

		void setInclination(double incline) { inclination = incline; }
		double getInclination() const { return inclination; }

		void setBottomNodes(const std::vector<unsigned int>& nodeIndices) { bottomNodes = nodeIndices; }
		const std::vector<unsigned int>& getBottomNodeIds() const { return bottomNodes; }
		std::vector<Point2D> getBottomNodePositions() const;

		void setTopNodes(const std::vector<unsigned int>& nodeIndices) { topNodes = nodeIndices; }
		const std::vector<unsigned int>& getTopNodeIds() const { return topNodes; }
		std::vector<Point2D> getTopNodePositions() const;

		bool intersectsEdge(const Point2D& edgeStart, const Point2D& edgeEnd) const;

		std::vector<int> getOverlappingAreaTransitionIds(const Map* kelojsonMap) const;

	protected:
		double inclination;
		std::vector<unsigned int> bottomNodes;
		std::vector<unsigned int> topNodes;
	};

	class LoadParking : public ZonePolygon {
	public:
		LoadParking(int featureId, const std::vector<Point2D>& coordinates, const std::string& name = "");
		virtual ~LoadParking() {}

		bool valid() const;

		void setPrimaryOpeningNodes(const std::vector<unsigned int>& nodeIndices) { primaryOpeningNodes = nodeIndices; }
		const std::vector<unsigned int>& getPrimaryOpeningNodes() const { return primaryOpeningNodes; }
		std::vector<Point2D> getPrimaryOpeningPositions() const;

		void setSecondaryOpeningNodes(const std::vector<unsigned int>& nodeIndices) { secondaryOpeningNodes = nodeIndices; }
		const std::vector<unsigned int>& getSecondaryOpeningNodes() const { return secondaryOpeningNodes; }
		std::vector<Point2D> getSecondaryOpeningPositions() const;

		void setLoadOrientation(float orientation) { loadOrientation = orientation; }
		float getLoadOrientation() const { return loadOrientation; }

		void setBelongsToLoadParkingGroup(const std::string& groupName);
		bool belongsToGroup(const std::string& groupName) const;
		const std::vector<std::string>& getAllGroupNames() const { return loadParkingGroups; }

		Pose2D getPrimaryOpeningCenterPose() const;

	protected:
		float loadOrientation{0.0F};
		std::vector<std::string> loadParkingGroups; // groups to which this load parking belongs
		std::vector<unsigned int> primaryOpeningNodes;
		std::vector<unsigned int> secondaryOpeningNodes;
	};

	class OcclusionRegion {
	public:
		OcclusionRegion(unsigned int internalId);
		virtual ~OcclusionRegion();

		void setFeatureId(int id) { featureId = id; }
		int getFeatureId() const;

		unsigned int getInternalId() const { return internalId; }

		void setName(const std::string& name);

		const std::string& getName() const;

		void addOcclusionLine(const kelo::geometry_common::Polyline2D& line);
		void addAreaTransition(const AreaTransition* transition);

		bool overlapsWithLineString(std::vector<Point2D> lineString) const;
		bool overlapsWithLineSegment(Point2D lineSegStart, Point2D lineSegEnd) const;

		bool getFirstPointOfContactWithLineString(std::vector<Point2D> lineString, Point2D& contactPoint) const;
		bool getFirstPointOfContactWithLineSegment(Point2D lineSegStart, Point2D lineSegEnd, Point2D& contactPoint) const;

		double dist(const Point2D& queryPoint) const;

		bool contains(const Point2D& point) const;

		bool empty() const;

		/**
		 * @brief Processes the occlusion region as a polygon using the end points of the associated occlusion lines
		 * as corners of the polygon. All the associated elements have to be connected in 
		 * the form of a single line string
		 * 
		 * @return true If there are atleast two sets of coordinate vectors (lines) and they form a connected line string
		 * @return false Otherwise
		 */
		bool generatePolygonCoords();

		/**
		 * @brief Get the polygonal representation of the occlusion regions as a closed line loop
		 * 
		 * @return const std::vector<Point2D>& coordinates of the single connected line string (forming a polygon) of all the associated coordinate vectors.
		 * 								   Otherwise an empty list.
		 */
		const std::vector<Point2D>& asPolygon() const;

	protected:
		unsigned int internalId;
		int featureId;
		std::string name;
		std::vector<kelo::geometry_common::Polyline2D> occlusionLines;
		std::vector<Point2D> polyCoordinates;
	};

	typedef ZoneNode WaitingLocation;
	typedef ZoneNode ChargingStation;

	class ZonesLayer : public Layer {
	public:
		ZonesLayer(const Map* map);
		virtual ~ZonesLayer();
		
		virtual void loadFeatures(const Map& map) override;
		virtual void loadGeometries(const Map& map) override;
		virtual void loadRelations(const Map& map) override;

		std::vector<const ZoneNode*> getAllChargingStations() const;
		std::vector<const ZoneNode*> getAllWaitingLocations() const;
		std::vector<const ZonePolygon*> getAllForbiddenAreas() const;

		const ZoneLine* getOcclusionLine(int featureId) const;
		std::vector<const ZoneLine*> getAllOcclusionLines() const;

		/**
		 * @brief Get the occlusions lines that intersect with the queried path
		 * 
		 * @param lineString The path of the robot as a set of 2D positions
		 * @return std::vector< std::vector<const ZoneLine*> > A list if occlusion lines that intersect with each segment of the queried path
		 */
		std::vector< std::vector<const ZoneLine*> > getIntersectingOcclusionLines(const std::vector<Point2D>& path) const;

		/**
		 * @brief Get the occlusions lines that intersect with the queried path
		 * 
		 * @param lineString The path of the robot as a set of 2D poses. Only the 2D positions will be considered of checking the intersection.
		 * @return std::vector<const ZoneLine*> A list if occlusion lines that intersect with each segment of the queried path
		 */
		std::vector< std::vector<const ZoneLine*> > getIntersectingOcclusionLines(const std::vector<Pose2D>& path) const;

		/**
		 * @brief Get the occlusion lines that are closest to the queried position
		 * 
		 * @param queryPosition The 2D position from where we wish to search for the occlusions
		 * @param searchRadius The maximum displacement between the queried point and the occlusion line to be considered as valid.
		 * @return std::vector<const ZoneLine*> A sorted list of occlusion lines with increasing distance from the queried point.
		 */
		std::vector<const ZoneLine*> getNearestOcclusionLines(const Point2D& queryPosition, double searchRadius) const;

		const std::map<unsigned int, OcclusionRegion>& getAllOcclusionRegions() const;

		/**
		 * @brief Get the occlusions regions that intersect with the queried path
		 * 
		 * @param lineString The path of the robot as a set of 2D positions
		 * @return std::vector< std::vector<const OcclusionRegion*> > A list if occlusion regions that intersect with each segment of the queried path
		 */
		std::vector< std::vector<const OcclusionRegion*> > getIntersectingOcclusionRegions(const std::vector<Point2D>& path) const;

		/**
		 * @brief Get the occlusions regions that intersect with the queried path
		 * 
		 * @param lineString The path of the robot as a set of 2D poses. Only the 2D positions will be considered of checking the intersection.
		 * @return std::vector<const OcclusionRegion*> A list if occlusion regions that intersect with each segment of the queried path
		 */
		std::vector< std::vector<const OcclusionRegion*> > getIntersectingOcclusionRegions(const std::vector<Pose2D>& path) const;

		/**
		 * @brief Get the occlusion regions that are closest to the queried position
		 * 
		 * @param queryPosition The 2D position from where we wish to search for the occlusions
		 * @param searchRadius The maximum displacement between the queried point and the occlusion line to be considered as valid.
		 * @return std::vector<const OcclusionRegion*> A sorted list of occlusion regions with increasing distance from the queried point.
		 */
		std::vector<const OcclusionRegion*> getNearestOcclusionRegions(const Point2D& queryPosition, double searchRadius) const;

		/**
		 * @brief Get a list of potential occlusion points per edge of the robot path
		 * 
		 * @param path A list of robot waypoints
		 * @return std::vector< std::vector<Point2D> > A list of potential occlusion points for each edge of the path
		 */
		std::vector< std::vector<Point2D> > getOcclusionPointAlongPath(const std::vector<Point2D>& path) const;

		/**
		 * @brief Get a list of potential occlusion points per edge of the robot path
		 * 
		 * @param path A list of robot waypoints
		 * @return std::vector< std::vector<Point2D> > A list of potential occlusion points for each edge of the path
		 */
		std::vector< std::vector<Point2D> > getOcclusionPointAlongPath(const std::vector<Pose2D>& path) const;

		/**
		 * @brief Get the first occlusion point along the robot path
		 * 
		 * @param path A list of robot waypoints
		 * @param occlusionPoint the first occlusion path along the path if found
		 * @return true occlusion point along the path found
		 * @return false no occlusions along the path
		 */
		bool getFirstOcclusionPointAlongPath(const std::vector<Point2D>& path, Point2D& occlusionPoint) const;

		/**
		 * @brief Get the first occlusion point along the robot path
		 * 
		 * @param path A list of robot waypoints
		 * @param occlusionPoint the first occlusion path along the path if found
		 * @return true occlusion point along the path found
		 * @return false no occlusions along the path
		 */
		bool getFirstOcclusionPointAlongPath(const std::vector<Pose2D>& path, Point2D& occlusionPoint) const;

		bool hasRamps() const;
		std::vector<const Ramp*> getAllRamps() const;
		std::vector<const Ramp*> getIntersectingRamps(const Point2D& start, const Point2D& end) const;
		std::vector<const Ramp*> getIntersectingRamps(const std::vector<Point2D>& lineString) const;

		bool hasLoadParkings() const;
		std::vector<const LoadParking*> getAllLoadParkings() const;
		std::vector<std::string> getAllLoadParkingGroupNames() const;
		std::vector<const LoadParking*> getAllLoadParkingsInGroup(const std::string& loadParkingGroupName) const;
		const LoadParking* getLoadParking(const std::string& name) const;

		bool insideForbiddenArea(const Point2D& pos) const;

		ZonesStore zones;

	protected:
		void loadOsmNodes(const Map& map);
		void loadOsmWays(const Map& map);
		void loadOcclusionRegions(const Map& map); // explicitly defined in the map
		void loadInterlayerAssociations(const Map& map);
		void loadIntralayerAssociations(const Map& map);

		bool loadRampEdges(const Map& map, const osm::Relation* relation);
		bool loadLoadParkingOpenings(const Map& map, const osm::Relation* relation);
		bool loadLoadParkingGroup(const osm::Relation* relation);

		void autoGenerateOcclusionRegions(const Map& map); // at junctions and doors

		std::vector<const ZoneNode*> getNodes(zoneTypes::ZoneTypes type) const;
		std::vector<const ZoneLine*> getLines(zoneTypes::ZoneTypes type) const;
		std::vector<const ZonePolygon*> getPolygons(zoneTypes::ZoneTypes type) const;

		std::string getZoneName(const osm::Primitive* primitive) const;
		zoneTypes::ZoneTypes getZoneType(const osm::Primitive* primitive) const;
		double getRampInclination(const osm::Primitive* primitive) const;

		std::map<unsigned int, OcclusionRegion> occlusionRegions;
	};
}
}

#endif // KELO_KELOJSON_ZONES_LAYER_H
