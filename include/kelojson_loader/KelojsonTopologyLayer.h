#ifndef KELO_KELOJSON_TOPOLOGY_LAYER_H
#define KELO_KELOJSON_TOPOLOGY_LAYER_H

#include "kelojson_loader/KelojsonLayer.h"

namespace kelojson {

	// Forward declarations
	class Area;
	class AreaTransition;
	class TopologyLayer;
	class Ramp;
	struct PlannerConfig;

	class TopologyNode {
	public:
		TopologyNode();
		TopologyNode(int featureId, const Point2D& pos);
		virtual ~TopologyNode() {}

		std::pair<bool, int> getOverlappingAreaId() const;

		Point2D position;
		int featureId;
		std::map<layerType::LayerType, std::set<int> > interlayerAssociations;
	};

	class TopologyEdge {
	public:
		int featureId;
		unsigned int edgeId;
		int startNodeId;
		int endNodeId;
	};

	class TopologyLayer : public Layer {
	public:
		typedef std::map<int, unsigned int> InternalIdMap;
		typedef std::map<int, unsigned int>::const_iterator InternalIdMapConstItr;
		typedef std::map<unsigned int, int> FeatureIdMap;
		typedef std::map<unsigned int, int>::const_iterator FeatureIdMapConstItr;

		TopologyLayer(const Map* map);
		virtual ~TopologyLayer() {}

		virtual void loadGeometries(const Map& map) /*override*/;
		virtual void loadRelations(const Map& map) /*override*/;

		void extractGraphElements(std::vector<Pose2D>& nodes, std::vector<Edge>& edges) const;

		std::vector<int> getNodesInArea(const Area* area) const;
		std::vector<unsigned int> getEdgesInArea(const Area* area, bool fullyContained = false) const;

		bool getInternalNodeId(int osmNodeId, unsigned int& internalNodeId) const;
		bool getOsmNodeId(unsigned int internalNodeId, int& osmNodeId) const;

		bool getEdgeId(int startOsmNodeId, int endOsmNodeId, unsigned int& edgeId) const;

		const TopologyNode* getClosestNodeInArea(const Point2D& point) const;
		const TopologyEdge* getClosestEdgeInArea(const Point2D& point, const std::vector<BlockedEdge>& blockedEdges = std::vector<BlockedEdge>()) const;
		std::vector<unsigned int> getConnectedEdges(const TopologyNode& node) const;

		double minDistanceToEdge(const Point2D& point, int edgeId) const;

		Plan computePath(int startNodeFeatureId, int endNodeFeatureId, const std::vector<BlockedEdge>& blockedEdges = std::vector<BlockedEdge>()) const;
		Plan computePath(const Point2D& start, const Point2D& end, const std::vector<BlockedEdge>& blockedEdges = std::vector<BlockedEdge>()) const;

		// Returns a point on the the path that is closest to query point
		Point2D getNearestPointOnPath(const Plan& path, const Point2D& point) const;

		std::vector< std::vector<const kelojson::AreaTransition*> > getAreaTransitionsAlongPath(const std::vector<Point2D>& path) const;
		std::vector< std::vector<const kelojson::AreaTransition*> > getAreaTransitionsAlongPath(const Plan& path) const;

		std::vector< std::vector<const Ramp*> > getRampsAlongPath(const std::vector<Point2D>& path) const;
		std::vector< std::vector<const Ramp*> > getRampsAlongPath(const Plan& path) const;

		TopologyNodeMap topologyNodes;
		TopologyEdgeMap topologyEdges;

	protected:
		void loadInterlayerAssociations(const Map& map);

		InternalIdMap nodeInternalIdMap;
		FeatureIdMap nodeFeatureIdMap;
	};
}

#endif // KELO_KELOJSON_TOPOLOGY_LAYER_H
