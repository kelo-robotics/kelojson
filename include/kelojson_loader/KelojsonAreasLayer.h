#ifndef KELO_KELOJSON_AREAS_LAYER_H
#define KELO_KELOJSON_AREAS_LAYER_H

#include "kelojson_loader/KelojsonLayer.h"

namespace kelojson {

	namespace areaTypes {
		enum AreaTypes {
			UNKNOWN = 0,
			ROOM,
			CORRIDOR,
			OPEN_AREA,
			COUNT
		};

		AreaTypes getType(std::string type);
		std::string getName(AreaTypes type);
	}

	namespace doorTypes {
		enum DoorTypes {
			NONE = 0,
			HINGED,
			SLIDING,
			COUNT
		};

		DoorTypes getType(std::string type);
		std::string getName(DoorTypes type);
	}

	class AreaTransition {
	public:
		friend bool operator<(const AreaTransition& lhs, const AreaTransition& rhs);

		bool isDoor() const;
		double width() const;

		std::vector<Pos> coordinates;
		doorTypes::DoorTypes doorType;
		int featureId;
		std::pair<int, int> associatedAreaIds;
	};

	class Area {
	public:
		int featureId;
		areaTypes::AreaTypes areaType;
		std::string name;
		std::vector<Pos> coordinates;

		std::vector<int> getAdjacentAreas() const;
		std::vector<AreaTransition> getTransitionsWithArea(int adjAreaId) const;
		std::map<int, std::vector<AreaTransition> > getAllDoors() const;
		std::vector<AreaTransition> getDoorsWithArea(int adjAreaId) const;

		bool insideBoundingBox(Pos point) const;
		double getBoundingBoxArea() const;
		bool contains(Pos point) const;

		std::map<int, std::set<AreaTransition> > transitions;
		std::pair<Pos, Pos> boundingBox;
	};

	class AreaBBoxComparator {
	public:
		bool operator() (const Area* a, const Area* b) const {
			return a->getBoundingBoxArea() > b->getBoundingBoxArea();
		}
	};

	class AreasLayer : public Layer {
	public:
		AreasLayer(const Map* map);
		virtual ~AreasLayer() {}

		virtual void loadGeometries(const Map& map) /*override*/;
		virtual void loadRelations(const Map& map) /*override*/;

		void loadAreaTransitions(const Map& map);

		const Area* getArea(int areaId) const;
		const Area* getArea(const std::string& areaName) const;
		const Area* getArea(const Pos& point) const;

		const AreaTransition* getAreaTransition(int featureId) const;
		std::map<int, const AreaTransition*> getAllAreaTransitions() const;
		bool getDistanceToTransition(const Pos& point, const AreaTransition* transition, double& distance) const;
		std::vector<const AreaTransition*> getNearestTransitions(const Pos& point,
														  double searchRadius = std::numeric_limits<double>::max(),
														  unsigned int maxNumTransitions = std::numeric_limits<unsigned int>::max()) const;

		std::string getAreaName(int areaId) const;
		std::pair<bool, int> getAreaId(const std::string& areaName) const;

		std::vector<int> computePath(int startAreaId, int goalAreaId) const;
		std::vector<int> computePath(const std::string& startAreaName, const std::string& goalAreaName) const;
		std::vector<int> computePath(const Pos& startPos, const Pos& goalPos) const;
		std::string getPrintablePath(const std::vector<int>& path) const;

		std::vector<const AreaTransition*> getIntersectingAreaTransitions(const Pos& start, const Pos& end) const;
		std::vector<const AreaTransition*> getIntersectingAreaTransitions(const std::vector<Pos>& lineString) const;

		// Returns true if the queried position overlaps with any of the mapped areas
		bool contains(const Pos& pos) const;

		std::map<int, Area> areas;

	protected:
		std::multiset<const Area*, AreaBBoxComparator> getAreasByBBoxSize() const;
	};
}

#endif // KELO_KELOJSON_AREAS_LAYER_H
