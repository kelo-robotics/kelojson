#ifndef KELO_KELOJSON_LAYER_H
#define KELO_KELOJSON_LAYER_H

#include "kelojson_loader/OsmPrimitives.h"
#include "kelojson_loader/KelojsonTypes.h"

namespace kelo {
namespace kelojson {

	// Forward declarations
	class Map;

	class OccupancyGrid {
	public:
		int featureId;
		std::string filename;
		double freeThreshold;
		double occupiedThreshold;
		bool negate;
		std::string name;
		double resolution;
		Pose2D origin;
	};


	class Layer {
	public:
		Layer(layerType::LayerType type, const Map* map);
		virtual ~Layer() {}

		void addOsmPrimitive(const osm::Primitive* primitive);
		virtual void loadFeatures(const Map& map);
		virtual void loadGeometries(const Map& /*map*/) {}
		virtual void loadRelations(const Map& /*map*/) {}

		unsigned int getNumOsmPrimitives(osm::primitiveType::PrimitiveType primType);

		const Map* kelojsonMap;
		layerType::LayerType layerType;
		std::map<osm::primitiveType::PrimitiveType, std::vector<int> > osmPrimitives;
	};

	class OccupancyGridLayer : public Layer {
	public:
		OccupancyGridLayer(const Map* map);
		virtual ~OccupancyGridLayer() {}
		
		virtual void loadGeometries(const Map& map) /*override*/;
		virtual void loadRelations(const Map& map) /*override*/;

		std::map<int, OccupancyGrid> occupancyGrids;
	};

	// class SubAreasLayer : public Layer {
	// public:
	// 	SubAreasLayer();
	// 	~SubAreasLayer(){}

	// 	virtual void loadGeometries(const Map& map) /*override*/;
	// 	virtual void loadRelations(const Map& map) /*override*/;
	// };
}
}

#endif // KELO_KELOJSON_LAYER_H
