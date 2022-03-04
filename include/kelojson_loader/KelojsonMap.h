#ifndef KELO_KELOJSON_MAP_H
#define KELO_KELOJSON_MAP_H

#include "kelojson_loader/KelojsonAreasLayer.h"
#include "kelojson_loader/KelojsonZonesLayer.h"
#include "kelojson_loader/KelojsonTopologyLayer.h"

namespace kelojson {

	class Map {
	public:
		Map();
		virtual ~Map();

		bool loadFile(std::string filename);
		bool loadFromString(std::string mapString);
		bool clear();

		const osm::Node* getOsmNode(int id) const;
		const osm::Way* getOsmWay(int id) const;
		const osm::Relation* getOsmRelation(int id) const;

		std::vector<layerType::LayerType> getAvailableLayerTypes() const;
		const AreasLayer* getAreasLayer() const;
		const ZonesLayer* getZonesLayer() const;
		const TopologyLayer* getTopologyLayer() const;
		const OccupancyGridLayer* getOccupancyLayer() const;
		// const SubAreasLayer* getSubAreasLayer() const;

		// Returns true if the queried position lies inside the map
		bool insideMap(const Point2D& pos) const;

	protected:
		void clearOsmPrimitives();
		void clearAllLayers();

		// OSM parsing methods 
		bool loadOsmPrimitives(const YAML::Node& yaml);
		bool initializeOsmPrimitiveStore();
		bool isOsmGeometry(const YAML::Node& feature) const;
		bool isOsmRelation(const YAML::Node& feature) const;
		bool isOsmNode(const YAML::Node& feature) const;
		bool isOsmWay(const YAML::Node& feature) const;

		// Layer parsing methods 
		bool loadlayers();
		void sortOsmPrimitivesIntoLayers();

		LayerMap layers;
		OsmPrimitiveStore osmPrimitiveStore;
	};
}

#endif // KELO_KELOJSON_MAP_H
