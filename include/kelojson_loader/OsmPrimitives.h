#ifndef KELO_OSM_PRIMITIVES_H
#define KELO_OSM_PRIMITIVES_H

#include "kelojson_loader/KelojsonTypes.h"
#include <yaml-cpp/yaml.h>
#include <boost/lexical_cast.hpp>
#include <iostream>

namespace kelo {
namespace kelojson {

	namespace osm {

		class Primitive {
		public:
			Primitive() {}
			Primitive(const YAML::Node& feature);
			virtual ~Primitive() {}

			Tags getOsmTags(const YAML::Node& feature);
			template<typename T>
			bool getTagValue(const std::string key, T& value) const;
			kelojson::layerType::LayerType getLayerType();

			int primitiveId;
			primitiveType::PrimitiveType primitiveType;
			Tags tags;
		};
		template<typename T>
		bool Primitive::getTagValue(const std::string key, T& value) const {
			if (tags.find(key) == tags.end()) {
				return false;
			}

			T val;
			try {
				val = boost::lexical_cast<T>(tags.at(key).c_str());
			} catch( boost::bad_lexical_cast const& ) {
				std::cout << "Failed to parse property value for " << key.c_str() << std::endl;
				return false;
			}
			value = val;
			return true;
		}

		class Node : public Primitive {
		public:
			Node();
			Node(const YAML::Node& feature);
			Node(const Node& node); // Copy constructor
			virtual ~Node() {}

			Point2D position;
		};

		class Way : public Primitive {
		public:
			Way();
			Way(const YAML::Node& feature);
			virtual ~Way() {}

			static std::vector<Node> extractNodes(const YAML::Node& feature);

			std::vector<int> nodeIds;
			std::string wayType;
		};

		class RelationMember {
		public:
			RelationMember(const YAML::Node& member);
			virtual ~RelationMember() {}

			primitiveType::PrimitiveType primitiveType;
			int primitiveId;
			std::string role;
		};

		class Relation : public Primitive {
		public:
			Relation();
			Relation(const YAML::Node& feature);
			virtual ~Relation() {}

			std::vector<RelationMember> members;
			std::string relationType;
		};
	}
}
}

#endif // KELO_OSM_PRIMITIVES_H
