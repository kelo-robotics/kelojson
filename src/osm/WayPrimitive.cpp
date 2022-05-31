#include <yaml_common/Parser2.h>

#include <kelojson_loader/osm/WayPrimitive.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool WayPrimitive::initialise(const YAML::Node& feature)
{
    return ( Primitive::initialise(feature) && 
             Parser::hasKey(feature, "geometry") &&
             Parser::read<std::string>(feature["geometry"], "type", type_) &&
             Parser::hasKey(feature["geometry"], "nodeIds") &&
             feature["geometry"]["nodeIds"].IsSequence() &&
             feature["geometry"]["nodeIds"].size() > 0 &&
             Parser::read<std::vector<int>>(feature["geometry"]["nodeIds"][0], node_ids_) );
}

bool WayPrimitive::extractNodes(const YAML::Node& feature, std::vector<NodePrimitive::Ptr>& nodes)
{
    if ( !Parser::hasKey(feature, "geometry") )
    {
        return false;
    }

    if ( !Parser::hasKey(feature["geometry"], "coordinates") ||
         !feature["geometry"]["coordinates"].IsSequence() ||
         !Parser::hasKey(feature["geometry"], "nodeIds") ||
         !feature["geometry"]["nodeIds"].IsSequence() )
    {
        return false;
    }

    const std::string type = Parser::get<std::string>(feature["geometry"], "type", "");
    bool is_polygon = ( type == "Polygon" );
    const YAML::Node& coords = ( is_polygon )
                               ? feature["geometry"]["coordinates"][0]
                               : feature["geometry"]["coordinates"];
    const YAML::Node& node_ids = feature["geometry"]["nodeIds"][0];
    if ( node_ids.size() != coords.size() )
    {
        return false;
    }

    nodes.clear();
    for ( size_t i = 0; i < coords.size(); i++ )
    {
        NodePrimitive::Ptr node = std::make_shared<NodePrimitive>();
        int node_id;
        if ( !Parser::read<int>(node_ids[i], node_id) ||
             !node->initialise(node_id, coords[i]) )
        {
            return false;
        }
        nodes.push_back(node);
    }
    return true;
}

const std::vector<int>& WayPrimitive::getNodeIds() const
{
    return node_ids_;
}

void WayPrimitive::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "node_ids: [";
    for ( size_t i = 0; i < node_ids_.size(); i++ )
    {
        out << node_ids_[i] << ", ";
    }
    out << "]>";
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
