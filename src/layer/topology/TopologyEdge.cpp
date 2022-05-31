#include <kelojson_loader/osm/PrimitiveUtils.h>
#include <kelojson_loader/layer/topology/TopologyEdge.h>

namespace kelo {
namespace kelojson {

bool TopologyEdge::initialise(int way_id, const osm::Primitive::Store& store)
{
    const osm::WayPrimitive::Ptr way = osm::PrimitiveUtils::getWay(store, way_id);
    if ( way == nullptr )
    {
        std::cout << Print::Err << "[TopologyEdge] Way id: " << way_id
                  << " does not exist." << Print::End << std::endl;
        return false;
    }

    primitive_id_ = way_id;
    if ( !way->readTag<std::string>("name", name_) )
    {
        name_ = "TopologyEdge_" + std::to_string(primitive_id_);
    }
    tags_ = way->getTags();
    return true;
}

int TopologyEdge::getPrimitiveId() const
{
    return primitive_id_;
}

const std::string& TopologyEdge::getName() const
{
    return name_;
}

std::ostream& operator << (std::ostream& out, const TopologyEdge& edge)
{
    out << "<TopologyEdge "
        << ", primitive_id: " << edge.primitive_id_
        << ", name: " << edge.name_;

    out << ", tags: [";
    osm::Tags::const_iterator tag_begin = edge.tags_.cbegin();
    for ( osm::Tags::const_iterator itr = edge.tags_.cbegin();
          itr != edge.tags_.cend();
          itr ++ )
    {
        if ( itr != tag_begin )
        {
            out << ", ";
        }
        out << itr->first << ": " << itr->second;
    }
    out << "]>";
    return out;
}

} // namespace kelojson
} // namespace kelo
