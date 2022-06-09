#include <kelojson_loader/Print.h>
#include <kelojson_loader/layer/topology/TopologyEdge.h>
#include <kelojson_loader/layer/Layer.h>
#include <kelojson_loader/layer/areas/AreasLayer.h>

using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::Polyline2D;

namespace kelo {
namespace kelojson {

bool TopologyEdge::initialise(
        const osm::WayPrimitive::ConstPtr& way,
        size_t internal_id,
        const TopologyNode::ConstPtr& start_node,
        const TopologyNode::ConstPtr& end_node)
{
    primitive_id_ = way->getId();
    internal_id_ = internal_id;
    if ( start_node == nullptr || end_node == nullptr )
    {
        std::cout << Print::Err << "[TopologyEdge] primitive_id: " << primitive_id_
                  << "start_node and/or end_node is nullptr"
                  << Print::End << std::endl;
        return false;
    }
    start_node_ = start_node;
    end_node_ = end_node;
    is_oneway_ = way->getTag<bool>("oneway", false);

    name_ = "TopologyEdge_"
          + std::to_string(primitive_id_) + "_"
          + std::to_string(internal_id_);
    tags_ = way->getTags();
    return true;
}

bool TopologyEdge::initialiseInterLayerAssociation(
        const Layer::Map& layers)
{
    LayerType layer_type = LayerType::AREAS; // only areas layer supported right now
    if ( layers.find(layer_type) == layers.end() )
    {
        std::cout << Print::Err << "[TopologyEdge] "
                  << "layers map does not contain "
                  << asString(layer_type) << " layer."
                  << Print::End << std::endl;
        return false;
    }
    if ( inter_layer_associations_.find(layer_type) == inter_layer_associations_.end() )
    {
        inter_layer_associations_[layer_type] = std::set<int>();
    }

    Layer::ConstPtr layer = layers.at(layer_type);
    const AreasLayer::ConstPtr areas_layer =
        std::static_pointer_cast<const AreasLayer>(layer);
    int start_node_area_id, end_node_area_id;
    bool start_node_in_area = start_node_->getOverlappingAreaId(start_node_area_id);
    bool end_node_in_area = end_node_->getOverlappingAreaId(end_node_area_id);
    if ( start_node_in_area )
    {
        inter_layer_associations_[layer_type].insert(start_node_area_id);
    }
    if ( end_node_in_area )
    {
        inter_layer_associations_[layer_type].insert(end_node_area_id);
    }
    if ( start_node_in_area && end_node_area_id &&
         start_node_area_id != end_node_area_id )
    {
        if ( !addAllOverlappingAreas(start_node_area_id, end_node_area_id, *areas_layer) )
        {
            return false;
        }
    }
    return true;
}

bool TopologyEdge::addAllOverlappingAreas(
        int start_node_area_id,
        int end_node_area_id,
        const AreasLayer& areas_layer)
{
    /* first check if start area and end area are adjacent to each other (optmisation) */
    const Area::ConstPtr start_area = areas_layer.getArea(start_node_area_id);
    const std::vector<int> start_area_adjacent_area_ids = start_area->getAdjacentAreaIds();
    if ( std::find(start_area_adjacent_area_ids.begin(),
                   start_area_adjacent_area_ids.end(),
                   end_node_area_id) != start_area_adjacent_area_ids.end() )
    {
        return true; // if they are, nothing to do since they would already have been added
    }

    std::set<int>& overlapping_areas = inter_layer_associations_[LayerType::AREAS];
    const Area::ConstPtr end_area = areas_layer.getArea(end_node_area_id);
    const LineSegment2D segment = getLineSegment();
    Area::ConstPtr area = start_area;
    size_t itr = 0, itr_limit = 50;
    while ( area->getId() != end_area->getId() )
    {
        itr ++;
        if ( itr >= itr_limit )
        {
            std::cout << Print::Warn << "[TopologyEdge] "
                      << "overlaps with > 2 areas but cannot find which areas "
                      << "since more than " << itr_limit << " area transitions "
                      << "intersect with edge line segment." << std::endl
                      << *this << Print::End << std::endl;
            return false;
        }
        const Transition::ConstVec area_transitions = area->getTransitions();
        bool found = false;
        for ( const Transition::ConstPtr& transition : area_transitions )
        {
            const Polyline2D& polyline = transition->getPolyline();
            if ( polyline.intersects(segment) )
            {
                const std::pair<Area::ConstPtr, Area::ConstPtr> associated_areas =
                    transition->getAssociatedAreas();
                const Area::ConstPtr other_area =
                    ( associated_areas.first->getId() == area->getId() )
                    ? associated_areas.second
                    : associated_areas.first;
                if ( overlapping_areas.find(other_area->getId()) == overlapping_areas.end() ||
                     end_area->getId() == other_area->getId() )
                {
                    area = other_area;
                    overlapping_areas.insert(area->getId());
                    found = true;
                    break;
                }
            }
        }
        if ( !found )
        {
            std::cout << Print::Warn << "[TopologyEdge] "
                      << "overlaps with > 2 areas but cannot find which areas "
                      << "since none of the transitions of " << area->getName()
                      << " intersect with edge line segment." << std::endl
                      << *this << Print::End << std::endl;
            return false;
        }
    }
    return true;
}

std::vector<int> TopologyEdge::getOverlappingAreaIds() const
{
    if ( inter_layer_associations_.find(LayerType::AREAS) != inter_layer_associations_.end() )
    {
        const std::set<int>& associations = inter_layer_associations_.at(LayerType::AREAS);
        return std::vector<int>(associations.begin(), associations.end());
    }
    return std::vector<int>();
}

bool TopologyEdge::isInArea(const Area& area) const
{
    if ( inter_layer_associations_.find(LayerType::AREAS) != inter_layer_associations_.end() )
    {
        const std::set<int>& associations = inter_layer_associations_.at(LayerType::AREAS);
        return ( associations.find(area.getId()) != associations.end() );
    }
    return false;
}

const LineSegment2D TopologyEdge::getLineSegment() const
{
    return LineSegment2D(start_node_->getPosition(), end_node_->getPosition());
}

int TopologyEdge::getPrimitiveId() const
{
    return primitive_id_;
}

const std::string& TopologyEdge::getName() const
{
    return name_;
}

const TopologyNode::ConstPtr& TopologyEdge::getStartNode() const
{
    return start_node_;
}

const TopologyNode::ConstPtr& TopologyEdge::getEndNode() const
{
    return end_node_;
}

bool TopologyEdge::isOneWay() const
{
    return is_oneway_;
}

const std::map<LayerType, std::set<int>>& TopologyEdge::getInterlayerAssociations() const
{
    return inter_layer_associations_;
}

const osm::Tags& TopologyEdge::getTags() const
{
    return tags_;
}

std::ostream& operator << (std::ostream& out, const TopologyEdge& edge)
{
    out << "<TopologyEdge "
        << ", primitive_id: " << edge.primitive_id_
        << ", internal_id: " << edge.internal_id_
        << ", name: " << edge.name_
        << ", start_node: " << edge.start_node_->getName()
        << ", end_node: " << edge.end_node_->getName()
        << ", is_oneway: " << edge.is_oneway_;

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
