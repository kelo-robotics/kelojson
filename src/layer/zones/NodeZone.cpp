#include <kelojson/osm/PrimitiveUtils.h>

#include <kelojson/layer/zones/NodeZone.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Pose2D;

namespace kelo {
namespace kelojson {

bool NodeZone::initialise(int node_id, const osm::Primitive::Store& store)
{
    const osm::NodePrimitive::Ptr node = osm::PrimitiveUtils::getNode(store, node_id);
    if ( node == nullptr )
    {
        return false;
    }

    Zone::initialise(node);
    pose_ = Pose2D(node->getPosition(), node->getTag("theta", 0.0f));
    return true;

}

const Pose2D& NodeZone::getPose() const
{
    return pose_;
}

const Point2D NodeZone::meanPoint() const
{
    return pose_.position();
}

void NodeZone::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "pose: " << pose_ << ">";
}

} // namespace kelojson
} // namespace kelo
