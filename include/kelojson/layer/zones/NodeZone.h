#ifndef KELO_KELOJSON_NODE_ZONE_H
#define KELO_KELOJSON_NODE_ZONE_H

#include <geometry_common/Pose2D.h>

#include <kelojson/layer/zones/Zone.h>

namespace kelo {
namespace kelojson {

class NodeZone : public Zone
{
    public:

        using Ptr = std::shared_ptr<NodeZone>;

        using ConstPtr = std::shared_ptr<const NodeZone>;

        using Vec = std::vector<NodeZone::Ptr>;

        using ConstVec = std::vector<const NodeZone::Ptr>;

        virtual ~NodeZone() = default;

        virtual bool initialise(
                int node_id,
                const osm::Primitive::Store& store) override;

        const geometry_common::Pose2D& getPose() const;

        const geometry_common::Point2D meanPoint() const override;

        virtual void write(std::ostream& out) const override;

    protected:

        geometry_common::Pose2D pose_;

        NodeZone(ZoneType type):
            Zone(type) {}

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_NODE_ZONE_H
