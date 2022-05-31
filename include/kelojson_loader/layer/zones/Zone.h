#ifndef KELO_KELOJSON_ZONE_H
#define KELO_KELOJSON_ZONE_H

#include <geometry_common/Point2D.h>

#include <kelojson_loader/osm/Primitive.h>
#include <kelojson_loader/osm/RelationPrimitive.h>
#include <kelojson_loader/layer/LayerType.h>
#include <kelojson_loader/layer/zones/ZoneType.h>

namespace kelo {
namespace kelojson {

// forward declaration
class Layer;

class Zone
{
    public:

        using Ptr = std::shared_ptr<Zone>;

        using ConstPtr = std::shared_ptr<const Zone>;

        using Map = std::map<int, Zone::Ptr>;

        using Store = std::map<ZoneType, Zone::Map>;

        virtual ~Zone() = default;

        virtual bool initialise(int id, const osm::Primitive::Store& store) = 0;

        virtual bool initialiseInterLayerAssociation(
                const osm::RelationPrimitive::Ptr& relation,
                const std::map<LayerType, std::shared_ptr<Layer>>& layers);

        int getId() const;

        ZoneType getType() const;

        osm::PrimitiveType getPrimitiveType() const;

        const std::string& getName() const;

        std::vector<int> getOverlappingAreaIds() const;

        virtual const geometry_common::Point2D meanPoint() const = 0;

        const std::map<LayerType, std::set<int>>& getInterlayerAssociations() const;

        virtual void write(std::ostream& out) const = 0;

    protected:

        int id_;
        std::string name_;
        std::map<LayerType, std::set<int>> inter_layer_associations_;
        osm::PrimitiveType primitive_type_{osm::PrimitiveType::INVALID};

        Zone(ZoneType type):
            type_(type) {}

        void initialise(const osm::Primitive::Ptr& primitive);

        void writeGeneric(std::ostream& out) const;

    private:

        ZoneType type_;

        Zone() = delete;

};

inline std::ostream& operator << (std::ostream& out, const Zone& zone)
{
    zone.write(out);
    return out;
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_ZONE_H
