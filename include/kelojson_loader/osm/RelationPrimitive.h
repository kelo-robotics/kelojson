#ifndef KELO_KELOJSON_OSM_RELATION_PRIMITIVE_H
#define KELO_KELOJSON_OSM_RELATION_PRIMITIVE_H

#include <kelojson_loader/osm/Primitive.h>

namespace kelo {
namespace kelojson {
namespace osm {

class RelationPrimitive : public Primitive
{
    public:

        using Ptr = std::shared_ptr<RelationPrimitive>;

        using ConstPtr = std::shared_ptr<const RelationPrimitive>;

        struct Member
        {
            PrimitiveType type;
            int id;
            std::string role;

            bool initialise(const YAML::Node& feature);
            void write(std::ostream& out) const;
        };

        RelationPrimitive():
            Primitive(PrimitiveType::RELATION) {}

        RelationPrimitive(const RelationPrimitive& relation):
            Primitive(relation),
            members_(relation.members_) {}

        virtual ~RelationPrimitive () = default;

        bool initialise(const YAML::Node& feature);

        const std::vector<Member>& getMembers() const;

        void write(std::ostream& out) const;

    private:

        std::vector<Member> members_;

};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_RELATION_PRIMITIVE_H
