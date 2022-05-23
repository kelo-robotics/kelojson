#include <yaml_common/Parser2.h>

#include <kelojson_loader/osm/RelationPrimitive.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

bool RelationPrimitive::initialise(const YAML::Node& feature)
{
    if ( !Primitive::initialise(feature) ||
         !Parser::hasKey(feature, "properties") ||
         !Parser::read<std::string>(feature["properties"], "type", type_ ) ||
         !Parser::hasKey(feature, "relation") ||
         !Parser::hasKey(feature["relation"], "members") ||
         !feature["relation"]["members"].IsSequence() )
    {
        return false;
    }

    for ( const YAML::Node& member_yaml : feature["relation"]["members"] )
    {
        Member member;
        if ( !member.initialise(member_yaml) )
        {
            return false;
        }
        members_.push_back(member);
    }
    return true;
}

bool RelationPrimitive::Member::initialise(const YAML::Node& member_yaml)
{
    std::string type_str;
    if ( !Parser::read<std::string>(member_yaml, "type", type_str) )
    {
        return false;
    }
    std::transform(type_str.begin(), type_str.end(), type_str.begin(), ::toupper);
    type = asPrimitiveType(type_str);
    if ( type == PrimitiveType::INVALID )
    {
        return false;
    }
    if ( !Parser::read<int>(member_yaml, "id", id) ||
         !Parser::read<std::string>(member_yaml, "role", role) )
    {
        return false;
    }
    return true;
}

void RelationPrimitive::Member::write(std::ostream& out) const
{
    out << "<Member "
        << "type: "<< asString(type)
        << ", id: " << id
        << ", role: " << role
        << ">";
}

const std::vector<RelationPrimitive::Member>& RelationPrimitive::getMembers() const
{
    return members_;
}

void RelationPrimitive::write(std::ostream& out) const
{
    writeGeneric(out);
    out << "members: [";
    for ( size_t i = 0; i < members_.size(); i++ )
    {
        out << "    ";
        members_[i].write(out);
        out << std::endl;
    }
    out << "]>";
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
