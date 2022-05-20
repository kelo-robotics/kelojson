#include <yaml_common/Parser2.h>

#include <kelojson_loader/osm/Primitive.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

Primitive::Primitive(const Primitive& primitive)
{
    type_ = primitive.type_;
    id_ = primitive.id_;
    tags_ = primitive.tags_;
}

bool Primitive::initialise(const YAML::Node& feature)
{
    return Parser::read<int>(feature, "id", id_) && parseTags(feature);
}

bool Primitive::parseTags(const YAML::Node& feature)
{
    if ( !Parser::hasKey(feature, "properties", false) )
    {
        return true;
    }

    return Parser::read<std::map<std::string, std::string>>(
            feature, "properties", tags_, true);
}

void Primitive::setId(int id)
{
    id_ = id;
}

int Primitive::getId() const
{
    return id_;
}

PrimitiveType Primitive::getType() const
{
    return type_;
}

const Tags& Primitive::getTags() const
{
    return tags_;
}

void Primitive::writeGeneric(std::ostream& out) const
{
    out << "<Primitive type: " << asString(getType())
        << ", id: " << getId() << ", ";

    const Tags& tags = getTags();
    out << "tags: [";
    for ( Tags::const_iterator itr = tags.begin(); itr != tags.end(); itr ++ )
    {
        out << itr->first << ": " << itr->second << ", ";
    }
    out << "], ";
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
