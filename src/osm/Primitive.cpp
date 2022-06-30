#include <yaml_common/Parser2.h>

#include <kelojson/osm/Primitive.h>

using Parser = kelo::yaml_common::Parser2;

namespace kelo {
namespace kelojson {
namespace osm {

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

PrimitiveType Primitive::getPrimitiveType() const
{
    return primitive_type_;
}

const std::string& Primitive::getType() const
{
    return type_;
}

const Tags& Primitive::getTags() const
{
    return tags_;
}

void Primitive::writeGeneric(std::ostream& out) const
{
    out << "<Primitive " << asString(getPrimitiveType())
        << ", id: " << getId()
        << ", type: " << getType();

    const Tags& tags = getTags();
    out << ", tags: [";
    for ( Tags::const_iterator itr = tags.cbegin(); itr != tags.cend(); itr ++ )
    {
        if ( itr != tags.cbegin() )
        {
            out << ", ";
        }
        out << itr->first << ": " << itr->second;
    }
    out << "], ";
}

template <>
std::string Primitive::convertStringTo<std::string>(const std::string& value)
{
    return value;
}

template <>
int Primitive::convertStringTo<int>(const std::string& value)
{
    return std::stoi(value);
}

template <>
size_t Primitive::convertStringTo<size_t>(const std::string& value)
{
    return std::stoul(value);
}

template <>
float Primitive::convertStringTo<float>(const std::string& value)
{
    return std::stof(value);
}

template <>
bool Primitive::convertStringTo<bool>(const std::string& value)
{
    if ( value == "yes" || value == "Yes" || value == "YES" ||
         value == "1" ||
         value == "true" || value == "True" || value == "TRUE" )
    {
        return true;
    }
    else if ( value == "no" || value == "No" || value == "NO" ||
              value == "0" ||
              value == "false" || value == "False" || value == "FALSE" )
    {
        return false;
    }

    throw std::invalid_argument("Could not convert to bool");
}

} // namespace osm
} // namespace kelojson
} // namespace kelo
