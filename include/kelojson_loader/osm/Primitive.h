#ifndef KELO_KELOJSON_OSM_PRIMITIVE_H
#define KELO_KELOJSON_OSM_PRIMITIVE_H

#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>
#include <boost/lexical_cast.hpp>

#include <kelojson_loader/osm/PrimitiveType.h>

namespace kelo {
namespace kelojson {
namespace osm {

using Tags = std::map<std::string, std::string>;

class Primitive
{
    public:

        using Ptr = std::shared_ptr<Primitive>;

        using ConstPtr = std::shared_ptr<const Primitive>;

        using Map = std::map<int, Primitive::Ptr>;

        using Store = std::map<PrimitiveType, Primitive::Map>;


        Primitive(const Primitive& primitive);

        virtual ~Primitive() = default;

        bool initialise(const YAML::Node& feature);

        template <typename T>
        bool readTag(const std::string& key, T& value) const
        {
            if ( tags_.find(key) == tags_.end() )
            {
                return false;
            }

            try
            {
                value = boost::lexical_cast<T>(tags_.at(key).c_str());
            }
            catch( const boost::bad_lexical_cast& )
            {
                std::cout << "[Primitive] Failed to parse property value for " << key << std::endl;
                return false;
            }
            return true;
        }

        template <typename T>
        T getTag(const std::string& key, const T& default_value) const
        {
            T value;
            return readTag(key, value) ? value : default_value;
        }

        void setId(int id);

        int getId() const;

        PrimitiveType getType() const;

        const Tags& getTags() const;

        virtual void write(std::ostream& out) const = 0;

    protected:

        Primitive(const PrimitiveType& type):
            type_(type) {}

        void writeGeneric(std::ostream& out) const;

    private:

        int id_;
        PrimitiveType type_;
        Tags tags_;

        bool parseTags(const YAML::Node& feature);

        Primitive() = delete;

};

inline std::ostream& operator << (std::ostream& out, const Primitive& primitive)
{
    primitive.write(out);
    return out;
};

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_H
