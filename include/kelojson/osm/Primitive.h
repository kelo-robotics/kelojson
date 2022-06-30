#ifndef KELO_KELOJSON_OSM_PRIMITIVE_H
#define KELO_KELOJSON_OSM_PRIMITIVE_H

#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

#include <kelojson/Print.h>
#include <kelojson/osm/PrimitiveType.h>

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


        Primitive(const Primitive& primitive):
            type_(primitive.type_),
            id_(primitive.id_),
            primitive_type_(primitive.primitive_type_),
            tags_(primitive.tags_) {}

        virtual ~Primitive() = default;

        virtual bool initialise(const YAML::Node& feature);

        template <typename T>
        bool readTag(const std::string& key, T& value) const
        {
            if ( tags_.find(key) == tags_.end() )
            {
                return false;
            }

            const std::string& key_value = tags_.at(key);
            try
            {
                value = Primitive::convertStringTo<T>(key_value);
            }
            catch( const std::exception& ex )
            {
                std::cout << Print::Err << "[Primitive] "
                          << "Failed to convert tag key: " << key << ", value: "
                          << key_value << " to required type with exception "
                          << ex.what() << Print::End << std::endl;
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

        PrimitiveType getPrimitiveType() const;

        const std::string& getType() const;

        const Tags& getTags() const;

        virtual void write(std::ostream& out) const = 0;

    protected:

        std::string type_;

        Primitive(const PrimitiveType& type):
            primitive_type_(type) {}

        void writeGeneric(std::ostream& out) const;

    private:

        int id_;
        PrimitiveType primitive_type_;
        Tags tags_;

        bool parseTags(const YAML::Node& feature);

        template <typename T>
        static T convertStringTo(const std::string& value);

        Primitive() = delete;

};

inline std::ostream& operator << (std::ostream& out, const Primitive& primitive)
{
    primitive.write(out);
    return out;
};

template <>
std::string Primitive::convertStringTo<std::string>(const std::string& value);

template <>
int Primitive::convertStringTo<int>(const std::string& value);

template <>
size_t Primitive::convertStringTo<size_t>(const std::string& value);

template <>
float Primitive::convertStringTo<float>(const std::string& value);

template <>
bool Primitive::convertStringTo<bool>(const std::string& value);

} // namespace osm
} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_OSM_PRIMITIVE_H
