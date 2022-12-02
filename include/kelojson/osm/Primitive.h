/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Dharmin Bakaraniya
 * Sushant Chavan
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

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
