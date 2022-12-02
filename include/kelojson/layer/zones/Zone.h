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

#ifndef KELO_KELOJSON_ZONE_H
#define KELO_KELOJSON_ZONE_H

#include <geometry_common/Point2D.h>

#include <kelojson/osm/Primitive.h>
#include <kelojson/osm/RelationPrimitive.h>
#include <kelojson/layer/LayerType.h>
#include <kelojson/layer/zones/ZoneType.h>

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
