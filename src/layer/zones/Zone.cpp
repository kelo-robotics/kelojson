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

#include <kelojson/Print.h>
#include <kelojson/layer/zones/Zone.h>
#include <kelojson/layer/Layer.h>
#include <kelojson/layer/areas/AreasLayer.h>

namespace kelo {
namespace kelojson {

void Zone::initialise(const osm::Primitive::Ptr& primitive)
{
    id_ = primitive->getId();
    primitive_type_ = primitive->getPrimitiveType();
    if ( !primitive->readTag<std::string>("name", name_) )
    {
        name_ = asString(getType()) + "_" + std::to_string(id_);
    }
}

bool Zone::initialiseInterLayerAssociation(
        const osm::RelationPrimitive::Ptr& relation,
        const Layer::Map& layers)
{
    const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
    for ( size_t i = 1; i < members.size(); i++ )
    {
        const osm::RelationPrimitive::Member& child_member = members[i];
        LayerType layer_type = asLayerType(child_member.role);
        if ( layers.find(layer_type) == layers.end() )
        {
            std::cout << Print::Err << "[Zone] "
                      << "layers map does not contain "
                      << asString(layer_type) << " layer."
                      << Print::End << std::endl;
            return false;
        }
        if ( inter_layer_associations_.find(layer_type) == inter_layer_associations_.end() )
        {
            inter_layer_associations_[layer_type] = std::set<int>();
        }

        Layer::ConstPtr layer = layers.at(layer_type);
        switch ( layer_type )
        {
            case LayerType::AREAS:
            {
                const AreasLayer::ConstPtr areas_layer =
                    std::static_pointer_cast<const AreasLayer>(layer);
                if ( child_member.type != osm::PrimitiveType::WAY )
                {
                    std::cout << Print::Warn << "[Zone] "
                              << "Association relation has child member of type "
                              << asString(child_member.type) << ", but only WAY "
                              << "is supported." << std::endl << *relation
                              << Print::End << std::endl;
                    return false;
                }
                const Area::ConstPtr area = areas_layer->getArea(child_member.id);
                if ( area == nullptr )
                {
                    std::cout << Print::Warn << "[Zone] "
                              << "Association relation has child member with id "
                              << child_member.id << ", but could not find area "
                              << "with that id" << std::endl << *relation
                              << Print::End << std::endl;
                    return false;
                }
                inter_layer_associations_[layer_type].insert(child_member.id);
                break;
            }
            default:
                // std::cout << Print::Warn << "[Zone] "
                //           << "No implementation found to establish inter-layer "
                //           << "association from ZONES layer to " << asString(layer_type)
                //           << " layer." << std::endl << *relation
                //           << Print::End << std::endl;
                break;
        }
    }
    return true;
}

std::vector<int> Zone::getOverlappingAreaIds() const
{
    if ( inter_layer_associations_.find(LayerType::AREAS) != inter_layer_associations_.end() )
    {
        const std::set<int>& associations = inter_layer_associations_.at(LayerType::AREAS);
        return std::vector<int>(associations.begin(), associations.end());
    }
    return std::vector<int>();
}

int Zone::getId() const
{
    return id_;
}

ZoneType Zone::getType() const
{
    return type_;
}

osm::PrimitiveType Zone::getPrimitiveType() const
{
    return primitive_type_;
}

const std::string& Zone::getName() const
{
    return name_;
}

const std::map<LayerType, std::set<int>>& Zone::getInterlayerAssociations() const
{
    return inter_layer_associations_;
}

void Zone::writeGeneric(std::ostream& out) const
{
    out << "<Zone " << asString(getType())
        << ", primitive_type: " << asString(getPrimitiveType())
        << ", id: " << getId()
        << ", name: " << getName() << ", ";
}

} // namespace kelojson
} // namespace kelo
