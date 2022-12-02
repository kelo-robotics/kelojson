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
#include <kelojson/osm/PrimitiveUtils.h>
#include <kelojson/layer/topology/TopologyNode.h>
#include <kelojson/layer/Layer.h>
#include <kelojson/layer/areas/AreasLayer.h>

namespace kelo {
namespace kelojson {

bool TopologyNode::initialise(
        int node_id,
        size_t internal_id,
        const osm::Primitive::Store& store)
{
    const osm::NodePrimitive::Ptr node = osm::PrimitiveUtils::getNode(store, node_id);
    if ( node == nullptr )
    {
        return false;
    }

    primitive_id_ = node->getId();
    internal_id_ = internal_id;
    if ( !node->readTag<std::string>("name", name_) )
    {
        name_ = "TopologyNode_"
              + std::to_string(primitive_id_) + "_"
              + std::to_string(internal_id_);
    }
    position_ = node->getPosition();
    return true;
}

bool TopologyNode::initialiseInterLayerAssociation(
        const osm::RelationPrimitive::Ptr& relation,
        const std::map<LayerType, std::shared_ptr<Layer>>& layers)
{
    const std::vector<osm::RelationPrimitive::Member>& members = relation->getMembers();
    for ( size_t i = 1; i < members.size(); i++ )
    {
        const osm::RelationPrimitive::Member& child_member = members[i];
        LayerType layer_type = asLayerType(child_member.role);
        if ( layers.find(layer_type) == layers.end() )
        {
            std::cout << Print::Err << "[TopologyNode] "
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
                    std::cout << Print::Warn << "[TopologyNode] "
                              << "Association relation has child member of type "
                              << asString(child_member.type) << ", but only WAY "
                              << "is supported." << std::endl << *relation
                              << Print::End << std::endl;
                    return false;
                }
                const Area::ConstPtr area = areas_layer->getArea(child_member.id);
                if ( area == nullptr )
                {
                    std::cout << Print::Warn << "[TopologyNode] "
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
                // std::cout << Print::Warn << "[TopologyNode] "
                //           << "No implementation found to establish inter-layer "
                //           << "association from TOPOLOGY layer to " << asString(layer_type)
                //           << " layer." << std::endl << *relation
                //           << Print::End << std::endl;
                break;
        }
    }

    /* sanity check */
    if ( inter_layer_associations_.find(LayerType::AREAS) != inter_layer_associations_.end() &&
         inter_layer_associations_.at(LayerType::AREAS).size() != 1 )
    {
        std::cout << Print::Warn << "[TopologyNode] "
                  << "TopologyNode with primitive_id: " << primitive_id_
                  << " is part of multiple areas." << std::endl << *relation
                  << Print::End << std::endl;
        return false;
    }
    return true;
}

bool TopologyNode::getOverlappingAreaId(int& area_id) const
{
    if ( inter_layer_associations_.find(LayerType::AREAS) != inter_layer_associations_.end() )
    {
        const std::set<int>& associations = inter_layer_associations_.at(LayerType::AREAS);
        area_id = *(associations.begin());
        return true;
    }
    return false;
}

bool TopologyNode::isInArea(const Area& area) const
{
    if ( inter_layer_associations_.find(LayerType::AREAS) != inter_layer_associations_.end() )
    {
        const std::set<int>& associations = inter_layer_associations_.at(LayerType::AREAS);
        return ( associations.find(area.getId()) != associations.end() );
    }
    return false;
}

int TopologyNode::getPrimitiveId() const
{
    return primitive_id_;
}

size_t TopologyNode::getInternalId() const
{
    return internal_id_;
}

const std::string& TopologyNode::getName() const
{
    return name_;
}

const geometry_common::Point2D& TopologyNode::getPosition() const
{
    return position_;
}

const std::map<LayerType, std::set<int>>& TopologyNode::getInterlayerAssociations() const
{
    return inter_layer_associations_;
}

std::ostream& operator << (std::ostream& out, const TopologyNode& node)
{
    out << "<TopologyNode "
        << ", primitive_id: " << node.primitive_id_
        << ", internal_id: " << node.internal_id_
        << ", name: " << node.name_
        << ", position: " << node.position_ << ">";
    return out;
}

} // namespace kelojson
} // namespace kelo
