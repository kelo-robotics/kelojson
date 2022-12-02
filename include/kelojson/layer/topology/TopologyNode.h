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

#ifndef KELO_KELOJSON_TOPOLOGY_NODE_H
#define KELO_KELOJSON_TOPOLOGY_NODE_H

#include <geometry_common/Point2D.h>

#include <kelojson/osm/Primitive.h>
#include <kelojson/osm/RelationPrimitive.h>
#include <kelojson/layer/LayerType.h>
#include <kelojson/layer/areas/Area.h>

namespace kelo {
namespace kelojson {

// forward declaration
class Layer;

class TopologyNode
{
    public:

        using Ptr = std::shared_ptr<TopologyNode>;

        using ConstPtr = std::shared_ptr<const TopologyNode>;

        using Vec = std::vector<TopologyNode::Ptr>;

        using ConstVec = std::vector<TopologyNode::ConstPtr>;

        TopologyNode() = default;

        virtual ~TopologyNode() = default;

        bool initialise(
                int node_id,
                size_t internal_id,
                const osm::Primitive::Store& store);

        bool initialiseInterLayerAssociation(
                const osm::RelationPrimitive::Ptr& relation,
                const std::map<LayerType, std::shared_ptr<Layer>>& layers);

        bool getOverlappingAreaId(int& area_id) const;

        bool isInArea(const Area& area) const;

        int getPrimitiveId() const;

        size_t getInternalId() const;

        const std::string& getName() const;

        const geometry_common::Point2D& getPosition() const;

        const std::map<LayerType, std::set<int>>& getInterlayerAssociations() const;

        friend std::ostream& operator << (std::ostream& out, const TopologyNode& node);

    protected:

        int primitive_id_;
        size_t internal_id_;
        std::string name_;
        std::map<LayerType, std::set<int>> inter_layer_associations_;
        geometry_common::Point2D position_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_NODE_H
