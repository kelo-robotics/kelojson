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

#ifndef KELO_KELOJSON_TOPOLOGY_EDGE_H
#define KELO_KELOJSON_TOPOLOGY_EDGE_H

#include <geometry_common/LineSegment2D.h>

#include <kelojson/osm/WayPrimitive.h>
#include <kelojson/layer/topology/TopologyNode.h>
#include <kelojson/layer/areas/AreasLayer.h>

namespace kelo {
namespace kelojson {

class TopologyEdge
{
    public:

        using Ptr = std::shared_ptr<TopologyEdge>;

        using ConstPtr = std::shared_ptr<const TopologyEdge>;

        using Vec = std::vector<TopologyEdge::Ptr>;

        using ConstVec = std::vector<TopologyEdge::ConstPtr>;

        using Matrix = std::vector<TopologyEdge::Vec>;

        TopologyEdge() = default;

        virtual ~TopologyEdge() = default;

        bool initialise(
                const osm::WayPrimitive::ConstPtr& way,
                size_t internal_id,
                const TopologyNode::ConstPtr& start_node,
                const TopologyNode::ConstPtr& end_node);

        bool initialiseInterLayerAssociation(
                const std::map<LayerType, std::shared_ptr<Layer>>& layers);

        std::vector<int> getOverlappingAreaIds() const;

        bool isInArea(const Area& area) const;

        const geometry_common::LineSegment2D getLineSegment() const;

        int getPrimitiveId() const;

        size_t getInternalId() const;

        const std::string& getName() const;

        const TopologyNode::ConstPtr& getStartNode() const;

        const TopologyNode::ConstPtr& getEndNode() const;

        bool isOneWay() const;

        const std::map<LayerType, std::set<int>>& getInterlayerAssociations() const;

        const osm::Tags& getTags() const;

        friend std::ostream& operator << (
                std::ostream& out,
                const TopologyEdge& edge);

    protected:

        int primitive_id_;
        size_t internal_id_;
        std::string name_;

        TopologyNode::ConstPtr start_node_;
        TopologyNode::ConstPtr end_node_;
        bool is_oneway_{false};

        std::map<LayerType, std::set<int>> inter_layer_associations_;
        osm::Tags tags_;

        bool addAllOverlappingAreas(
                int start_node_area_id,
                int end_node_area_id,
                const AreasLayer& areas_layer);
};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_EDGE_H
