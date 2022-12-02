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

#ifndef KELO_KELOJSON_TOPOLOGY_LAYER_H
#define KELO_KELOJSON_TOPOLOGY_LAYER_H

#include <kelojson/layer/Layer.h>
#include <kelojson/layer/areas/AreasLayer.h>
#include <kelojson/layer/topology/TopologyNode.h>
#include <kelojson/layer/topology/TopologyEdge.h>
#include <kelojson/layer/topology/TopologyPlanner.h>

namespace kelo {
namespace kelojson {

class TopologyLayer : public Layer
{
    public:

        using Ptr = std::shared_ptr<TopologyLayer>;

        using ConstPtr = std::shared_ptr<const TopologyLayer>;

        TopologyLayer():
            Layer(LayerType::TOPOLOGY) {}

        virtual ~TopologyLayer() = default;

        bool initialise(const osm::Primitive::Store& store) override;

        bool initialiseInterLayerAssociation(
                const Layer::Map& layers,
                const osm::Primitive::Store& store) override;

        // ====================================================================
        // Nodes
        // ====================================================================

        const TopologyNode::ConstPtr getNodeWithInternalId(size_t internal_id) const;

        const TopologyNode::ConstPtr getNodeWithPrimitiveId(int primitive_id) const;

        const TopologyNode::ConstVec getNodesInArea(const Area& area) const;

        const TopologyNode::ConstPtr getNearestNodeInArea(
                const geometry_common::Point2D& point) const;

        const TopologyNode::ConstVec getAdjacentNodes(const TopologyNode& node) const;

        const TopologyNode::ConstVec getAllNodes() const;

        const TopologyNode::ConstVec computePath(
                const TopologyNode& start,
                const TopologyNode& goal,
                const TopologyPlanner::SearchType& search_type =
                        TopologyPlanner::SearchType::BFS) const;

        // ====================================================================
        // Edges
        // ====================================================================

        const TopologyEdge::ConstPtr getEdgeWithInternalId(size_t internal_id) const;

        const TopologyEdge::ConstVec getEdgesInArea(const Area& area) const;

        const TopologyEdge::ConstPtr getNearestEdgeInArea(
                const geometry_common::Point2D& point) const;

        const TopologyEdge::ConstPtr getNearestEdgeInArea(
                const geometry_common::Pose2D& pose,
                bool only_oneway = false,
                float theta_tolerance = M_PI/4) const;

        const TopologyEdge::ConstVec getAllEdges() const;

        const TopologyEdge::Matrix& getAdjacencyMatrix() const;

        friend std::ostream& operator << (
                std::ostream& out,
                const TopologyLayer& layer);

    private:

        TopologyNode::Vec nodes_;
        TopologyEdge::Vec edges_;
        TopologyEdge::Matrix adjacency_matrix_;
        std::map<int, size_t> primitive_id_to_internal_id_;
        AreasLayer::ConstPtr areas_layer_;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_LAYER_H
