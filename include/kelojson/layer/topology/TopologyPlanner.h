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

#ifndef KELO_KELOJSON_TOPOLOGY_PLANNER_H
#define KELO_KELOJSON_TOPOLOGY_PLANNER_H

#include <kelojson/layer/topology/TopologyNode.h>
#include <kelojson/layer/topology/TopologyEdge.h>

namespace kelo {
namespace kelojson {

class TopologyPlanner
{
    public:

        enum class SearchType
        {
            BFS,
            DIJKSTRA
        };

        static const TopologyNode::ConstVec plan(
                const TopologyNode::Vec& nodes,
                const TopologyEdge::Matrix& adjacency_matrix,
                const TopologyNode& start,
                const TopologyNode& goal,
                const SearchType& search_type = SearchType::BFS);

    protected:

        struct Node
        {
            float g{0.0f}, h{0.0f}, f{0.0f};
            size_t topology_node_id;
        };

        static const TopologyNode::ConstVec backtrack(
                const TopologyNode::Vec& nodes,
                const std::vector<size_t>& parent_of,
                size_t start_id,
                size_t goal_id);

        static bool greaterNode(const Node& n1, const Node& n2)
        {
            return ( n1.f > n2.f );
        }

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_TOPOLOGY_PLANNER_H
