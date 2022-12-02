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

#ifndef KELO_KELOJSON_LAYER_H
#define KELO_KELOJSON_LAYER_H

#include <memory>
#include <map>

#include <kelojson/layer/LayerType.h>
#include <kelojson/osm/Primitive.h>

namespace kelo {
namespace kelojson {

class Layer
{
    public:

        using Ptr = std::shared_ptr<Layer>;

        using ConstPtr = std::shared_ptr<const Layer>;

        using Map = std::map<LayerType, Layer::Ptr>;

        virtual ~Layer() = default;

        virtual bool initialise(const osm::Primitive::Store& store) = 0;

        virtual bool initialiseInterLayerAssociation(
                const Layer::Map& layers,
                const osm::Primitive::Store& store)
        {
            layers.size(); // to suppress unused variable warning
            store.size(); // to suppress unused variable warning
            return true;
        }

        LayerType getType() const
        {
            return type_;
        }

    protected:

        Layer(const LayerType& type):
            type_(type) {}

    private:

        LayerType type_;

        Layer() = delete;

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_LAYER_H
