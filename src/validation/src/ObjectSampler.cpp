/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectSampler.h>


ObjectSampler::~ObjectSampler()
{ }


void ObjectSampler::setObjectName(const std::string& object_name)
{
    std::string err = "ObjectSampler::setObjectBame. Base class method not implemented.";
    throw(std::runtime_error(err));
}
