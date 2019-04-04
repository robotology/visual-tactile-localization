/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CONTACTDETECTION_H
#define CONTACTDETECTION_H

#include <unordered_map>

class ContactDetection
{
public:
    virtual std::pair<bool, std::unordered_map<std::string, bool>> getActiveFingers() = 0;
};

#endif /* CONTACTDETECTION_H */
