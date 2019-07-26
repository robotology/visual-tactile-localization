/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <SingleVideoResult.h>

#include <iostream>


int main(int argc, char** argv)
{
    std::string result = "/home/xenvre/validation";
    std::string data = "/mnt/ycb_video/640_480";

    SingleVideoResult svr("004_sugar_box", "0049", data, result);

    std::cout << svr.getObjectPose(1000).translation() << std::endl << svr.getObjectPose(1).rotation() << std::endl;
    std::cout << svr.getGroundTruthPose(1000).translation() << std::endl << svr.getGroundTruthPose(1).rotation() << std::endl;
    
    return 0;
}
