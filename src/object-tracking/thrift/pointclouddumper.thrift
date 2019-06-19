/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

/**
 * PointCloudDumperIDL
 *
 */

service PointCloudDumperIDL
{
    /**
     * Reset stored data.
     *
     * @return true/false on success/failure.
     */
    bool reset();

    /**
     * Save stored frames.
     *
     * @return true/false on success/failure.
     */
    bool save_frames();
}
