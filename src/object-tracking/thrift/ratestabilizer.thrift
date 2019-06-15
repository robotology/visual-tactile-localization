/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

/**
 * RateStabilizerIDL
 *
 */

service RateStabilizerIDL
{
    /**
     * Reset the rate stabilizer
     *
     */
    void reset();

    /**
     * Set the fps set point.
     *
     */
    void set_fps(1:double fps);
}
