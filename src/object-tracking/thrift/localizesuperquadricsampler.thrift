/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

/**
 * LocalizeSuperquadricSamplerIDL
 *
 */

service LocalizeSuperquadricSamplerIDL
{
    /**
     * Sent the most recent modelled superquadric to the viewer.
     *
     * @return true/false on sucess/failure
     */
    bool send_superquadric_to_viewer();
}
