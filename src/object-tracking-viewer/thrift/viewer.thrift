/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

/**
 * ViewerIDL
 *
 */

service ViewerIDL
{
    /**
     * Enable visualization using a superquadric.
     *
     * @param method a string with the name of point estimate extraction method to be used.
     *
     * @return true/false on success/failure.
     */
    bool use_superquadric(1:double size_x, 2:double size_y, 3:double size_z, 4:double eps_1, 5:double eps_2, 6:double x, 7:double y, 8:double z, 9:double phi, 10:double theta, 11:double psi);
}
