/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

/**
 * ModelInitializationIDL
 *
 */

service ModelInitializationIDL
{
    /**
     * Initialize model.
     *
     * @return true/false on success/failure.
     */
    bool initialize_model(1:string object_name);
}
