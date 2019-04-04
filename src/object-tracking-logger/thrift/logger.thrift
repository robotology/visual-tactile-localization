/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

/**
 * ObjectTrackingLoggerIDL
 *
 */

service ObjectTrackingLoggerIDL
{
    /**
     * Run the logger
     *
     * @return true/false on success/failure.
     */
    bool run();

    /**
     * Stop the logger.
     *
     * @return true/false on success/failure.
     */
    bool stop();

    /**
     * Quit the logger.
     *
     * @return true/false on success/failure.
     */
    bool quit();
}
