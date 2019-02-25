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
