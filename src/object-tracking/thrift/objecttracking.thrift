/**
 * ObjectTrackingIDL
 *
 */

service ObjectTrackingIDL
{
    /**
     * Initialize and run the filter.
     *
     * @return true/false on success/failure.
     */
    bool run_filter();

    /**
     * Reset the filter.
     *
     * @return true/false on success/failure.
     */
    bool reset_filter();

    /**
     * Stop and reset the filter.
     *
     * @return true/false on success/failure.
     */
    bool stop_filter();

    /**
     * Pause the filter.
     *
     */
    void pause_filter();

    /**
     * Resume the filter.
     *
     */
    void resume_filter();

    /**
     * Enable/disable use of contacts.
     *
     */
    void contacts(1:bool enable);

    /**
     * Enable/Disable skipping the filtering step specified in what_step.
     * what_step can be one of the following:
     *
     *  1) prediction: skips the whole prediction step
     *
     *  2) state: skips the prediction step related to the state transition
     *
     *  3) exogenous: skips the prediction step related exogenous inputs
     *
     *  4) correction: skips the whole correction step
     *
     * @param what_step the step to skipping
     * @param status enable/disable skipping
     *
     * @return true/false on success/failure.
     */
    bool skip_step(1:string what_step, 2:bool status);

    /**
     * Get information on the point estimates extraction.
     *
     * @return a string containing the requested information.
     */
    list<string> get_point_estimate_info();

    /**
     * Change the current point estimate extraction method.
     *
     * @param method a string with the name of point estimate extraction method to be used.
     *
     * @return true/false on success/failure.
     */
    bool set_point_estimate_method(1:string method);

    /**
     * Change the window size of the history buffer used for point estimate extraction.
     *
     * @param window specifies the window size.
     *
     * @return true/fale on success/failure.
     */
    bool set_history_window(1:i16 window);

    /**
     * Quit the filter in graceful way.
     */
    bool quit();
}
