/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

service ObjectTrackingManipulationIDL
{
    string set_hand(1:string laterality);
    string unset_hand();
    string gaze_track_on();
    string gaze_track_off();
    string move_up();
    string move_down();
    string move_left();
    string move_right();
    string move_in();
    string move_out();
    string move_rest(1:string laterality);
    string move_home(1:string laterality);
    string latch_approach();
    string coarse_approach();
    string precise_approach();
    string test_coarse_approach();
    string test_precise_approach();
    string open_hand();
    string close_hand();
    string start_motion();
    string get_hand_pose(1:string laterality);
    string stop();
    string quit();
    string do_pour();
}