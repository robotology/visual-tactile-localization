service ObjectTrackingManipulationIDL
{
    string set_hand(1:string laterality);
    string unset_hand();
    string move_up();
    string move_down();
    string move_left();
    string move_right();
    string move_in();
    string move_out();
    string move_rest(1:string laterality);
    string move_home(1:string laterality);
    string stop();
    string quit();
}