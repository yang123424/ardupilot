#include "Copter.h"

#if MODE_GUIDED_ENABLED

/*
 * Init and run calls for guided flight mode
 */
// init - initialise guided controller
bool modestar::init(bool ignore_checks)
{
    if(position_ok()||ignore_checks){
    auto_yaw.set_mode_to_default(false);
    // start in position control mode
    pos_control_start();
    return true;
}else{
    return false;
}
}

// run - runs the guided controller
// should be called at 100hz or more
void modestar::run()
{
pos_control_start()
 }


// initialise position controller
void modestar::pva_control_start()
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    // initialise terrain alt
    guided_pos_terrain_alt = false;
}

// initialise guided mode's position controller
void modestar::pos_control_start()
{
    wp_nav->wp_and_spline_init();
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);
    wp_nav->set_wp_destination(stopping_point,false);
    auto_yaw.set_mode_to_default(false);
}
