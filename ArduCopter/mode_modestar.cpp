#include "Copter.h"

#if MODE_GUIDED_ENABLED

/*
 * Init and run calls for guided flight mode
 */
// init - initialise guided controller
bool modestar::init(bool ignore_checks)
{
    // start in velaccel control mode
    velaccel_control_start();
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    send_notification = false;

    // clear pause state when entering guided mode
    _paused = false;

    return true;
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
    // set to position control mode
    guided_mode = SubMode::Pos;

    // initialise position controller
    pva_control_start();
}
