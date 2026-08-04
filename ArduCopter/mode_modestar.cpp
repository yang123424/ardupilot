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

    path_num = 0;
    generate_path();
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
    if(path_num <6){
        if(wp_nav->reached_wp_destination()){
            path_num++;
            wp_nav->set_wp_destination(path[path_num],false);
        }
    }
    pos_control_run()
    
}

void modestar::generate_path()
{
    float radius_cm = 1000.0;

    wp_nav->get_wp_stopping_point(path[0]);
    path[1] = path[0] + Vector3f(1.0f, 0.0, 0.0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];
}
// initialise guided mode's position controller
void modestar::pos_control_start()
{
    wp_nav->wp_and_spline_init();

    wp_nav->set_wp_destination(path[0],false);
    auto_yaw.set_mode_to_default(false);
}

void modestar::pos_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (guided_pos_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // send position and velocity targets to position controller
    guided_accel_target_cmss.zero();
    guided_vel_target_cms.zero();

    // stop rotating if no updates received within timeout_ms
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    float pos_offset_z_buffer = 0.0; // Vertical buffer size in m
    if (guided_pos_terrain_alt) {
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin() * 100.0, 0.5 * fabsF(guided_pos_target_cm.z));
    }
    pos_control->input_pos_xyz(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}