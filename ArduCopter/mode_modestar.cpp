#include "Copter.h"

#if MODE_MODESTAR_ENABLED

/*
 * Init and run calls for modestar flight mode
 * Files a five-pointed star pattern around the current position
 */
// init - initialise modestar controller
bool modestar::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        auto_yaw.set_mode_to_default(false);

        path_num = 0;
        generate_path();
        // start in position control mode
        pos_control_start();
        return true;
    } else {
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
    pos_control_run();
    
}

void modestar::generate_path()
{
    float radius_cm = g2.star_r_cm;

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

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller - advances along the star path and feeds
    // position/velocity targets into the position controller
    wp_nav->update_wpnav();

    // WP_Nav has set the vertical position control targets
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// allows arming with this mode selected
bool modestar::allows_arming(AP_Arming::Method method) const
{
    return true;
}

// wp_distance - distance to active waypoint
uint32_t modestar::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

// wp_bearing - bearing to next waypoint
int32_t modestar::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

// crosstrack_error - horizontal error of the copter relative to the active track
float modestar::crosstrack_error() const
{
    return wp_nav->crosstrack_error();
}
#endif