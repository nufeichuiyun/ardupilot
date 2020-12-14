#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for guided flight mode
 */


// guided_init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{

    path_num = 0;

    generate_path();

    // start in position control mode
    pos_control_start();
    return true;
}

void ModeDrawStar::generate_path()
{
    float radius_cm = g2.star_radius_cm;

    wp_nav->get_wp_stopping_point(path[0]);

    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];
}

// initialise guided mode's position controller
void ModeDrawStar::pos_control_start()
{

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();


    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}


// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeDrawStar::run()
{
    if(path_num < 6){
        if(wp_nav->reached_wp_destination()){
            path_num ++;
            wp_nav->set_wp_destination(path[path_num],false);
        }
    }

    pos_control_run();
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void ModeDrawStar::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}


#endif
