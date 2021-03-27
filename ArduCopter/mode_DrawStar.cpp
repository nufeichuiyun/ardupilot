#include "Copter.h"

#if MODE_DRAWSTAR_ENABLED == ENABLED

/*
 * 五角星航线模式初始化
 */
bool ModeDrawStar::init(bool ignore_checks)
{
    path_num = 0;  // 航点号清零，从而切到其他模式再切回来后，可以飞出一个新的五角星航线
    generate_path();  // 生成五角星航线

    pos_control_start();  // 开始位置控制

    return true;
}

// 生成五角星航线
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

// 开始位置控制
void ModeDrawStar::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// 此模式的周期调用
void ModeDrawStar::run()
{
    if (path_num < 6) {  // 五角星航线尚未走完
        if (wp_nav->reached_wp_destination()) {  // 到达某个端点
            path_num++;
            wp_nav->set_wp_destination(path[path_num], false);  // 将下一个航点位置设置为导航控制模块的目标位置
        }
    } else if ((path_num == 6) && wp_nav->reached_wp_destination()) {  // 五角星航线运行完成，自动进入Loiter模式
        gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into loiter mode");
        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
    }

    pos_control_run();  // 位置控制器
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void ModeDrawStar::pos_control_run()  // 注意，此函数直接从mode_guided.cpp中复制过来，不需要该其中的内容
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
