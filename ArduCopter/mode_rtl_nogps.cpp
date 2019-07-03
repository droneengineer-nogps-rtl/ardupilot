#include "Copter.h"


/*
 * Init and run calls for RTL no GPS, flight mode
 */

// RTLNoGPS - initialise 
bool ModeRTLNoGPS::init(bool ignore_checks)
{
    _state_gps = true;
    return ModeRTL::init(ignore_checks);
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void ModeRTLNoGPS::run()
{
    if (!motors->armed()) {
        return;
    }

    if ((copter.gps.status() == AP_GPS::NO_FIX) || (copter.gps.status() == AP_GPS::NO_FIX)) {
        _state_gps = false;

        pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
        pos_control->set_max_accel_z(g.pilot_accel_z);

        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();
        calculate_home_distance();

        attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, home_az, true);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();


        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

        // get pilot's desired yaw rate
        //float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

        // call attitude controller
//        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);



        return;
    }
    _state_gps = true;
    set_loc();
    ModeRTL::run(true);
}

bool ModeRTLNoGPS::set_loc()
{
    Location home_loc;

    last_loc_alt = copter.current_loc.alt; // 高度
    last_loc_lat = copter.current_loc.lat; // 緯度
    last_loc_lng = copter.current_loc.lng; // 経度

    home_loc = copter.inertial_nav.get_origin();
    home_loc_alt = home_loc.alt;
    home_loc_lat = home_loc.lat;
    home_loc_lng = home_loc.lng;

    return true;
}

bool ModeRTLNoGPS::calculate_home_distance()
{
    double x1, y1, x2, y2, theta;

    x1 = last_loc_lat/1e7 * M_PI/180;
    y1 = last_loc_lng/1e7 * M_PI/180;
    x2 = home_loc_lat/1e7 * M_PI/180;
    y2 = home_loc_lng/1e7 * M_PI/180;

    home_distance = acos((sin(y1) * sin(y2))  +  (cos(y1) * cos(y2) * cos(x2 - x1))) * earth_radius;

    theta = atan2(sin(x2 - x1) ,(cos(y1) * tan(y2)) - (sin(y1) * cos(x2 - x1)));
    home_az = theta >= 0 ? theta * 180/M_PI: (theta + (M_2PI * 180/M_PI));
    return true;
}

void ModeRTLNoGPS::init_target()
{
    // initialise position controller
    pos_control->set_desired_velocity_xy(0.0f,0.0f);
    pos_control->set_desired_accel_xy(0.0f,0.0f);
    pos_control->init_xy_controller();

    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_xy(inertial_nav.get_velocity().length());
    pos_control->set_max_accel_xy(BRAKE_MODE_DECEL_RATE);
    pos_control->calc_leash_length_xy();

    // set target position
    Vector3f stopping_point;
    pos_control->get_stopping_point_xy(stopping_point);
    pos_control->set_xy_target(stopping_point.x, stopping_point.y);
}
