#include "Copter.h"

/*
 * Init and run calls for tornado flight mode
 */

bool ModeTornado::init(bool ignore_checks)
{
    pilot_yaw_override = false;
    _angular_vel_max = 0.0f;
    _radius = g2.torn_radius_start;
    _gain_vector = 1;

    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    pos_control->set_desired_accel_xy(0.0f,0.0f);
    pos_control->set_desired_velocity_xy(0.0f,0.0f);
    pos_control->init_xy_controller();

    // set initial position target to reasonable stopping point
    pos_control->set_target_to_stopping_point_xy();
    pos_control->set_target_to_stopping_point_z();

    // get stopping point
    const Vector3f& stopping_point = pos_control->get_pos_target();

    _center.x = stopping_point.x + g2.torn_radius_start.get() * ahrs.cos_yaw();
    _center.y = stopping_point.y + g2.torn_radius_start.get() * ahrs.sin_yaw();
    _center.z = stopping_point.z;

    // calculate velocities
    calc_velocities(true);

    // set starting angle from vehicle heading
    init_start_angle(true);


    return true;
}

// should be called at 100hz or more
void ModeTornado::run()
{
    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);
    pos_control->set_leash_length_xy(g2.torn_radius_max*2);

    // get pilot's desired yaw rate (or zero if in radio failsafe)
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        pilot_yaw_override = true;
    }

    // get pilot desired climb rate (or zero if in radio failsafe)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    update();

    // call attitude controller
    if (pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(),
                                                                      pos_control->get_pitch(),
                                                                      target_yaw_rate);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(),
                                                           pos_control->get_pitch(),
                                                           _yaw, true);
    }

    // update altitude target and call position controller
    // protects heli's from inflight motor interlock disable
    if (motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::GROUND_IDLE && !copter.ap.land_complete) {
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
    } else {
        pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    }
    pos_control->update_z_controller();
}

uint32_t ModeTornado::wp_distance() const
{
    return pos_control->get_distance_to_target();
}

int32_t ModeTornado::wp_bearing() const
{
    return pos_control->get_bearing_to_target();
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void ModeTornado::calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius <= 0) {
        _angular_vel_max = ToRad(g2.torn_curve_rate.get());
        _angular_accel = MAX(fabsf(_angular_vel_max),ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));  // reach maximum yaw velocity in 1 second
    }else{
        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float velocity_max = MIN(pos_control->get_max_speed_xy(), safe_sqrt(0.5f*pos_control->get_max_accel_xy()*_radius));

        // angular_velocity in radians per second
        _angular_vel_max = velocity_max/_radius;
        _angular_vel_max = constrain_float(ToRad(g2.torn_curve_rate.get()),-_angular_vel_max,_angular_vel_max);

        // angular_velocity in radians per second
        _angular_accel = MAX(pos_control->get_max_accel_xy()/_radius, ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));
    }

    // initialise angular velocity
    if (init_velocity) {
        _angular_vel = 0;
    }
}

void ModeTornado::update()
{
    // calculate dt
    float dt = pos_control->time_since_last_xy_update();
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // ramp angular velocity to maximum
    if (_angular_vel < _angular_vel_max) {
        _angular_vel += fabsf(_angular_accel) * dt;
        _angular_vel = MIN(_angular_vel, _angular_vel_max);
    }
    if (_angular_vel > _angular_vel_max) {
        _angular_vel -= fabsf(_angular_accel) * dt;
        _angular_vel = MAX(_angular_vel, _angular_vel_max);
    }

    // update the target angle and total angle traveled
    float angle_change = _angular_vel * dt;
    _angle += angle_change;
    _angle = wrap_PI(_angle);
    _angle_total += angle_change;

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    if (!is_zero(_radius)) {
        if (_radius >= g2.torn_radius_max) {
            _gain_vector = -1;
        }

        if (_radius < 0) {
            _gain_vector = 1;
        }

        _radius += _gain_vector * g2.torn_radius_gain;

        // calculate target position
        Vector3f target;

        target.z = pos_control->get_alt_target();
        target.x = _center.x + _radius * cosf(-_angle);
        target.y = _center.y + _radius * sinf(-_angle);

        // update position controller target
        pos_control->set_xy_target(target.x, target.y);

        // heading is from vehicle to center of circle
        _yaw = get_bearing_cd(inertial_nav.get_position(), _center);
    } else {
        // set target position to center
        Vector3f target;
        target.x = _center.x;
        target.y = _center.y;
        target.z = pos_control->get_alt_target();

        // update position controller target
        pos_control->set_xy_target(target.x, target.y);

        // heading is same as _angle but converted to centi-degrees
        _yaw = _angle * DEGX100;
    }

    // update position controller
    pos_control->update_xy_controller();
}

void ModeTornado::init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius <= 0) {
        _angle = ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        _angle = wrap_PI(ahrs.yaw-M_PI);
    } else {
        const Vector3f &curr_pos = inertial_nav.get_position();
        if (is_equal(curr_pos.x,_center.x) && is_equal(curr_pos.y,_center.y)) {
            _angle = wrap_PI(ahrs.yaw-M_PI);
        } else {
            float bearing_rad = atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x);
            _angle = wrap_PI(bearing_rad);
        }
    }
}