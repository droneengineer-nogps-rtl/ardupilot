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
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();
        calculate_home_distance();

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
