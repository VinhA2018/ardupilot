#include <AP_HAL/AP_HAL.h>
#include "AP_L1_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_L1_Control::var_info[] = {
    // @Param: PERIOD
    // @DisplayName: L1 control period
    // @Description: Period in seconds of L1 tracking loop. This parameter is the primary control for agressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default of 20 is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PERIOD",    0, AP_L1_Control, _L1_period, 17),

    // @Param: DAMPING
    // @DisplayName: L1 control damping ratio
    // @Description: Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
    // @Range: 0.6 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DAMPING",   1, AP_L1_Control, _L1_damping, 0.75f),

    // @Param: XTRACK_I
    // @DisplayName: L1 control crosstrack integrator gain
    // @Description: Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.
    // @Range: 0 0.1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XTRACK_I",   2, AP_L1_Control, _L1_xtrack_i_gain, 0.02),

    // @Param: L_DIST
    // @DisplayName: L distance
    // @Description: L distance to perform upwind mode
    // @Units: m
    // @Range: 1 90
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("L_DIST",    3, AP_L1_Control, _L_dist, 40),

    // @Param: L_USE
    // @DisplayName: Using L 
    // @Description: L distance to perform upwind mode 1, or 0 to use L1
    // @Units: m
    // @Range: 0 1
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("L_USE",    4, AP_L1_Control, _L_use, 0),

    // @Param: LIM_BANK
    // @DisplayName: Loiter Radius Bank Angle Limit
    // @Description: The sealevel bank angle limit for a continous loiter. (Used to calculate airframe loading limits at higher altitudes). Setting to 0, will instead just scale the loiter radius directly
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO_FRAME("LIM_BANK",   5, AP_L1_Control, _loiter_bank_limit, 0.0f, AP_PARAM_FRAME_PLANE),

    

    AP_GROUPEND
};

//Bank angle command based on angle between aircraft velocity vector and reference vector to path.
//S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
//Proceedings of the AIAA Guidance, Navigation and Control
//Conference, Aug 2004. AIAA-2004-4900.
//Modified to use PD control for circle tracking to enable loiter radius less than L1 length
//Modified to enable period and damping of guidance loop to be set explicitly
//Modified to provide explicit control over capture angle


/*
  Wrap AHRS yaw if in reverse - radians
 */
float AP_L1_Control::get_yaw()
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.yaw);
    }
    return _ahrs.yaw;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
int32_t AP_L1_Control::get_yaw_sensor() const
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t AP_L1_Control::nav_roll_cd(void) const
{
    float ret;
    ret = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * 0.101972f) * 100.0f); // 0.101972 = 1/9.81
    ret = constrain_float(ret, -9000, 9000);
    return ret;
}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_L1_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AP_L1_Control::nav_bearing_cd(void) const
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AP_L1_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t AP_L1_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float AP_L1_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    return MIN(wp_radius, _L1_dist);
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float AP_L1_Control::turn_distance(float wp_radius, float turn_angle) const
{
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float AP_L1_Control::loiter_radius(const float radius) const
{
    // prevent an insane loiter bank limit
    float sanitized_bank_limit = constrain_float(_loiter_bank_limit, 0.0f, 89.0f);
    float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;

    float nominal_velocity_sea_level;
    if(_spdHgtControl == nullptr) {
        nominal_velocity_sea_level = 0.0f;
    } else {
        nominal_velocity_sea_level =  _spdHgtControl->get_target_airspeed();
    }

    float eas2tas_sq = sq(_ahrs.get_EAS2TAS());

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            // If we've told the plane that its sea level radius is unachievable fallback to
            // straight altitude scaling
            return radius * eas2tas_sq;
        } else {
            // select the requested radius, or the required altitude scale, whichever is safer
            return MAX(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool AP_L1_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */
void AP_L1_Control::_prevent_indecision(float &Nu)
{
    const float Nu_limit = 0.9f*M_PI;
    if (fabsf(Nu) > Nu_limit &&
        fabsf(_last_Nu) > Nu_limit &&
        labs(wrap_180_cd(_target_bearing_cd - get_yaw_sensor())) > 12000 &&
        Nu * _last_Nu < 0.0f) {
        // we are moving away from the target waypoint and pointing
        // away from the waypoint (not flying backwards). The sign
        // of Nu has also changed, which means we are
        // oscillating in our decision about which way to go
        Nu = _last_Nu;
    }
}

/*
// update L1 control for waypoint navigation
void AP_L1_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{

    struct Location _current_loc;
    float Nu;
    float xtrackVel;
    float ltrackVel;

    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1) {
        dt = 0.1;
        _L1_xtrack_i = 0.0f;
    }
    _last_update_waypoint_us = now;

    // Calculate L1 gain required for specified damping
    float K_L1 = 4.0f * _L1_damping * _L1_damping;
    //hal.console->printf("K1: %2f\n", K_L1);

    // Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/1/pipi
    _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);
    //hal.console->printf("L1 dist: %2f\n", _L1_dist);

    // Calculate the NE position of WP B relative to WP A
    Vector2f AB = prev_WP.get_distance_NE(next_WP);
    float AB_length = AB.length();

    // Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = _current_loc.get_distance_NE(next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
    AB.normalize();

    // Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = prev_WP.get_distance_NE(_current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = A_air % AB;

    //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference poin
    
    //Otherwise do normal L1 guidance
    float WP_A_dist = A_air.length();
    float alongTrackDist = A_air * AB;
    if (WP_A_dist > _L1_dist && alongTrackDist/MAX(WP_A_dist, 1.0f) < -0.7071f)                             //Aircraft behind first waypoint
    {
        //hal.console->println("IF n1");
        //Calc Nu to fly To WP A
        Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft
        xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else if (alongTrackDist > AB_length + groundSpeed*3) {    
        //hal.console->println("IF n2");                                                                                                    // we have passed point B by 3 seconds. Head towards B
        // Calc Nu to fly To WP B
        const Vector2f B_air = next_WP.get_distance_NE(_current_loc);
        Vector2f B_air_unit = (B_air).normalized(); // Unit vector from WP B to aircraft
        xtrackVel = _groundspeed_vector % (-B_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-B_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-B_air_unit.y , -B_air_unit.x); // bearing (radians) from AC to L1 point
    } else {                                                                                                //Calc Nu to fly along AB line

        //hal.console->println("IF n3");
        //hal.console->printf("CrossTrackError: %2f\n", _crosstrack_error);
        //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
        xtrackVel = _groundspeed_vector % AB; // Velocity cross track
        ltrackVel = _groundspeed_vector * AB; // Velocity along track
        float Nu2 = atan2f(xtrackVel,ltrackVel);
        //Calculate Nu1 angle (Angle to L1 reference point)
        float sine_Nu1 = _crosstrack_error/MAX(_L1_dist, 0.1f);
        //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
        sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);
        float Nu1 = asinf(sine_Nu1);

       
        // compute integral error component to converge to a crosstrack of zero when traveling
        // straight but reset it when disabled or if it changes. That allows for much easier
        // tuning by having it re-converge each time it changes.
        if (_L1_xtrack_i_gain <= 0 || !is_equal(_L1_xtrack_i_gain.get(), _L1_xtrack_i_gain_prev)) {
            _L1_xtrack_i = 0;
            _L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
        } else if (fabsf(Nu1) < radians(5)) {
            _L1_xtrack_i += Nu1 * _L1_xtrack_i_gain * dt;

            // an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
            _L1_xtrack_i = constrain_float(_L1_xtrack_i, -0.1f, 0.1f);
        }
       
        // to converge to zero we must push Nu1 harder
        Nu1 += _L1_xtrack_i;

        Nu = Nu1 + Nu2;
        _nav_bearing = atan2f(AB.y, AB.x) + Nu1; // bearing (radians) from AC to L1 point
    }

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    //Limit Nu to +-(pi/2)
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    // Waypoint capture status is always false during waypoint following
    _WPcircle = false;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
*/

/*
// update L1 control for loitering
void AP_L1_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
    struct Location _current_loc;

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    radius = loiter_radius(fabsf(radius));

    // Calculate guidance gains used by PD loop (used during circle tracking)
    float omega = (6.2832f / _L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    float K_L1 = 4.0f * _L1_damping * _L1_damping;
    //float K_L1 = 2.0f;

    //Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = MAX(_groundspeed_vector.length() , 1.0f);


    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(center_WP);


    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/pi
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;
    //_L1_dist = 80.0f;
    //Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = center_WP.get_distance_NE(_current_loc);

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_unit;
    if (A_air.length() > 0.1f) {
        A_air_unit = A_air.normalized();
    } else {
        if (_groundspeed_vector.length() < 0.1f) {
            A_air_unit = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            A_air_unit = _groundspeed_vector.normalized();
        }
    }

    //Calculate Nu to capture center_WP
    float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
    float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    //Calculate radial position and velocity errors
    float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
    float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle

    // keep crosstrack error for reporting
    _crosstrack_error = xtrackErrCirc;

    //hal.console->printf("CrossTrackError: : %2f\n", _crosstrack_error);
    //hal.console->printf("L1 dist: %2f\n", _L1_dist);

    //Calculate PD control correction to circle waypoint_ahrs.roll
    float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

    //Calculate tangential velocity
    float velTangent = xtrackVelCap * float(loiter_direction);

    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
        latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
    }

    // Calculate centripetal acceleration demand
    float latAccDemCircCtr = velTangent * velTangent / MAX((0.5f * radius), (radius + xtrackErrCirc));

    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
    float latAccDemCirc = loiter_direction * (latAccDemCircPD + latAccDemCircCtr);

    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle
    if (xtrackErrCirc > 0.0f && loiter_direction * latAccDemCap < loiter_direction * latAccDemCirc) {
        //hal.console->printf("IF N1\n");
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else {
        //hal.console->printf("IF N2\n");
        _latAccDem = latAccDemCirc;
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians)from AC to L1 point
    }

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
*/

// update L control for waypoint navigation - UPWIND project
void AP_L1_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{

    //float L = 30.0f;
    struct Location _current_loc;
    float Nu;
    float xtrackVel;
    float ltrackVel;

    float L_use = _L_use;

    // Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/1/pipi
    //_L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);

    // Calculate the NE position of WP B relative to WP A
    Vector2f AB = prev_WP.get_distance_NE(next_WP);
    //float AB_length = AB.length();

    // Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = _current_loc.get_distance_NE(next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
    AB.normalize();

    // Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = prev_WP.get_distance_NE(_current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = A_air % AB;
    
     if (L_use < 1)
    {
        _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);
    }
    else{
        _L1_dist = sqrt(sq(_L_dist)+sq(_crosstrack_error));
    }

    //hal.console->printf("L1 dist: %2f\n", _L1_dist);
   
   
    //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference poin
    
    //Otherwise do normal L1 guidance
    float WP_A_dist = A_air.length();
    float alongTrackDist = A_air * AB;
    
    //hal.console->printf("AlongTrackDist:: %2f\n", alongTrackDist);

    //Calc Nu to fly along AB line
    if (WP_A_dist > _L1_dist && alongTrackDist/MAX(WP_A_dist, 1.0f) < -0.7071f)                             //Aircraft behind first waypoint
    {
        //hal.console->println("Too Far\n");
        //Calc Nu to fly To WP A
        Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft
        xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else 
    {
        //hal.console->println("Normal\n");
        //hal.console->printf("CrossTrackError: %2f\n", _crosstrack_error);
        //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
        xtrackVel = _groundspeed_vector % AB; // Velocity cross track
        ltrackVel = _groundspeed_vector * AB; // Velocity along track
        float Nu2 = atan2f(xtrackVel,ltrackVel);  
        //Calculate Nu1 angle (Angle to L1 reference point)
        float sine_Nu1 = _crosstrack_error/MAX(_L1_dist, 0.1f);
        //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
        sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);
        float Nu1 = asinf(sine_Nu1);
    
        Nu = Nu1 + Nu2;
        _nav_bearing = atan2f(AB.y, AB.x) + Nu1; // bearing (radians) from AC to L1 point
    }

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    //Limit Nu to +-(pi/2)
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);

     if (L_use < 1)
    {
        //hal.console->println("L_use = 0");
        float K_L1 = 4.0f * _L1_damping * _L1_damping;
        _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
    }
    else{
        //hal.console->println("L_use = 1");
        _latAccDem = 2.0f * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);      
    }

    // Waypoint capture status is always false during waypoint following
    _WPcircle = false;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    _data_is_stale = false; // status are correctly updated with current waypoint data
}



// update L control for loitering - UPWIND project
void AP_L1_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
    struct Location _current_loc;
    //float L = 30.0f;
    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    radius = loiter_radius(fabsf(radius));
    //hal.console->printf("Radius: %2f\n", radius);

    // Calculate guidance gains used by PD loop (used during circle tracking)
    float omega = (6.2832f / _L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    //float K_L1 = 4.0f * _L1_damping * _L1_damping;

    //Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = MAX(_groundspeed_vector.length() , 1.0f);

    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(center_WP);


    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/pi
    //_L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

    //Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = center_WP.get_distance_NE(_current_loc);

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_unit;
    if (A_air.length() > 0.1f) {
        A_air_unit = A_air.normalized();
    } else {
        if (_groundspeed_vector.length() < 0.1f) {
            A_air_unit = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            A_air_unit = _groundspeed_vector.normalized();
        }
    }

    //Calculate Nu to capture center_WP
    float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
    float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    //Calculate radial position and velocity errors
    float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
    float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle
    
    // keep crosstrack error for reporting
    _crosstrack_error = A_air.length() - radius;

    _L1_dist = sqrt(sq(_L_dist)+sq(_crosstrack_error));
    //hal.console->printf("CrossTrackError: : %2f\n", _crosstrack_error);

    //Waypoint too far
    if (_L1_dist > radius)
    {
        //hal.console->printf("Waypoint too far!\n");
        _L1_dist = radius;
    }

    //hal.console->printf("L1 dist: %2f\n", _L1_dist);

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    float latAccDemCap = 2.0f * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
    
    //Calculate PD control correction to circle waypoint_ahrs.roll
    float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

    //Calculate tangential velocity
    float velTangent = xtrackVelCap * float(loiter_direction);

    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
        latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
    }

    // Calculate centripetal acceleration demand
    float latAccDemCircCtr = velTangent * velTangent / MAX((0.5f * radius), (radius + xtrackErrCirc));

    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
    float latAccDemCirc = loiter_direction * (latAccDemCircCtr + latAccDemCircPD);

    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle 

    //need to add a new IF, to when the plane is too far from the point! maybe L1 isn't correct

    if (xtrackErrCirc > 0.0f && loiter_direction * latAccDemCap < loiter_direction * latAccDemCirc) {
        //hal.console->printf("IF N1\n");
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else {
        //hal.console->printf("IF N2\n");
        _latAccDem = latAccDemCirc;
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians)from AC to L1 point
    }

    _data_is_stale = false; // status are correctly updated with current waypoint data 
}

// update L1 control for fly8 figure in PLANE
void AP_L1_Control::update_loiter_ellipse(const struct Location &center_loc, const int32_t maxradius_cm, const float minmaxratio, const float psi, const int8_t orientation, struct Location &aircraft_loc, Vector3f &aircraft_vel, struct Location &desired_loc)
{
    struct Location _current_loc;

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    // maxradius_cm *= sq(_ahrs.get_EAS2TAS());

    // Calculate guidance gains used by PD loop (used during circle tracking)
    const float omega = (6.2832f / _L1_period);
    const float Kx = omega * omega;
    const float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    const float K_L1 = 4.0f * _L1_damping * _L1_damping;
    

    // get current position and velocity in NED frame
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }
        // position of aircraft relative to the center of the ellipse; vector components in meters
    const Vector3f posav(center_loc.get_distance_NED(_current_loc));
    // store absolute aircraft position for external use
    aircraft_loc = _current_loc;

    // lateral projection
    const Vector2f posalv(posav.x, posav.y);
    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(center_loc);

    // velocity of aircraft in NED coordinate system
    Vector3f velav;
    // only use if ahrs.have_inertial_nav() is true
    if (_ahrs.get_velocity_NED(velav)) {
    }
    else {Vector2f(velav.x, velav.y)=_ahrs.groundspeed_vector();
          /* if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
                  velav = gps.velocity().z;
              } else {
                  velav = -barometer.get_climb_rate();
              }; */
          velav.z = 0;
    }

    // store aircraft velocity
    aircraft_vel = velav;

    const Vector2f velalv(velav.x,velav.y);
    const float velal = MAX(velalv.length(), 1.0f);

    // unit vector pointing from the center to the  aircraft
    Vector2f erlv;
    erlv = posalv;
    if (erlv.length() > 0.1f) {
        erlv = posalv.normalized();
    } else {
        if (velalv.length() < 0.1f) {
            erlv = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            erlv = velalv.normalized();
        }
    }

    // Calculate time varying control parameters
    
    

    
    //hal.console->printf("CrossTrackError: : %2f\n", _crosstrack_error);

    //Calculate Nu to capture center_WP
    const float xtrackVelCap = erlv % velalv; // Velocity across line - perpendicular to radial inbound to WP
    const float ltrackVelCap = - (velalv * erlv); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    

    // calculate desired position on ellipse with major and minor principal axes along unit vectors e1 and e2, respectively
    // for given position vector posalv(phia) = ra(cos(phia)e1 + cos(theta)sin(phia)e2) of the aircraft
    //
    //hal.console->println(maxradius_cm);
    const float cos_psi = cosf(radians(psi));
    const float sin_psi = sinf(radians(psi));
    // trigonometric functions of angle at which a circle has to be inclined in order to yield the ellipse
    // poselv(phi) = cos(phi)e1 + cos(theta)sin(phi)e2
    const float cos_theta = minmaxratio;
    const float sin_theta = sqrt(1.0f - sq(minmaxratio));
    // unit vectors e1 and e2 into the direction of the major and minor principal axes, respectively
    const Vector2f e1(cos_psi,sin_psi);
//    hal.console->print("e1: ");
//    hal.console->print(e1.x);
//    hal.console->print(", ");
//    hal.console->println(e1.y);

    const Vector2f e2(-e1.y,e1.x);
    //hal.console->print(e1.x);
    //hal.console->print(" ");
    //hal.console->println(e1.y);
    // projections of the aircraft's position onto e1 and e2
    const float posal1 = posalv * e1;
    const float posal2 = posalv * e2;
    // determine parametrization (ra,phia) of the aircraft's position from lateral components
    // posal1 = ra cos(phia)
    // posal2 = ra cos(theta)sin(phia);
    // projections of the aircraft's velocity onto e1 and e2
    
    //const float velal1 = velalv * e1;
    //const float velal2 = velalv * e2;

    // distance of the aircraft from the curve;
    float dae;
    // unit tangent vector
    Vector2f etelv;
    // unit outer normal vector at point of the ellipse closest to the aircraft's position relative to center_loc
    Vector2f enelv;
    // curvature at point of the ellipse closest to the aircraft's position relative to center_loc
    float kappa;

    //hal.console->print("cos_theta: ");
    // hal.console->print(cos_theta);
    if (!is_zero(cos_theta)){
      // non-degenerate ellipse
      // distance of the aircraft from the center of the ellipse in meter
      const float ra = sqrt(sq(posal1) + sq(1/cos_theta * posal2));
//      hal.console->print("ra: ");
//      hal.console->print(ra);
//           hal.console->println("cos_theta: ");
//           hal.console->println(cos_theta);
//           hal.console->println(S1ctoalv.x);
//           hal.console->println(S1ctoalv.y);
//           hal.console->println(e1.x);
//           hal.console->println(e1.y);
       const float rho = ra - maxradius_cm/100.0f;
       // trigononometric functions of curve parameter phia at the aircraft's position
       const float cos_phia = posal1/ra;
       const float sin_phia = orientation * posal2/(ra * cos_theta);
       // first oder correction to curve parameter to approximate parameter at point of the ellipse closest to the aircraft's position
       const float dphi = - rho * sq(sin_theta) * sin_phia * cos_phia /(ra * (1 - sq(sin_theta * cos_phia)));
       const float cos_dphi = cosf(dphi);
       const float sin_dphi = sinf(dphi);
       // trigonometric functions of phi = phia + dphi, which is the first-order value of the curve parameter of the ellipse at the nearest point of the aircraft
       const float cos_phiapdphi = cos_phia * cos_dphi - sin_phia * sin_dphi;
       const float sin_phiapdphi = cos_phia * sin_dphi + sin_phia * cos_dphi;
       //hal.console->print(cos_phiapdphi);
       //hal.console->print(" ");
       //hal.console->println(sin_phiapdphi);
       // distance of the aircraft from the ellipse;
       dae = rho * cos_theta /sqrt(1 - sq(sin_theta * cos_phia));
       // position vector of point of the ellipse closest to the aircraft's position relative to center_loc
       // const Vector2f poselv = Vector2f((e1 * cos_phiapdphi + e2 * cos_theta * sin_phiapdphi * orientation) * maxradius_cm/100.0f);
       // projections onto e1 and e2
       //const float poselv1 = poselv * e1;
       //const float poselv2 = poselv * e2;
       const Vector2f telv = Vector2f(-e1 * sin_phiapdphi + e2 * cos_theta * cos_phiapdphi * orientation);
       const float telvnorm = telv.length();
       // unit tangent vector at point of the ellipse closest to the aircraft's position relative to center_loc
       etelv = telv / telvnorm;
       // unit outer normal vector at point of the ellipse closest to the aircraft's position relative to center_loc
       enelv(etelv.y * orientation, -etelv.x * orientation);
       // curvature at point of the ellipse closest to the aircraft's position relative to center_loc
       kappa = cos_theta /(ra * powf(telvnorm,3));
       //hal.console->print(cos_theta);
       //hal.console->print(" ");
       //hal.console->println(norm);

       //Calculate radial position and velocity errors
       const float xtrackVelCirc = enelv * velalv ; // normal outbound velocity
       const float xtrackErrCirc = dae; // Radial distance from the loiter circle
//       hal.console->print(xtrackVelCirc);
//       hal.console->print(", ");
//       hal.console->println(xtrackErrCirc);

       //hal.console->print("should be zero: ");
       //hal.console->println((posalv-poselv) * enelv - dae);
       const float ltrackVelCirc = etelv * velalv; // tangential velocity in the tangential direction (depends on orientation)

       // keep crosstrack error for reporting
       _crosstrack_error = xtrackErrCirc;

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    //Calculate the L1 length required for specified period
    //0.3183099 = 1/pi
    float L_use = _L_use;
    float latAccDemCap;

    if (L_use < 1)
    {
        //hal.console->println("L_use = 0");
        _L1_dist = 0.3183099f * _L1_damping * _L1_period * velal;
        latAccDemCap = K_L1 * velal * velal / _L1_dist * sinf(Nu);
    }
    else{
        //hal.console->println("L_use = 1");
        _L1_dist = sqrt(sq(_L_dist)+sq(_crosstrack_error));
        latAccDemCap = 2.0f * velal * velal / _L1_dist * sinf(Nu);      
    }
    
    //hal.console->printf("CrossTrackError: : %2f\n", _crosstrack_error);
       //Calculate PD control correction to circle waypoint_ahrs.roll
       float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

       //Calculate tangential velocity
       float velTangent = ltrackVelCirc; // * float(orientation);

//    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
//    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
//        latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
//    }

       // Calculate centripetal acceleration demand
       float latAccDemCircCtr = velTangent * velTangent * kappa;
       //hal.console->print(velTangent);
       //hal.console->print("kappa: ");
       //hal.console->println(1/kappa);

       //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
       float latAccDemCirc = orientation * (latAccDemCircPD + latAccDemCircCtr);
//       hal.console->println(latAccDemCircPD);

       // Perform switchover between 'capture' and 'circle' modes at the
       // point where the commands cross over to achieve a seamless transfer
       // Only fly 'capture' mode if outside the circle
       if ((xtrackErrCirc > maxradius_cm * (1-minmaxratio)/100.0f && orientation * latAccDemCap < orientation * latAccDemCirc)){ //|| posalv.length()>= maxradius_cm/100.0f) {
           _latAccDem = latAccDemCap;
           _WPcircle = false;
           _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
           _nav_bearing = atan2f(-posalv.y , -posalv.x); // bearing (radians) from AC to L1 point
           //hal.console->println("capture");
       } else {
           _latAccDem = latAccDemCirc;
           _WPcircle = true;
           _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
           _nav_bearing = atan2f(-posalv.y , -posalv.x); // bearing (radians)from AC to L1 point
        //hal.console->println("loiter_ellipse");
       }

       _data_is_stale = false; // status are correctly updated with current waypoint data

    } else {
        //hal.console->println("degenerate ellipse: ");
        // if cos_theta == 0, the ellipse is degenerate; its lateral projection is a straight line spanned by e1

        Vector2f deltav = e1 * maxradius_cm / 100.0f * 2.0 * orientation;
        struct Location start = center_loc;
        start.offset(-deltav.x, -deltav.y);
        struct Location end = center_loc;
        end.offset(deltav.x, deltav.y);

//        hal.console->print("deltav: ");
//        hal.console->print(deltav.x);
//        hal.console->print(", ");
//        hal.console->println(deltav.y);


        update_waypoint(start, end);
//        hal.console->print("_latAccDem: ");
//        hal.console->println(_latAccDem);


    }

    // current location of the aircraft
}

// update L1 control for fly8 figure
void AP_L1_Control::update_loiter_3d(const struct Location &S2center, const Vector3f &ercv, int32_t S2radius, const float &theta_r, int8_t orientation, struct Location &aircraft_loc, Vector3f &aircraft_vel, struct Location &desired_loc)
{
// CALCULATE DERIVED PARAMETERS FROM ARGUMENT LIST OF THE FUNCTION

    const float cos_thetar = cosf(radians(theta_r));
    const float sin_thetar = sinf(radians(theta_r));
    // distance between the center of the sphere and the center of the circle
    const int32_t D = S2radius * cos_thetar;
    // radius of the circle
    const int32_t S1radius = S2radius * sin_thetar;
    //const Vector3f S2ctoS1cv = ercv * D/100.0f;
    // Location of the center of the circle
    struct Location S1center = S2center;
    S1center.offset(ercv.x * D/100.0f, ercv.y * D/100.0f);
    
    S1center.alt = S1center.alt - ercv.z * D;
     // trigonometric functions of the polar angle
     const float cos_theta = -ercv.z;
     const float sin_theta = sqrt(1.0f - sq(cos_theta));
      // calculate desired position on ellipse (= lateral projection of the circle) with major and minor principal axes along unit vectors e1 and e2, respectively
      // position vector of the aircraft parameterized as: posalv(phia) = ra(cos(phia)e1 + cos(theta)sin(phia)e2)
      // trigonometric functions of the azimuth angle
      float cos_psi;
      float sin_psi;
      // if theta == 0, i.e. the circle is horizontal, choose psi = 0
      if (is_zero(sin_theta)){
          cos_psi = 1;
          sin_psi = 0;
      } else {
          cos_psi = ercv.x / sin_theta;
          sin_psi = ercv.y / sin_theta;
      }
      // trigonometric functions of angle at which a circle has to be inclined in order to yield the ellipse as its lateral projection
      // unit vectors e1 and e2 into the direction of the major and minor principal axes, respectively
      //const Vector2f e1(cos_psi,sin_psi);
      //const Vector2f e2(-e1.y,e1.x);
      // minor and major principal axes directions have to be exchanged for the inclined circle
      // this can be accomplished by a 90 degree rotation: e1 -> e2, e2 -> -e1 which preserves the orientation
      const Vector2f e1(-sin_psi,cos_psi); //unit vector pointing along the major principal axis; directed towards east for psi = 0
      const Vector2f e2(-e1.y,e1.x);//unit vector pointing along the minor principal axis; directed towards south for psi = 0

      // minimal height for flying unconstrained outside the sphere
      int32_t heightmin_cm = 2000;
      // radius (half distance) between the two points of the segment lying above the minimal height
      int32_t segradius_cm = sqrt(sq(S1radius)-sq(heightmin_cm));
      // vector is pointing in the direction of motion from start_loc to end_loc on the upper hemicircle (for inclination theta >0) given by orientation
      Vector2f maxv = - e1 * segradius_cm / 100.0f * orientation;
      struct Location start_loc = S1center;
      
      start_loc.offset(-maxv.x, -maxv.y);
      start_loc.alt = start_loc.alt + heightmin_cm;
      struct Location end_loc = S1center;
      end_loc.offset(maxv.x, maxv.y);

     /* 
      end_loc.alt = end_loc.alt + heightmin_cm;
      hal.console->print("start_loc: ");
      hal.console->print(start_loc.lat);
      hal.console->print(", ");
      hal.console->print(start_loc.lng);
      hal.console->print(", ");
      hal.console->println(start_loc.alt);
      hal.console->print("end_loc: ");
      hal.console->print(end_loc.lat);
      hal.console->print(", ");
      hal.console->print(end_loc.lng);
      hal.console->print(", ");
      hal.console->println(end_loc.alt);
    */

// GET CURRENT POSITON AND VELOCITY
     // current location of the aircraft
     struct Location _current_loc;

     // get current position in NED coordinate system
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }
    // store aircraft's position for external use
    aircraft_loc = _current_loc;
    // aircraft's position vector (direction of ideal (straight) tether) from the center of the sphere
    /*Vector3f S2ctoav = S2center.get_distance_NED(_current_loc);
    hal.console->print("S2ctoav: ");
    hal.console->print(S2ctoav.x);
    hal.console->print(", ");
    hal.console->print(S2ctoav.y);
    hal.console->print(", ");
    hal.console->println(S2ctoav.z);
    */

    // aircraft's position vector from the center of the circle
    Vector3f S1ctoav = S1center.get_distance_NED(_current_loc);
    // lateral projection of the aircraft's relative position vector
    Vector2f S1ctoalv(S1ctoav.x, S1ctoav.y);
    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(S1center);

    // track velocity in NED coordinate system
    Vector3f velav;
    // only use if ahrs.have_inertial_nav() is true
    if (_ahrs.get_velocity_NED(velav)) {
    }
    else {Vector2f(velav.x, velav.y) =_ahrs.groundspeed_vector();
          /* if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
                  velav = gps.velocity().z;
              } else {
                  velav = -barometer.get_climb_rate();
              }; */
          velav.z = 0;
    }
    // store aircraft velocity for external use
    aircraft_vel = velav;
    // lateral projection of the aircraft's velocity vector
    Vector2f velalv(velav.x,velav.y);
    // lateral velocity; protect against becoming zero
    float velal = MAX(velalv.length(), 1.0f);


    // unit tangent vector at point on a circle which is closest to the aircraft; lies in the plane containing the circle
     Vector3f etv = (S1ctoav % ercv) * orientation;
     etv = etv.normalized();
     // lateral projection of the unit tangent vector
     Vector2f etlv(etv.x, etv.y);
     etlv = etlv.normalized();

     // outer unit normal (radial) vector at point on circle that is closest to the aircraft
     Vector3f env = (ercv % etv) * orientation;
     env = env.normalized(); // renormalize in order to compensate for numerical inaccuracies
     // lateral renormalized projection of the unit normal vector; this is the radial vector of the point of the ellipse (the lateral projection of the circle) but NOT its normal vector;
     Vector2f erlv(env.x, env.y);
     if (erlv.length() > 0.1f) {
         erlv = erlv.normalized();
     } else {
         if (velalv.length() < 0.1f) {
             erlv = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
         } else {
             erlv = velalv.normalized();
         }
     }


    // Calculate guidance gains used by PD loop (used during circle tracking)
      const float omega = (6.2832f / _L1_period);
      const float Kx = omega * omega;
      const float Kv = 2.0f * _L1_damping * omega;
      // Calculate L1 gain required for specified damping (used during waypoint capture)
      const float K_L1 = 4.0f * _L1_damping * _L1_damping;


// NAVIGATION ON THE LATERAL PROJECTION OF THE INCLINED CICRLE ((DEGENERATE) ELLIPSE)


      // projections of the aircraft's position onto e1 and e2
      const float posal1 = S1ctoalv * e1;
      const float posal2 = S1ctoalv * e2;
      // determine parametrization (ra,phia) of the aircraft's position from lateral components
      // posal1 = ra cos(phia)
      // posal2 = ra cos(theta)sin(phia);

      // distance of the aircraft from the curve;
      float dae;
      // unit tangent vector
      Vector2f etelv;
      // unit outer normal vector at point of the ellipse closest to the aircraft's position relative to center_loc
      Vector2f enelv;
      // curvature at point of the ellipse closest to the aircraft's position relative to center_loc
      float kappa;

      //hal.console->print("cos_theta: ");
      // hal.console->print(cos_theta);
      if (!is_zero(cos_theta)){
          // non-degenerate ellipse
          // distance of the aircraft from the center of the ellipse in meter
          const float ra = sqrt(sq(posal1) + sq(1/cos_theta * posal2));
//           hal.console->println("cos_theta: ");
//           hal.console->println(cos_theta);
//           hal.console->println(S1ctoalv.x);
//           hal.console->println(S1ctoalv.y);
//           hal.console->println(e1.x);
//           hal.console->println(e1.y);
          const float rho = ra - S1radius/100.0f;
          // trigonometric functions of curve parameter phia at the aircraft's position
          const float cos_phia = posal1/ra;
          const float sin_phia = orientation * posal2/(ra * cos_theta);
          // first oder correction to curve parameter to approximate parameter at point of the ellipse closest to the aircraft's position
          const float dphi = - rho * sq(sin_theta) * sin_phia * cos_phia /(ra * (1 - sq(sin_theta * cos_phia)));
          const float cos_dphi = cosf(dphi);
          const float sin_dphi = sinf(dphi);
          // trigonometric functions of phi = phia + dphi, which is the first-order value of the curve parameter of the ellipse at the nearest point of the aircraft
          const float cos_phiapdphi = cos_phia * cos_dphi - sin_phia * sin_dphi;
          const float sin_phiapdphi = cos_phia * sin_dphi + sin_phia * cos_dphi;
          //hal.console->print(cos_phiapdphi);
          //hal.console->print(" ");
          //hal.console->println(sin_phiapdphi);
          // distance of the aircraft from the ellipse;
          dae = rho * cos_theta /sqrt(1 - sq(sin_theta * cos_phia));
          // position vector of point of the ellipse closest to the aircraft's position relative to center_loc
          // const Vector2f S1ctoelv = Vector2f((e1 * cos_phiapdphi + e2 * cos_theta * sin_phiapdphi * orientation) * S1radius/100.0f);
          // projections onto e1 and e2
          // const float S1ctoelv1 = S1ctoelv * e1;
          // const float S1ctoelv2 = S1ctoelv * e2;
          //hal.console->print(posel1);
          //hal.console->print(" ");
          //hal.console->println(posel2);
          const Vector2f telv = Vector2f(-e1 * sin_phiapdphi + e2 * cos_theta * cos_phiapdphi * orientation);
          const float telvnorm = telv.length();
          // unit tangent vector at point of the ellipse closest to the aircraft's position relative to center_loc
          etelv = telv / telvnorm;
          // unit outer normal vector at point of the ellipse closest to the aircraft's position relative to center_loc
          enelv = Vector2f(etelv.y * orientation, -etelv.x * orientation);
          // curvature at point of the ellipse closest to the aircraft's position relative to center_loc
          kappa = cos_theta /(ra * powf(telvnorm,3));

          // velocities and accelerations for capturing the center

          // crosstrack lateral velocity: velocity component of the aircraft along tangent vector  <=> velocity perpendicular to the path heading to the target point
          float xtrackVelCap = velalv * etlv * orientation;
          // along the track lateral velocity: velocity component of the aircraft radial inbound towards the target point
          float ltrackVelCap = - velalv * erlv;


          float Nu = atan2f(xtrackVelCap,ltrackVelCap);
          _prevent_indecision(Nu);
          _last_Nu = Nu;
          Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2
          //Calculate lateral acceleration demand to capture center_WP (use L1 guidance law)
          float latAccDemCap = K_L1 * velal * velal / _L1_dist * sinf(Nu);


          // deviations of _current_loc from desired point
          // difference of the lengths of the direct lateral projections
          //float xtrackErrCirc = S2ctoalv.length() - S2ctocirclelv.length();
          // deviation of the position of the aircraft from the ellipse
          float xtrackErrCirc = dae;
          // keep crosstrack error for reporting
          _crosstrack_error = xtrackErrCirc;

          // velocities and accelerations for circulating around the center

          //Vector2f veldeviationlv(velav.x,velav.y);
          // lateral velocity component in the direction of the outer normal vector
          float xtrackVelCirc = velalv * enelv;
          // lateral velocity component in the tangential direction of the ellipse
          float ltrackVelCirc = velalv * etelv;

          // calculate lateral acceleration for following the ellipse (the lateral projection of the circle)
          float latAccDemCircCtr = ltrackVelCirc * ltrackVelCirc * kappa;

          // calculate PD control correction to lateral acceleration
          // flight path outside desired circle -> positive correction
          // velocity pointing outwards -> positive correction
          float latAccDemCircPD = xtrackErrCirc * Kx + xtrackVelCirc * Kv;

          //Calculate tangential velocity
          float velTangent = xtrackVelCap * float(orientation);

          //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
          if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
              latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
          }

          float tetherErr = 0;
          // sign of lateral acceleration corresponds to sign of roll angle
          // roll angle and hence acceleration is negative if orientation is positive
          float latAccDemCirc = orientation * (latAccDemCircPD + latAccDemCircCtr + tetherErr);

          // Perform switchover between 'capture' and 'circle' modes at the
          // point where the commands cross over to achieve a seamless transfer
          // Only fly 'capture' mode if outside the circle
          if (xtrackErrCirc > 0.0f && orientation * latAccDemCap < orientation * latAccDemCirc) {
              // capture mode
              // hal.console->println("non-degenerate case: capture");
              _latAccDem = latAccDemCap;
              _WPcircle = false;
              _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
              _nav_bearing = 0;//atan2f(-erlv.y , -erlv.x); // bearing (radians) from AC to L1 point
              // desired target: location of point closest to the inclined circle
              desired_loc = S1center;
              desired_loc.offset(env.x * S1radius / 100.0f, env.y * S1radius / 100.0f);
              desired_loc.alt = S1center.alt - env.z * S1radius;
              //hal.console->print("desired_loc.alt: ");
              //hal.console->println(desired_loc.alt);
          } else {
              // loiter
              // hal.console->println("loiter");
              _latAccDem = latAccDemCirc;
              _WPcircle = true;
              _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
              _nav_bearing = atan2f(-erlv.y , -erlv.x); // bearing (radians)from AC to L1 point
              // desired target: point closest to the inclined circle
              desired_loc = S1center;
              desired_loc.offset(env.x * S1radius / 100.0f, env.y * S1radius / 100.0f);
              desired_loc.alt = S1center.alt - env.z * S1radius;


              //hal.console->println(desired_loc.alt - S2center.alt);
              //hal.console->print(" ");
              //hal.console->println(S1center.alt-S2center.alt);

              // hal.console->print("height demanded: ");
              // hal.console->println(height);
          }
} else {
          //hal.console->println("degenerate case");
          // ellipse is degenerate to a line with unit tangent vector -e1, normal vector e2 running between start_loc and end_loc
          // aircraft should move along -e1 * orientation, i.e. in accord with the orientation of the upper (for theta = 90) hemicircle
          // e2, -e1, e_down form a right-handed coordinate system: looking downwards onto the (e1,e2)-plane in the -e1 direction, e2 points to the left
          // distance of the aircraft from the line
          // a deviation to the left requires a positive roll angle correction and hence positive acceleration
          // => deviation dae from path should be projection of the relative position onto e2 and should have sign given by orientation
          dae = posal2;
          etelv = -e1;
          enelv = e2;
//          hal.console->print("e1: ");
//          hal.console->print(e1.x);
//          hal.console->print(", ");
//          hal.console->println(e1.y);
//          hal.console->print("e2: ");
//          hal.console->print(e2.x);
//          hal.console->print(", ");
//          hal.console->println(e2.y);

          // determine point on the line closest to the aircraft
          // posal1 is projection of the aircraft's position onto the line: abs(posal1) > segradius_cm: projection lies outside segment
          //                                                          and   sgn(posal1) = +/-1: projection behind start point / in front of finish point
          struct Location capture_loc;
          // hal.console->print(posal1);
          // hal.console->print(", ");
          // hal.console->println(segradius_cm/100.0f);
          if (abs(posal1) >= segradius_cm/100.0f){
              if (posal1 >= 0) {
                  capture_loc = start_loc;
//                  hal.console->print("capture_loc = start_loc: ");
//                  hal.console->print(capture_loc.lat);
//                  hal.console->print(", ");
//                  hal.console->print(capture_loc.lng);
//                  hal.console->print(", ");
//                  hal.console->println(capture_loc.alt);
              } else {
                  capture_loc = end_loc;
//                  hal.console->print("capture_loc = end_loc: ");
//                  hal.console->print(capture_loc.lat);
//                  hal.console->print(", ");
//                  hal.console->print(capture_loc.lng);
//                  hal.console->print(", ");
//                  hal.console->println(capture_loc.alt);
              }
          } else {
              // int32_t heightsq = sq(S1radius)-sq(posal1* 100.0f);
              capture_loc = S1center;
              capture_loc.offset(e1.x * posal1, e1.y * posal1);
              capture_loc.alt = capture_loc.alt + sqrt(sq(S1radius)-sq(posal1* 100.0f));
//              hal.console->print("capture_loc = S1center + (");
//              hal.console->print(e1.x * posal1);
//              hal.console->print(", ");
//              hal.console->print(e1.y * posal1);
//              hal.console->print(", ");
//              hal.console->print(sqrt(sq(S1radius)-sq(posal1* 100.0f)));
//              hal.console->println(")");
//              hal.console->print(capture_loc.lat);
//              hal.console->print(", ");
//              hal.console->print(capture_loc.lng);
//              hal.console->print(", ");
//              hal.console->println(capture_loc.alt);
//              hal.console->print("capture_loc generic: ");
//              hal.console->print(e1.x * posal1);
//              hal.console->print(", ");
//              hal.console->println(e1.y * posal1);
//              hal.console->print("S1radius, posal1: ");
//              hal.console->print(S1radius);
//              hal.console->print(", ");
//              hal.console->println(posal1 * 100.0f);
          }






          // redefine radial vector in case of the degenerate ellipse as the vector on the line segment closest to the aircraft
          Vector2f rlv = capture_loc.get_distance_NE(_current_loc);
          erlv = rlv.normalized();
          // redefine tangential vector for approach to the target point (erlv, etlv, e_down) should form a right-handed system
          etlv = Vector2f(- erlv.y, erlv.x);

          // curvature of the straight line
          kappa = 0;

          // VELOCITIES AND ACCELERATIONS FOR CAPTURING THE CENTRAL POINT

          // crosstrack lateral velocity: velocity component of the aircraft along tangent vector  <=> velocity perpendicular to the path heading to the target point
          //  deviation should be positive if the inbound aircraft deviates towards left from the straight path to the target
          // (erlv, etlv) should form a right-handed system
          float xtrackVelCap = velalv * etlv * orientation;
          // along the track lateral velocity: velocity component of the aircraft radial inbound towards the target point
          float ltrackVelCap = - velalv * erlv;


          float Nu = atan2f(xtrackVelCap,ltrackVelCap);
          _prevent_indecision(Nu);
          _last_Nu = Nu;
          Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2
          //Calculate lateral acceleration demand to capture center_WP (use L1 guidance law)
          float latAccDemCap = K_L1 * velal * velal / _L1_dist * sinf(Nu);


          // deviation of the position of the aircraft from the ellipse
          float xtrackErrCirc = dae;
          // keep crosstrack error for reporting
          _crosstrack_error = xtrackErrCirc;



          // update _target_bearing_cd
          _target_bearing_cd = _current_loc.get_bearing_to(end_loc);


          //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
//                         xtrackVel = velalv % e1; // Velocity cross track
          float xtrackVelCirc = velalv * enelv;
//                         ltrackVel = velalv * e1;
          float ltrackVelCirc = velalv * etelv;
          float Nu2 = atan2f(xtrackVelCirc,ltrackVelCirc);
          //Calculate Nu1 angle (Angle to L1 reference point)
          float sine_Nu1 = _crosstrack_error / MAX(_L1_dist, 0.1f);
          //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
          sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);
          float Nu1 = asinf(sine_Nu1);
          float NuCirc = Nu1 + Nu2;
          _nav_bearing = atan2f(e1.y, e1.x) + Nu1; // bearing (radians) from AC to L1 point


          _prevent_indecision(NuCirc);
          _last_Nu = NuCirc;

          //Limit Nu to +-pi
          NuCirc = constrain_float(NuCirc, -1.5708f, +1.5708f);
          float latAccDemCirc = K_L1 * velal * velal / _L1_dist * sinf(NuCirc);

          // Waypoint capture status is always false during waypoint following
          _WPcircle = false;

          _bearing_error = NuCirc; // bearing error angle (radians), +ve to left of track


      // Perform switchover between 'capture' and 'circle' modes at the
      // point where the commands cross over to achieve a seamless transfer
      // Only fly 'capture' mode if outside the circle
          if (xtrackErrCirc > 0.0f && orientation * latAccDemCap < orientation * latAccDemCirc) {
              // capture mode
              // hal.console->println("degenerate case: capture");
              _latAccDem = latAccDemCap;
              _WPcircle = false;
              _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
              _nav_bearing = 0;//atan2f(-erlv.y , -erlv.x); // bearing (radians) from AC to L1 point
              // desired target: location of point closest to the inclined circle
              desired_loc = S1center;
              desired_loc.offset(env.x * S1radius / 100.0f, env.y * S1radius / 100.0f);
              desired_loc.alt = S1center.alt - env.z * S1radius;
              //hal.console->print("desired_loc.alt: ");
              //hal.console->println(desired_loc.alt);
          } else {
              // loiter
              // hal.console->println("update_waypoint");
             _latAccDem = latAccDemCirc;
             _WPcircle = true;
             _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
             _nav_bearing = atan2f(-erlv.y , -erlv.x); // bearing (radians)from AC to L1 point
              // desired target: point closest to the inclined circle
            desired_loc = capture_loc;
//            location_offset(desired_loc, env.x * S1radius / 100.0f, env.y * S1radius / 100.0f);
//            desired_loc.alt = S1center.alt - env.z * S1radius;
//            //hal.console->println(desired_loc.alt - S2center.alt);
              //hal.console->print(" ");
              //hal.console->println(S1center.alt-S2center.alt);

              // hal.console->print("height demanded: ");
              // hal.console->println(height);
          }
//          desired_loc = capture_loc;
//          hal.console->print("start_loc: ");
//           hal.console->print(start_loc.lat);
//           hal.console->print(", ");
//           hal.console->print(start_loc.lng);
//           hal.console->print(", ");
//           hal.console->println(start_loc.alt);
//           hal.console->print("desired_loc: ");
//          hal.console->print(desired_loc.lat);
//          hal.console->print(", ");
//          hal.console->print(desired_loc.lng);
//          hal.console->print(", ");
//          hal.console->println(desired_loc.alt);
//          hal.console->print("end_loc: ");
//          hal.console->print(end_loc.lat);
//          hal.console->print(", ");
//          hal.console->print(end_loc.lng);
//          hal.console->print(", ");
//          hal.console->println(end_loc.alt);
//          hal.console->print(location_diff(start_loc, end_loc).x);
//          hal.console->print(location_diff(start_loc, end_loc).y);
//
//
//
//          update_waypoint(start_loc, end_loc);
      }

}

// update L1 control for heading hold navigation
void AP_L1_Control::update_heading_hold(int32_t navigation_heading_cd)
{
    // Calculate normalised frequency for tracking loop
    const float omegaA = 4.4428f/_L1_period; // sqrt(2)*pi/period
    // Calculate additional damping gain

    int32_t Nu_cd;
    float Nu;

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = wrap_180_cd(navigation_heading_cd);
    _nav_bearing = radians(navigation_heading_cd * 0.01f);

    Nu_cd = _target_bearing_cd - wrap_180_cd(_ahrs.yaw_sensor);
    Nu_cd = wrap_180_cd(Nu_cd);
    Nu = radians(Nu_cd * 0.01f);

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();

    // Calculate time varying control parameters
    _L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
    float VomegaA = groundSpeed * omegaA;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _crosstrack_error = 0;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    // Limit Nu to +-pi
    Nu = constrain_float(Nu, -M_PI_2, M_PI_2);
    _latAccDem = 2.0f*sinf(Nu)*VomegaA;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for level flight on current heading
void AP_L1_Control::update_level_flight(void)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = _ahrs.yaw_sensor;
    _nav_bearing = _ahrs.yaw;
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
