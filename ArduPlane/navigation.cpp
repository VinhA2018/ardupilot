#include "Plane.h"

// set the nav_controller pointer to the right controller
void Plane::set_nav_controller(void)
{
    switch ((AP_Navigation::ControllerType)g.nav_controller.get()) {

    default:
    case AP_Navigation::CONTROLLER_DEFAULT:
        // no break, fall through to L1 as default controller

    case AP_Navigation::CONTROLLER_L1:
        nav_controller = &L1_controller;
        break;
    }
}

/*
  reset the total loiter angle
 */
void Plane::loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
    loiter.reached_target_alt = false;
    loiter.unable_to_acheive_target_alt = false;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
void Plane::loiter_angle_update(void)
{
    static const int32_t lap_check_interval_cd = 3*36000;

    const int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;

    if (loiter.sum_cd == 0 && !reached_loiter_target()) {
        // we don't start summing until we are doing the real loiter
        loiter_delta_cd = 0;
    } else if (loiter.sum_cd == 0) {
        // use 1 cd for initial delta
        loiter_delta_cd = 1;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd = lap_check_interval_cd;
    } else {
        loiter_delta_cd = target_bearing_cd - loiter.old_target_bearing_cd;
    }

    loiter.old_target_bearing_cd = target_bearing_cd;
    loiter_delta_cd = wrap_180_cd(loiter_delta_cd);
    loiter.sum_cd += loiter_delta_cd * loiter.direction;

    if (labs(current_loc.alt - next_WP_loc.alt) < 500) {
        loiter.reached_target_alt = true;
        loiter.unable_to_acheive_target_alt = false;
        loiter.next_sum_lap_cd = loiter.sum_cd + lap_check_interval_cd;

    } else if (!loiter.reached_target_alt && labs(loiter.sum_cd) >= loiter.next_sum_lap_cd) {
        // check every few laps for scenario where up/downdrafts inhibit you from loitering up/down for too long
        loiter.unable_to_acheive_target_alt = labs(current_loc.alt - loiter.start_lap_alt_cm) < 500;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd += lap_check_interval_cd;
    }
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void Plane::navigate()
{
    // allow change of nav controller mid-flight
    set_nav_controller();

    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    SpdHgt_Controller->set_path_proportion(auto_state.wp_proportion);

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    update_navigation();
}

void Plane::calc_airspeed_errors()
{
    float airspeed_measured = 0;
    
    // we use the airspeed estimate function not direct sensor as TECS
    // may be using synthetic airspeed
    ahrs.airspeed_estimate(&airspeed_measured);

    // FBW_B/cruise airspeed target
    if (!failsafe.rc_failsafe && (control_mode == &mode_fbwb || control_mode == &mode_cruise)) {
        if (g2.flight_options & FlightOptions::CRUISE_TRIM_AIRSPEED) {
            target_airspeed_cm = aparm.airspeed_cruise_cm;
        } else if (g2.flight_options & FlightOptions::CRUISE_TRIM_THROTTLE) {
            float control_min = 0.0f;
            float control_mid = 0.0f;
            const float control_max = channel_throttle->get_range();
            const float control_in = get_throttle_input();
            switch (channel_throttle->get_type()) {
                case RC_Channel::RC_CHANNEL_TYPE_ANGLE:
                    control_min = -control_max;
                    break;
                case RC_Channel::RC_CHANNEL_TYPE_RANGE:
                    control_mid = channel_throttle->get_control_mid();
                    break;
            }
            if (control_in <= control_mid) {
                target_airspeed_cm = linear_interpolate(aparm.airspeed_min * 100, aparm.airspeed_cruise_cm,
                                                        control_in,
                                                        control_min, control_mid);
            } else {
                target_airspeed_cm = linear_interpolate(aparm.airspeed_cruise_cm, aparm.airspeed_max * 100,
                                                        control_in,
                                                        control_mid, control_max);
            }
        } else {
            target_airspeed_cm = ((int32_t)(aparm.airspeed_max - aparm.airspeed_min) *
                                  get_throttle_input()) + ((int32_t)aparm.airspeed_min * 100);
        }

    } else if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        // Landing airspeed target
        target_airspeed_cm = landing.get_target_airspeed_cm();
    } else if ((control_mode == &mode_auto) &&
               (quadplane.options & QuadPlane::OPTION_MISSION_LAND_FW_APPROACH) &&
							 ((vtol_approach_s.approach_stage == Landing_ApproachStage::APPROACH_LINE) ||
							  (vtol_approach_s.approach_stage == Landing_ApproachStage::VTOL_LANDING))) {
        float land_airspeed = SpdHgt_Controller->get_land_airspeed();
        if (is_positive(land_airspeed)) {
            target_airspeed_cm = SpdHgt_Controller->get_land_airspeed() * 100;
        } else {
            // fallover to normal airspeed
            target_airspeed_cm = aparm.airspeed_cruise_cm;
        }
    } else {
        // Normal airspeed target
        target_airspeed_cm = aparm.airspeed_cruise_cm;
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (auto_throttle_mode &&
    	aparm.min_gndspeed_cm > 0 &&
    	control_mode != &mode_circle) {
        int32_t min_gnd_target_airspeed = airspeed_measured*100 + groundspeed_undershoot;
        if (min_gnd_target_airspeed > target_airspeed_cm) {
            target_airspeed_cm = min_gnd_target_airspeed;
        }
    }

    // Bump up the target airspeed based on throttle nudging
    if (throttle_allows_nudging && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    // Apply airspeed limit
    if (target_airspeed_cm > (aparm.airspeed_max * 100))
        target_airspeed_cm = (aparm.airspeed_max * 100);

    // use the TECS view of the target airspeed for reporting, to take
    // account of the landing speed
    airspeed_error = SpdHgt_Controller->get_target_airspeed() - airspeed_measured;
}

void Plane::calc_gndspeed_undershoot()
{
    // Use the component of ground speed in the forward direction
    // This prevents flyaway if wind takes plane backwards
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
	      Vector2f gndVel = ahrs.groundspeed_vector();
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
        if (!yawVect.is_zero()) {
            yawVect.normalize();
            float gndSpdFwd = yawVect * gndVel;
            groundspeed_undershoot = (aparm.min_gndspeed_cm > 0) ? (aparm.min_gndspeed_cm - gndSpdFwd*100) : 0;
        }
    } else {
        groundspeed_undershoot = 0;
    }
}

void Plane::update_loiter(uint16_t radius)
{
    if (radius <= 1) {
        // if radius is <=1 then use the general loiter radius. if it's small, use default
        radius = (abs(aparm.loiter_radius) <= 1) ? LOITER_RADIUS_DEFAULT : abs(aparm.loiter_radius);
        if (next_WP_loc.loiter_ccw == 1) {
            loiter.direction = -1;
        } else {
            loiter.direction = (aparm.loiter_radius < 0) ? -1 : 1;
        }
    }

    if (loiter.start_time_ms != 0 &&
        quadplane.guided_mode_enabled()) {
        if (!auto_state.vtol_loiter) {
            auto_state.vtol_loiter = true;
            // reset loiter start time, so we don't consider the point
            // reached till we get much closer
            loiter.start_time_ms = 0;
            quadplane.guided_start();
        }
    } else if ((loiter.start_time_ms == 0 &&
                (control_mode == &mode_auto || control_mode == &mode_guided) &&
                auto_state.crosstrack &&
                current_loc.get_distance(next_WP_loc) > radius*3) ||
               (control_mode == &mode_rtl && quadplane.available() && quadplane.rtl_mode == 1)) {
        /*
          if never reached loiter point and using crosstrack and somewhat far away from loiter point
          navigate to it like in auto-mode for normal crosstrack behavior

          we also use direct waypoint navigation if we are a quadplane
          that is going to be switching to QRTL when it gets within
          RTL_RADIUS
        */
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    } else {
        nav_controller->update_loiter(next_WP_loc, radius, loiter.direction);
    }

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() ||
            auto_state.wp_proportion > 1) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
            if (control_mode == &mode_guided || control_mode == &mode_avoidADSB) {
                // starting a loiter in GUIDED means we just reached the target point
                gcs().send_mission_item_reached_message(0);
            }
            if (quadplane.guided_mode_enabled()) {
                quadplane.guided_start();
            }
        }
    }
}

void Plane::update_zero_plane()
{
    eight_in_R2.set_current_segment(eight_in_R2.aircraft_loc, eight_in_R2.aircraft_vel);

    Vector3f rav =  location_3d_diff_NED(eight_in_R2.center_loc, eight_in_R2.aircraft_loc);
    Vector3f _rxaplanev = rav - eight_in_R2.erxv * (eight_in_R2.erxv * rav);
    int8_t _current_quadrant;
    // north or south
    if (_rxaplanev * eight_in_R2.ethetaxv >= 0) {_current_quadrant = -2;} else {_current_quadrant = -1;}
    // east or west
    if (_rxaplanev * eight_in_R2.epsixv >= 0) {_current_quadrant = _current_quadrant + 2;} else {_current_quadrant = -_current_quadrant + 1;};
    // current_quadrant is set to integer 0,1,2,3, where: NE:0, SE:1, SW:2, NW:3
    

        Vector3f crv = rav - eight_in_R2.current_cv;
        Vector3f ctv = eight_in_R2.current_tv;

////
//    struct Location g1start;
//    struct Location g1end;
//    struct Location g2start;
//    struct Location g2end;
//    if(eight_in_R2.orientation == 1){
//        g1start = eight_in_R2.c2g1_loc;
//        g1end = eight_in_R2.g1c1_loc;
//        g2start = eight_in_R2.c1g2_loc;
//        g2end = eight_in_R2.g2c2_loc;
//    } else {
//        g1start = eight_in_R2.g1c1_loc;
//        g1end = eight_in_R2.c2g1_loc;
//        g2start = eight_in_R2.g2c2_loc;
//        g2end = eight_in_R2.c1g2_loc;
//    }

    float angle;
    switch(eight_in_R2.current_segment){

    case 0:
        // g1
        //hal.console->println("segment 0: g1");
        angle = atan2f(eight_in_R2.etg1v.y, eight_in_R2.etg1v.x) * RAD_TO_DEG_DOUBLE;
        //angle = 90 - angle;
        nav_controller->update_loiter_ellipse(eight_in_R2.center_loc, 3.0f * eight_in_R2.S1_radius_cm, 0.0f, angle, eight_in_R2.orientation, eight_in_R2.aircraft_loc, eight_in_R2.aircraft_vel, eight_in_R2.desired_loc);
        break;
    case 1:
        // c1
        //hal.console->println("segment 1: c1");
        nav_controller->update_loiter_ellipse(eight_in_R2.c1_loc, eight_in_R2.S1_radius_cm, 1.0f, eight_in_R2.azimuth_deg, eight_in_R2.orientation, eight_in_R2.aircraft_loc, eight_in_R2.aircraft_vel, eight_in_R2.desired_loc);
        break;
    case 2:
        // g2
        //hal.console->println("segment 2: g2");
        angle = atan2f(eight_in_R2.etg2v.y, eight_in_R2.etg2v.x) * RAD_TO_DEG_DOUBLE;
        // angle = 90 - angle;
        nav_controller->update_loiter_ellipse(eight_in_R2.center_loc, 3.0f * eight_in_R2.S1_radius_cm, 0.0f, angle, eight_in_R2.orientation, eight_in_R2.aircraft_loc, eight_in_R2.aircraft_vel, eight_in_R2.desired_loc);
        break;
    case 3:
        // c2
        //hal.console->println("segment 3: c2");
        nav_controller->update_loiter_ellipse(eight_in_R2.c2_loc, eight_in_R2.S1_radius_cm, 1.0f, eight_in_R2.azimuth_deg, -eight_in_R2.orientation, eight_in_R2.aircraft_loc, eight_in_R2.aircraft_vel, eight_in_R2.desired_loc);
        break;
    }

}

void Plane::update_eight_sphere() {

    //    //nav_controller->update_loiter_3d(eight_in_S2.S2_loc, eight_in_S2.segments_ercv[eight_in_S2.current_segment], eight_in_S2.S2_radius_cm, eight_in_S2.segments_theta_r[eight_in_S2.current_segment], eight_in_S2.segments_orientation[eight_in_S2.current_segment], eight_in_S2.aircraft_posccenter, eight_in_S2.aircraft_vel, eight_in_S2.desired_loc);
        Vector3f current_ercv = eight_in_S2.current_ercv();
        int32_t current_theta_r = eight_in_S2.current_theta_r();
        int8_t current_orientation = eight_in_S2.current_orientation();


    //    Vector3f current_ercv = eight_in_S2.ercvs[eight_in_S2.current_segment];
    //    int32_t current_theta_r = eight_in_S2.thetars[eight_in_S2.current_segment];
    //    int8_t current_orientation = eight_in_S2.orientations[eight_in_S2.current_segment];

        //    hal.console->print("trigof chihalf: ");
    //    hal.console->print(eight_in_S2.sin_theta_r);
    //    hal.console->print(", ");
    //    hal.console->print(eight_in_S2.cos_theta_r);

    //    hal.console->print("eight_in_S2.erg1v: ");
    //    hal.console->print(eight_in_S2.erg1v.x);
    //    hal.console->print(", ");
    //    hal.console->print(eight_in_S2.erg1v.y);
    //    hal.console->print(", ");
    //    hal.console->println(eight_in_S2.erg1v.z);
    //
    //    hal.console->print("eight_in_S2.current_ercv() :");
    //    hal.console->print(current_ercv.x);
    //    hal.console->print(", ");
    //    hal.console->print(current_ercv.y);
    //    hal.console->print(", ");
    //    hal.console->println(current_ercv.z);

    //    hal.console->print("eight_in_S2.current_theta_r() :");
    //    hal.console->println(current_theta_r);
    //
    //    hal.console->print("eight_in_S2.current_orientation() :");
    //    hal.console->println(current_orientation);

//        hal.console->println("before loiter_3d: ");
//        hal.console->println(micros());
        nav_controller->update_loiter_3d(eight_in_S2.S2_loc, current_ercv, eight_in_S2.S2_radius_cm, current_theta_r, current_orientation, eight_in_S2.aircraft_loc, eight_in_S2.aircraft_vel, eight_in_S2.desired_loc);
    //    nav_controller->update_loiter_3d(eight_in_S2.S2_loc, Vector3f(0,0,-1), eight_in_S2.S2_radius_cm, 30.0f, 1, eight_in_S2.aircraft_loc, eight_in_S2.aircraft_vel, eight_in_S2.desired_loc);
//        hal.console->println("after loiter_3d: ");
//        hal.console->println(micros());
        // Vector3f adiff = eight_in_S2.S2_loc.get_distance_NED(eight_in_S2.aircraft_loc);
        // Vector3f desdiff = eight_in_S2.S2_loc.get_distance_NED(eight_in_S2.desired_loc);
        // hal.console->print("height, desired height: ");
        // hal.console->print(adiff.z);
        // hal.console->print(", ");
        // hal.console->println(desdiff.z);


        //eight_in_S2.set_current_segment(eight_in_S2.aircraft_loc, eight_in_S2.aircraft_vel);

//    // hal.console->print("aircraft_posS2center: ");
//    struct Location aloc = eight_in_S2.aircraft_loc;
//    Vector3f rav = location_3d_diff_NED(eight_in_S2.S2_loc, aloc);
//    Vector3f cv;
//    int8_t _current_quadrant = eight_in_S2.current_quadrant;
//    int8_t _current_segment = eight_in_S2.current_segment;
//    hal.console->print("aircraft_posS1center: ");
//    Vector3f S1avec = rav - eight_in_S2.erc1v * eight_in_S2.dist_cm / 100.0f;
//    hal.console->print(S1avec.x);
//    hal.console->print(", ");
//    hal.console->print(S1avec.y);
//    hal.console->print(", ");
//    hal.console->println(S1avec.z);

    struct Location aloc = eight_in_S2.aircraft_loc;
     Vector3f rav = eight_in_S2.S2_loc.get_distance_NED(aloc);

    // position vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
    Vector3f _rxaplanev = eight_in_S2.rxaplanev(rav);
    // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
    float _mindistxaplane = 0.25f * eight_in_S2.S2_radius_cm / 100.0f * eight_in_S2.sin_theta_0; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
    //
    eight_in_S2.close_to_crossing_point = bool(_rxaplanev.length() <= _mindistxaplane);
    // set internal variable _current quadrant in dependence of the location of the aircraft

    int8_t _current_quadrant = eight_in_S2.quadrant(rav);
    // set internal variable _current segment
    int8_t _current_segment = eight_in_S2.current_segment;
    int8_t _next_quadrant = eight_in_S2.quadrants[(eight_in_S2.quadrant_count[eight_in_S2.current_quadrant] + eight_in_S2.orientation) % 4];


//    hal.console->print("current_quadrant, _next_quadrant: ");
//    hal.console->print(eight_in_S2.current_quadrant);
//    hal.console->print(", ");
//    hal.console->println(_next_quadrant);
//    hal.console->print("_current_quadrant, _current_segment: ");
//    hal.console->print(_current_quadrant);
//    hal.console->print(", ");
//    hal.console->println(_current_segment);
//
//    hal.console->print("switch entered_new_quadrant? ");
//    hal.console->print(!eight_in_S2.close_to_crossing_point);
//    hal.console->print(eight_in_S2.current_quadrant != _current_quadrant);
//    hal.console->println(eight_in_S2.entered_next_quadrant);

    if(eight_in_S2.close_to_crossing_point){
//        // aircraft is too close to the crossing point
//        // disable quadrant and segment switching unless aircraft moves against orientation
//      if(eight_in_S2.moving_matches_orientation){
          // aircraft moves roughly in accord with the orientation
          // keep stored current_quadrant
          _current_quadrant = eight_in_S2.current_quadrant;
          // keep stored current_quadrant
          _current_segment = eight_in_S2.current_segment;
//      } else{
//          // aircraft moves roughly against the orientation
//          // change current_quadrant in dependence of the current quadrant and velocity vector
//          // switch current_segment (geodesics)
//          if (_current_segment == 0){
//              // current segment is g1
//              if (eight_in_S2.aircraft_vel * eight_in_S2.etg2xv >=0){
//                  // aircraft's velocity vector points rather into quadrant 3 than quadrant 1
//                  // switch current quadrant to quadrant 3
//                  _current_quadrant = 3;
//              } else {
//                  // aircraft's velocity vector points rather into quadrant 1 than quadrant 3
//                  // switch current quadrant to quadrant 1
//                  _current_quadrant = 1;
//              }
//              // switch current segment to segment g2
//              _current_segment = 2;
//          } else {
//              // current segment is g2
//              if (eight_in_S2.aircraft_vel * eight_in_S2.etg1xv >=0){
//                  // aircraft's velocity vector points rather into quadrant 0 than quadrant 2
//                  // switch current quadrant to quadrant 0
//                  _current_quadrant = 0;
//              } else {
//                  // aircraft's velocity vector points rather into quadrant 2 than quadrant 0
//                  // switch current quadrant to quadrant 0
//                  _current_quadrant = 2;
//              }
//              // switch current segment to segment g1
//              _current_segment == 0;
//          }
//      }
    } else {
        // aircraft is not too close to the crossing point
        // switching the current quadrant and current segment has to be checked and performed
        if (eight_in_S2.current_quadrant != _current_quadrant){
            // hal.console->print("entered new quadrant: ");
            // hal.console->println(_current_quadrant);
            // aircraft has entered another quadrant
            // determine if aircraft entered the next quadrant
            eight_in_S2.entered_next_quadrant = bool(_current_quadrant == _next_quadrant);
            // hal.console->print("new quadrant is next quadrant? ");
            // hal.console->println(eight_in_S2.entered_next_quadrant);
            if (eight_in_S2.entered_next_quadrant) {
                // aircraft has entered correct next quadrant
                // switch to next quadrant
                eight_in_S2.current_quadrant = _current_quadrant;
            // after switching the quadrant the aircraft has in any case left the initial quadrant
            eight_in_S2.in_initial_quadrant = false;

            }
        }
    }


    // center vector associated with the current quadrant
    Vector3f _current_cv = eight_in_S2.centervectors[eight_in_S2.current_quadrant];
    eight_in_S2.current_cv = _current_cv;
    // tangent vector at the transgression point between two segments associated with the current quadrant
    Vector3f _current_tv = eight_in_S2.tangentvectors[eight_in_S2.current_quadrant];
    eight_in_S2.current_tv = _current_tv;
    // position vector from center of the current turning circle to the aircraft
    // direction of flight in the current quadrant: +1:outbound, -1:inbound
    int8_t _current_direction = eight_in_S2.directions[eight_in_S2.current_quadrant];

    Vector3f _rcav = rav - _current_cv;
    eight_in_S2.rcav = _rcav;
    Vector2f _rcavl(_rcav.x,_rcav.y);
    Vector2f _current_tvl(_current_tv.x,_current_tv.y);
    eight_in_S2.projection = _rcavl.normalized() * _current_tvl * 100.0f + 50.0f;
    //hal.console->print("projection: ");
    //hal.console->println(eight_in_S2.projection);
    //hal.console->print("after projection: ");
    //hal.console->println(micros());

    // true if the current segment is the first in the quadrant: transgression point of that quadrant will be passed
    eight_in_S2.switch_to_2nd_segment_in_quadrant  = bool(eight_in_S2.projection >=0);//bool(_rcav * _current_tv >= 0);
    // true if the velocity vector of the aircraft is outbound / inbound  in the quadrants (0,3) / (1,2) for orientation = +1 and vice versa for orientation = -1
    eight_in_S2.moving_matches_orientation = bool(eight_in_S2.aircraft_vel * _current_cv * _current_direction > 0);

    //hal.console->print("eight_in_S2.in_initial_quadrant: ");
    //hal.console->println(eight_in_S2.in_initial_quadrant);



    //          if(close_to_crossing_point) {
    //              // aircraft is in the vicinity of the crossing point
    //              // select geodesic segment with orientation best aligned with the velocity vector of the aircraft
    //              if(vav * (etg1xv - etg2xv) >= 0){
    //                  // select segment corresponding to g1
    //                  _current_segment = 0;
    //              } else {
    //                  // select segment corresponding to g2
    //                  _current_segment = 2;
    //            }
    //          } else {

    // switch to the second segment in the quadrant if required
    if (eight_in_S2.close_to_crossing_point){
        // aircraft is in the vicinity  to crossing point
        // select geodesic segment with orientation best aligned with the velocity vector of the aircraft
//              if(vav * (etg1v - etg2v) >= 0){
//                  // select segment corresponding to g1
//                  _current_segment = 0;
//              } else {
//                  // select segment corresponding to g2
//                  _current_segment = 2;
//              }
    } else {
        // aircraft is not in the vicinity of the crossing point
        // set/leave current segment to/at the first segment
        // switch from first to second segment in the quadrant if
        //_current_segment = eight_in_S2.firstsegments[eight_in_S2.current_quadrant];
        if ((eight_in_S2.in_initial_quadrant || eight_in_S2.entered_next_quadrant) && eight_in_S2.switch_to_2nd_segment_in_quadrant){
            // hal.console->println("entering second segment of current quadrant.");
            // aircraft is in the first quadrant where figure-eight pattern is initialized or aircraft has entered next quadrant and switching to second segment is required
            // switch to second segment of the current quadrant
            _current_segment = eight_in_S2.secondsegments[eight_in_S2.current_quadrant];
            //current_segment = _current_segment;
            // reset
            eight_in_S2.entered_next_quadrant = false;
            eight_in_S2.in_initial_quadrant = false;

        }
    }

    eight_in_S2.current_quadrant = _current_quadrant;
    eight_in_S2.current_segment = _current_segment;

}


/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
void Plane::update_cruise()
{
    if (!cruise_state.locked_heading &&
        channel_roll->get_control_in() == 0 &&
        rudder_input() == 0 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        gps.ground_speed() >= 3 &&
        cruise_state.lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        cruise_state.lock_timer_ms = millis();
    }
    if (cruise_state.lock_timer_ms != 0 &&
        (millis() - cruise_state.lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        cruise_state.locked_heading = true;
        cruise_state.lock_timer_ms = 0;
        cruise_state.locked_heading_cd = gps.ground_course_cd();
        prev_WP_loc = current_loc;
    }
    if (cruise_state.locked_heading) {
        next_WP_loc = prev_WP_loc;
        // always look 1km ahead
        next_WP_loc.offset_bearing(cruise_state.locked_heading_cd*0.01f, prev_WP_loc.get_distance(current_loc) + 1000);
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }
}


/*
  handle speed and height control in FBWB or CRUISE mode. 
  In this mode the elevator is used to change target altitude. The
  throttle is used to change target airspeed or throttle
 */
void Plane::update_fbwb_speed_height(void)
{
    uint32_t now = micros();
    if (now - target_altitude.last_elev_check_us >= 100000) {
        // we don't run this on every loop as it would give too small granularity on quadplanes at 300Hz, and
        // give below 1cm altitude change, which would result in no climb or descent
        float dt = (now - target_altitude.last_elev_check_us) * 1.0e-6;
        dt = constrain_float(dt, 0.1, 0.15);

        target_altitude.last_elev_check_us = now;
        
        float elevator_input = channel_pitch->get_control_in() / 4500.0f;
    
        if (g.flybywire_elev_reverse) {
            elevator_input = -elevator_input;
        }

        int32_t alt_change_cm = g.flybywire_climb_rate * elevator_input * dt * 100;
        change_target_altitude(alt_change_cm);
        
        if (is_zero(elevator_input) && !is_zero(target_altitude.last_elevator_input)) {
            // the user has just released the elevator, lock in
            // the current altitude
            set_target_altitude_current();
        }
        
        target_altitude.last_elevator_input = elevator_input;
    }
    
    check_fbwb_minimum_altitude();

    altitude_error_cm = calc_altitude_error_cm();
    
    calc_throttle();
    calc_nav_pitch();
}

/*
  calculate the turn angle for the next leg of the mission
 */
void Plane::setup_turn_angle(void)
{
    int32_t next_ground_course_cd = mission.get_next_ground_course_cd(-1);
    if (next_ground_course_cd == -1) {
        // the mission library can't determine a turn angle, assume 90 degrees
        auto_state.next_turn_angle = 90.0f;
    } else {
        // get the heading of the current leg
        int32_t ground_course_cd = prev_WP_loc.get_bearing_to(next_WP_loc);

        // work out the angle we need to turn through
        auto_state.next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
    }
}    

/*
  see if we have reached our loiter target
 */
bool Plane::reached_loiter_target(void)
{
    if (quadplane.in_vtol_auto()) {
        return auto_state.wp_distance < 3;
    }
    return nav_controller->reached_loiter_target();
}
    
