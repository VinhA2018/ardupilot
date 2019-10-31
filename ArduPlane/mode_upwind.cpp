#include "mode.h"
#include "Plane.h"

bool ModeUpwind::_enter()
{
    hal.console->println("Initialization of UPWING mode");
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.do_eight_sphere();

    return true;
}

void ModeUpwind::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}