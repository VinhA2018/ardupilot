#include "mode.h"
#include "Plane.h"

bool ModeUpwind::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.do_fly8();

    return true;
}

void ModeUpwind::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}