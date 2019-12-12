#include "mode.h"
#include "Plane.h"

bool ModeNavigate::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.do_zero_plane();

    return true;
}

void ModeNavigate::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}