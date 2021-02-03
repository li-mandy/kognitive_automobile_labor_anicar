
#include "Math.h"
const double Math::INF = DBL_MAX;


const double Math::PI = 3.14159265;

const double Math::SQRT2 = 1.41421356237309504880;

double Math::deg2rad(double degrees)
{
    return degrees * Math::PI / 180.0;
}

double Math::deg2signed(double degrees)
{
    degrees = fmod(degrees, 360.0);

    return (degrees > 180.0) ? -1.0 * fmod(degrees, 180.0) : degrees;
}


bool Math::equals(double a, double b, double precision)
{
    if (a == Math::INF && b == Math::INF)
        return true;

    return (fabs(a - b) < precision);
}

bool Math::greater(double a, double b, double precision)
{
    if (a == Math::INF && b == Math::INF)
        return false;

    return a - precision > b;
}


bool Math::less(double a, double b, double precision)
{
    if (a == Math::INF && b == Math::INF)
        return false;

    return a + precision < b;
}


double Math::rad2deg(double radians)
{
    return radians * 180.0 / Math::PI;
}


double Math::rad2signed(double radians)
{
    radians = fmod(radians, (2.0 * Math::PI));

    return (radians > Math::PI) ? -1.0 * fmod(radians, Math::PI) : radians;
}