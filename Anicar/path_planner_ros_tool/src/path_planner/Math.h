

#ifndef MATH_H
#define MATH_H
#include <cmath>
#include <cfloat>

class Math {
public:

    static const double INF;
    static const double PI;
    static const double SQRT2;

    static double deg2rad(double degrees);
    static double deg2signed(double degrees);
    static bool equals(double a, double b, double precision = 0.000000000000001);
    static bool greater(double a, double b, double precision = 0.000000000000001);
    static bool less(double a, double b, double precision = 0.000000000000001);
    static double rad2deg(double radians);
    static double rad2signed(double radians);
};


#endif //MATH_H
