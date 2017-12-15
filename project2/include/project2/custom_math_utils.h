#ifndef CUSTOM_MATH_UTILS_H
#define CUSTOM_MATH_UTILS_H
#include <cmath>
#include <cstdlib>

#define DOUBLE_INFINITE 99999999999
#define PI 3.14159265358979323846

double getRandomDouble();
double lerp(double a, double b, double bpart);
double lerp_theta(double a, double b, double bpart);

double clamp(double min, double max, double x);
double clampToPi(double value);
double getLinearlyInterpolatedValue(double x1, double y1, double x2, double y2, double x);
double getExponentiallyInterpolatecValue(double max_y, double min_y, double x);
double myabs(double value);

#endif


