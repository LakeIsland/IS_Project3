#include "custom_math_utils.h"


double getRandomDouble()
{
	return (double)rand() / (double)RAND_MAX;
}

double lerp(double a, double b, double bpart)
{
	return a + (b - a) * bpart;
}

double lerp_theta(double a, double b, double bpart)
{
	double ax = cos(a);
	double ay = sin(a);
	
	double bx = cos(b);
	double by = sin(b);
	
	double nx = lerp(ax, bx, bpart);
	double ny = lerp(ay, by, bpart);
	
	return atan2(ny, nx);
}

double clamp(double min, double max, double x)
{
	if(x > max)
		return max;
	if(x < min)
		return min;
	return x;
}

double clampToPi(double value){
	float result = value;
	while(result >= PI)
		result -= 2 * PI;
	while(result < -PI)
		result += 2 * PI;
	return result;
}
double getExponentiallyInterpolatecValue(double max_y, double min_y, double x)
{
	double dy = max_y - min_y;
	return dy * exp(-20 * x) + min_y;
}
double getLinearlyInterpolatedValue(double x1, double y1, double x2, double y2, double x)
{
	if(x1 > x2)
	{
		if(x < x2)
			return y2;
		if(x > x1)
			return y1;
	}
	else if(x1 < x2)
	{
		if(x > x2)
			return y2;
		if(x < x1)
			return y1;
	}
		
	double a = (y2 - y1) / (x2 - x1);	
	return (a * (x - x1) + y1);
}

double myabs(double value)
{
	if(value < 0)
		return -value;
	return value;
}


