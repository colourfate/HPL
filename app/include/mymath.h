#ifndef _MYMATH_H_
#define _MYMATH_H_
#define ONE_PI   (3.14159265)
#define REAL float

#define my_abs(x) ( (x)>0?(x):-(x) )
float inline my_absf(float x)
{
	return (x)>0?(x):-(x);
}

double my_sin(double rad);
double my_cos(double rad);
float my_sqrt(float number);
REAL fast_atan2(REAL y, REAL x);
float my_atof(char *str);
char *my_ftoa(const double x);

#endif
