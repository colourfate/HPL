// the atof() function and math lib is not be surported in the SDK...

// following code come from mymath.c in F407_FC_ANO project
#include <cs_types.h>
#include "mymath.h"

#define ONE_PI   (3.14159265)

static double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	s8 flag = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

double my_cos(double rad)
{
	s8 flag = 1;
	rad += ONE_PI/2.0;

	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}

	return my_sin(rad)*flag;
}

// return 10^n
double pow10(int8_t n)
{
	double ret = 1;
	if(n < 0){
		for(uint8_t i=0; i<-n; i++){
			ret *= 0.1f;
		}
	}else{
		for(uint8_t i=0; i<n; i++){
			ret *= 10;
		}
	}
	
	return ret;
}

