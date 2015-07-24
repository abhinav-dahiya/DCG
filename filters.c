#include "declare.h"
#include "filters.h"

float discrete_diff(float xi, double dT, float freq)
{
	static double x_old[2] = {0.0,0.0};
	static double dx_old[2] = {0.0,0.0};
	double tau = 1.0/(2.0*3.14159*freq);

	double A = 2.0*(dT/(pow((2.0*tau + dT),2.0)));
	double B = -1.0 * A;
	double C = (2.0*(2.0*tau - dT))/(2.0*tau +dT);
	double D = -1.0 * (pow(((2.0*tau - dT)/(2.0*tau + dT)),2.0));
	static float itr = 1;

	float derivative = A*xi + B*x_old[1] + C*dx_old[0] + D*dx_old[1];

	if(itr < 2.5)
	{
		derivative = (xi - x_old[0])/dT;
		++itr;
	}

	dx_old[1] = dx_old[0];
	dx_old[0] = derivative;

	x_old[1] = x_old[0];
	x_old[0] = xi;

	printf("Diff. Done. and dx = %f\n", derivative);			 
	return derivative;
}



float low_pass_filter(float xi, double dT, float freq)
{
	static float x_old[2] = {0.0,0.0};
	static float xf_old[2] = {0.0,0.0};

	float a = 2.0 * 3.14159 * freq;
	float p = 2.0/dT;

	float A = pow((a/(a+p)),2.0);
	float B = 2.0 * A;
	float C = A;
	float D = -2.0 * (a-p) / (a+p);
	float E = -1.0 * pow(((a-p)/(a+p)),2.0);
	static float itr = 1;

	float filtered = (A*xi) + B*x_old[0] + C*x_old[1] + D*xf_old[0] + E*xf_old[1];

	if(itr < 2.5)
	{
		filtered = xi;
		++itr;
	}

	xf_old[1] = xf_old[0];
	xf_old[0] = filtered;

	x_old[1]  = x_old[0];
	x_old[0]  = xi;

	printf("Filtering done and xf = %f\n", filtered);
	return filtered;
}


float discrete_intg(float pow, double dT)
{
 	static float pow_old;
	static float E_old;

	float A = dT/2.0;
	static float itr = 1.0;

	float E = E_old + A*(pow + pow_old);

	if(itr < 1.5)
	{
		E = (pow * dT)/2.0;
		++itr;
	}

	E_old = E ;

	pow_old = pow;

	return E;
}
