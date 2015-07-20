#include "declare.h"
#include "filters.h"

float discrete_diff(float xi, double dT, int freq)
{
	static double x_old[2];
	static double dx_old[2];
	double tau = 1/(2*3.14159*freq);

	double A = 2.0*dT/pow((2.0*tau + dT),2);
	double B = -1 * A;
	double C = 2.0*(2.0*tau - dT)/(2.0*tau +dT);
	double D = -1 * pow(((2.0*tau - dT)/(2.0*tau + dT)),2);
	static float itr = 1;

	dx = A*xi + B*x_old[1] + C*dx_old[0] + D*dx_old[1];

	if(itr < 2.5)
	{
		dx = (xi - x_old[0])/dT;
		++itr;
	}

	dx_old[1] = dx_old[0];
	dx_old[0] = dx;

	x_old[1] = x_old[0];
	x_old[0] = xi;

	printf("Diff. Done. \n");
	return dx;
}



float low_pass_filter(float xi, double dT, int freq)
{
	static float x_old[2];
	static float xf_old[2];

	double a = 2 * 3.14159 * freq;
	double p = 2.0/dT;

	double A = pow((a/(a+p)),2);
	double B = 2 * A;
	double C = A;
	double D = -2 * (a-p) / (a+p);
	double E = -1 * pow(((a-p)/(a+p)),2);
	static float itr = 1;

	xf = A*xi + B*x_old[0] + C*x_old[1] + D*xf_old[0] + E*xf_old[1];

	if(itr < 2.5)
	{
		xf = xi;
		++itr;
	}

	xf_old[1] = xf_old[0];
	xf_old[0] = xf;

	x_old[1]  = x_old[0];
	x_old[0]  = xi;

	printf("iltering done.\n");
	return xf;
}


float discrete_intg(float pow, double dT)
{
 	static float pow_old;
	static float E_old;

	float A = dT/2.0;
	static float itr = 1;

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
