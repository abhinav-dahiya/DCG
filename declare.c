#include "declare.h"


// declaring various global variables

int encoder_flag	= 0;		  // set when encoder calculation done
int magnet_flag 	= 0;		  // set when magnet calculation done
double dT_PD    	= 1;        // sampling time for PD loop in seconds
double dT_XD    	= 2;        // sampling time for xd loop in seconds
double dT_EN    	= 3;        // sampling time for power sampling loop in seconds   
int sample_encoder 	= 0;		  // flag for sampling encoder
int sample_magnet  	= 0;		  // flag for sampling magnetic sensor
int sample_energy  	= 0;		  // flag for sampling power
float x         	= 0.0;		  // position
float xf 		   	= 0.0;		  // filtered value of position
float xd         	= 0.0;		  // desired value of position
float dx         	= 0.0;		  // motor speed
float I_ref      	= 0.0;		  // reference current (control action)
float kp         	= 2.0;		  // kp for PD loop
float kd            = 1.0;		  // kd for PD loop
float I_range    	= 0.0;		  // for calculating PWM value
float power 		= 0.0;		  // instantaneous power measured by IC

