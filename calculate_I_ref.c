#include "calculate_I_ref.h"
#include "declare.h"


float calculate_I_ref(void)
{	
	printf("I_ref\n");
	
	//calculate I_ref and convert to pwm value
	I_ref = (kp)*(xd - xf) - (kd)*(dx);
	I_range = 2;     // for range -2 to +2
	float pwm_value = (I_ref + I_range) * (512 / I_range); // I_range = range set suring motor controller setup
	
	//reset flags
	encoder_flag = 0;
	
	return pwm_value;
	
}
