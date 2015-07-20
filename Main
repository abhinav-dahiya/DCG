// including the required functions' header files

#include "declare.h"
#include "timer_thread.h"
#include "encoder_thread.h" 
#include "magnet_thread.h"
#include "filters.h" 
#include "energy_thread.h" 
#include "calculate_I_ref.h" 

int main(void) 
{
	 bcm2835_init();
	 
	// setting PWM_PIN as pwm from channel 0 in markspace mode with range = RANGE
	bcm2835_gpio_fsel(PWM_PIN, BCM2835_GPIO_FSEL_ALT5);  //ALT5 is pwm mode
	bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16); // pwm freq = 19.2 / 16 MHz
	bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1);		     // markspace mode
	bcm2835_pwm_set_range(PWM_CHANNEL, RANGE);
	
	// creating and running threads
	pthread_t th1, th2, th3, th4, th5, th6;
	pthread_create(&th1, NULL, (void*)encoder_time_thread, NULL);
	pthread_create(&th2, NULL, (void*)magnet_time_thread, NULL);
	pthread_create(&th3, NULL, (void*)encoder_thread, NULL);
	pthread_create(&th4, NULL, (void*)magnet_thread, NULL);
	pthread_create(&th5, NULL, (void*)energy_time_thread, NULL);
	pthread_create(&th6, NULL, (void*)calculate_energy, NULL);
	
	while(1)
	{
		if(encoder_flag) // encoder flag = 1 means xf and dx are calculated
		{
			//calculate I_ref and corresponding pwm value
			
			float pwm_value = calculate_I_ref();
			bcm2835_pwm_set_data(PWM_CHANNEL, pwm_value);
	
			//reset flag
			encoder_flag = 0;
		}
	}
    return 0;
}
