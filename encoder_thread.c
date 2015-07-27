#include "declare.h"
#include "encoder_thread.h"

/*
// declaring various global variables

int encoder_flag	= 0;		  // set when encoder calculation done
int sample_encoder  = 0;		  // flag for sampling encoder
int sample_magnet   = 0;		  // flag for sampling magnetic sensor
float x         	= 0.0;		  // position
float xf 			= 0.0;		  // filtered value of position
float dx        	= 0.0;		  // motor speed

*/

void encoder_thread(void)
{
	bcm2835_gpio_fsel(ENC_PIN, BCM2835_GPIO_FSEL_OUTP);

	// setting modes of counter pins

	bcm2835_gpio_fsel(OE_COUNT, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(SEL1, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(SEL2, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(D0, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(D1, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(D2, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(D3, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(D4, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(D5, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(D6, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(D7, BCM2835_GPIO_FSEL_INPT);

	printf("Encoder Thread started.\n");
	
	while(1)
	{
		if(sample_encoder)
		{
			bcm2835_gpio_write(ENC_PIN, HIGH);		  // for the oscilloscope
			
			// code for obtaining value from encoder IC
			x = calculate_encoder();

			// code for calculating x and dx
			dx = discrete_diff(x,dT_PD,100);	   // calling practical differentiator
			xf = low_pass_filter(x,dT_PD,100); // getting filtered value of x

			bcm2835_gpio_write(ENC_PIN, LOW);

			// reset time flag
			sample_encoder = 0;  //reset sampling flag

			//set encoder flag high
		    encoder_flag = 1;   // means encoder calculations done
	  	}
	}

}



float calculate_encoder(void)
{
	float encoder_array[24];

	// setting OE for counter
	bcm2835_gpio_write(OE_COUNT, LOW);

	// reading LSB (0-7)

	bcm2835_gpio_write(SEL1, HIGH);
	bcm2835_gpio_write(SEL2, LOW);

	encoder_array[0] = bcm2835_gpio_lev(D0);
	encoder_array[1] = bcm2835_gpio_lev(D1);
	encoder_array[2] = bcm2835_gpio_lev(D2);
	encoder_array[3] = bcm2835_gpio_lev(D3);
	encoder_array[4] = bcm2835_gpio_lev(D4);
	encoder_array[5] = bcm2835_gpio_lev(D5);
	encoder_array[6] = bcm2835_gpio_lev(D6);
	encoder_array[7] = bcm2835_gpio_lev(D7);

	// reading 2nd byte (8-15)

	bcm2835_gpio_write(SEL1, LOW);
	bcm2835_gpio_write(SEL2, LOW);

	encoder_array[8]  = bcm2835_gpio_lev(D0);
	encoder_array[9]  = bcm2835_gpio_lev(D1);
	encoder_array[10] = bcm2835_gpio_lev(D2);
	encoder_array[11] = bcm2835_gpio_lev(D3);
	encoder_array[12] = bcm2835_gpio_lev(D4);
	encoder_array[13] = bcm2835_gpio_lev(D5);
	encoder_array[14] = bcm2835_gpio_lev(D6);
	encoder_array[15] = bcm2835_gpio_lev(D7);

	// reading 3rd Byte (16-23)

	bcm2835_gpio_write(SEL1, HIGH);
	bcm2835_gpio_write(SEL2, HIGH);

	encoder_array[16] = bcm2835_gpio_lev(D0);
	encoder_array[17] = bcm2835_gpio_lev(D1);
	encoder_array[18] = bcm2835_gpio_lev(D2);
	encoder_array[19] = bcm2835_gpio_lev(D3);
	encoder_array[20] = bcm2835_gpio_lev(D4);
	encoder_array[21] = bcm2835_gpio_lev(D5);
	encoder_array[22] = bcm2835_gpio_lev(D6);
	encoder_array[23] = bcm2835_gpio_lev(D7);

	// reset OE value
	bcm2835_gpio_write(OE_COUNT, LOW);

	// convert data to decimal

	x = 0;
	int i;
	for (i = 0; i < 24; i++)
	{
//		printf("for loop entered, Di = %d \n",encoder_array[i]);
		x = x + (encoder_array[i] * pow(2,i));
	}

//	printf("Decimal value of x = %f \n",x);

	return x;
}
