/*******************************************************************************
*
*   Main_Code.c
*
********************************************************************************/

// including libraries

#include<math.h>
#include<stdio.h>
#include<bcm2835.h>
#include<pthread.h>
#include<unistd.h>
#include<time.h>


// define various pins for testing

#define TIME_PIN RPI_BPLUS_GPIO_J8_11  //time flag
#define PWM_PIN RPI_BPLUS_GPIO_J8_12   //pwm
#define ENC_PIN RPI_BPLUS_GPIO_J8_38
#define MAG_PIN RPI_BPLUS_GPIO_J8_40

#define PWM_CHANNEL 0
#define RANGE 1024		 // pwm range = (0-1024)
#define BILLION 1E9


// defining pins for quadrature decoder (encoder counter)

#define OE_COUNT RPI_BPLUS_GPIO_J8_13  // output enable for encoder counter
#define SEL1 RPI_BPLUS_GPIO_J8_16      // sel 1 for encoder counter
#define SEL2 RPI_BPLUS_GPIO_J8_18      // sel 2 for encoder counter
#define D0 RPI_BPLUS_GPIO_J8_22        // 8 data pins
#define D1 RPI_BPLUS_GPIO_J8_33
#define D2 RPI_BPLUS_GPIO_J8_32
#define D3 RPI_BPLUS_GPIO_J8_31
#define D4 RPI_BPLUS_GPIO_J8_29
#define D5 RPI_BPLUS_GPIO_J8_26
#define D6 RPI_BPLUS_GPIO_J8_24
#define D7 RPI_BPLUS_GPIO_J8_35


// declaring thread functions

void encoder_time_thread(void);
void magnet_time_thread(void);
void encoder_thread(void);
void magnet_thread(void);
void calculate_energy(void);

//declaring other functions
float discrete_diff(float x, double dT, int freq);
float low_pass_filter(float x, double dT, int freq);
float calculate_encoder(void);
float discrete_intg(float x, double dT);
void clock_delay(double interval);

// declaring various global variables
int encoder_flag = 0;		  // set when encoder calculation done
int magnet_flag  = 0;		  // set when magnet calculation done
double dT_PD     = 1;         // sampling time for PD loop in seconds
double dT_XD     = 1;         // sampling time for xd loop in seconds
int sample_encoder = 0;		  // flag for sampling encoder
int sample_magnet = 0;		  // flag for sampling magnetic sensor
float x          = 0.0;		  // position
float xf 		 = 0.0;		  // filtered value of position
float xd         = 0.0;		  // desired value of position
float dx         = 0.0;		  // motor speed
float I_ref      = 0;		  // reference current (control action)
float kp         = 2.0;		  // kp for PD loop
float kd         = 1.0;		  // kd for PD loop
float I_range    = 0.0;		  // for calculating PWM value


// the main

int main(int argc, char **argv)
{
    if(!bcm2835_init())
            return 1;

	// setting PWM_PIN as pwm from channel 0 in markspace mode with range = RANGE

	bcm2835_gpio_fsel(PWM_PIN, BCM2835_GPIO_FSEL_ALT5);  //ALT5 is pwm mode
	bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16); // pwm freq = 19.2 / 16 MHz
	bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1);		     // markspace mode
	bcm2835_pwm_set_range(PWM_CHANNEL, RANGE);


	// initializing threads

	pthread_t th1, th2, th3, th4, th5;
	pthread_create(&th1, NULL, (void*)encoder_time_thread, NULL);
	pthread_create(&th2, NULL, (void*)magnet_time_thread, NULL);
	pthread_create(&th3, NULL, (void*)encoder_thread, NULL);
	pthread_create(&th4, NULL, (void*)magnet_thread, NULL);
	pthread_create(&th5, NULL, (void*)calculate_energy, NULL);


	// now that the four threads are running, start calculating I_ref

	while(1)
	{
		if(encoder_flag)
		{
			//calculate I_ref and convert to pwm value
			I_ref = (kp)*(xd - xf) - (kd)*(dx);
			I_range = 2;     // for range -2 to +2
			double pwm_value = (I_ref + I_range) * (512 / I_range); // I_range = range set suring motor controller setup
			bcm2835_pwm_set_data(PWM_CHANNEL, pwm_value);

			//reset flags
			encoder_flag = 0;
		}
	}
	bcm2835_close();
	return 0;
}



void encoder_time_thread(void)
{
	while(1)
	{
		sample_encoder = 1;   // set sampling flag for encoder
		clock_delay(dT_PD);
	}
}




void magnet_time_thread(void)
{
	while(1)
	{
		sample_magnet = 1;   // set sampling flag for magnet
		clock_delay(dT_XD);
	}
}




void encoder_thread(void)
{
	bcm2835_gpio_fsel(ENC_PIN, BCM2835_GPIO_FSEL_OUTP);
/*	int sel_1 = 0;
	int sel_2 = 0;
	int oe = 0;
*/
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



void magnet_thread(void)
{
	//set MAG_PIN as output
	bcm2835_gpio_fsel(MAG_PIN, BCM2835_GPIO_FSEL_OUTP);

	while(1)
	{
		if(sample_magnet)
	    {
			// bcm2835_gpio_set_eds(PIN1);
			bcm2835_gpio_write(MAG_PIN, HIGH);


			//code for obtaining value from iC-MU
			bcm2835_spi_begin();
	    		bcm2835_gpio_fsel(D6, BCM2835_GPIO_FSEL_INPT);   // ATTENTION: If there's a problem in reading encoder data (the D6 and/or D5 Pins), this is the issue. Can be solved by changing the library.
			bcm2835_gpio_fsel(D5, BCM2835_GPIO_FSEL_INPT);
    			bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    			bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
	    		bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32); // 32 = 7.8125 MHz, 128 = 1.95 Mhz
    		//bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    		//bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default


			char mag_buf[] = { 0xA6, 0xF0, 0x12 };   //  Data to send: first byte is op code, rest are useless
			bcm2835_spi_transfern(mag_buf, sizeof(mag_buf));     // mag_buf will now be filled with the data that was read from the sensor

			double mag_reading = (256.0 * mag_buf[1]) + mag_buf[2];

			bcm2835_spi_end();

			// code for calculating xd

			xd = pow(sin(mag_reading),-0.5);

			bcm2835_gpio_write(MAG_PIN, LOW);

			// reset sampling flag
			sample_magnet = 0;
			
			//set magnet flag high
			magnet_flag = 1;
	  	}
	}
}



void calculate_energy(void)
{
	uint8_t send;
	uint8_t data;
	char volt[2];
	uint8_t write_address = 0x80;
	uint8_t read_address = 0x81;
	char start_read[3];
	start_read[0] = 0x80; //write
	start_read[1] = 0x01; //voltage register address
	start_read[2] = 0x81; //read
	float voltage = 0;
	
	while(1)
	{
		if(sample_encoder)
		{
			bcm2835_i2c_begin();  // I2C begin

    		bcm2835_i2c_setSlaveAddress(write_address);		 //write
    		bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
	
			//data = 0x01;  // shunt voltage register
			send = bcm2835_i2c_write(start_read, 3);
			send = bcm2835_i2c_read(volt, 2);
			if(volt[0] > 7)		   // if sign bit is 1
			{
				volt[0] = volt[0] - 8;
				voltage = (float)(-1.0 * ((256.0 * volt[0])+ volt[1]));
			}
			else
			{
				voltage = (float)((256.0 * volt[0])+ volt[1]);
			}
			
			float power = pow(voltage,2) / 0.01;   //  V^2 / R
			float energy = discrete_intg(power, dT_PD);
		//	printf("Energy = %f \n",energy);

			bcm2835_i2c_end(); // I2C end
			
			bcm2835_gpio_set_eds(TIME_PIN);
		}
			
			
	}
}



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

//	printf("Diff. Done. \n");
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

//	printf("xf_old = %f \n", xf_old[0]);
	return xf;
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



void clock_delay(double interval)
{
	struct timespec requestStart, requestEnd;
	double elapsed;

	clock_gettime(CLOCK_REALTIME, &requestStart);
	clock_gettime(CLOCK_REALTIME, &requestEnd);
	elapsed = (double)( requestEnd.tv_sec - requestStart.tv_sec ) + (double)( requestEnd.tv_nsec - requestStart.tv_nsec )/ BILLION;

	while( elapsed  < interval ) //interval in seconds
	{
		clock_gettime(CLOCK_REALTIME, &requestEnd);
		elapsed = (double)( requestEnd.tv_sec - requestStart.tv_sec ) + (double)( requestEnd.tv_nsec - requestStart.tv_nsec )/ BILLION;
	}

}
