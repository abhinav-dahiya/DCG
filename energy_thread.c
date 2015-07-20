#include "declare.h"
#include "energy_thread.h"

float calculate_energy(void)
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
	
	printf("Energy thread started.\n");
	
	while(1)
	{
		if(sample_energy)
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
			
			sample_energy = 0;
			
			return power;
			
		}
	}
}
