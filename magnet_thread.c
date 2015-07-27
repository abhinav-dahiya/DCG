#include "declare.h"
#include "magnet_thread.h"

void magnet_thread(void)
{
	printf("Magnet thread started.\n");

	while(1)
	{
		if(sample_magnet)
	    {
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
