# DCG
BLDC servo motor control, using R-Pi. Discrete-time control. Multi-threading
The functions are in sepatrate header files.

Code Details:

1.	Libraries used (other than standard): bcm2835.h, pthread.h, time.h 
So, while compiling with gcc, link with these libraries: -lbcm2835, -lm, -pthread, -lrt

2.	It runs seven threads (other than the main). Three for keeping up the time and other two for encoder measurement and magnet calculations. These threads keep on updating values of x (motor position), dx (motor speed) and xd (desired position) while I_ref (control action) is calculated in a separate thread. The last thread is for measuring the energy consumed.

3.	Time_thread : Based on the preset values for the three time periods (for encoder, magnetic sensor and power), corresponding sampling flags are set. The other three threads keep on waiting for their respective flag to set. Once recognised, the threads start doing their calculations and resets this flag.
NOTE: This can also be done by merely changing the state of some PIN instead of a variable. Doing that will make it possible to see the behaviour (toggling speed, timer accuracy) of the timer thread and also allows to input from timer from outside source, if needed in future.

4.	Encoder_thread: This thread first input value of motor encoder position from the quadrature decoder, converts it into decimal value of motor position (x), then calculates speed (dx) and filtered value of x (xf). Every iteration it calls three functions: discrete_diff, low_pass_filter and calculate_encoder.

IMPORTANT: Do not connect the index pin of the encoder to the quadrature decoder, because it resets the encoder. The PCB has provision to connect the index pin to connector, so the connection has to be removed somehow. For now, I have removed the contact from the IC connector of the decoder IC.

5.	Magnet_thread: This thread gets absolute position data from the magnetic sensor and calculates the desired position of x (xd). But before desired data can be read from the sensor, the EEPROM is to be initialised properly based on the required tuning (which depends on various unavoidable things).

6.	Discrete-time differentiator: Calculates derivative of the input signal samples, using difference equation based on practical model of a differentiator circuit.

7.	Low_pass_filter: Applies a discrete second-order low pass filter on the input signal.

8.	Main _loop: Waits for the encoder _thread and the magnet_thread to complete the calculations for current samples of data, and then calculates the I_ref using x, dx, and xd. Converts this into pwm value (0-1024) and output through PIN 12. 
Note: This conversion to pwm is dependent on the settings of the motor controller which are done using ESCON software.

9.	Energy_calculator: This thread reads the voltage value from the power_measurement IC and using discrete time integrator, it converts this data to total energy consumed.

 
EEPROM Code:

•	This code is for writing the data to the EEPROM for sensor configuration.
•	It asks to enter an address and then the value we want to write to that address. Everything is to be written in Hexadecimal format. In case, a wrong value is entered, press Ctrl+C and re-enter that address.
NOTE: Apparently, the EEPROM is not needed, because the sensor itself can be configure using the SPI communication (which is being used to read its data), after it is powered on. So, just write the code for the registers that are to be configured (see the code, details and example code are commented).