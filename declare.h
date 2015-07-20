#ifndef DEC_H
#define DEC_H


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


extern int encoder_flag	  ;		  // set when encoder calculation done
extern int magnet_flag 	  ;		  // set when magnet calculation done
extern double dT_PD       ;        // sampling time for PD loop in seconds
extern double dT_XD    	  ;        // sampling time for xd loop in seconds
extern double dT_EN    	  ;
extern int sample_encoder ;		  // flag for sampling encoder
extern int sample_magnet  ;		  // flag for sampling magnetic sensor
extern int sample_energy  ;
extern float x            ;		  // position
extern float xf 		  ;		  // filtered value of position
extern float xd           ;		  // desired value of position
extern float dx           ;		  // motor speed
extern float I_ref        ;		  // reference current (control action)
extern float kp           ;		  // kp for PD loop
extern float kd           ;		  // kd for PD loop
extern float I_range      ;		  // for calculating PWM value
extern float power 		  ;		  // instantaneous power measured by IC


#endif

