#ifndef ENC_THREADS_H
#define ENC_THREADS_H

/*
// declaring various global variables

extern int encoder_flag	  ;		  // set when encoder calculation done
extern int sample_encoder ;		  // flag for sampling encoder
extern int sample_magnet  ;		  // flag for sampling magnetic sensor
extern float x            ;		  // position
extern float xf 		  ;		  // filtered value of position
extern float dx           ;		  // motor speed

  */


void encoder_thread(void);
float calculate_encoder(void);

#endif

