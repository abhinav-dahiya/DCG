#ifndef FILT_H
#define FILT_H

float discrete_diff(float x, double dT, int freq);
float low_pass_filter(float x, double dT, int freq);
float discrete_intg(float x, double dT);

#endif

