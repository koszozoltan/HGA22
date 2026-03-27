#ifndef __GOERTZEL_H__
#define __GOERTZEL_H__


extern float coef1;
extern float coef2;


int     search_freq ( uint16_t samples[], int depth);
float   Goertzel_coef(float freq, int samp_rate) ;
float   Goertzel_mag(uint16_t samples[], int depth, float coef, int adc_midpoint);


int16_t Goertzel_coef_q15(float freq, int samp_rate);
int32_t Goertzel_mag_fast_q15(const int16_t *samples,
                              int depth,
                              int16_t coef,
                              int32_t adc_mid);




#endif /* __GOERTZEL_H__ */
