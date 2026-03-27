/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include <math.h>
#include <limits.h>
#include "util.h"
#include "main.h"
#include "ringbuffer.h"
#include "goertzel.h"

float coef1;
float coef2;


int search_freq ( uint16_t samples[], int depth){


     float max = -1;
     int max_id = 0;
     float TONE_ONE_MAG;
     static int first = 0;

     int minimum = INT_MAX;
     int maximum = INT_MIN;

     for(int i = 0; i < depth; i++) {
    	  if (minimum >(int) samples[i]) minimum = (int) samples[i];
    	  if (maximum <(int) samples[i]) maximum = (int) samples[i];
     }
     int avg = (maximum+minimum)/2;
     if (first == 0) {
        first = 1;
         Uart2_printf("FREQ:");
         for (int i = 1700 ; i < 2100 ; i=i+10){

            Uart2_printf("%d;", i);

         }
          Uart2_printf("\r\n");
     }

     float sum = 0;
     int   avg_cnt = 0;
     float p1 = 0;
     float p2 = 0;
     int   avg_first = 1;

     for (int i = 1700 ; i < 2100 ; i=i+10){

    	float coef =  Goertzel_coef(i,  SAMPLE_RATE);


        TONE_ONE_MAG = Goertzel_mag(samples,depth,coef,avg);
        if (TONE_ONE_MAG > max) {
                max = TONE_ONE_MAG;
                max_id = i;
        }
        if ((i ==FREQ_ONE) || (i ==FREQ_TWO)){
//            Uart2_printf("[%d];", (int)TONE_ONE_MAG / 100);
        }else {
//            Uart2_printf("%d;", (int)TONE_ONE_MAG  / 100);
        }
        if (i == FREQ_ONE) {
            p1 = TONE_ONE_MAG;
        }
        if (i == FREQ_TWO) {
            p2 = TONE_ONE_MAG;
        }
        if (avg_first){
            avg_first = 0;
            sum += TONE_ONE_MAG;
        }

        sum += TONE_ONE_MAG;
        sum = sum /2;

     }
     {

      float snr1 = 100 * log10 ( p1 / sum );
      float snr2 = 100 * log10 ( p2 / sum );
       Uart2_printf("%d;%d",(int)snr1,(int)snr2 );
     }
      Uart2_printf("\r\n");

     //Uart2_printf("MAX FEQ %d\r\n",max_id);
    return max_id;
}



float Goertzel_coef(float freq, int samp_rate) {
  float coef = 2.0 * cos((2.0 * PI * freq) / samp_rate);
  return coef;
}

float Goertzel_mag(uint16_t samples[], int depth, float coef, int adc_midpoint) {
	float Q1 = 0;
	float Q2 = 0;
	for (int n = 0; n < depth; n++) {
		float Q0 = coef * Q1 - Q2 + (float)((float)samples[n] - adc_midpoint);
		Q2 = Q1;
		Q1 = Q0;
	}
	return sqrt(Q1 * Q1 + Q2 * Q2 - coef * Q1 * Q2);

}


int16_t Goertzel_coef_q15(float freq, int samp_rate)
{
    float w = (2.0f * (float)M_PI * freq) / (float)samp_rate;
    float c = 2.0f * cosf(w);

    return (int16_t)(c * 32768.0f);   // Q15
}

int32_t Goertzel_mag_fast_q15(const int16_t *samples,
                              int depth,
                              int16_t coef,
                              int32_t adc_mid)
{
    int32_t q0 = 0;
    int32_t q1 = 0;
    int32_t q2 = 0;

    for (int n = 0; n < depth; n++) {

        int32_t x = (int32_t)samples[n] - adc_mid;

        // 32bit * 16bit → 32bit (gyors)
        q0 = x + ((coef * q1) >> 15) - q2;

        q2 = q1;
        q1 = q0;
    }


    // magnitude² (csak itt 64bit → OK)
    int64_t mag =
          (int64_t)q1 * q1
        + (int64_t)q2 * q2
        - ((int64_t)coef * q1 >> 15) * q2;
    mag = sqrt((double)mag);

    if (mag < 0) mag = 0;
    if (mag > INT32_MAX) mag = INT32_MAX;

    return (int32_t)mag;
}
