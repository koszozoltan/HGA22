#ifndef __UTIL_H__
#define __UTIL_H__


#if 1
#define FREQ_ONE      (1730)
#define FREQ_TWO      (FREQ_ONE + 340)
#define TIMER3_FREQ   (135600-2000)
#endif

#define TIMER_FREQUENCY_HZ    (8000UL)          /// ADC mintavetelezesi frekvencia

#define SAMPLE_RATE TIMER_FREQUENCY_HZ


#ifndef MAX
    #define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif
#ifndef MIN
    #define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

#ifndef FLT_MAX
    #define FLT_MAX 3.402823466e+38F /* max value */
#endif
#ifndef FLT_MIN
    #define FLT_MIN 1.175494351e-38F /* min positive value */
#endif


#define SAMPLE_RATE TIMER_FREQUENCY_HZ

#define DEPTH         (ADC_CONVERTED_DATA_BUFFER_SIZE*4)        // 4 ms
#define OFFSET_BITS   5
#define BITBUF_SIZE   16





#if 0
#warning DCF49
#define FREQ_ONE      (1730)
#define FREQ_TWO      (FREQ_ONE + 340)
#define TIMER3_FREQ   (129100-2000)


#warning DCF39
#define FREQ_ONE      (1720)
#define FREQ_TWO      (FREQ_ONE+340)
#define TIMER3_FREQ   (139000-2000)

#endif


#define NOISE_FLOOR   5000
#ifndef PI
    # define PI           3.14159265358979323846  /* pi */
#endif

#ifndef ITEMNUM
    #define ITEMNUM(x)          (sizeof(x) / sizeof((x)[0]))
#endif




#endif /* __UTIL_H__ */
