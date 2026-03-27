
#ifndef APPLICATION_USER_IQ_H_
#define APPLICATION_USER_IQ_H_

#define TABLE_SIZE 1024
#define Q15_SCALE 32768


typedef struct {

    int32_t phase;        // rad * 32768 (Q15 típusú fázisérték)
    int32_t i_filt;       // Q15
    int32_t q_filt;       // Q15
    int32_t prev_i;       // Q15
    int32_t prev_q;       // Q15

    int32_t sampleRate;   // integer (Hz)
    int32_t mixFreq;      // integer (Hz)

    int32_t lpfAlpha;     // Q15
    int32_t omega;        // rad * 32768 per sample (Q15)

} iq_st;

extern iq_st iq_1;
extern iq_st iq_2;


typedef struct {
    uint16_t freq;     // keverési frekvencia (Hz)
    int32_t omega_q15; // Q15 rad/sample
    int32_t alpha_q15; // Q15 LPF alpha
} mix_const_t;


void iq_st_init(iq_st *iq, uint16_t mixFreq);
void initSinCosTable(void);
int16_t scale_to_x15_int(int32_t x, int32_t min, int32_t max, int64_t range );
int32_t iq_power_q15(int16_t sample, iq_st* st);



#endif /* APPLICATION_USER_IQ_H_ */
