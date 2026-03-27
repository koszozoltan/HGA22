
#include <string.h>
#include <math.h>
#include <limits.h>

#include "main.h"
#include "agc.h"
#include "goertzel.h"
#include "iq.h"
#include "decoder.h"
#include "util.h"
#include "ringbuffer.h"
#include "date.h"

// ============================================================================
//  DECODER STRUCT + INIT
// ============================================================================



DecoderState goertzel_dec = {
 .idx = 0,
};
DecoderState iq_dec = {
 .idx = 1,
};

int max_snr1 =0;
int max_snr2 =0;
int min_snr1 =0;
int min_snr2 =0;




uint8_t bit_to_byte_st(uint8_t bit, uint8_t *out, bit_to_byte_state_t *st)
{
    // shift update
    st->shift = (st->shift >> 1) | (bit ? 0x80 : 0);

    // counter update
    st->cnt++;

    // completed byte?
    if (st->cnt == 8) {
        *out = st->shift;
        st->cnt = 0;
        st->shift = 0;
        return 1;
    }

    return 0;
}

uint8_t bit_to_byte(uint8_t bit, uint8_t *out) {
  static uint8_t shift = 0;
  static uint8_t cnt = 0;

  shift = (shift >> 1) | (bit ? 0x80 : 0);
  cnt++;

  if (cnt == 8) {
    *out = shift;
    cnt = 0;
    shift = 0;
    return 1;
  }
  return 0;
}

uint32_t getBits(const uint8_t *data, uint32_t pos, uint32_t len) {
  uint32_t v = 0;
  for (uint32_t i = 0; i < len; i++) {
    uint8_t b = (data[(pos+i)/8] >> ((pos+i)%8)) & 1;
    v |= (b << i);
  }
  return v;
}



void decoder_init(DecoderState* st) {
    st->state = 0;
    st->byte = 0;
    st->parity = 0;
    st->start = 0;
    st->bit_cnt = 0;
    st->byte_state.shift = 0;
    st->byte_state.cnt = 0;
}

// ============================================================================
//  COMMON BIT DECODER — USED BY BOTH GOERTZEL AND IQ DEMOD
// ============================================================================

void decode_loop(DecoderState* st, uint8_t bit)
{

    // ---------------- Majority filter 5 mintával ----------------
    int16_t bit_int = bit;
    sq_push(&st->q_fsk, (int16_t*)&bit_int, 1);

    if (sq_size(&st->q_fsk) < 5)
    return;

    uint16_t bits[5] = {0};
    sq_peek(&st->q_fsk, (int16_t*)bits, 5);
    sq_discard(&st->q_fsk, 1);

    uint8_t sum = 0;
    for (int i = 0; i < 5; i++)
        sum += bits[i];

    bit = (sum >= 3) ? 1 : 0;


    // ----------- Startbit keresése -----------
    if (st->start == 0) {

        if (bit == 0) {


             if (++st->start_zero_count >= 3) {
                st->start = 1;
                st->send = 5;
                st->bit_cnt = 0;
                st->start_zero_count = 0;   // indulás után lenullázható
            } else {
                return;
            }


        } else {
            st->start_zero_count = 0;
            return;
        }
    }

    if (++st->send < 5)
        return;

    st->send = 0;

    switch (st->state) {

        case 0:{ // START
            if (bit == 0) {
                st->state = 1;
                st->byte = 0;
                st->send = 0;
            }
            }break;

        case 1:{// DATA bitek
            if (bit_to_byte_st(bit, &st->byte,&st->byte_state)) {
                st->state = 2;
            }
            }break;

        case 2:{ // PARITY
            st->parity = __builtin_parity(st->byte);
            if (st->parity == bit)
                st->state = 3;
            else {

                decoder_init(st);

            }
            }break;

        case 3:{ // STOP
            if (bit == 1) {

                uint16_t data = st->byte;
                sq_push(&st->q_byte, (int16_t*)&data, 1);

                // ------------ frame összerakása ------------
                while (sq_size(&st->q_byte) >= 16) {

                    int16_t raw[16];
                    uint8_t bytes[16];

                    sq_peek(&st->q_byte, raw, 16);
                    sq_discard(&st->q_byte, 1);

                    for (int i = 0; i < 16; i++)
                        bytes[i] = raw[i];

                    parse_frame(bytes,st->idx);
                }
            }

            // reset
            decoder_init(st);
            }
            break;
        default :{
            decoder_init(st);
        }
    }

    if (++st->bit_cnt >= 11) {
        decoder_init(st);
    }
}

// ============================================================================
//  GOERTZEL DEMODULATION → decode_loop()
// ============================================================================

void process_goertzel(DecoderState* st, int16_t* buffer, int depth)
{
    int32_t m1 = Goertzel_mag_fast_q15(buffer, depth, coef1, 0);
    int32_t m2 = Goertzel_mag_fast_q15(buffer, depth, coef2, 0);


    max_snr1 = MAX((int)m1,max_snr1);
    max_snr2 = MAX((int)m2,max_snr2);
    min_snr1 = MIN((int)m1,min_snr1);
    min_snr2 = MIN((int)m2,min_snr2);

    uint8_t bit = (m1 > m2) ? 1 : 0;

    decode_loop(st, bit);
}

// ============================================================================
//  IQ DEMODULATION → decode_loop()
// ============================================================================

void process_iq(DecoderState* st, int16_t* buffer, int depth)
{
    int64_t sum1 = 0;
    int64_t sum2 = 0;

    for (int i = 0; i < depth; i++) {
        sum1 += iq_power_q15(buffer[i], &iq_1);
        sum2 += iq_power_q15(buffer[i], &iq_2);
    }

    uint8_t bit = (sum1 > sum2) ? 1 : 0;

    decode_loop(st, bit);
}

// ============================================================================
//  MAIN LOOP — RUN BOTH IN PARALLEL
// ============================================================================

void loop(int16_t* buffer, int depth) {
    min = INT_MAX;
    max = INT_MIN;

    // ------------------------ Min/max ------------------------
    for (int i = 0; i < depth; i++) {
        if (buffer[i] < min) min = buffer[i];
        if (buffer[i] > max) max = buffer[i];
    }

    int32_t  range = max - min;
    int32_t  avg = (max + min) / 2;
    int16_t  scaled_buffer[depth];

    // ------------------------ AGC ------------------------
    AGC_HW_process(range);

    for(int i = 0; i < (depth/4) ; i++) {
        scaled_buffer[i] = scale_to_x15_int(buffer[i], min,max,range);
    }
    AGC_SW_process(buffer, depth, range, avg);



    // ------------------------ FSK demodok párhuzamosan ------------------------

    process_goertzel(&goertzel_dec, buffer, depth);
    process_iq(&iq_dec, scaled_buffer, depth/4);


}

// ============================================================================
//  INIT CALL (call once in setup())
// ============================================================================

void init_demods(void){



    coef1 =  Goertzel_coef_q15(FREQ_ONE,SAMPLE_RATE);
    coef2 =  Goertzel_coef_q15(FREQ_TWO,SAMPLE_RATE);


    iq_st_init(&iq_1,FREQ_ONE);
    iq_st_init(&iq_2,FREQ_TWO);


    static int16_t goertzel_byte_buffer[32];
    static int16_t goertzel_fsk[8];
    static int16_t iq_byte_buffer[32];
    static int16_t iq_fsk[8];

    decoder_init(&goertzel_dec);
    sq_init(&goertzel_dec.q_byte, (int16_t*)goertzel_byte_buffer, 32);
    sq_init(&goertzel_dec.q_fsk,  (int16_t*)goertzel_fsk, 8);

    decoder_init(&iq_dec);
    sq_init(&iq_dec.q_byte, (int16_t*)iq_byte_buffer, 32);
    sq_init(&iq_dec.q_fsk,  (int16_t*)iq_fsk, 8);
}

void parse_frame(uint8_t *frame, uint8_t idx) {

  if (frame[0]!=0x68 || frame[1]!=0x0A || frame[2]!=0x0A || frame[3]!=0x68)
    return;

  uint8_t sum = 0;
  for (int i=4;i<14;i++) sum += frame[i];
  if (frame[14]!=sum || frame[15]!=0x16) return;



  for (int i = 0; i < 16; i++){
    Uart2_printf("0x");
    if (frame[i]<16){
        Uart2_printf("0");
    }
    Uart2_printf("%X,",frame[i]);
  }

  uint8_t *d = &frame[7];

//  uint32_t ms  = getBits(d,  0,10);
  uint32_t sec = getBits(d, 10, 6);
  uint32_t min = getBits(d, 16, 6);
  uint32_t hr  = getBits(d, 24, 5);
  uint32_t day = getBits(d, 32, 5);
  uint32_t mon = getBits(d, 40, 4);
  uint32_t yr  = getBits(d, 48, 7);
  uint8_t  dst = getBits(d, 31, 1);

  Uart2_printf("[%d]",idx);
  Uart2_printf("TIME: 20");
  Uart2_printf("%d",yr);
  Uart2_printf("-");
  Uart2_printf("%s",mon<10?"0":"");
  Uart2_printf("%d",mon);
  Uart2_printf("-");
  Uart2_printf("%s",day<10?"0":"");
  Uart2_printf("%d",day);
  Uart2_printf(" ");
  Uart2_printf("%s",hr<10?"0":"");
  Uart2_printf("%d",hr);
  Uart2_printf(":");
  Uart2_printf("%s",min<10?"0":"");
  Uart2_printf("%d",min);
  Uart2_printf(":");
  Uart2_printf("%s",sec<10?"0":"");
  Uart2_printf("%d",sec);
  Uart2_printf("[DST %d] HWAGC[%d]\r\n",dst,Get_HW_Agc());

  datetime_t dt;

  dt.yr = yr+2000;
  dt.mon = mon;
  dt.day = day;
  dt.hr  = hr;
  dt.min = min;
  dt.sec = sec;

  uint64_t now = datetime_to_unix_us(&dt, 0);

  /* RTC -> dt mezők feltöltése */
  NVIC_DisableIRQ(SysTick_IRQn);
  unix_us = now;
  NVIC_EnableIRQ(SysTick_IRQn);


  if (send_array) {
	  send_array = 0;

  #if 0

	  printf("\r\nDATA [%d]:\r\n",sq_size(&q_voice));
	  while(1){
	 	if (sq_size(&q_voice) > 0){
	      uint16_t buffer[32];
	 	  sq_peek(&q_voice,buffer,32);
	 	  sq_discard(&q_voice,32);
		  for (int i = 0;i < 32; i++){
			  printf("%d;",buffer[i]);
			  printf("\r\n");
		  }
		  //printf("\r\n");
	 	} else {
	 		break;
	 	}
	  }

#endif
  }

}
