#ifndef __DECODER_H__
#define __DECODER_H__

#include "queue.h"



typedef struct {
    uint8_t shift;
    uint8_t cnt;
} bit_to_byte_state_t;


typedef struct {

    uint8_t idx;
    uint8_t state;
    uint8_t byte;
    uint8_t parity;
    uint8_t start;
    uint8_t bit_cnt;
    uint8_t send;

    uint8_t start_zero_count;


    stream_q_t q_fsk;
    stream_q_t q_byte;
    bit_to_byte_state_t byte_state;

} DecoderState;


extern int max_snr1;
extern int max_snr2;
extern int min_snr1;
extern int min_snr2;


uint8_t bit_to_byte_st(uint8_t bit, uint8_t *out, bit_to_byte_state_t *st);


void    decoder_init        (DecoderState* st);
void    decode_loop         (DecoderState* st, uint8_t bit);
void    process_goertzel    (DecoderState* st, int16_t* buffer, int depth);
void    process_iq          (DecoderState* st, int16_t* buffer, int depth);
void    init_demods         (void);


void    loop                (int16_t* buffer, int depth);

void    parse_frame         (uint8_t *frame, uint8_t idx);


#endif /* __DECODER_H__ */
