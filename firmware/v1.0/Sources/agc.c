#include "main.h"
#include <string.h>
#include "util.h"

#include "queue.h"
#include "math.h"
#include "ringbuffer.h"
#include "tcp_server.h"
#include "arm_math.h"
#include "adc.h"
#include "agc.h"



static int gain = 0;


void cmd_agc_power(int range) {

    //Uart2_printf("AGC - %d\r\n", range);

    switch (range) {
        case 0 : {
           AGC1_high();
           AGC2_high();
           AGC3_high();
           AGC4_high();
        } break;
        case 1 : {
           AGC1_high();
           AGC2_high();
           AGC3_high();
           AGC4_low();
        } break;
        case 2 : {
           AGC1_high();
           AGC2_high();
           AGC3_low();
           AGC4_low();
        } break;
        case 3 : {
           AGC1_high();
           AGC2_low();
           AGC3_low();
           AGC4_low();
        } break;
        case 4 : {
           AGC1_low();
           AGC2_low();
           AGC3_low();
           AGC4_low();
        } break;
        default : {
        }
    }
}

int Get_HW_Agc(void){
    return gain;
}

void AGC_HW_process(int range) {
    //static int gain = 0;

    static int range_avg = 0;
    static int delay = 0;
    static int start = 1;

    if (start){
        start = 0;
        range_avg = range;

        if (GETBIT(agc,0) == 0) {
            gain  = 4;
        } else
        if (GETBIT(agc,1) == 0) {
            gain  = 3;
        }  else
        if (GETBIT(agc,2) == 0) {
            gain  = 2;
        }
        else
        if (GETBIT(agc,3) == 0) {
            gain  = 1;
        }
    }
    range_avg = range;
    range_avg /=  2;

    if (agc_disable == 1) {
        delay = -59000;
        agc_disable = 0;
    }


    if (++delay < 10000) return;

    datetime_t dt;
	uint64_t now  = get_unix_time_us();
	unix_us_to_datetime(now, &dt);
    if (dt.sec % 10 != 3) return;

    delay = 0;

     if (range_avg > 2000) {
        if (gain > 0) gain --;
     } else

     if (range_avg <  1000) {
        if (gain < 4) gain ++;
     }

    //Uart2_printf("AGC_process range %d, agc %d, ",range_avg,gain);

    cmd_agc_power(gain);


}


void AGC_SW_process(int16_t* buffer, int depth , int range , int avg) {

    uint8_t shift = 0;
    if (range > 0) {
        if (range < 256)       shift = 4; // ~16x erõsítés
        else if (range < 512)  shift = 3; // ~8x erõsítés
        else if (range < 1024) shift = 2; // ~4x erõsítés
        else if (range < 2048) shift = 1; // ~2x erõsítés
        else                   shift = 0; // ~1x erõsítés
    }

    for(int i = 0; i < depth; i++) {
        buffer[i] = (int16_t)((buffer[i] - avg) << shift);
    }

}
