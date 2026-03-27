#ifndef __AGC_H__
#define __AGC_H__


int  Get_HW_Agc(void);


void cmd_agc_power(int range);
void AGC_HW_process(int range);
void AGC_SW_process(int16_t* buffer, int depth , int range , int avg);


#endif /* __AGC_H__ */
