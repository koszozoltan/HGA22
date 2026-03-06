/*
 * date.c
 *
 *  Created on: Mar 5, 2026
 *      Author: development
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include "queue.h"
#include "math.h"
#include "ringbuffer.h"
#include "tcp_server.h"
#include "arm_math.h"
#include "date.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

static const uint8_t days_in_month_tbl[12] =
{
    31,28,31,30,31,30,31,31,30,31,30,31
};

uint64_t get_unix_time_us(void){
	return unix_us;
}

int is_leap(uint32_t y)
{
    return ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
}

uint32_t days_in_month(uint32_t y, uint32_t m)
{
    if (m == 2 && is_leap(y)) return 29;
    return days_in_month_tbl[m-1];
}

/* =========================
   unix_us -> datetime
   ========================= */
void unix_us_to_datetime(uint64_t unix_us, datetime_t *dt)
{
    uint64_t sec = unix_us / 1000000ULL;

    dt->sec = sec % 60; sec /= 60;
    dt->min = sec % 60; sec /= 60;
    dt->hr  = sec % 24; sec /= 24;

    // sec most napok száma 1970 óta
    uint32_t days = (uint32_t)sec;

    uint32_t y = 1970;

    while (1)
    {
        uint32_t dy = is_leap(y) ? 366 : 365;
        if (days >= dy)
        {
            days -= dy;
            y++;
        }
        else break;
    }

    dt->yr = y;

    uint32_t m = 1;
    while (1)
    {
        uint32_t dm = days_in_month(y, m);
        if (days >= dm)
        {
            days -= dm;
            m++;
        }
        else break;
    }

    dt->mon = m;
    dt->day = days + 1;
}

/* =========================
   datetime -> unix_us
   ========================= */
uint64_t datetime_to_unix_us(const datetime_t *dt, uint32_t usec)
{
    uint64_t days = 0;

    for (uint32_t y = 1970; y < dt->yr; y++)
        days += is_leap(y) ? 366 : 365;

    for (uint32_t m = 1; m < dt->mon; m++)
        days += days_in_month(dt->yr, m);

    days += (dt->day - 1);

    uint64_t sec =
        days * 86400ULL +
        dt->hr * 3600ULL +
        dt->min * 60ULL +
        dt->sec;

    return sec * 1000000ULL + usec;
}
