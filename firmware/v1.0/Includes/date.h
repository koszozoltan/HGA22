/*
 * date.h
 *
 *  Created on: Mar 5, 2026
 *      Author: development
 */

#ifndef APPLICATION_USER_DATE_H_
#define APPLICATION_USER_DATE_H_

extern volatile	uint64_t unix_us ;

uint64_t	get_unix_time_us	(void);
int			is_leap				(uint32_t y);
uint32_t	days_in_month		(uint32_t y, uint32_t m);
void 		unix_us_to_datetime	(uint64_t unix_us, datetime_t *dt);
uint64_t 	datetime_to_unix_us	(const datetime_t *dt, uint32_t usec);


#endif /* APPLICATION_USER_DATE_H_ */
