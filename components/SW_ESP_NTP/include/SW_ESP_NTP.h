

#ifndef MAIN_NTP_H_
#define MAIN_NTP_H_

#include "esp_sntp.h"
#include <stdio.h>
#include <time.h>
//#include "RTC.h"
#include <sys/time.h>

void NTP_Init(void);
void NTP_GetTime();
//void time_sync_notification_cb(struct timeval *tv);
extern struct tm timep;

#endif /* MAIN_NTP_H_ */
