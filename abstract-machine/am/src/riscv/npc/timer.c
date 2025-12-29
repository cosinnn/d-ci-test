#include <am.h>
#include "../riscv.h"
void __am_timer_init() {
}

void __am_timer_uptime(AM_TIMER_UPTIME_T *uptime) {
  uint32_t mcycle;
  uint32_t mcycleh;
  uint64_t res;
  asm volatile ("csrr %0, 2816" : "=r"(mcycle) :  : );
  asm volatile ("csrr %0, 2944" : "=r"(mcycleh) :  : );
  res = mcycleh;
  uptime->us = ((res<<32)|mcycle)*5000;
}

void __am_timer_rtc(AM_TIMER_RTC_T *rtc) {
  rtc->second = 0;
  rtc->minute = 0;
  rtc->hour   = 0;
  rtc->day    = 0;
  rtc->month  = 0;
  rtc->year   = 1900;
}
