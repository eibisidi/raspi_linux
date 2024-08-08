/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include <osal.h>
#include <rtthread.h>
#include <rthw.h>
#include <gtimer.h>
#include <cpuport.h>
#include <sys/time.h>

#define  timercmp(a, b, CMP)                                \
  (((a)->tv_sec == (b)->tv_sec) ?                           \
   ((a)->tv_usec CMP (b)->tv_usec) :                        \
   ((a)->tv_sec CMP (b)->tv_sec))
#define  timeradd(a, b, result)                             \
  do {                                                      \
    (result)->tv_sec = (a)->tv_sec + (b)->tv_sec;           \
    (result)->tv_usec = (a)->tv_usec + (b)->tv_usec;        \
    if ((result)->tv_usec >= 1000000)                       \
    {                                                       \
       ++(result)->tv_sec;                                  \
       (result)->tv_usec -= 1000000;                        \
    }                                                       \
  } while (0)
#define  timersub(a, b, result)                             \
  do {                                                      \
    (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;           \
    (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;        \
    if ((result)->tv_usec < 0) {                            \
      --(result)->tv_sec;                                   \
      (result)->tv_usec += 1000000;                         \
    }                                                       \
  } while (0)

#define USECS_PER_SEC   1000000
#define USECS_PER_TICK  (USECS_PER_SEC / RT_TICK_PER_SECOND)

#if 0
/* Workaround for rt-labs defect 776.
 * Default implementation of udelay() didn't work correctly when tick was
 * shorter than one millisecond.
 */
void udelay (uint32_t us)
{
   tick_t ticks = (us / USECS_PER_TICK) + 1;
   task_delay (ticks);
}

/*
	conflicting types for 'gettimeofday'
	previous declaration of 'gettimeofday' was here
	int gettimeofday(struct timeval *tv, struct timezone *tz);
*/
int gettimeofday(struct timeval *tp, void *tzp)
{
   tick_t tick = tick_get();
   tick_t ticks_left;

   ASSERT (tp != NULL);

   tp->tv_sec = tick / CFG_TICKS_PER_SECOND;

   ticks_left = tick % CFG_TICKS_PER_SECOND;
   tp->tv_usec = ticks_left * USECS_PER_TICK;
   ASSERT (tp->tv_usec < USECS_PER_SEC);

   return 0;
}
#endif

int osal_usleep (uint32 usec)
{
//husytodo 使用RT_DEBUG_NOT_IN_INTERRUPT非中断环境调用
	rt_hw_us_delay(usec);
	return 0;
}

int osal_gettimeofday(struct timeval *tv, struct timezone *tz)
{
	//husytodo 使用RT_DEBUG_NOT_IN_INTERRUPT非中断环境调用

	rt_uint64_t cntpct, us;
	cntpct = rt_hw_get_cntpct_val();
	rt_hw_dsb();
	us = rt_hw_cntpct2us(cntpct);

	tv->tv_sec = us / 1000000;
	tv->tv_usec = us % 1000000;

   	return 0;
}

ec_timet osal_current_time (void)
{
   struct timeval current_time;
   ec_timet return_value;

   osal_gettimeofday (&current_time, 0);
   return_value.sec = current_time.tv_sec;
   return_value.usec = current_time.tv_usec;
   return return_value;
}

uint64 osal_current_time_ns()
{
	uint64 ns = rt_hw_get_current_ns();
	//2023-11-17 00:00:00  1700150400000 ms
	//2000-01-01 00:00:00  946684800000ms
	uint64 base = 753465600000000ULL;
	return ns + base;  
}

void osal_timer_start (osal_timert * self, uint32 timeout_usec)
{
   struct timeval start_time;
   struct timeval timeout;
   struct timeval stop_time;

   osal_gettimeofday(&start_time, 0);
   timeout.tv_sec = timeout_usec / USECS_PER_SEC;
   timeout.tv_usec = timeout_usec % USECS_PER_SEC;
   timeradd (&start_time, &timeout, &stop_time);

   self->stop_time.sec = stop_time.tv_sec;
   self->stop_time.usec = stop_time.tv_usec;
}

boolean osal_timer_is_expired (osal_timert * self)
{
   struct timeval current_time;
   struct timeval stop_time;
   int is_not_yet_expired;

   osal_gettimeofday (&current_time, 0);
   stop_time.tv_sec = self->stop_time.sec;
   stop_time.tv_usec = self->stop_time.usec;
   is_not_yet_expired = timercmp (&current_time, &stop_time, <);

   return is_not_yet_expired == FALSE;
}

void *osal_malloc(size_t size)
{
   return rt_malloc(size);
}

void osal_free(void *ptr)
{
   rt_free(ptr);
}

int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
	rt_kprintf("Function[%s] shall not be used.\n", __FUNCTION__);											  \
	RT_ASSERT(0)
   	return FALSE;
}

int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
	rt_kprintf("Function[%s] shall not be used.\n", __FUNCTION__);											  \
	RT_ASSERT(0)
   	return FALSE;
}
