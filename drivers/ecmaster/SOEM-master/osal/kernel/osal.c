/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include <osal.h>
#include <linux/preempt.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/kthread.h>

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
extern u64 tick_get_current_ns(void);
#else
u64 tick_get_current_ns(void)
{
	return 0;
}
#endif

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
	WARN_ON(in_interrupt());
	fsleep(usec);
	return 0;
}

struct timeval {
	__kernel_old_time_t	tv_sec;		/* seconds */
	__kernel_suseconds_t	tv_usec;	/* microseconds */
};

static int osal_gettimeofday(struct timeval *tv, struct timezone *tz)
{
	uint64 now_ns = tick_get_current_ns();

	WARN_ON(in_interrupt());

	tv->tv_sec  = now_ns / 1000000000;
	tv->tv_usec = (now_ns % 1000000000) / 1000000;

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
	uint64 ns = tick_get_current_ns();
	return ns;
	//2023-11-17 00:00:00  1700150400000 ms
	//2000-01-01 00:00:00  946684800000ms
	//uint64 base = 753465600000000ULL;
	//return ns + base;  
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
	pr_err("Function[%s] shall not be used.\n", __FUNCTION__);
	WARN_ON(1);
   	return NULL;
}

void osal_free(void *ptr)
{
	pr_err("Function[%s] shall not be used.\n", __FUNCTION__);
	WARN_ON(1);
}

int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
	WARN_ON(in_interrupt());
	OSAL_THREAD_HANDLE new_kt= kthread_run(func, param, "osalkthread");
	if (IS_ERR(new_kt))
	{
		pr_err("kthread_run() failed!\n");
		return FALSE;
	}

	*((OSAL_THREAD_HANDLE * )thandle) = new_kt;

   	return FALSE;
}

int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
	pr_err("Function[%s] shall not be used.\n", __FUNCTION__);
	WARN_ON(1);
   	return FALSE;
}
