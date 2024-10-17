#ifndef ECK_GLOBALS
#define ECK_GLOBALS

#include "common.h"
#include "osal.h"
#include "net.h"

#include <linux/wait.h>

#define MEASURE_TIMING
//#define LOGPOINTS
/****************************************************************************/

#ifndef UNUSED
#define UNUSED(a) (void)(a)
#endif

#define ECK_INFO(fmt, args...) \
	printk(KERN_INFO "%s:%d ECK: " fmt, __FILE__, __LINE__, ##args)

#define ECK_ERR(fmt, args...) \
	printk(KERN_ERR "%s:%d ECK ERROR: " fmt, __FILE__, __LINE__, ##args)

#define ECK_WARN(fmt, args...) \
	printk(KERN_WARNING "%s:%d ECK WARNING: " fmt, __FILE__, __LINE__, ##args)

#define ECK_DBG(fmt, args...) \
	do { \
		printk(KERN_ERR "%s:%d ECK DEBUG: " fmt, __FILE__, __LINE__, \
##args); \
	} while (0)

		struct ECK;
		typedef struct ECK eck_t;

		enum{
			NRT_ERR_OK = 0,
			NRT_ERR_SIDERING_FULL = (1 << 0),
			NRT_ERR_CONFIRM_TIMEOUT = (1 << 1),
		};

enum {
	PERIOD_EOK 	     = 0x00,
	PERIOD_ERECV     = (1 << 0),		//读邮箱错误
	PERIOD_EOV       = (1 << 1),
	PERIOD_EMBOXFULL = (1 << 2),		//邮箱满
	PERIOD_ESTIMER	 = (1 << 3),		//STIMER间隔时间过长
	PERIOD_ESCAN	 = (1 << 4),		//扫描从站数目不正确
	PERIOD_EPREOP	 = (1 << 5),		//进入PREOP超时
	PERIOD_EDC		 = (1 << 6),		//计算DC相关寄存器错误
	PERIOD_ESDO		 = (1 << 7),		//初始化SDO下载失败
	PERIOD_EPDO		 = (1 << 8),		//配置PDO失败
	PERIOD_ESYNC0	 = (1 << 9),		//SYNC0使能失败
	PERIOD_ESAFEOP   = (1 << 10),		//进入SAFEOP超时
	PERIOD_EOP		 = (1 << 11),		//进入OP超时
	PERIOD_ESETUPNIC = (1 << 12),		//初始化网卡失败
	PERIOD_SLOW		 = (1 << 13),		//处理过慢
	PERIOD_SLOW2     = (1  << 14),
	PERIOD_BWR900	 = (1 << 15),
};

enum{
	PERIOD_W_OK = 0,
	PERIOD_WRECV_TIMO = (1 << 0),   	//报文接收超时
	PERIOD_WNOTSYNC  = (1 << 1),		//站时钟长时间未同步
};

void set_nrt_error(uint32_t e);

#if 0
void tick_sethook(void (*hook)(void));
void *tick_gethook(void);
u64 tick_get_expected_ns(void);
#else
static inline void tick_sethook(void (*hook)(void)) {}
static inline void *tick_gethook(void) {return NULL;}
static inline u64 tick_get_expected_ns(void) {return 0;}
#endif

void period_start(void);
void period_stop(void);
void slaveinfo(void);
void slaveinfo_readonly(void);
void eck_rt_init_out_cb_funcs(eck_t *eck);

extern uint32_t period_err;
extern uint32_t period_warn;
extern uint8_t  period_run;
extern uint32_t nrt_error;
extern uint8_t  period_startout;
extern uint32_t period_counter;
extern int      period_wkc;
extern uint8_t	period_op;
extern uint8_t	period_skip_lrw;

//Timer related global variables defined in kernel/time/tick-common.c
#if 0
extern	u64 expected_cnt;
extern	s64 jitter_max;
extern	s64 jitter_min;
extern s64 real_times;
extern s64 hookcost_max;
extern u32 timer_step;
extern u8 overwrite_spsr_el1;
#else
static	u64 expected_cnt;
static	s64 jitter_max;
static	s64 jitter_min;
static s64 real_times;
static s64 hookcost_max;
static u32 timer_step;
static u8 overwrite_spsr_el1;
#endif

extern wait_queue_head_t generic_timer_wait_queue;
extern u64 generic_timer_wait_queue_flag;

#endif
