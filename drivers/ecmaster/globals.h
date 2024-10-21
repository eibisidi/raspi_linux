#ifndef ECK_GLOBALS
#define ECK_GLOBALS

#include "common.h"

#include <linux/wait.h>

#define MEASURE_TIMING
//#define LOGPOINTS
/****************************************************************************/

#ifndef UNUSED
#define UNUSED(a) (void)(a)
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "%s %s:%d " fmt, KBUILD_MODNAME, __FILE__, __LINE__

#if 0
#define pr_info(fmt, args...) \
	printk(KERN_INFO "%s:%d ECK: " fmt, __FILE__, __LINE__, ##args)
#define pr_err(fmt, args...) \
	printk(KERN_ERR "%s:%d ECK ERROR: " fmt, __FILE__, __LINE__, ##args)

#define pr_warn(fmt, args...) \
	printk(KERN_WARNING "%s:%d ECK WARNING: " fmt, __FILE__, __LINE__, ##args)

#define pr_dbg(fmt, args...) \
	do { \
		printk(KERN_ERR "%s:%d ECK DEBUG: " fmt, __FILE__, __LINE__, \
##args); \
	} while (0)
#endif

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


void period_start(void);
void period_stop(void);
void slaveinfo(void);
void slaveinfo_readonly(void);
void eck_rt_init_out_cb_funcs(eck_t *eck);


#endif
