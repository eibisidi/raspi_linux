#include <linux/types.h>
#include <linux/page-flags.h>
#include <linux/semaphore.h>

#include "eck.h"
#include "fops.h"

int eck_file_open(eck_t *eck)
{
#if 0
	//等待W5500链路UP，初始化完成
	if (wait_for_completion_interruptible(&eck->linkup))
		return -EINTR;

	if (period_err & PERIOD_ESETUPNIC)	 	//PERIOD_ESETUPNIC被设置，W5500初始化失败
		return -EIO;

	memset(master_state, 0, sizeof(master_state_t));
#endif
	return 0;
}

void eck_file_close(eck_t *eck)
{
#if 0
	memset(master_state, 0, sizeof(master_state_t));
	period_stop();

	eck->mapped = false;
#endif
	return;
}

