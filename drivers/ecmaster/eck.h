#ifndef ECK_H
#define ECK_H

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/completion.h>

#include "eck_cdev.h"
#include "common.h"

typedef struct ECK{
	unsigned int 	index;
	eck_cdev_t		cdev;
	struct device 	*class_device; /**< Master class device. */
	struct semaphore sem;
	struct completion linkup;					//W5500 链路UP

	bool			rt_task_valid;

	bool			mapped;
	master_state_t	*master_state;
	unsigned long 	master_state_size;

	out_control_t	*out_cbs;					//实时输出控制块
	unsigned long	out_cbs_size;

	out_buffer_t	*out_buffers;				//实时输出缓存
	unsigned long	out_buffers_size;

	outcb_func_t	*out_cb_funcs;				//实时输出回调函数列表
	unsigned long	out_cb_funcs_size;

	request_t		*pending_requests;			//请求处理控制块
	unsigned long	pending_requests_size;

	RequestState	*request_states;			//请求状态
	unsigned long	request_states_size;

	decl_control_t	*decl_cbs;					//减速停止控制块
	unsigned long	decl_cbs_size;

	syncmove_cb_t   *syncmove_cbs;      		//同步运动控制块
	unsigned long	syncmove_cbs_size;

	axis_state_t 	*axis_states;
	unsigned long	axis_states_size;

	slave_xml_config_t 	*slave_xml_configs;
	unsigned long		slave_xml_configs_size;

	physical_unit_t		*initconfig_physical_unit;
	unsigned long		initconfig_physical_unit_size;

	period_t		*period_struct;
	unsigned long	period_struct_size;

	device_stats_t	*ndev_stats;
	unsigned long	ndev_stats_size;

	void 			*process_data;
	unsigned long 	process_data_size;
}eck_t;

#endif
