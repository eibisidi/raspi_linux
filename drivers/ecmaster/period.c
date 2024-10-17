#include "ethercat.h"
#include "common.h"
#include "net.h"
#include "eck.h"
#include "eck_cdev.h"
#include "globals.h"
#include "ecbus.h"

#include <linux/delay.h>

uint32_t period_err;			//错误变量
uint32_t period_warn;			//报警变量
uint8_t  period_run;			//开始执行周期函数
uint32_t nrt_error;				//非实时线程产生错误
uint8_t  period_startout;		//从站PDO已经配置完成，可以发送过程数据
uint32_t period_counter;		//执行次数
int      period_wkc;			//过程数据wkc，每个周期更新
uint8_t	 period_op;				//已经进入OP模式
uint8_t	 period_skip_lrw;		//调试用不产生过程数据交互

#define SKIP_LRW			(1)
#define TOGGLE_OUTPUT		(0)

static void handle_decl(out_control_t	*ocb)
{
	uint16_t		j, axis;
	uint8_t			all_stopped = 1;

	int32_t			current_spd;

	decl_control_t *decl_cb;
	for (j = 0; j < ocb->data_info.slave_count; ++j)
	{
		axis = ocb->data_info.slave_data[j].slave_pos;
		decl_cb = &rt_lrw_decl_cbs[axis];

		if (1 == ocb->decl)
		{
			decl_cb->start_spd 	= GET_AXIS_TARGET_POS(axis) - decl_cb->last_target;
			decl_cb->current_spd = decl_cb->start_spd;
			decl_cb->spd_delta 	 = decl_cb->current_spd / CONFIG_ECAT_DECL_CYCLES;
			if (0 == decl_cb->spd_delta)
			{
				if (decl_cb->current_spd > 0)
					decl_cb->spd_delta = 1;
				if (decl_cb->current_spd < 0)
					decl_cb->spd_delta = -1;
			}
			// rt_printf("-------------axis = %u,start_spd = %d, current_spd = %d, spd_delta=%d\n",axis, decl_cb->start_spd, decl_cb->current_spd, decl_cb->spd_delta);
			// rt_printf("-------------last_target=%d, target=%d\n",decl_cb->last_target, GET_AXIS_TARGET_POS(axis));
		}

		current_spd = decl_cb->current_spd - decl_cb->spd_delta;
		if (decl_cb->start_spd > 0 && current_spd < 0)
			current_spd = 0;
		if (decl_cb->start_spd < 0 && current_spd > 0)
			current_spd = 0;

		SET_AXIS_TARGET_POS(axis, GET_AXIS_TARGET_POS(axis) + current_spd);
		decl_cb->current_spd = current_spd;

		//        if (j == ocb->data_info.slave_count - 1)
		//        {
		//            rt_printf("------target=%10d %10d \n", GET_AXIS_TARGET_POS(0), GET_AXIS_TARGET_POS(1));
		//        }
		if (0 != current_spd)
			all_stopped = 0;
	}

	if (ocb->decl == 1)
	{
		__sync_synchronize();
		ocb->out_state = OUT_DONE;
		END_OCB_CB(ocb);					//终止move_until_homed
	}
	++ocb->decl;

	if ( ocb->decl >= 2 * CONFIG_ECAT_DECL_CYCLES		//长时间未停下
	     || all_stopped)
	{
		//rt_printf("-----done, ocb->decl=%u\n. ", ocb->decl);
		__sync_synchronize();
		ocb->decl = 0;
	}
	return;
}

//由于内核不支持浮点数运算
//将轨迹计算转换为Scaled Integer运算
static int32_t syncmove_displace_t(const syncmove_cb_t *sync_move_ptr)
{
	int32_t q;
	uint64_t tms = sync_move_ptr->ts * (initconfig_dc_cycle_us / 1000);		//当前周期转换为毫秒数
	const syncmove_tparam_t  *tpara = &(sync_move_ptr->sync_move_para);
	if ( (tms >= 0)
	     && (tms < tpara->Ta))
	{
		q = (tpara->vmax * tms * tms ) / (tpara->tacc) / 2000;
	}
	else if ((tms >= tpara->Ta)
		 && (tms < (tpara->Ta + tpara->Tv)))
	{
		q = tpara->sacc + tpara->vlim * (tms - tpara->Ta) / 1000 / 1000;
	}
	else if ( (tms >= (tpara->Ta + tpara->Tv))
		  && (tms < tpara->T))
	{
		q = tpara->q1 - (tpara->vmax) * (tpara->T - tms) * (tpara->T - tms) / (tpara->tacc) / 2000;
	}
	else
	{
		q = tpara->q1;
	}

	if (tpara->negative )
		return -q;

	return  q;
}

static syncmove_state_t syncmove_fsm_wait_inpos(syncmove_cb_t *cb)
{
	if (SV_IN_POSITION(cb->axis))
	{
		SET_AXIS_REQ_CMD_POS(cb->axis, cb->dst_pos)		//更新轴命令请求位置
									//cb->enabled         = 0;
			cb->done_do_notify	= 1;
		return SYNCMOVE_STATE_DONE;
	}

	return SYNCMOVE_STATE_BUSY;
}

static syncmove_state_t syncmove_fsm_udpate_targetpos(syncmove_cb_t *cb)
{
	int32_t plan_pos =  syncmove_displace_t(cb);

	int32_t	target_pos = cb->base_position + plan_pos;
	SET_AXIS_TARGET_POS(cb->axis, target_pos)

		if(++cb->ts >= cb->sync_move_para.cycles)
		{
			cb->fsm_state = syncmove_fsm_wait_inpos;
			cb->dst_pos   = target_pos;
		}

	return SYNCMOVE_STATE_BUSY;
}

static void syncmove_rt_do(void)
{
	int i;
	uint8_t		sync_in = 0;
	syncmove_cb_t   *sync_move_ptr;
	for (i = 0; i < MAX_SYNC_MOVE_COUNT; ++i)
	{
		sync_move_ptr = &(syncmove_cbs[i]);
		if (0 == sync_move_ptr->paras_valid)   continue;
		IO_STATUS_GET_IN_BIT(sync_move_ptr->inbit, sync_in);

		if (sync_move_ptr->enabled == 0)
			goto UPDATE;

		if (SYNCMOVE_STATE_DONE == sync_move_ptr->state)
		{
			if (sync_move_ptr->req_disable)
			{//请求关闭同步运动使能
				sync_move_ptr->enabled = 0;
				sync_move_ptr->req_disable = 0;
				sync_move_ptr->dst_pos     = GET_AXIS_REQ_CMD_POS(sync_move_ptr->axis);
				sync_move_ptr->done_do_notify	= 1;
				goto UPDATE;
			}

			if (0 == sync_in
			    && sync_move_ptr->inbit_value != 0
			    && sync_move_ptr->done_do_notify == 0)
			{
				if (sync_move_ptr->sync_move_para.vmax != sync_move_ptr->editing_move_para.vmax
				    || sync_move_ptr->sync_move_para.tacc != sync_move_ptr->editing_move_para.tacc
				    || sync_move_ptr->sync_move_para.q1 != sync_move_ptr->editing_move_para.q1
				    || sync_move_ptr->sync_move_para.negative != sync_move_ptr->editing_move_para.negative)
				{//编辑区参数发生改变
					sync_move_ptr->sync_move_para.vmax 		= sync_move_ptr->editing_move_para.vmax;
					sync_move_ptr->sync_move_para.tacc 		= sync_move_ptr->editing_move_para.tacc;
					sync_move_ptr->sync_move_para.q1 		= sync_move_ptr->editing_move_para.q1;
					sync_move_ptr->sync_move_para.negative 	= sync_move_ptr->editing_move_para.negative;
					//计算用变量
					sync_move_ptr->sync_move_para.Ta 		= sync_move_ptr->editing_move_para.Ta;
					sync_move_ptr->sync_move_para.Tv 		= sync_move_ptr->editing_move_para.Tv;
					sync_move_ptr->sync_move_para.T 		= sync_move_ptr->editing_move_para.T;
					sync_move_ptr->sync_move_para.vlim 		= sync_move_ptr->editing_move_para.vlim;
					sync_move_ptr->sync_move_para.sacc 		= sync_move_ptr->editing_move_para.sacc;
					sync_move_ptr->sync_move_para.cycles 	= sync_move_ptr->editing_move_para.cycles;
				}

				sync_move_ptr->ts = 0;
				sync_move_ptr->base_position	= GET_AXIS_CUR_POS(sync_move_ptr->axis);
				sync_move_ptr->fsm_state       = syncmove_fsm_udpate_targetpos;
				sync_move_ptr->state			= SYNCMOVE_STATE_BUSY;
			}
		}

		if (SYNCMOVE_STATE_BUSY ==  sync_move_ptr->state)
		{
			if (rt_lrw_busy(sync_move_ptr->axis))
			{//运动冲突
				master_state->syncmove_alarm_conflict = 1;
			}
			syncmove_state_t new_state = sync_move_ptr->fsm_state(sync_move_ptr);
			sync_move_ptr->state     = new_state;
		}

UPDATE:
		sync_move_ptr->inbit_value = sync_in;
	}
}

static void do_outputs(void)
{
	int	i;
	out_control_t	*ocb;
	uint16_t		j, axis;
	uint16_t		data_offset;			//pdo data offset
	uint16_t		buffer_offset;			//offset with each buffer
	uint8_t 		*buffer_ptr;

	//TODO 内核不支持浮点运算
	//syncmove_rt_do();		//处理同步运动

	for (i = 0; i < initconfig_slave_count; ++i)
	{
		ocb = rt_lrw_out_cbs + i;

		if (CB_BUSY == ocb->cb_state)
			ocb->cb_func(&pending_requests[i]);

		if (ocb->decl)
		{
			handle_decl(ocb);
		}

		if (OUT_DONE == ocb->out_state) continue;

		for (j = 0; j < ocb->data_info.slave_count; ++j)
		{
			axis = ocb->data_info.slave_data[j].slave_pos;
			rt_lrw_decl_cbs[axis].last_target 	= GET_AXIS_TARGET_POS(axis);

			data_offset 	= ocb->data_info.slave_data[j].data_offset;
			buffer_offset	= (ocb->current_cycle) % ELEMENT_COUNT;
			buffer_ptr		= BUFFER_PTR(ocb->data_info.slave_data[j].slave_pos, ocb->buffer_index, buffer_offset);

			switch(ocb->data_info.slave_data[j].data_len)
			{
			case 1:
				EC_WRITE_U8(domain0_pd + data_offset, *((uint8_t *)(buffer_ptr)));
				break;
			case 2:
				EC_WRITE_U16(domain0_pd + data_offset, *((uint16_t *)(buffer_ptr)));
				break;
			case 4:
				EC_WRITE_U32(domain0_pd + data_offset, *((uint32_t *)(buffer_ptr)));
				break;
			case 8: 			//currently unsupported
			default:
				break;
				//assert(0);
			}
		}

		//rt_printf("axis(%d), [%4u/%4u], target=%d, current=%d, sw=0x%x\n", i, ocb->current_cycle, ocb->cycle_count, GET_AXIS_TARGET_POS(i), GET_AXIS_CUR_POS(i), GET_AXIS_STATUS_WORD(i));

		if (++(ocb->current_cycle) >= ocb->cycle_count)
		{
			ocb->data_info.slave_count = 0;
			__sync_synchronize();
			ocb->out_state = OUT_DONE;
		}
		else if (0 == (ocb->current_cycle % ELEMENT_COUNT) )
		{//switch buffer
			ocb->refill_index  = ocb->buffer_index;
			__sync_synchronize();
			ocb->buffer_index = (ocb->buffer_index + 1)%BUFFER_COUNT;
		}
	}
}

int32 axis0_target = 1;
uint8 positive = 0;
static void period_hook(void)
{
	int ecat_recv;

	period_run = 1;

	//发送出当前W5500发送缓存中的数据
	nicdrv_send_processdata();

	//从SideBuffer中取出一条报文进行发送
	nicdrv_send_side_buffer();
	///////////////////////////////////////////////////////////////////////////
	ecat_recv = nicdrv_recv_frames();

	if (period_startout)
	{
		period_wkc = nicdrv_receive_processdata();
		if (period_op)
		{
			ecbus_check(period_wkc);
		}

		do_outputs();

#if TOGGLE_OUTPUT
		if (positive)
		{
			SET_AXIS_TARGET_POS(0, axis0_target);
		}
		else
		{
			SET_AXIS_TARGET_POS(0, -axis0_target);
		}
		positive = !positive;
#endif

#if SKIP_LRW
		if (period_skip_lrw == 0)
			nicdrv_queue_processdata();		//将LRW+FRMV+FPWR报文写入W5500发送缓存
		else
			--period_skip_lrw;
#else
		nicdrv_queue_processdata();		//将LRW+FRMV+FPWR报文写入W5500发送缓存
#endif
	}
	else
		nicdrv_queue_sync_frame();

	++period_counter;
}

void period_start(void)
{
	period_startout = 0;
	period_err = 0;
	period_warn = 0;
	period_run = 0;
	nrt_error = 0;
	period_startout = 0;
	period_counter = 0;
	period_op	= 0;

	tick_sethook(period_hook);
}

void period_stop(void)
{
	tick_sethook(NULL);
	period_run = 0;
}

