#include "globals.h"
#include "eck.h"

#define __ENTRY__ 

void out_cb_test(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	ECK_DBG("--------xxxxxxxxxxxx, slave=%u.", req->slave_pos);

	END_OCB_CB(ocb);
}

void static request_cbfunc_svon_delay(void *priv_data)
{
	request_t		*req		= (request_t *)priv_data;
	out_control_t	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);

	if (++req->u.svon_priv.delay_cycles >= 20)
	{
		ocb->cb_return	= 0;
		END_OCB_CB(ocb);
	}
}

void static request_cbfunc_svon_do(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		status_word = GET_AXIS_STATUS_WORD(req->slave_pos);
	uint16_t		control_word = 0;

	if (SV_ENABLED(status_word))
	{
		if (OPMODE_CSP == GET_SLAVE_XML_OPMODE(req->slave_pos))
		{
			uint32_t	cur_pos = GET_AXIS_CUR_POS(req->slave_pos);
			SET_AXIS_TARGET_POS(req->slave_pos, cur_pos);			//init target_pos to current pos
			SET_AXIS_REQ_CMD_POS((req->slave_pos), cur_pos);
		}
		else if (OPMODE_CST == GET_SLAVE_XML_OPMODE(req->slave_pos))
		{
			SET_AXIS_TARGET_TORQUE(req->slave_pos, 0);				//目标扭矩设置为0
		}

		req->u.svon_priv.delay_cycles = 0;
		CONTINUE_OCB_CB(ocb, request_cbfunc_svon_delay);

		return;
	}

	if (SV_FAULT(status_word))
	{
		control_word = 0x80;
	}
	else if (SV_DISABLED(status_word))
	{
		control_word = 0x06;
	}
	else if(SV_RDY_TO_SWITCH_ON(status_word))
	{
		control_word = 0x07;
	}
	else if(SV_SWITCHED_ON(status_word))
	{
		if (OPMODE_CSP == GET_SLAVE_XML_OPMODE(req->slave_pos))
		{
			uint32_t	cur_pos = GET_AXIS_CUR_POS(req->slave_pos);
			SET_AXIS_TARGET_POS(req->slave_pos, cur_pos);			//init target_pos to current pos
		}
		else if (OPMODE_CST == GET_SLAVE_XML_OPMODE(req->slave_pos))
		{
			SET_AXIS_TARGET_TORQUE(req->slave_pos, 0);				//目标扭矩设置为0
		}
		//rt_printf("SV_SWITCHED_ON, %d, opmode_display=%d, sw=0x%x, cur_pos=%d.\n", req->slave_pos, GET_AXIS_OPMODE_DISPLAY(req->slave_pos), GET_AXIS_STATUS_WORD(req->slave_pos), GET_AXIS_CUR_POS(req->slave_pos));
		control_word	= 0x0f;
	}

	SET_AXIS_CONTROL_WORD(req->slave_pos, control_word);
}

void __ENTRY__ request_cbfunc_svon_wait_disabled(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		status_word = GET_AXIS_STATUS_WORD(req->slave_pos);

	SET_AXIS_CONTROL_WORD(req->slave_pos, 0x0000);

	if (!SV_ENABLED(status_word))
	{
		//when in homing process, after it was immediate stop, we want to switch back to CSP MODE.
		SET_AXIS_OPMODE(req->slave_pos,  GET_SLAVE_XML_OPMODE(req->slave_pos));

		CONTINUE_OCB_CB(ocb, request_cbfunc_svon_do);
	}

	return;
}

void __ENTRY__ request_cbfunc_svoff_wait_disabled(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		status_word = GET_AXIS_STATUS_WORD(req->slave_pos);

	SET_AXIS_CONTROL_WORD(req->slave_pos, 0x0000);
	if (!SV_ENABLED(status_word))
	{
		END_OCB_CB(ocb);
	}
}

static void  request_cbfunc_homemove_wait_cspmode(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);

	if (GET_SLAVE_XML_OPMODE(req->slave_pos) == GET_AXIS_OPMODE_DISPLAY(req->slave_pos))
	{	
		ocb->cb_return	= 0;
		END_OCB_CB(ocb);
	}
}

static void request_cbfunc_homemove_delay(void *priv_data)
{
	request_t		*req		= (request_t *)priv_data;
	out_control_t	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);

	++req->u.hmove_priv.delay_cycles;
	if (req->u.hmove_priv.delay_cycles >= 150)
	{
		SET_AXIS_TARGET_POS(req->slave_pos, 0);
		//end homing
		SET_AXIS_CONTROL_WORD(req->slave_pos, 0x0F);
		SET_AXIS_OPMODE(req->slave_pos,  GET_SLAVE_XML_OPMODE(req->slave_pos));//switch back to CSP

		SET_AXIS_REQ_CMD_POS(req->slave_pos, 0);
		SET_AXIS_HOME_OFFSET(req->slave_pos, 0);

		CONTINUE_OCB_CB(ocb, request_cbfunc_homemove_wait_cspmode);
	}
}

void request_cbfunc_homemove_wait_homed(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		status_word = GET_AXIS_STATUS_WORD(req->slave_pos);
	int32_t 		curpos;

	//start homing
	SET_AXIS_CONTROL_WORD(req->slave_pos, 0x001f);

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//! some driver will generate overspeed alarm when switch back to CSP
	//! if not  updating target position in homing mode.
	curpos = GET_AXIS_CUR_POS(req->slave_pos);
	SET_AXIS_TARGET_POS(req->slave_pos, curpos);

	if (req->u.hmove_priv.stop_at_once)
	{
		uint8_t		hw = GET_AXIS_HOME_SWITCH(req->slave_pos);
		if (hw)
		{//HOME_SWITCH is on, stop homing
		 //end homing
			SET_AXIS_CONTROL_WORD(req->slave_pos, 0x0F);

			SET_AXIS_TARGET_POS(req->slave_pos, curpos);
			SET_AXIS_OPMODE(req->slave_pos,  GET_SLAVE_XML_OPMODE(req->slave_pos));	//switch back to CSP

			//rt_printf("request_cbfunc_homemove_wait_homed %d, curpos=%d, target_pos=%d, status_word=0x%x\n", req->slave_pos, GET_AXIS_CUR_POS(req->slave_pos), GET_AXIS_TARGET_POS(req->slave_pos), status_word);

			SET_AXIS_REQ_CMD_POS(req->slave_pos, curpos);
			SET_AXIS_HOME_OFFSET(req->slave_pos, curpos);

			ocb->cb_return 	= 0;
			END_OCB_CB(ocb);
		}
	}
	else
	{
		if (SV_HOME_ATTAINED(status_word)			//原点完成
		    && (  !GET_AXIS_SW_BIT10_VALID(req->slave_pos) || SV_TARGET_REACHED(status_word)))		//位置到达
		{
			//rt_printf("request_cbfunc_homemove_wait_homed %d, curpos=%d\n", req->slave_pos, curpos);
			req->u.hmove_priv.delay_cycles = 0;
			CONTINUE_OCB_CB(ocb, request_cbfunc_homemove_delay);

#if 0
			SET_AXIS_TARGET_POS(req->slave_pos, 0);
			//end homing
			SET_AXIS_CONTROL_WORD(req->slave_pos, 0x0F);
			SET_AXIS_OPMODE(req->slave_pos,  GET_SLAVE_XML_OPMODE(req->slave_pos));//switch back to CSP

			SET_AXIS_REQ_CMD_POS(req->slave_pos, 0);
			SET_AXIS_HOME_OFFSET(req->slave_pos, 0);

			ocb->cb_return 	= 0;
			END_OCB_CB(ocb);
#endif
		}
		else if (SV_HOME_ERROR(status_word))
		{
			SET_AXIS_CONTROL_WORD(req->slave_pos, 0x0F);
			SET_AXIS_OPMODE(req->slave_pos,  GET_SLAVE_XML_OPMODE(req->slave_pos));//switch back to CSP

			SET_AXIS_REQ_CMD_POS(req->slave_pos, GET_AXIS_CUR_POS(req->slave_pos));
			SET_AXIS_HOME_OFFSET(req->slave_pos, GET_AXIS_CUR_POS(req->slave_pos));

			ocb->cb_return 	= -1;
			END_OCB_CB(ocb);
		}
	}
}

void __ENTRY__ request_cbfunc_homemove_wait_hmmode(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);

	SET_AXIS_OPMODE(req->slave_pos, OPMODE_HM);
	if (OPMODE_HM == GET_AXIS_OPMODE_DISPLAY(req->slave_pos))
	{	
		//start homing
		SET_AXIS_CONTROL_WORD(req->slave_pos, 0x001F);
		CONTINUE_OCB_CB(ocb, request_cbfunc_homemove_wait_homed);
	}
}

void __ENTRY__ request_cbfunc_move_until_homed_waithomed(void *priv_data)
{
	int32_t cur_target_pos;
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint8_t 		home_switch = GET_AXIS_HOME_SWITCH(req->slave_pos);
	priv_move_until_homed_t *priv		= &(req->u.move_until_homed_priv);
	long moved_dist = GET_AXIS_CUR_POS(req->slave_pos) - priv->startpos;

	if (moved_dist < 0)
		moved_dist = -moved_dist;

	if (home_switch)
	{
		ocb->cb_return 	= 0;
		END_OCB_CB(ocb);
	}
	else if (moved_dist > priv->max_dist )
	{
		ocb->cb_return 	= -1;       //长时间运动
		END_OCB_CB(ocb);
	}
	else
	{
		if (priv->low_speed > 0
		    && (priv->current_speed + 1 <= priv->low_speed) )
		{//正向运动，低速尚未到达，速度+1
			++(priv->current_speed);
		}
		else if (priv->low_speed < 0
			 && (priv->current_speed - 1 >= priv->low_speed))
		{//负向运动，低速尚未到达，速度+1
			--(priv->current_speed);
		}

		cur_target_pos = GET_AXIS_TARGET_POS(req->slave_pos);
		SET_AXIS_TARGET_POS(req->slave_pos, cur_target_pos + (priv->current_speed));
	}
}

void request_cbfunc_immediate_stop_wait_enabled(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		axis;
	uint16_t total_axis = req->u.immediate_stop_priv.total_axis;
	int i;
	int32_t actual;

#ifdef DEBUG_DECL
	uint16_t    sw = GET_AXIS_STATUS_WORD(req->slave_pos);
	if (sw != last_sw)
	{
		rt_printf("------wait enabled cw=0x%x sw=0x%x\n", GET_AXIS_CONTROL_WORD(req->slave_pos), sw);
	}
	last_sw = sw;
#endif

	uint8_t disabled = 0;
	for (i = 0; i < total_axis; ++i)
	{
		axis = req->u.immediate_stop_priv.axis_array[i];
		if (!SV_STATE_ON(GET_AXIS_STATUS_WORD(axis)))
		{//任意轴丢掉使能
			disabled = 1;
			break;
		}

		if (!SV_ENABLED(GET_AXIS_STATUS_WORD(axis)) )
			break;  //尚未从quick stop active 恢复
	}

	if (++req->u.immediate_stop_priv.waitcycles > 1000
	    || disabled)
	{
		for ( i = 0; i < total_axis; ++i)
		{
			axis = req->u.immediate_stop_priv.axis_array[i];
			SET_AXIS_CONTROL_WORD(axis, 0x000F);
			actual = GET_AXIS_CUR_POS(axis);
			SET_AXIS_TARGET_POS(axis, actual);		//减速停止完成后，设置当前目标位置，防止控制字切换后位置突变
			SET_AXIS_REQ_CMD_POS(axis,	actual);
			SET_AXIS_OPMODE(axis, GET_SLAVE_XML_OPMODE(axis));
		}
		req->u.immediate_stop_priv.error_flag  = 1;
		END_OCB_CB(ocb);
		return; //掉使能
	}

	if (i < total_axis)
		return; //尚未从quick stop active 恢复

	END_OCB_CB(ocb);
}

void request_cbfunc_immediate_stop_nosvoff_enable(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		axis;
	uint16_t 		total_axis = req->u.immediate_stop_priv.total_axis;
	int             i;

	for ( i = 0; i < total_axis; ++i)
	{
		axis = req->u.immediate_stop_priv.axis_array[i];
		SET_AXIS_CONTROL_WORD(axis, 0x000f);
	}

	CONTINUE_OCB_CB(ocb, request_cbfunc_immediate_stop_wait_enabled);
}

void request_cbfunc_immediate_stop_nosvoff_wait_quick_stop_active(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		axis;
	uint16_t 		total_axis = req->u.immediate_stop_priv.total_axis;
	uint8_t         disabled   = 0;
	int i;
	uint16_t	status ;
	int32_t actual;

#ifdef DEBUG_DECL
	uint16_t    sw = GET_AXIS_STATUS_WORD(req->slave_pos);
	if (sw != last_sw)
	{
		rt_printf("---cw=0x%x sw=0x%x\n", GET_AXIS_CONTROL_WORD(req->slave_pos), sw);
	}
	last_sw = sw;
#endif

	for ( i = 0; i < total_axis; ++i)
	{
		axis = req->u.immediate_stop_priv.axis_array[i];

		status = GET_AXIS_STATUS_WORD(axis);

		if (!SV_STATE_ON(status))
		{//电机丢掉使能
			disabled = 1;
			break;
		}

		if (!SV_QUICK_STOP_ACTIVE(status)) break;        //尚未进入quick stop active 状态

		if (!SV_TARGET_REACHED(status))     break;       //bit 10未设置, 未成功停下
	}

	if (++req->u.immediate_stop_priv.waitcycles > 1000
	    || disabled)
	{
		for ( i = 0; i < total_axis; ++i)
		{
			axis = req->u.immediate_stop_priv.axis_array[i];
			SET_AXIS_CONTROL_WORD(axis, 0x000F);
			actual = GET_AXIS_CUR_POS(axis);
			SET_AXIS_TARGET_POS(axis, actual);		//减速停止完成后，设置当前目标位置，防止控制字切换后位置突变
			SET_AXIS_REQ_CMD_POS(axis,	actual);
			SET_AXIS_OPMODE(axis, GET_SLAVE_XML_OPMODE(axis));
		}
		req->u.immediate_stop_priv.error_flag  = 1;
		END_OCB_CB(ocb);
		return; //掉使能
	}

	if (i < total_axis)
		return; //未停止

	for ( i = 0; i < total_axis; ++i)
	{
		axis = req->u.immediate_stop_priv.axis_array[i];

		actual = GET_AXIS_CUR_POS(axis);
		SET_AXIS_TARGET_POS(axis, actual);		//减速停止完成后，设置当前目标位置，防止控制字切换后位置突变
		SET_AXIS_REQ_CMD_POS(axis,	actual);
		SET_AXIS_OPMODE(axis, GET_SLAVE_XML_OPMODE(axis));
	}

	CONTINUE_OCB_CB(ocb, request_cbfunc_immediate_stop_nosvoff_enable);
	//req->ocb->cb_func(req->ocb->priv_data);
}

void __ENTRY__ request_cbfunc_immediate_stop_nosvoff_do_quickstop(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		axis;
	uint16_t 		total_axis = req->u.immediate_stop_priv.total_axis;
	int             i;

	SET_AXIS_CONTROL_WORD(req->slave_pos, SV_FAST_STOP_CW);		//fast stop
	for ( i = 0; i < total_axis; ++i)
	{
		axis = req->u.immediate_stop_priv.axis_array[i];
		SET_AXIS_CONTROL_WORD(axis, SV_FAST_STOP_CW);
	}

	CONTINUE_OCB_CB(ocb, request_cbfunc_immediate_stop_nosvoff_wait_quick_stop_active);
}

//等待固定的周期数目后将控制字置为0，避免总线异常后，急停操作长时间等待
void __ENTRY__ request_cbfunc_immediate_stop_wait_cycles(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	uint16_t		axis;
	uint16_t 		total_axis = req->u.immediate_stop_priv.total_axis;
	uint16_t        sw;
	int i;
	int32_t actual ;

	SET_AXIS_CONTROL_WORD(req->slave_pos, 0x0000);	//去掉使能

	for ( i = 0; i < total_axis; ++i)
	{
		axis = req->u.immediate_stop_priv.axis_array[i];
		SET_AXIS_CONTROL_WORD(axis, 0x0000);	//去掉使能

		actual = GET_AXIS_CUR_POS(axis);
		SET_AXIS_REQ_CMD_POS(axis,	actual);	//设置命令位置，response函数返回命令位置
	}

	if ((req->u.immediate_stop_priv.waitcycles)++ < 100)
	{//等待固定个周期数
		return;
	}

	sw = GET_AXIS_STATUS_WORD(req->slave_pos);
	if (SV_FAULT(sw))
	{
		SET_AXIS_CONTROL_WORD(req->slave_pos, 0x80);
	}

	END_OCB_CB(ocb);
}

static void request_cbfunc_set_sync_tmove_enable_wait_inpos(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	int				slot        = req->u.set_sync_tmove_enable_priv.slot;
	struct syncmove_cb * cb = &syncmove_cbs[slot];

	if (SV_IN_POSITION(cb->axis))
	{
		SET_AXIS_REQ_CMD_POS(cb->axis, cb->dst_pos) 	//更新轴命令请求位置
			END_OCB_CB(ocb);
		return;
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


static void request_cbfunc_set_sync_tmove_enable_update_targetpos(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	int				slot        = req->u.set_sync_tmove_enable_priv.slot;
	struct syncmove_cb * cb = &syncmove_cbs[slot];

	int32_t plan_pos =	syncmove_displace_t(cb);

	int32_t target_pos = cb->base_position + plan_pos;
	SET_AXIS_TARGET_POS(cb->axis, target_pos);

	if(++cb->ts >= cb->sync_move_para.cycles)
	{
		cb->dst_pos	 = target_pos;
		CONTINUE_OCB_CB(ocb, request_cbfunc_set_sync_tmove_enable_wait_inpos);
	}

	return ;
}

void __ENTRY__ request_cbfunc_set_sync_tmove_enable(void *priv_data)
{
	request_t 		*req 		= (request_t *)priv_data;
	out_control_t 	*ocb		= OUT_CB_PTR_NOCHECK(req->slave_pos);
	int				slot        = req->u.set_sync_tmove_enable_priv.slot;
	struct syncmove_cb * sync_move_ptr = &syncmove_cbs[slot];
	uint8_t		sync_in = 0;

	if (sync_move_ptr->req_disable)
	{
		END_OCB_CB(ocb);
		return;
	}

	IO_STATUS_GET_IN_BIT(sync_move_ptr->inbit, sync_in);

	if (0 == sync_in
	    && sync_move_ptr->inbit_value != 0)
	{//检测到下降沿
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
		CONTINUE_OCB_CB(ocb, request_cbfunc_set_sync_tmove_enable_update_targetpos);
		return;
	}

	sync_move_ptr->inbit_value = sync_in;
	return;
}

void eck_rt_init_out_cb_funcs(eck_t *eck)
{
	int i;
	eck->out_cb_funcs[REQUEST_CBFUNC_SVON_WAIT_DISABLED]  = request_cbfunc_svon_wait_disabled;
	eck->out_cb_funcs[REQUEST_CBFUNC_SVOFF_WAIT_DISABLED] = request_cbfunc_svoff_wait_disabled;
	eck->out_cb_funcs[REQUEST_CBFUNC_HOMEMOVE_WAIT_HMMODE] = request_cbfunc_homemove_wait_hmmode;
	eck->out_cb_funcs[REQUEST_CBFUNC_MOVE_UNTIL_HOMED_WAITHOMED] = request_cbfunc_move_until_homed_waithomed;
	eck->out_cb_funcs[REQUEST_CBFUNC_IMMEDIATE_STOP_NOSVOFF_DO_QUICKSTOP] = request_cbfunc_immediate_stop_nosvoff_do_quickstop;
	eck->out_cb_funcs[REQUEST_CBFUNC_IMMEDIATE_STOP_WAIT_CYCLES] = request_cbfunc_immediate_stop_wait_cycles;
	eck->out_cb_funcs[REQUEST_CBFUNC_SET_SYNC_TMOVE_ENABLE] = request_cbfunc_set_sync_tmove_enable;
	eck->out_cb_funcs[OUT_CB_TEST] = out_cb_test;

	for (i = 0; i < OUTCB_FUNC_NUM; ++i)
	{
		ECK_INFO("out_cb_funcs[%d] @ 0x%p.",i, eck->out_cb_funcs[i]);
		}
}

