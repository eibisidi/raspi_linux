#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/kobject.h>

#include "globals.h"
#include "ethercat.h"

static struct proc_dir_entry *proc_ecmaster_dir;		//文件/proc/ecmaster
static struct dentry *debug_ecmaster_dir;				//目录/sys/kernel/debug/ecmaster

static ssize_t debug_file_skip_lrw_write(struct file *f, const char __user *buffer, size_t len, loff_t *offset)
{
	int slave_pos = 0;
	int ret;
	unsigned long long res;
	ret = kstrtoull_from_user(buffer, len, 10, &res);
	if (0 == ret) 
	{
		period_skip_lrw = res;
	}
	else
		period_skip_lrw = 1;

	return len;
}

static const struct file_operations debug_file_skip_lrw_fops = {
    .owner = THIS_MODULE,
    .write = debug_file_skip_lrw_write,
};

static ssize_t debug_file_slaveinfo_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int len = 0;

	if (0 == *offset)
	{
		if (master_state->activated)
			slaveinfo_readonly();
		else
			slaveinfo();
	}

	len += scnprintf(tmpbuf, sizeof(tmpbuf), "It is dangerous to do so. Please use dmesg to read out slave info.\n");
	
	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_slaveinfo_fops = {
    .owner = THIS_MODULE,
    .read = debug_file_slaveinfo_read,
};

static ssize_t debug_file_offset_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	slave_type_t type;
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;
	int i;
	unsigned int port;
	
	len += scnprintf(tmpbuf + len, size - len, "%6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s\n", 
				"slave", "OUT", "OBytes", "IN", "INBytes" ,"CW", "OPMD", "TARGET", "ALARM", "SW", "ACTUAL", "OPDSP", "DI");

	for (i = 0; i < initconfig_slave_count; ++i)
	{
		type = GET_SLAVE_TYPE(i);
		if (SLAVE_AXIS == type)
		{
			len += scnprintf(tmpbuf + len, size - len, "[%4d] %6u %6u %6u %6u %6u %6u %6u %6u %6u %6u %6u %6u\n", 
					i,
					*SLAVE_STATUS_OFF_OUTPUT_PTR(i),
					*SLAVE_STATUS_OUTPUT_SIZE_PTR(i),
					*SLAVE_STATUS_OFF_INPUT_PTR(i),
					*SLAVE_STATUS_INPUT_SIZE_PTR(i),
					*AXIS_STATUS_OFF_CONTROL_WORD_PTR(i),
					*AXIS_STATUS_OFF_OPMODE_PTR(i),
					*AXIS_STATUS_OFF_TARGET_POS_PTR(i),
					*AXIS_STATUS_OFF_ALARM_CODE_PTR(i),
					*AXIS_STATUS_OFF_STATUS_WORD_PTR(i),
					*AXIS_STATUS_OFF_CUR_POS_PTR(i),
					*AXIS_STATUS_OFF_OPMODE_DISPLAY_PTR(i),
					*AXIS_STATUS_OFF_DIGITAL_INPUT_PTR(i));		
		}
		else
		{
			len += scnprintf(tmpbuf + len, size - len, "[%4d] %6u %6u %6u %6u \n", 
					i,
					*SLAVE_STATUS_OFF_OUTPUT_PTR(i),
					*SLAVE_STATUS_OUTPUT_SIZE_PTR(i),
					*SLAVE_STATUS_OFF_INPUT_PTR(i),
					*SLAVE_STATUS_INPUT_SIZE_PTR(i));	
		}
	}

	len += scnprintf(tmpbuf + len, size - len, "------IO Module Info\n");
	len += scnprintf(tmpbuf + len, size - len, "input_port_num:%u input_portno_to_offset[]:\n", input_port_num);
	for (port = 0; port < input_port_num; ++port)
	{
		len += scnprintf(tmpbuf + len, size - len, "%4u " , input_portno_to_offset[port]);
		if (0 == (port + 1) % 16)
			len += scnprintf(tmpbuf + len, size - len, "\n");
	}
	len += scnprintf(tmpbuf + len, size - len, "\n");
	
	len += scnprintf(tmpbuf + len, size - len, "output_port_num:%u output_portno_to_offset[]:\n", output_port_num);		
	for (port = 0; port < output_port_num; ++port)
	{
		len += scnprintf(tmpbuf + len, size - len, "%4u " , output_portno_to_offset[port]);
		if (0 == (port + 1) % 16)
			len += scnprintf(tmpbuf + len, size - len, "\n");
	}
	len += scnprintf(tmpbuf + len, size - len, "\n");
	
	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_offset_fops = {
    .owner = THIS_MODULE,
    .read = debug_file_offset_read,
};

static ssize_t debug_file_process_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;
	int i;

	len += scnprintf(tmpbuf + len, size - len, "------Output Size:%d\n", domain0_size);
	if (domain0_pd)
	{
		for (i = 0; i < domain0_size; ++i)
		{
			len += scnprintf(tmpbuf + len, size -len, "%02X ", domain0_pd[i]);

	        if ((i + 1) % 16 == 0) {
	            len += scnprintf(tmpbuf + len, size -len, "\n");
	        }
	    }
	}
	
	len += scnprintf(tmpbuf + len, size - len, "\n------Input  Size:%d\n", domain1_size);
	if (domain1_pd)
	{
		for (i = 0; i < domain1_size; ++i)
		{
			len += scnprintf(tmpbuf + len, size -len, "%02X ", domain1_pd[i]);

			if ((i + 1) % 16 == 0) {
				len += scnprintf(tmpbuf + len, size -len, "\n");
			}
		}
	}
	len += scnprintf(tmpbuf + len, size -len, "\n");

	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_process_fops = {
    .owner = THIS_MODULE,
    .read = debug_file_process_read,
};

static ssize_t debug_file_axis_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;
	int logical;
	len += scnprintf(tmpbuf + len, size - len, "%6s %6s %6s %10s %6s %6s %10s %6s %8s\n", 
				"logical" ,"CW", "OPMD", "TARGET", "ALARM", "SW", "ACTUAL", "OPDSP", "DI");

	for (logical = 0; logical < initconfig_slave_count; ++logical)
	{
		if (SLAVE_AXIS != GET_SLAVE_TYPE(logical))
			continue;
		
		len += scnprintf(tmpbuf + len, size - len, "[%5d] 0x%04x %6u %10d 0x%04x 0x%04x %10d %6u 0x%08x\n", 
				logical,
				GET_AXIS_CONTROL_WORD(logical),
				GET_AXIS_OPMODE(logical),
				GET_AXIS_TARGET_POS(logical),
				GET_AXIS_ALARM_CODE(logical),
				GET_AXIS_STATUS_WORD(logical),
				GET_AXIS_CUR_POS(logical),
				GET_AXIS_OPMODE_DISPLAY(logical),
				GET_AXIS_DIGITAL_INPUT(logical));		
	}
	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_axis_fops = {
    .owner = THIS_MODULE,
    .read = debug_file_axis_read,
};

static ssize_t debug_file_master_state_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;
	
	len += scnprintf(tmpbuf + len, size - len, "rt_lrw_ready:%u.\n" , master_state->rt_lrw_ready);
	len += scnprintf(tmpbuf + len, size - len, "ec_bus_inited:%u.\n", master_state->ec_bus_inited);
	len += scnprintf(tmpbuf + len, size - len, "ec_bus_error:%u.\n", master_state->ec_bus_error);
		len += scnprintf(tmpbuf + len, size - len, "accumlated:%u.\n", master_state->accumlated);
	len += scnprintf(tmpbuf + len, size - len, "al_status:0x%x.\n", master_state->al_status);
	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_master_state_fops = {
    .owner = THIS_MODULE,
    .read = debug_file_master_state_read,
};

static ssize_t debug_file_period_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;

	len += scnprintf(tmpbuf + len, size - len, "period_err:0x%x.\n" , period_err);
	len += scnprintf(tmpbuf + len, size - len, "period_run:%u.\n", period_run);
	len += scnprintf(tmpbuf + len, size - len, "nrt_error:0x%x.\n", nrt_error);	
	len += scnprintf(tmpbuf + len, size - len, "period_startout:%u.\n", period_startout);
	len += scnprintf(tmpbuf + len, size - len, "period_counter:%u.\n", period_counter);
	len += scnprintf(tmpbuf + len, size - len, "period_wkc:%d.\n", period_wkc);
	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_period_fops = {
    .owner = THIS_MODULE,
    .read = debug_file_period_read,
};
	
static ssize_t debug_file_io_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;
	unsigned int port;
	uint8_t portdata;

	len += scnprintf(tmpbuf + len, size - len, "------Inputs:\n");
	for (port = 0; port < input_port_num; ++port)
	{
		IO_STATUS_GET_IN_DATA(port, portdata);
		len += scnprintf(tmpbuf + len, size - len, "%02x " , portdata);
		if (0 == (port + 1) % 16)
			len += scnprintf(tmpbuf + len, size - len, "\n");
	}
	len += scnprintf(tmpbuf + len, size - len, "\n");
	
	len += scnprintf(tmpbuf + len, size - len, "-------Outputs:\n");
	for (port = 0; port < output_port_num; ++port)
	{
		IO_STATUS_GET_OUT_DATA(port, portdata);
		len += scnprintf(tmpbuf + len, size - len, "%02x " , portdata);
		if (0 == (port + 1) % 16)
			len += scnprintf(tmpbuf + len, size - len, "\n");
	}
	len += scnprintf(tmpbuf + len, size - len, "\n");

	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_io_fops = {
	.owner = THIS_MODULE,
	.read = debug_file_io_read,
};

static ssize_t debug_file_reg_timediff_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;
	char sign;
	int physical, slave;
	int wkc;
	int32_t val;
	uint32  abs_sync_diff;

	if (0 != *offset)
		return 0;

	len += scnprintf(tmpbuf + len, size - len, "------Read Register Time Difference 0x092C:\n");

	for (physical = 0; physical < initconfig_physical_count; ++physical)
	{
		slave = physical + 1;
		if (!ec_slave[slave].hasdc)
		{
			len += scnprintf(tmpbuf + len, size - len, "slave %d is not DC capable.\n", physical);
			continue;
		}
	
		wkc = ec_FPRD(ec_slave[slave].configadr, ECT_REG_DCSYSDIFF, sizeof(val), &val, EC_TIMEOUTRET); 
		if (wkc <= 0)
		{
			len += scnprintf(tmpbuf + len, size - len, "slave %d read register error.\n", physical);
			continue;
		}

		abs_sync_diff = etohl(val) & 0x7fffffff;
		sign = (etohl(val) & 0x80000000) ? ('-'):('+');
		len += scnprintf(tmpbuf + len, size - len, "slave %d %c %u(ns).\n", physical, sign, abs_sync_diff);
	
	}
	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_reg_timediff_fops = {
	.owner = THIS_MODULE,
	.read = debug_file_reg_timediff_read,
};

static ssize_t debug_file_reg_watchdogs_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;
	int physical, slave;
	int wkc;
	uint32  abs_sync_diff;
	uint16 wd_status;		//Watchdog Status Process Data (0x0440:0x0441) 0:expired 1:active or disabled
	uint16 wd_div;			//Watchdog Divider (0x0400:0x0401)
	uint16 wd_pd;			//Watchdog Time Process Data (0x0420:0x0421)
	int32 t_wd_div;
	int32 t_wd_pd;
	uint8	sm2_control;	//Register Control Register SyncManager y (0x0804+y*8) y=2
	uint8   wd_counter;		//Register Watchdog Counter Process Data (0x0442)
	int     wd_counter_wkc;

	if (0 != *offset)
		return 0;

	len += scnprintf(tmpbuf + len, size - len, "------Read Process Data Related Registers:\n");
	len += scnprintf(tmpbuf + len, size - len, "tWD_Div tWD_PD SM2_WatchdogTriggerEnabled WatchDogStatus WatchDogCounter\n");

	for (physical = 0; physical < initconfig_physical_count; ++physical)
	{
		slave = physical + 1;

		wkc = ec_FPRD(ec_slave[slave].configadr, 0x0400, sizeof(wd_div), &wd_div, EC_TIMEOUTRET); 
		if (wkc <= 0)
		{
			wd_div = 0xFFFF;
		}

		wkc = ec_FPRD(ec_slave[slave].configadr, 0x0420, sizeof(wd_pd), &wd_pd, EC_TIMEOUTRET); 
		if (wkc <= 0)
		{
			wd_pd = 0xFFFF;
		}

		if (wd_div != 0xFFFF)
		{
			t_wd_div = (wd_div + 2)*40;	
		}
		else
			t_wd_div = -1;

		if (wd_div != 0xFFFF 
			&& wd_pd != 0xFFF)
		{
			t_wd_pd = t_wd_div * wd_pd;
		}
		else
			t_wd_pd = -1;
	
		wkc = ec_FPRD(ec_slave[slave].configadr, 0x0804 + 2 * 8, sizeof(sm2_control), &sm2_control, EC_TIMEOUTRET); 
		if (wkc <= 0)
		{
			sm2_control = 0xFF;
		}
		
		wkc = ec_FPRD(ec_slave[slave].configadr, 0x0440, sizeof(wd_status), &wd_status, EC_TIMEOUTRET); 
		if (wkc <= 0)
		{
			wd_status = 0xFFFF;
		}

		wd_counter_wkc = ec_FPRD(ec_slave[slave].configadr, 0x0442, sizeof(wd_counter), &wd_counter, EC_TIMEOUTRET); 
		
		len += scnprintf(tmpbuf + len, size - len, "%d %d %8s ", t_wd_div, t_wd_pd, (sm2_control & 0x40)? "Enabled":"Disabled");
		len += scnprintf(tmpbuf + len, size - len, "0x%04x ", wd_status);
		if (wd_counter_wkc == 1)
			len += scnprintf(tmpbuf + len, size - len, "%u", wd_counter);
		else
			len += scnprintf(tmpbuf + len, size - len, "RErr");
		len += scnprintf(tmpbuf + len, size - len, "\n");
	}
	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_reg_watchdogs_fops = {
	.owner = THIS_MODULE,
	.read = debug_file_reg_watchdogs_read,
};

static ssize_t debug_file_syncmove_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	static uint8_t tmpbuf[4096];
	int size = sizeof(tmpbuf);
	int len = 0;

	int i;
	for (i = 0; i < MAX_SYNC_MOVE_COUNT; ++i)
	{
		len += scnprintf(tmpbuf + len, size - len, "------[%d]\n" , i);
		len += scnprintf(tmpbuf + len, size - len, "enabled=%u\n" , syncmove_cbs[i].enabled);
		len += scnprintf(tmpbuf + len, size - len, "inbit=%u\n" , syncmove_cbs[i].inbit);
		len += scnprintf(tmpbuf + len, size - len, "axis=%u\n" , syncmove_cbs[i].axis);
		len += scnprintf(tmpbuf + len, size - len, "paras_valid=%u\n" , syncmove_cbs[i].paras_valid);
		len += scnprintf(tmpbuf + len, size - len, "ts=%u\n" , syncmove_cbs[i].ts);
		len += scnprintf(tmpbuf + len, size - len, "inbit_value=%u\n" , syncmove_cbs[i].inbit_value);
		len += scnprintf(tmpbuf + len, size - len, "done_do_notify=%u\n" , syncmove_cbs[i].done_do_notify);
		len += scnprintf(tmpbuf + len, size - len, "dst_pos=%d\n" , syncmove_cbs[i].dst_pos);
		len += scnprintf(tmpbuf + len, size - len, "sync_move_para:\n");
		len += scnprintf(tmpbuf + len, size - len, "vmax=%lld\n" , syncmove_cbs[i].sync_move_para.vmax);
		len += scnprintf(tmpbuf + len, size - len, "tacc=%lld\n" , syncmove_cbs[i].sync_move_para.tacc);
		len += scnprintf(tmpbuf + len, size - len, "q1=%lld\n" , syncmove_cbs[i].sync_move_para.q1);
		len += scnprintf(tmpbuf + len, size - len, "Ta=%lld\n" , syncmove_cbs[i].sync_move_para.Ta);
		len += scnprintf(tmpbuf + len, size - len, "Tv=%lld\n" , syncmove_cbs[i].sync_move_para.Tv);
		len += scnprintf(tmpbuf + len, size - len, "T=%lld\n" , syncmove_cbs[i].sync_move_para.T);
		len += scnprintf(tmpbuf + len, size - len, "vlim=%lld\n" , syncmove_cbs[i].sync_move_para.vlim);
		len += scnprintf(tmpbuf + len, size - len, "sacc=%lld\n" , syncmove_cbs[i].sync_move_para.sacc);
		len += scnprintf(tmpbuf + len, size - len, "cycles=%u\n" , syncmove_cbs[i].sync_move_para.cycles);
	}

	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_syncmove_fops = {
	.owner = THIS_MODULE,
	.read = debug_file_syncmove_read,
};

static void *ecmaster_proc_seq_start(struct seq_file *seq, loff_t *pos)
{
	return (*pos == 0) ? pos : NULL;
}

static void *ecmaster_proc_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos >= 1)
		return NULL;
	return pos;
}

static void ecmaster_proc_seq_stop(struct seq_file *seq, void *v)
{

}

static int ecmaster_proc_seq_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "------tick------\n");
	seq_printf(seq, "next_cnt:%llu maxjitter:%lld minjitter%lld real_times=%lld\n", expected_cnt, jitter_max, jitter_min, real_times);
	seq_printf(seq, "timer_step:%u\n", timer_step);
	seq_printf(seq, "hookcost_max:%lld\n", hookcost_max);
	seq_printf(seq, "------period------\n");
	seq_printf(seq, "run:%u err:0x%x warn:0x%x counter:%u.\n", period_run, period_err, period_warn, period_counter);
	seq_printf(seq, "startout:%u nrt_error:0x%x\n", period_startout, nrt_error);

	net_show_stats(seq, v);
	return 0;
}

static const struct seq_operations ecmaster_proc_seq_ops = {
	.start = ecmaster_proc_seq_start,
	.next  = ecmaster_proc_seq_next,
	.stop  = ecmaster_proc_seq_stop,
	.show  = ecmaster_proc_seq_show,
};
	
int eck_debugfs_init(void)
{
	int ret = -ENOENT;
	struct dentry * new_dentry;
	proc_ecmaster_dir = proc_create_seq("ecmaster", 0, NULL, &ecmaster_proc_seq_ops);
	if (!proc_ecmaster_dir)
	{
		pr_err("proc_create_seq() failed.\n");
		return -ENOENT;
	}

	debug_ecmaster_dir = debugfs_create_dir("ecmaster", NULL);
	if (!debug_ecmaster_dir)
	{	
		pr_err("debugfs_create_dir() failed.\n");
		goto OUT_REMOVE_PROC;
	}

	new_dentry = debugfs_create_file("skip_lrw", S_IWUSR, debug_ecmaster_dir, NULL, &debug_file_skip_lrw_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() skip_lrw failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

	new_dentry = debugfs_create_file("slaveinfo", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_slaveinfo_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() startsv failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

	new_dentry= debugfs_create_file("offset", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_offset_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() offset failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

	new_dentry= debugfs_create_file("process", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_process_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() process failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}
	
	new_dentry= debugfs_create_file("axis", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_axis_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() axis failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}
	
	new_dentry= debugfs_create_file("master_state", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_master_state_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() master_state failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

	new_dentry= debugfs_create_file("period", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_period_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() period failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

	new_dentry= debugfs_create_file("io", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_io_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() io failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}
	
	new_dentry= debugfs_create_file("reg_timediff", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_reg_timediff_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() reg_timediff failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}
	
	new_dentry= debugfs_create_file("reg_watchdogs", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_reg_watchdogs_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() reg_watchdogs failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

	new_dentry= debugfs_create_file("syncmove", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_syncmove_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() syncmove failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

	return 0;

OUT_REMOVE_DEBUGFS:
	debugfs_remove_recursive(debug_ecmaster_dir);

OUT_REMOVE_PROC:
	proc_remove(proc_ecmaster_dir);

	return ret;
}

void eck_debugfs_exit(void)
{
	if (debug_ecmaster_dir)
	{
		debugfs_remove_recursive(debug_ecmaster_dir);
		debug_ecmaster_dir = NULL;
	}

	if (proc_ecmaster_dir)
	{
		proc_remove(proc_ecmaster_dir);
		proc_ecmaster_dir = NULL;
	}
}
