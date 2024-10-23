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

static struct proc_dir_entry *proc_ecmaster_dir;	//文件/proc/ecmaster
static struct dentry *debug_ecmaster_dir;		//目录/sys/kernel/debug/ecmaster
static uint8_t tmpbuf[8192];				//possible race conditions???

static ssize_t debug_file_skip_lrw_write(struct file *f, const char __user *buffer, size_t len, loff_t *offset)
{
	int ret;
	unsigned long long res;
	ret = kstrtoull_from_user(buffer, len, 10, &res);
	if (0 == ret) 
	{
		period_struct->skip_lrw = res;
	}
	else
		period_struct->skip_lrw  = 1;

	return len;
}

static const struct file_operations debug_file_skip_lrw_fops = {
	.owner = THIS_MODULE,
	.write = debug_file_skip_lrw_write,
};

static ssize_t debug_file_cmd_write(struct file *f, const char __user *buffer, size_t len, loff_t *offset)
{
	uint8_t command[128] = {0};

	if (len >= sizeof(command)) return len;
	if (copy_from_user(command, buffer, len))
	{
		return len;
	}

	command[sizeof(command) - 1] = '\0';

	pr_info("cmd = %s\n", command);

	if (0 == strcmp(command, "reset_jitter\n"))
	{
		period_struct->reset_jitter = 1;
		pr_info("reset_jitter\n");
	}

	return len;
}

static const struct file_operations debug_file_cmd_fops = {
	.owner = THIS_MODULE,
	.write = debug_file_cmd_write,
};
#if 0
static ssize_t debug_file_slaveinfo_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
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
#endif

static ssize_t debug_file_offset_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	slave_type_t type;
	ssize_t retval;
	int size = sizeof(tmpbuf);
	int len = 0;
	int i;
	unsigned int port;

	len += scnprintf(tmpbuf + len, size - len, "%6s %6s %6s %6s %6s %6s\n",  "slave", "type", "OUT", "OBytes", "IN", "INBytes" );
	for (i = 0; i < initconfig_slave_count; ++i)
	{
		type = GET_SLAVE_TYPE(i);
		len += scnprintf(tmpbuf + len, size - len, "[%4d] %6s %6u %6u %6u %6u\n", 
				 i,
				 (type == SLAVE_AXIS) ? "AXIS" : "IO", 
				 *SLAVE_STATUS_OFF_OUTPUT_PTR(i),
				 *SLAVE_STATUS_OUTPUT_SIZE_PTR(i),
				 *SLAVE_STATUS_OFF_INPUT_PTR(i),
				 *SLAVE_STATUS_INPUT_SIZE_PTR(i));
	}

	len += scnprintf(tmpbuf + len, size - len, "%6s %6s %6s %6s %6s | %6s %6s %6s %6s %6s %6s %6s\n", 
			 "slave", "CW", "OPMOD", "TARPOS", "TARTOR", "ALARM", "SW", "ACTPOS", "OPDSP", "DI", "INNTOR", "ACTTOR");

	len += scnprintf(tmpbuf + len, size - len, "%6s %6s %6s %6s %6s | %6s %6s %6s %6s %6s %6s %6s\n", 
			 "slave", "6040h", "6060h", "607Ah", "6071h" ,"603Fh", "6041h", "6064h", "6061h", "60FDh", "6074h", "6077h");
	for (i = 0; i < initconfig_slave_count; ++i)
	{
		type = GET_SLAVE_TYPE(i);
		if (SLAVE_AXIS != type) continue;
		len += scnprintf(tmpbuf + len, size - len, "[%4d] %6d %6d %6d %6d | %6d %6d %6d %6d %6d %6d %6d\n", 
				 i,
				 *AXIS_STATUS_OFF_CONTROL_WORD_PTR(i),
				 *AXIS_STATUS_OFF_OPMODE_PTR(i),
				 *AXIS_STATUS_OFF_TARGET_POS_PTR(i),
				 *AXIS_STATUS_OFF_TARGET_TORQUE_PTR(i),
				 *AXIS_STATUS_OFF_ALARM_CODE_PTR(i),
				 *AXIS_STATUS_OFF_STATUS_WORD_PTR(i),
				 *AXIS_STATUS_OFF_CUR_POS_PTR(i),
				 *AXIS_STATUS_OFF_OPMODE_DISPLAY_PTR(i),
				 *AXIS_STATUS_OFF_DIGITAL_INPUT_PTR(i),
				 *AXIS_STATUS_OFF_DEMAND_TORQUE_PTR(i),
				 *AXIS_STATUS_OFF_ACTUAL_TORQUE_PTR(i));
	}		

	len += scnprintf(tmpbuf + len, size - len, "------IO Module Info------\n");
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
	int size = sizeof(tmpbuf);
	int len = 0;
	int logical;

	len += scnprintf(tmpbuf + len, size - len, "%8s %6s %6s %10s %6s | %6s %6s %10s %6s %10s %6s %6s\n", 
			 "logical" ,"CW", "OPMOD", "TARPOS", "TARTOR", "ALARM", "SW", "ACTPOS", "OPDSP", "DI", "INNTOR", "ACTTOR");
	len += scnprintf(tmpbuf + len, size - len, "%8s %6s %6s %10s %6s | %6s %6s %10s %6s %10s %6s %6s\n", 
			 "logical", "6040h", "6060h", "607Ah", "6071h" ,"603Fh", "6041h", "6064h", "6061h", "60FDh", "6074h", "6077h");

	for (logical = 0; logical < initconfig_slave_count; ++logical)
	{
		if (SLAVE_AXIS != GET_SLAVE_TYPE(logical))
			continue;

		len += scnprintf(tmpbuf + len, size - len, "[%6d] 0x%04x %6u %10d %6d | 0x%04x 0x%04x %10d %6u 0x%08x %6d %6d\n", 
				 logical,
				 GET_AXIS_CONTROL_WORD(logical),
				 GET_AXIS_OPMODE(logical),
				 GET_AXIS_TARGET_POS(logical),
				 GET_AXIS_TARGET_TORQUE(logical),
				 GET_AXIS_ALARM_CODE(logical),
				 GET_AXIS_STATUS_WORD(logical),
				 GET_AXIS_CUR_POS(logical),
				 GET_AXIS_OPMODE_DISPLAY(logical),
				 GET_AXIS_DIGITAL_INPUT(logical),
				 GET_AXIS_DEMAND_TORQUE(logical),
				 GET_AXIS_ACTUAL_TORQUE(logical));		
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
	int size = sizeof(tmpbuf);
	int len = 0;
	int i;

	len += scnprintf(tmpbuf + len, size - len, "jiffies:%llu.\n" , period_struct->jiffies);
	len += scnprintf(tmpbuf + len, size - len, "wkc:%d.\n" , period_struct->wkc);
	len += scnprintf(tmpbuf + len, size - len, "skip_lrw:%u.\n", period_struct->skip_lrw);
	len += scnprintf(tmpbuf + len, size - len, "jitter_max:%lld.\n", period_struct->jitter_max);	
	len += scnprintf(tmpbuf + len, size - len, "jitter_min:%lld.\n", period_struct->jitter_min);
	len += scnprintf(tmpbuf + len, size - len, "hookcost_max:%lld.\n", period_struct->hookcost_max);
	len += scnprintf(tmpbuf + len, size - len, "error:0x%x.\n", period_struct->error);	
	len += scnprintf(tmpbuf + len, size - len, "warn:0x%x.\n", period_struct->warn);
	len += scnprintf(tmpbuf + len, size - len, "nrt_error:0x%x.\n", period_struct->nrt_error);

	for (i = 0; i < PERIOD_MAX_PCNT; ++i)
	{
		len += scnprintf(tmpbuf + len, size - len, "pcnt[%d]:%llu ", i, period_struct->pcnt[i]);

		if (i > 0)
		{
			len += scnprintf(tmpbuf + len, size - len, "diff:%llu ", (int64_t)(period_struct->pcnt[i] - period_struct->pcnt[i - 1]));
		}
		len += scnprintf(tmpbuf + len, size - len, "\n");
	}

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

static ssize_t debug_file_syncmove_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
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

static ssize_t debug_file_ndev_stats_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	int size = sizeof(tmpbuf);
	int len = 0;

	len += scnprintf(tmpbuf + len, size - len, "tx_errors:%llu\n" , ndev_stats->tx_errors);
	len += scnprintf(tmpbuf + len, size - len, "tx_dropped:%llu\n" , ndev_stats->tx_dropped);
	len += scnprintf(tmpbuf + len, size - len, "tx_fifo_errors:%llu\n" , ndev_stats->tx_fifo_errors);
	len += scnprintf(tmpbuf + len, size - len, "tx_packets:%llu\n" , ndev_stats->tx_packets);
	len += scnprintf(tmpbuf + len, size - len, "tx_bytes:%llu\n" , ndev_stats->tx_bytes);
	len += scnprintf(tmpbuf + len, size - len, "rx_dropped:%llu\n" , ndev_stats->rx_dropped);
	len += scnprintf(tmpbuf + len, size - len, "rx_packets:%llu\n" , ndev_stats->rx_packets);
	len += scnprintf(tmpbuf + len, size - len, "rx_bytes:%llu\n" , ndev_stats->rx_bytes);
	len += scnprintf(tmpbuf + len, size - len, "rx_missed_errors:%llu\n" , ndev_stats->rx_missed_errors);
	len += scnprintf(tmpbuf + len, size - len, "rx_length_errors:%llu\n" , ndev_stats->rx_length_errors);
	len += scnprintf(tmpbuf + len, size - len, "rx_frame_errors:%llu\n" , ndev_stats->rx_frame_errors);

	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_ndev_stats_fops = {
	.owner = THIS_MODULE,
	.read = debug_file_ndev_stats_read,
};

static ssize_t debug_file_request_ocb_read(struct file *f, char __user *buffer, size_t buffer_len, loff_t *offset)
{
	ssize_t retval;
	int size = sizeof(tmpbuf);
	int len = 0;
	int logical;

	len += scnprintf(tmpbuf + len, size - len, 
			 "%8s %6s %6s %10s %10s | %10s %11s %13s %8s %10s\n", 
			 "logical", "state", "type", "fsm", "response", "out_state" ,"cycle_count", "current_cycle", "cb_state", "cb_func");

	for (logical = 0; logical < initconfig_slave_count; ++logical)
	{
		len += scnprintf(tmpbuf + len, size - len, "[%6d] %6d %6d 0x%8px 0x%8px | %10d %11u %13u %8d 0x%8px\n", 
				 logical,
				 (int)request_states[logical].state,
				 (int)pending_requests[logical].ind_type,
				 pending_requests[logical].state,
				 pending_requests[logical].response,
				 (int)rt_lrw_out_cbs[logical].out_state,
				 rt_lrw_out_cbs[logical].cycle_count,
				 rt_lrw_out_cbs[logical].current_cycle,
				 (int)rt_lrw_out_cbs[logical].cb_state,
				 rt_lrw_out_cbs[logical].cb_func);	
	}

	retval = simple_read_from_buffer(buffer, buffer_len, offset, tmpbuf, len);

	return retval;
}

static const struct file_operations debug_file_request_ocb_fops = {
	.owner = THIS_MODULE,
	.read = debug_file_request_ocb_read,
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
	seq_printf(seq, "nothing to show.\n");

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

	new_dentry = debugfs_create_file("cmd", S_IWUSR, debug_ecmaster_dir, NULL, &debug_file_cmd_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() cmd failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

#if 0
	new_dentry = debugfs_create_file("slaveinfo", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_slaveinfo_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() startsv failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}
#endif

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

	new_dentry= debugfs_create_file("ndev_stats", S_IRUSR, debug_ecmaster_dir, NULL, &debug_file_ndev_stats_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() ndev_stats failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}

	new_dentry = debugfs_create_file("request_ocb", S_IWUSR, debug_ecmaster_dir, NULL, &debug_file_request_ocb_fops);
	if (!new_dentry)
	{
		pr_err("debugfs_create_file() request_ocb failed.\n");
		goto OUT_REMOVE_DEBUGFS;
	}
#if 0
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
#endif

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
