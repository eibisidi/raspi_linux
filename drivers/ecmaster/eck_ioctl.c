#include <linux/mm.h>
#include <linux/mman.h>

#include "ethercat.h"
#include "eck.h"
#include "eck_ioctl.h"
#include "eck_cdev.h"
#include "ecbus.h"

static uint64_t expected_wakeup;
static uint32_t TIMER_STEP = (54000 * 2);

static int64_t counter2ns(int64_t counter)
{
	return 0;
}

static int eck_ioctl_start_rt_task(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	unsigned long ctlr, cntkctl_el1, sctlr_el1;
	int cpu = smp_processor_id();

	if (CONFIG_ECAT_AFF_CPUID != cpu)
	{
		ECK_ERR("eck_ioctl_disable_timer.\n");
		return -EACCES;
	}

	//disable arm generic timer
	ctlr = read_sysreg(CNTP_CTL_EL0);
	ctlr &= ~(1ULL << 0);
	ctlr |= 0x2;
	write_sysreg(ctlr, CNTP_CTL_EL0);

	//allow access CNTPCT_EL0 from EL0
	cntkctl_el1 = read_sysreg(CNTKCTL_EL1);
	cntkctl_el1 |= 0x01;				//EL0PCTEN, bit [0]
	write_sysreg(cntkctl_el1, CNTKCTL_EL1);

	//allow access PSTATE.{D, A, I, F} from EL0
	sctlr_el1 = read_sysreg(SCTLR_EL1);
	sctlr_el1 |= (1ULL << 9);			//UMA, bit [9]
	write_sysreg(sctlr_el1, SCTLR_EL1);

	/*
	   RCU GP thread rcu_sched will continue to send  IPI_RESCHEDULE to this CPU
	   if rcu_qs() is not called.
	   The fllowing call stack emit IPI_IPI_RESCHEDULE:
	   [  109.184351]  smp_send_reschedule+0x64/0x68
	   [  109.184356]  resched_curr+0x7c/0xd8
	   [  109.184360]  resched_cpu+0xc8/0xd0
	   [  109.184363]  rcu_implicit_dynticks_qs+0x304/0x350
	   [  109.184369]  force_qs_rnp+0x164/0x268
	   [  109.184373]  rcu_gp_fqs_loop+0x404/0x568
	   [  109.184378]  rcu_gp_kthread+0x214/0x248
	   [  109.184384]  kthread+0x110/0x120
	   [  109.184388]  ret_from_fork+0x10/0x20
	   We invoke rcu_report_dead() to let RCU know that this CPU is 'dead'.
	   */
	rcu_report_dead(CONFIG_ECAT_AFF_CPUID);

	isb();

	return 0;
}

static int eck_ioctl_stop_rt_task(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	unsigned long ctlr;
	int cpu = smp_processor_id();

	if (CONFIG_ECAT_AFF_CPUID != cpu)
	{
		ECK_ERR("eck_ioctl_stop_rt_task error.\n");
		return -EACCES;
	}

	ctlr = read_sysreg(CNTP_CTL_EL0);
	ctlr |= 0x01;
	ctlr &= ~(1ULL << 1);
	write_sysreg(ctlr, CNTP_CTL_EL0);
	isb();
	pr_info("physical timer is enabled again.\n");

	return 0;
}

static int eck_ioctl_prepare_cpu_core(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	unsigned long cntkctl_el1, sctlr_el1, cntp_ctl_el0;
	int cpu = smp_processor_id();

	//allow access from EL0
	cntkctl_el1 = read_sysreg(CNTKCTL_EL1);
	cntkctl_el1 |= 0x01;				//EL0PCTEN, bit [0]  allows EL0 to access cntpct_el0 + cntfrq_el0
	cntkctl_el1 |= 0x200;				//EL0PTEN, bit [9]   allows EL0 to access CNTP_CTL_EL0
	write_sysreg(cntkctl_el1, CNTKCTL_EL1);

	//allow access PSTATE.{D, A, I, F} from EL0
	sctlr_el1 = read_sysreg(SCTLR_EL1);
	sctlr_el1 |= (1ULL << 9);			//UMA, bit [9]
	write_sysreg(sctlr_el1, SCTLR_EL1);
	isb();

	cntp_ctl_el0 = read_sysreg(CNTP_CTL_EL0);
	pr_info("CORE:%d cntp_ctl_el0=0x%lx.\n", cpu, cntp_ctl_el0);
	pr_info("CORE:%d prepare_cpu_core success.\n", cpu);

	return 0;
}

static int eck_ioctl_get_jitter(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	eck_ioctl_jitter_t data;
	data.avg = 0;			//non-sense
	data.min = 0;
	data.max = 0;
	data.gmin = 0;
	data.gmax = counter2ns(jitter_max);
	data.overruns = 0;
	data.goverruns = 0;
	data.gmaxcyclecost = counter2ns(hookcost_max);

	if (copy_to_user((void __user *) arg, &data, sizeof(eck_ioctl_jitter_t)))
	{
		return -EFAULT;
	}

	return 0;
}

static int eck_ioctl_map_buffer_to_user(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	int ret;
	eck_ioctl_map_buffer_to_user_t data;

	//if (eck->mapped)
	//	return -EBUSY;

	priv->ctx.src_vaddr = eck->master_state;
	data.master_state = (void *)vm_mmap(filp, 0, eck->master_state_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->out_cbs;
	data.out_cbs = (void *)vm_mmap(filp, 0, eck->out_cbs_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->out_buffers;
	data.out_buffers = (void *)vm_mmap(filp, 0, eck->out_buffers_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->out_cb_funcs;
	data.out_cb_funcs = (void *)vm_mmap(filp, 0, eck->out_cb_funcs_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->pending_requests;
	data.pending_requests = (void *)vm_mmap(filp, 0, eck->pending_requests_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->request_states;
	data.request_states = (void *)vm_mmap(filp, 0, eck->request_states_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->decl_cbs;
	data.decl_cbs = (void *)vm_mmap(filp, 0, eck->decl_cbs_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->syncmove_cbs;
	data.syncmove_cbs = (void *)vm_mmap(filp, 0, eck->syncmove_cbs_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->axis_states;
	data.axis_states = (void *)vm_mmap(filp, 0, eck->axis_states_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->slave_xml_configs;
	data.slave_xml_configs = (void *)vm_mmap(filp, 0, eck->slave_xml_configs_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->initconfig_physical_unit;
	data.initconfig_physical_unit = (void *)vm_mmap(filp, 0, eck->initconfig_physical_unit_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->period_struct;
	data.period_struct = (void *)vm_mmap(filp, 0, eck->period_struct_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->ndev_stats;
	data.ndev_stats = (void *)vm_mmap(filp, 0, eck->ndev_stats_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	priv->ctx.src_vaddr = eck->process_data;
	data.process_data = (void *)vm_mmap(filp, 0, eck->process_data_size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

	if (IS_ERR_VALUE(data.master_state)
	    || IS_ERR_VALUE(data.out_cbs)
	    || IS_ERR_VALUE(data.out_buffers)
	    || IS_ERR_VALUE(data.out_cb_funcs)
	    || IS_ERR_VALUE(data.pending_requests)
	    || IS_ERR_VALUE(data.request_states)
	    || IS_ERR_VALUE(data.decl_cbs)
	    || IS_ERR_VALUE(data.syncmove_cbs)
	    || IS_ERR_VALUE(data.axis_states)
	    || IS_ERR_VALUE(data.slave_xml_configs)
	    || IS_ERR_VALUE(data.initconfig_physical_unit)
	    || IS_ERR_VALUE(data.period_struct)
	    || IS_ERR_VALUE(data.ndev_stats)
	    || IS_ERR_VALUE(data.process_data)
	   )
	{
		ECK_ERR("map to user space failed.\n");
		return -EFAULT;
	}

	//zero out
	memset(eck->master_state, 0, eck->master_state_size);
	memset(eck->out_cbs, 0, eck->out_cbs_size);
	memset(eck->out_buffers, 0, eck->out_buffers_size);
	//DO NOT memset out_cb_funcs
	memset(eck->pending_requests, 0, eck->pending_requests_size);
	memset(eck->request_states, 0, eck->request_states_size);
	memset(eck->decl_cbs, 0, eck->decl_cbs_size);
	memset(eck->syncmove_cbs, 0, eck->syncmove_cbs_size);
	memset(eck->axis_states, 0, eck->axis_states_size);
	memset(eck->slave_xml_configs, 0, eck->slave_xml_configs_size);
	memset(eck->initconfig_physical_unit, 0, eck->initconfig_physical_unit_size);
	memset(eck->period_struct, 0, eck->period_struct_size);
	memset(eck->ndev_stats, 0, eck->ndev_stats_size);
	memset(eck->process_data, 0, eck->process_data_size);

	ECK_INFO("master_state 0x%px	mapped to 0x%px.", eck->master_state,  data.master_state);
	ECK_INFO("out_cbs 0x%px	mapped to 0x%px.", eck->out_cbs, data.out_cbs);
	ECK_INFO("out_buffers 0x%px	mapped to 0x%px.", eck->out_buffers,  data.out_buffers);
	ECK_INFO("out_cb_funcs 0x%px	mapped to 0x%px.", eck->out_cb_funcs,  data.out_cb_funcs);
	ECK_INFO("pending_requests 0x%px	mapped to 0x%px.", eck->pending_requests, data.pending_requests);
	ECK_INFO("request_states 0x%px	mapped to 0x%px.", eck->request_states,  data.request_states);
	ECK_INFO("decl_cbs 0x%px	mapped to 0x%px.", eck->decl_cbs,  data.decl_cbs);
	ECK_INFO("syncmove_cbs 0x%px	mapped to 0x%px.", eck->syncmove_cbs,  data.syncmove_cbs);
	ECK_INFO("axis_states 0x%px	mapped to 0x%px.", eck->axis_states,  data.axis_states);
	ECK_INFO("slave_xml_configs 0x%px	mapped to 0x%px.", eck->slave_xml_configs,  data.slave_xml_configs);
	ECK_INFO("initconfig_physical_unit 0x%px	mapped to 0x%px.", eck->initconfig_physical_unit,  data.initconfig_physical_unit);
	ECK_INFO("period_struct 0x%px	mapped to 0x%px.", eck->period_struct,  data.period_struct);
	ECK_INFO("ndev_stats 0x%px	mapped to 0x%px.", eck->ndev_stats,  data.ndev_stats);
	ECK_INFO("process_data 0x%px	mapped to 0x%px.", eck->process_data,  data.process_data);

	if (copy_to_user((void __user *) arg, &data, sizeof(eck_ioctl_map_buffer_to_user_t)))
	{
		ECK_ERR("copy_to_user() failed.\n");
		return -EFAULT;
	}

	eck->mapped = true;

	return 0;
}

static int eck_ioctl_bus_reset(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	if (ecbus_reset())
	{
		return -EACCES;
	}

	return 0;
}

static int eck_ioctl_master_activate(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	eck_ioctl_master_activate_t data;
	if (copy_from_user(&data, (void __user *) arg, sizeof(data))) {
		return -EFAULT;
	}

	master_state->activated = 1;

	initconfig_dc_cycle_us			= data.initconfig_dc_cycle_us;
	initconfig_dc_start_shift_us	= data.initconfig_dc_start_shift_us;
	initconfig_slave_count			= data.initconfig_slave_count;
	initconfig_physical_count		= data.initconfig_physical_count;
	initconfig_io_count 			= data.initconfig_io_count;
	initconfig_axis_count			= data.initconfig_axis_count;
	domain0_size					= data.domain0_size;
	domain1_size					= data.domain1_size;

	calculate_io_offset();

	domain0_pd  = eck->process_data;
	domain1_pd   = domain0_pd + domain0_size;

#if 0
	if (initconfig_dc_cycle_us != (1000000 / CONFIG_HZ))
	{
		ECK_ERR("wrong dc cycle configured.\n");
		return -EACCES;
	}

	ECK_INFO("slave_count:%d, physical_count:%d", initconfig_slave_count, initconfig_physical_count);
	ECK_INFO("dc-cycle:%d(us), dc-shift:%d(us)", initconfig_dc_cycle_us, initconfig_dc_start_shift_us);


	if (ecbus_activate())
	{
		ECK_ERR("ecbus_activate failed.\n");
		return -EACCES;
	}


	data.domain0_size = domain0_size;
	data.domain1_size = domain1_size;
	if (copy_to_user((void __user *) arg, &data, sizeof(eck_ioctl_master_activate_t)))
	{
		return -EACCES;
	}
#endif

	return 0;
}

static unsigned int my_counter = 0;

static int eck_ioctl_wait_period(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	uint64_t now_counter;
	unsigned long ctlr;
	eck_ioctl_wait_period_t io = {0};
	int64_t diff;

	do {
		isb();
		now_counter = __arch_counter_get_cntpct();
		diff = (int64_t)(now_counter - expected_wakeup);
	}while(diff < 0);

	//rcu_mark_qs();
	//rcu_report_dead(3);

	unsigned long spsr, daif;
	spsr = read_sysreg(SPSR_EL1);
	daif = read_sysreg(DAIF);
	//printk("------spsr =0x%lx daif=0x%lx\n", spsr, daif);


	expected_wakeup += TIMER_STEP;

	++my_counter;
	io.ov 	= my_counter;
	io.pcnt = now_counter;

	if (copy_to_user((void __user *)arg, &io, sizeof(io))) {
		return -EFAULT;
	}

	return 0;
}

static int eck_ioctl_slave_sdo_upload(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	int wkc, size;
	uint16 slave;
	eck_ioctl_slave_sdo_upload_t io;
	uint8_t buffer[MAX_SDO_DATASIZE];
	if (copy_from_user(&io, (void __user *) arg, sizeof(io))) {
		return -EFAULT;
	}

	if (io.target_size <= 0)
		return 0;

	if (io.slave_position >= initconfig_physical_count
	    || io.target_size > sizeof(buffer))
		return -EIO;

	slave = io.slave_position + 1;
	size  = io.target_size;
	wkc = ec_SDOread(slave, io.sdo_index, io.sdo_entry_subindex, FALSE, &size, buffer, EC_TIMEOUTRXM);

	if (wkc > 0) {
		io.data_size  = size;
		io.abort_code = 0;
		if (copy_to_user((void __user *) io.target, buffer, size)) {
			return -EFAULT;
		}

		if (copy_to_user((void __user *)arg, &io, sizeof(io))) {
			return -EFAULT;
		}
	}

	return (wkc > 0) ? 0 : -EIO;
}

static int eck_ioctl_slave_sdo_download(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	int wkc;
	uint16 slave;
	eck_ioctl_slave_sdo_download_t io;
	uint8_t buffer[MAX_SDO_DATASIZE];
	if (copy_from_user(&io, (void __user *) arg, sizeof(io))) {
		return -EFAULT;
	}

	if (io.data_size <= 0)
		return 0;

	if (io.slave_position >= initconfig_physical_count)
		return -EIO;

	if (copy_from_user(buffer, (void __user *) io.data, io.data_size)) {
		return -EFAULT;
	}

	slave = io.slave_position + 1;
	wkc = ec_SDOwrite(slave, io.sdo_index, io.sdo_entry_subindex, io.complete_access, io.data_size, buffer, EC_TIMEOUTRXM);

	return (wkc > 0) ? 0 : -EIO;
}

static int eck_ioctl_slave_reg_read(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	int slave, wkc;
	eck_ioctl_slave_reg_t io;
	uint8_t buffer[MAX_REG_SIZE];
	uint16_t adp;
	if (copy_from_user(&io, (void __user *) arg, sizeof(io))) {
		return -EFAULT;
	}

	if (io.size <= 0)
		return 0;

	if (io.slave_position >= initconfig_physical_count
	    || io.size > sizeof(buffer))
		return -EIO;

	slave = io.slave_position + 1;
	adp = ec_slave[slave].configadr;
	wkc = ec_FPRD(adp, io.address, io.size, buffer, EC_TIMEOUTRET);

	if (wkc > 0) {
		if (copy_to_user((void __user *) io.data, buffer, io.size)) {
			return -EFAULT;
		}
	}

	return (wkc > 0) ? 0 : -EIO;
}

static int eck_ioctl_slave_reg_write(eck_t *eck, struct file *filp, eck_cdev_priv_t *priv, void __user *arg)
{
	int slave, wkc;
	eck_ioctl_slave_reg_t io;
	uint8_t buffer[MAX_REG_SIZE];
	uint16_t adp;

	if (copy_from_user(&io, (void __user *) arg, sizeof(io))) {
		return -EFAULT;
	}

	if (io.size <= 0)
		return 0;

	if (io.slave_position >= initconfig_physical_count
	    || io.size > sizeof(buffer))
		return -EIO;

	if (copy_from_user(buffer, (void __user *) io.data, io.size)) {
		return -EFAULT;
	}

	slave = io.slave_position + 1;
	adp = ec_slave[slave].configadr;
	wkc = ec_FPWR(adp, io.address, io.size, buffer, EC_TIMEOUTRET);

	return (wkc > 0) ? 0 : -EIO;
}

long eck_ioctl(struct file *filp, unsigned int cmd, void __user *arg)
{
	int ret;
	eck_cdev_priv_t *priv = (eck_cdev_priv_t *) filp->private_data;
	eck_t *eck = priv->cdev->eck;

	switch (cmd) {
	case ECK_IOCTL_START_RT_TASK:
		ret = eck_ioctl_start_rt_task(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_GET_JITTER:
		ret = eck_ioctl_get_jitter(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_MAP_BUFFER_TO_USER:
		ret = eck_ioctl_map_buffer_to_user(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_BUS_RESET:
		ret = eck_ioctl_bus_reset(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_MASTER_ACTIVATE:
		ret = eck_ioctl_master_activate(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_WAIT_PERIOD:
		ret = eck_ioctl_wait_period(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_SLAVE_SDO_UPLOAD   :
		ret = eck_ioctl_slave_sdo_upload(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_SLAVE_SDO_DOWNLOAD :
		ret = eck_ioctl_slave_sdo_download(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_SLAVE_REG_READ	  :
		ret = eck_ioctl_slave_reg_read(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_SLAVE_REG_WRITE	  :
		ret = eck_ioctl_slave_reg_write(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_STOP_RT_TASK:
		ret = eck_ioctl_stop_rt_task(eck, filp, priv, arg);
		break;
	case ECK_IOCTL_PREPARE_CPU_CORE:
		ret = eck_ioctl_prepare_cpu_core(eck, filp, priv, arg);
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

