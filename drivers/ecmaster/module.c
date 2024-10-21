#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <generated/utsrelease.h>

#include "common.h"
#include "eck.h"
#include "eck_cdev.h"
#include "globals.h"
#include "eck_debugfs.h"

#define EC_IOMAP_SIZE  (4096)
#define MAKE_PDO_ENTRY(index, subindex, bitnum) (((index) << 16) | ((subindex)<<8) | (bitnum))

MODULE_AUTHOR("HuSuYang");
MODULE_DESCRIPTION("EtherCAT Master Kernel Module");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");

#define DEVICE_COUNT (1)
static dev_t	device_number;
static struct class *class; /**< Device class. */
static eck_t	*eck_array;
static unsigned long eck_array_size;
struct task_struct *eck_kthread;

static void * eck_alloc_buffer(unsigned long size)
{
	void *vabase = vmalloc(size);
	char *vaddr;
	char *vaend = (char *)vabase + size;
	struct page *p;

	if (!vabase)
	{
		return NULL;
	}

	for (vaddr = vabase ; vaddr < vaend; vaddr += PAGE_SIZE)
	{

		p = vmalloc_to_page(vaddr);
		SetPageReserved(p);
	}

	memset(vabase, 0, size);
	return vabase;
}

static void  eck_free_buffer(void *vabase, unsigned long size)
{
	char *vaddr;
	char *vaend = (char *)vabase + size;
	struct page *p;
	for (vaddr = vabase ; vaddr < vaend; vaddr += PAGE_SIZE)
	{

		p = vmalloc_to_page(vaddr);
		ClearPageReserved(p);
	}
	vfree(vabase);
}

static void data_init(void )
{
	int i;
	for (i = 0; i < MAX_SLAVE_COUNT; ++i)
	{
		rt_lrw_out_cbs[i].out_state = OUT_DONE;
		rt_lrw_out_cbs[i].cb_state = CB_DONE;
		pending_requests[i].slave_pos = i;
	}
}

static int eck_init(eck_t *eck, int index)
{
	int ret;
	eck->index  = index;
	eck->mapped = 0;

	sema_init(&eck->sem, 1);
	init_completion(&eck->linkup);

	if ((ret = eck_cdev_init(&eck->cdev, eck, device_number)))
	{
		goto out_return;
	}

	eck->class_device = device_create(class, NULL,
					  MKDEV(MAJOR(device_number), eck->index), NULL,
					  "ECK%u", eck->index);
	if (IS_ERR(eck->class_device))
	{
		pr_err( "Failed to create class device!\n");
		ret = PTR_ERR(eck->class_device);
		goto out_clear_cdev;
	}

	eck->master_state_size = sizeof(master_state_t);
	eck->master_state = eck_alloc_buffer(eck->master_state_size);
	if (!eck->master_state)
	{
		ret = -ENOMEM;
		goto out_unregister_device;
	}

	eck->out_cbs_size = sizeof(out_control_t) * MAX_SLAVE_COUNT;
	eck->out_cbs = eck_alloc_buffer(eck->out_cbs_size);
	if (!eck->out_cbs)
	{
		ret = -ENOMEM;
		goto OUT_FREE_MASTER_STATE;
	}

	eck->out_buffers_size = sizeof(out_buffer_t) * MAX_SLAVE_COUNT;
	eck->out_buffers = eck_alloc_buffer(eck->out_buffers_size);
	if (!eck->out_buffers)
	{
		ret = -ENOMEM;
		goto OUT_FREE_CBS;
	}

	eck->out_cb_funcs_size = sizeof(outcb_func_t) * OUTCB_FUNC_NUM;
	eck->out_cb_funcs = eck_alloc_buffer(eck->out_cb_funcs_size);
	if (!eck->out_cb_funcs)
	{
		ret = -ENOMEM;
		goto OUT_FREE_BUFFERS;
	}

	eck->pending_requests_size = sizeof(request_t) * MAX_SLAVE_COUNT;
	eck->pending_requests = eck_alloc_buffer(eck->pending_requests_size);
	if (!eck->pending_requests)
	{
		ret = -ENOMEM;
		goto OUT_FREE_OCB_FUNCS;
	}

	eck->request_states_size = sizeof(RequestState) * MAX_SLAVE_COUNT;
	eck->request_states = eck_alloc_buffer(eck->request_states_size);
	if (!eck->request_states)
	{
		ret = -ENOMEM;
		goto OUT_FREE_PENDING_REQUESTS;
	}

	eck->decl_cbs_size = sizeof(decl_control_t) * MAX_SLAVE_COUNT;
	eck->decl_cbs = eck_alloc_buffer(eck->decl_cbs_size);
	if (!eck->decl_cbs)
	{
		ret = -ENOMEM;
		goto OUT_FREE_REQUEST_STATES;
	}

	eck->syncmove_cbs_size = sizeof(syncmove_cb_t) * MAX_SLAVE_COUNT;
	eck->syncmove_cbs = eck_alloc_buffer(eck->syncmove_cbs_size);
	if (!eck->syncmove_cbs)
	{
		ret = -ENOMEM;
		goto OUT_FREE_DECL_CBS;
	}

	eck->axis_states_size = sizeof(axis_state_t) * MAX_SLAVE_COUNT;
	eck->axis_states = eck_alloc_buffer(eck->axis_states_size);
	if (!eck->axis_states)
	{
		ret = -ENOMEM;
		goto OUT_FREE_SYNCMOVE_CBS;
	}

	eck->slave_xml_configs_size = sizeof(slave_xml_config_t) * MAX_SLAVE_COUNT;
	eck->slave_xml_configs = eck_alloc_buffer(eck->slave_xml_configs_size);
	if (!eck->slave_xml_configs)
	{
		ret = -ENOMEM;
		goto OUT_FREE_AXIS_STATES;
	}

	eck->initconfig_physical_unit_size = sizeof(physical_unit_t) * MAX_PHYSICAL_COUNT;
	eck->initconfig_physical_unit = eck_alloc_buffer(eck->initconfig_physical_unit_size);
	if (!eck->initconfig_physical_unit)
	{
		ret = -ENOMEM;
		goto OUT_FREE_SLAVE_XML_CONFIGS;
	}

	eck->period_struct_size = sizeof(period_t);
	eck->period_struct = eck_alloc_buffer(eck->period_struct_size);
	if (!eck->period_struct)
	{
		ret = -ENOMEM;
		goto OUT_FREE_INITCONFIG_PHYSICAL_UNIT;
	}

	eck->ndev_stats_size = sizeof(device_stats_t);
	eck->ndev_stats = eck_alloc_buffer(eck->ndev_stats_size);
	if (!eck->ndev_stats)
	{
		ret = -ENOMEM;
		goto OUT_FREE_PERIOD_STRUCT;
	}

	eck->process_data_size = EC_IOMAP_SIZE;
	eck->process_data = eck_alloc_buffer(eck->process_data_size);
	if (!eck->initconfig_physical_unit)
	{
		ret = -ENOMEM;
		goto OUT_FREE_NDEV_STATS_STRUCT;
	}

	//eck_rt_init_out_cb_funcs(eck);

	//初始化各指针
	master_state	= eck->master_state;
	rt_lrw_out_cbs	= eck->out_cbs;
	rt_lrw_buffers	= eck->out_buffers;
	rt_lrw_decl_cbs = eck->decl_cbs;
	syncmove_cbs	= eck->syncmove_cbs;	  //同步运动控制块
	request_states	= eck->request_states;
	pending_requests=eck->pending_requests;
	axis_states	= eck->axis_states;
	out_cb_funcs	= eck->out_cb_funcs;
	slave_xml_configs = eck->slave_xml_configs;
	initconfig_physical_unit = eck->initconfig_physical_unit;
	period_struct	 = eck->period_struct;
	ndev_stats		 = eck->ndev_stats;

	domain0_pd  = eck->process_data;
	domain1_pd  = NULL;

	data_init();

	return 0;

OUT_FREE_NDEV_STATS_STRUCT:
	eck_free_buffer(eck->ndev_stats ,eck->ndev_stats_size);

OUT_FREE_PERIOD_STRUCT:
	eck_free_buffer(eck->period_struct ,eck->period_struct_size);

OUT_FREE_INITCONFIG_PHYSICAL_UNIT:
	eck_free_buffer(eck->initconfig_physical_unit ,eck->initconfig_physical_unit_size);

OUT_FREE_SLAVE_XML_CONFIGS:
	eck_free_buffer(eck->slave_xml_configs ,eck->slave_xml_configs_size);

OUT_FREE_AXIS_STATES:
	eck_free_buffer(eck->axis_states ,eck->axis_states_size);

OUT_FREE_SYNCMOVE_CBS:
	eck_free_buffer(eck->syncmove_cbs, eck->syncmove_cbs_size);

OUT_FREE_DECL_CBS:
	eck_free_buffer(eck->decl_cbs, eck->decl_cbs_size);

OUT_FREE_REQUEST_STATES:
	eck_free_buffer(eck->request_states, eck->request_states_size);

OUT_FREE_PENDING_REQUESTS:
	eck_free_buffer(eck->pending_requests, eck->pending_requests_size);

OUT_FREE_OCB_FUNCS:
	eck_free_buffer(eck->out_cb_funcs, eck->out_cb_funcs_size);

OUT_FREE_BUFFERS:
	eck_free_buffer(eck->out_buffers, eck->out_buffers_size);

OUT_FREE_CBS:
	eck_free_buffer(eck->out_cbs, eck->out_cbs_size);

OUT_FREE_MASTER_STATE:
	eck_free_buffer(eck->master_state, eck->master_state_size);

out_unregister_device:
	device_unregister(eck->class_device);

out_clear_cdev:
	eck_cdev_clear(&eck->cdev);

out_return:
	return ret;
}

static void eck_clear(eck_t *eck)
{
	device_unregister(eck->class_device);
	eck_cdev_clear(&eck->cdev);
	eck_free_buffer(eck->master_state, eck->master_state_size);
	eck_free_buffer(eck->out_cbs, eck->out_cbs_size);
	eck_free_buffer(eck->out_buffers, eck->out_buffers_size);
	eck_free_buffer(eck->out_cb_funcs, eck->out_cb_funcs_size);
	eck_free_buffer(eck->pending_requests, eck->pending_requests_size);
	eck_free_buffer(eck->request_states, eck->request_states_size);
	eck_free_buffer(eck->decl_cbs, eck->decl_cbs_size);
	eck_free_buffer(eck->syncmove_cbs, eck->syncmove_cbs_size);
	eck_free_buffer(eck->axis_states, eck->axis_states_size);
	eck_free_buffer(eck->slave_xml_configs, eck->slave_xml_configs_size);
	eck_free_buffer(eck->initconfig_physical_unit, eck->initconfig_physical_unit_size);
	eck_free_buffer(eck->period_struct, eck->period_struct_size);
	eck_free_buffer(eck->process_data, eck->process_data_size);
}

#if 0
static int ecmaster_init_proc(void *data)
{
	int i;
	//手动状态切换,防止在ec_config_map()时，自动尝试切换到SAFEOP
	ecx_context.manualstatechange = 1;

	//等待W5500链路UP，并打开W5500设备
	if (FALSE == ec_init(IFACE))
	{//配置EtherCAT网卡设备
		period_err |= PERIOD_ESETUPNIC;
		pr_err("ec_init() failed.");
	}

	for (i = 0; i < DEVICE_COUNT; i++)
	{
		complete_all(&eck_array[i].linkup);
	}

	module_put_and_kthread_exit(0);
}
#endif

//防盗版措施验证CPU Serial-number
//返回:	0			验证通过
//		<0 			验证失败
//备注: 读取ec_igb.ko的"HACK"数据段中的数据，该数据对应hack.ko中全局变量digest_data值
//		首次执行时，读取结果与初始数据相等，写入Serial-number到数据段中
//		之后执行，读取结果与Serial-number进行比较，如不相等，判定为验证失败
static int check_hack_data(void)
{
	char 	HACK_SECTION[] 		= "HACK";
	char	NET_DRIVER[] 		=  "/usr/lib/modules/" UTS_RELEASE "/kernel/drivers/hack/hack.ko";
	const 	int HACK_DATA_SIZE	= 16;
	char	ORIGINAL_DATA[] 	= {0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8,
		0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8};
	char	actual_data[16]		= {0};
	const int NEW_DATA_OFFSET 	= 0;
	const int NEW_DATA_SIZE	  	= 16;

	const char * serial;
	int error = -ENOEXEC;

	struct device_node *np;
	np = of_find_node_by_path("/");
	if (!np){
		pr_err("Failed to find root of!\n");
		return -ENOEXEC;

	}

	if (of_property_read_string(np, "serial-number", &serial))
	{//读取/proc/cpuinfo中Serial，16字节 0结尾
		of_node_put(np);
		pr_err("Failed to read serial-number!\n");
		goto out;
	}

	//pr_info("serial=%s\n", serial);

	const uint8_t * new_data    = serial;

	struct file * filep = filp_open(NET_DRIVER, O_RDWR, 0);
	int retval, i, j;

	struct elfhdr elf_ex;
	struct elf_shdr *elf_shdata;	//section headers
	struct elf_shdr *elf_shstrtab;	//.shstrtab section header
	struct elf_shdr *elf_pshdata;
	char *sh_names;
	loff_t offset = 0;

	if (IS_ERR(filep))
	{
		printk("ec_check_hack_data Open file %s error\n", NET_DRIVER);
		goto out;
	}

	//Read Elf header
	retval = kernel_read(filep, (char *) &elf_ex, sizeof(elf_ex), &offset);
	if (retval != sizeof(elf_ex))
	{
		printk("ec_check_hack_data kernel_read ELF header failed, retval = %d.\n", retval);
		goto out;
	}

	j = sizeof(struct elf_shdr) * elf_ex.e_shnum;

	error = -ENOMEM;
	elf_shdata = kmalloc(j, GFP_KERNEL);
	if (!elf_shdata)
		goto out;

	//read Section headers
	error = -ENOEXEC;
	offset = elf_ex.e_shoff;
	retval = kernel_read(filep, (char *)elf_shdata, j, &offset);
	if (retval != j)
	{
		printk("ec_check_hack_data kernel_read Section Headers failed, retval = %d.\n", retval);
		goto out_free_sh_buffer;
	}

	//read .shstrtab
	elf_shstrtab = &elf_shdata[elf_ex.e_shstrndx];

	sh_names = kmalloc(elf_shstrtab->sh_size, GFP_KERNEL);
	if (!sh_names)
		goto out_free_sh_buffer;

	error = -ENOEXEC;
	offset = elf_shstrtab->sh_offset;
	retval = kernel_read(filep,  sh_names, elf_shstrtab->sh_size, &offset);
	if (retval != elf_shstrtab->sh_size)
	{
		printk("ec_check_hack_data kernel_read .shstrtab  failed, retval = %d.\n", retval);
		goto out_free_sh_names;
	}

	//search data section named "HACK"
	elf_pshdata = elf_shdata;
	for (i = 0; i < elf_ex.e_shnum; i++, ++elf_pshdata)
	{
		if (0 == strcmp(sh_names + elf_pshdata->sh_name, HACK_SECTION)	//段名称匹配
		    && SHT_PROGBITS == elf_pshdata->sh_type)					//数据段
		{
			break;
		}
	}

	if (i >= elf_ex.e_shnum)
	{
		printk("ec_check_hack_data HACK section not found.\n");
		error = -ENOEXEC;
		goto out_free_sh_names;
	}

	//read acutal data
	offset = elf_pshdata->sh_offset;
	retval = kernel_read(filep, actual_data, HACK_DATA_SIZE, &offset);
	if (retval != HACK_DATA_SIZE)
	{
		printk("ec_check_hack_data read HACK sectioin data failed, retval = %d.\n", retval);
		error = -ENOEXEC;
		goto out_free_sh_names;
	}

	retval = memcmp(ORIGINAL_DATA, actual_data, HACK_DATA_SIZE);
	if (0 == retval)
	{//write new data
		offset = elf_pshdata->sh_offset + NEW_DATA_OFFSET;
		retval = kernel_write(filep, new_data, NEW_DATA_SIZE, &offset);
		if (retval != NEW_DATA_SIZE)
		{
			error = -ENOEXEC;
			goto out_free_sh_names;
		}
	}
	else
	{//compare mac address
		retval = memcmp(new_data, actual_data + NEW_DATA_OFFSET, NEW_DATA_SIZE);
		if (retval)
		{
			error = -ENOEXEC;
			goto out_free_sh_names;
		}
	}

	error = 0;

out_free_sh_names:
	kfree(sh_names);

out_free_sh_buffer:
	kfree(elf_shdata);
	filp_close(filep, NULL);
out:
	of_node_put(np);
	return error;
}

static int __init ecmaster_init_module(void)
{
	int i, ret = -EPERM;

#ifdef CONFIG_ECAT_DC_SYNC64
	pr_info("CONFIG_ECAT_DC_SYNC64\n");
#endif

#ifdef CONFIG_ECAT_IO_FREERUN
	pr_info("CONFIG_ECAT_IO_FREERUN\n");
#endif

#ifdef CONFIG_ECAT_CHECK_HACK
	pr_info("CONFIG_ECAT_CHECK_HACK\n");

	if (check_hack_data() < 0)
	{//绑定CPU验证失败
		return -EPERM;
	}
#endif

	pr_info("home move delay 150 cycles.\n");

	if (eck_debugfs_init())
	{
		pr_err("Failed to Create DebugFs!\n");
		return -ENOENT;
	}

	if (alloc_chrdev_region(&device_number, 0, DEVICE_COUNT, "ECK"))
	{
		pr_err("Failed to obtain device number(s)!\n");
		ret = -EBUSY;
		goto OUT_REMOVE_DEBUGFS;
	}

	class = class_create(THIS_MODULE, "ECK");
	if (IS_ERR(class)) {
		pr_err("Failed to create device class.\n");
		ret = PTR_ERR(class);
		goto out_cdev;
	}

	eck_array_size = DEVICE_COUNT * sizeof(eck_t);
	eck_array = eck_alloc_buffer(eck_array_size);
	if (!eck_array)
	{
		ret = -ENOMEM;
		goto out_class;
	}

	for (i = 0; i < DEVICE_COUNT; i++) {
		ret = eck_init(&eck_array[i], i);
		if (ret)
			goto out_clear_eck;
	}

	return 0;

	//	//创建内核线程并运行
	//	if (!try_module_get(THIS_MODULE))
	//		goto out_clear_eck;
	//	eck_kthread = kthread_run(ecmaster_init_proc, NULL, "kecmaster");
	//	if (IS_ERR(eck_kthread))
	//		goto out_put_module;

	//	return 0;

	//out_put_module:
	//	module_put(THIS_MODULE);

out_clear_eck:
	for (i--; i >= 0; i--)
		eck_clear(&eck_array[i]);

	eck_free_buffer(eck_array, eck_array_size);

out_class:
	class_destroy(class);

out_cdev:
	unregister_chrdev_region(device_number, DEVICE_COUNT);

OUT_REMOVE_DEBUGFS:
	eck_debugfs_exit();

	return ret;
}
module_init(ecmaster_init_module);

static void __exit ecmaster_exit_module(void)
{
	int i;

	for (i = 0; i < DEVICE_COUNT; i++)
		eck_clear(&eck_array[i]);

	eck_free_buffer(eck_array, eck_array_size);

	class_destroy(class);

	eck_debugfs_exit();
}
module_exit(ecmaster_exit_module);

