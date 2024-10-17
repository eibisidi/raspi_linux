#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/kthread.h>

#include "ethercat.h"
#include "globals.h"

#define MAX_SLAVE_DOMAIN_ENTRIES (16)
#define MAX_PDO_NUM (16)
#define DOMAIN_OUT_IDX (0)
#define DOMAIN_IN_IDX (1)
#define MAKE_PDO_ENTRY(index, subindex, bitnum) (((index) << 16) | ((subindex)<<8) | (bitnum))

#define EC_DC_MAX_SYNC_DIFF_NS 10000
#define EC_DC_SYNC_WAIT_MS 5000
#define DOMAIN_WKC_TOLERANCE (100)      //总线异常连续容忍次数

typedef enum {
	EC_WD_DEFAULT, /**< Use the default setting of the sync manager. */
	EC_WD_ENABLE, /**< Enable the watchdog. */
	EC_WD_DISABLE, /**< Disable the watchdog. */
} ec_watchdog_mode_t;

typedef enum {
	EC_DIR_INVALID, /**< Invalid direction. Do not use this value. */
	EC_DIR_OUTPUT, /**< Values written by the master. */
	EC_DIR_INPUT, /**< Values read by the master. */
	EC_DIR_COUNT /**< Number of directions. For internal use only. */
} ec_direction_t;

typedef struct {
	uint16_t index; /**< PDO entry index. */
	uint8_t subindex; /**< PDO entry subindex. */
	uint8_t bit_length; /**< Size of the PDO entry in bit. */
} ec_pdo_entry_info_t;

typedef struct
{
	uint16_t                pdo_index;			//pdo索引
	ec_pdo_entry_info_t     *entries;			//pdo条目信息
	uint8_t                 n_entries;			//pdo条目数目
}slave_pdo_info_t;

//物理从站pdo定义
typedef struct
{
	uint16_t				n_entries;                                  //该站所有的pdo条目总数
	ec_pdo_entry_info_t		entry_vec[MAX_SLAVE_DOMAIN_ENTRIES];		//该站所有的pdo条目定义
	uint16_t				byte_offset_vec[MAX_SLAVE_DOMAIN_ENTRIES];  //pdo条目在从站中的字节偏移
	uint16_t                bit_offset_vec[MAX_SLAVE_DOMAIN_ENTRIES];   //pdo条目在字节内的偏移
	uint16_t                logical[MAX_SLAVE_DOMAIN_ENTRIES];          //条目所属逻辑从站
	pdo_entry_meaning_t		meaning[MAX_SLAVE_DOMAIN_ENTRIES];

	slave_pdo_info_t        pdo_infos[MAX_PDO_NUM];					//pdo的条目定义
	uint8_t					pdo_num;								//该从站pdo数目
}slave_entries_info_t;

typedef struct {
	uint16_t index; /**< PDO index. */
	unsigned int n_entries; /**< Number of PDO entries in \a entries to map.
				  Zero means, that the default mapping shall be
				  used (this can only be done if the slave is
				  present at bus configuration time). */
	ec_pdo_entry_info_t *entries; /**< Array of PDO entries to map. Can either
					be \a NULL, or must contain at
					least \a n_entries values. */
} ec_pdo_info_t;

typedef struct {
	uint8_t index; /**< Sync manager index. Must be less
			 than #EC_MAX_SYNC_MANAGERS for a valid sync manager,
			 but can also be \a 0xff to mark the end of the list. */
	ec_direction_t dir; /**< Sync manager direction. */
	unsigned int n_pdos; /**< Number of PDOs in \a pdos. */
	ec_pdo_info_t *pdos; /**< Array with PDOs to assign. This must contain
			       at least \a n_pdos PDOs. */
	ec_watchdog_mode_t watchdog_mode; /**< Watchdog mode. */
} ec_sync_info_t;

typedef struct
{
	ec_pdo_info_t   rxpdos[MAX_PDO_NUM];
	ec_pdo_info_t   txpdos[MAX_PDO_NUM];
	ec_sync_info_t	syncs[5];	//[0]输出 	[1]输入 		[2]结束元素
}pdo_config_t;

typedef struct
{
	ec_direction_t 			dir;						//输入输出域
	slave_entries_info_t	entries[MAX_PHYSICAL_COUNT];//每个从站的pdo条目定义
	uint16_t				total_entries;				//所有从站pdo条目数目总和
}domain_config_t;

static pdo_config_t pdo_configs[MAX_PHYSICAL_COUNT];

static domain_config_t domain_configs[2] = {
	{EC_DIR_OUTPUT},	//输出域
	{EC_DIR_INPUT},		//输入域
};

static int expectedWKC;

//BWR ESC register 0x900 Receive Times
static uint8 bwr900_frame[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x88, 0xa4, 0x10, 0x10,
	0x08, 0x01, 0x00, 0x00, 0x00, 0x09, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t  bwr900_state;			//0等待queue 1等待send 2sent
static uint64_t bwr900_mastertime;
static uint8_t  bwr900_idx;
static uint16_t bwr900_wkc;
static struct task_struct *ecbus_reset_kthread;

void bwr900_hook(void)
{
	ec_comt* 	datagramP;
	uint8_t 	recvBuff[EC_BUFSIZE];
	uint16_t 	wkc;
	int recv;
	uint64_t	now;
	int64_t		cost;

	if (0 == bwr900_state)
	{
		bwr900_wkc = EC_NOFRAME;
		datagramP = (ec_comt*)&bwr900_frame[ETH_HEADERSIZE];
		bwr900_idx++;
		datagramP->index = bwr900_idx;
		bwr900_mastertime = tick_get_expected_ns();			//下个周期时钟
		net_queue(bwr900_frame, sizeof(bwr900_frame));		//加入W5500缓存
		bwr900_state = 1;
		return;
	}

	if (1 == bwr900_state)
	{
		net_send();											//发送当前缓存
		bwr900_state = 2;
		return;
	}

	now = osal_current_time_ns();
	cost = (int64_t)(now - bwr900_mastertime);
	if (cost > 1000000000 / CONFIG_HZ * CONFIG_ECAT_RCV_TIMEOUT )
	{//长时间未收到响应
		tick_sethook(NULL);
		return;
	}

	recv = net_recv(recvBuff, EC_BUFSIZE);
	if (recv != sizeof(bwr900_frame))
		return;

	datagramP = (ec_comt*)&recvBuff[ETH_HEADERSIZE];
	if (bwr900_idx != datagramP->index)
		return;

	wkc = recvBuff[0x1E] + (recvBuff[0x1F] << 8);
	if (wkc != initconfig_physical_count)
		return;

	bwr900_wkc = wkc;
	tick_sethook(NULL);
}

int ecbus_wait_enter_preop(void)
{
	int wkc;
	uint16 state;
	int retry = 10;
	uint8 success = 0;
	unsigned long timeout;

	while (retry--)
	{
		wkc = ec_config_init(FALSE);
		if (wkc >= initconfig_physical_count)
		{
			success = 1;
			break;
		}
		pr_info("waiting... found slave count = %d.", wkc);
		timeout = msleep_interruptible(1000);
		if (timeout)
		{//收到用户空间信号
			return -1;
		}
	};

	if (!success)
	{
		period_err |= PERIOD_ESCAN;
		return -1;
	}

	pr_info("%d slaves found on the bus.", wkc);

	success = 0;
	retry = 10;
	ec_slave[0].state = EC_STATE_PRE_OP | EC_STATE_ACK;
	ec_writestate(0);

	while (retry)
	{
		state = ec_statecheck(0, EC_STATE_PRE_OP,	EC_TIMEOUTSTATE * 4);
		if (EC_STATE_PRE_OP == state)
		{
			success = 1;
			break;
		}

		pr_info("waiting enter pre op, current state = 0x%x.", state);
		timeout = msleep_interruptible(1000);
		if (timeout)
		{//收到用户空间信号
			return -1;
		}
	}

	if (!success)
	{
		period_err |= PERIOD_EPREOP;
		return -1;
	}

	ec_readstate();
	int slave;
	for (slave = 1; slave <= ec_slavecount; slave++)
	{
		pr_info("slave[%d] in state:0x%x\n", slave, ec_slave[slave].state);
	}
	pr_info("slaves all in pre op.\n");

	return 0;
}

static int download_init_sdos(void)
{
	int 		retval = -1;
	uint8_t 	data[8];
	int i, j;
	int wkc;

	slave_xml_config_t	*xml_slave_config;
	init_sdo_t			*init_sdo;
	for (i = 0;  i < initconfig_slave_count; ++i)
	{
		xml_slave_config = &slave_xml_configs[i];

		for (j = 0; j < xml_slave_config->init_sdo_count; ++j)
		{
			init_sdo = &(xml_slave_config->init_sdos[j]);

			switch(init_sdo->size)
			{
			case 1:
				EC_WRITE_U8(data, init_sdo->value);
				break;

			case 2:
				EC_WRITE_U16(data, init_sdo->value);
				break;

			case 4:
				EC_WRITE_U32(data, init_sdo->value);
				break;

			case 8:
				EC_WRITE_U64(data, init_sdo->value);
				break;
			default:
				ECK_ERR("Data len invalid. pos = %d, index=0x%x, subindex=0x%x, size=%d, value=%d.",
					i, init_sdo->index, init_sdo->subidx, init_sdo->size, init_sdo->value);
				goto EXIT;
			}

			wkc = ec_SDOwrite(GET_SLAVE_PHYSICAL(i) + 1, init_sdo->index, init_sdo->subidx, FALSE, init_sdo->size, data, EC_TIMEOUTRXM);
			if (1 != wkc)
			{
				ECK_ERR("Sdo download failed. pos = %d, index=0x%x, subindex=0x%x, size=%d, value=%d.",
					i, init_sdo->index, init_sdo->subidx, init_sdo->size, init_sdo->value);
				return -1;
			}

			ECK_INFO("Sdo download success. pos = %d, index=0x%x, subindex=0x%x, size=%d, value=%d.",
				 i, init_sdo->index, init_sdo->subidx, init_sdo->size, init_sdo->value);
		}
	}

	retval = 0;
EXIT:
	return retval;
}

////////////////////////////////////////////////////////////////
//从配置文件解析结果中构建输入输出域中的pdo映射
////////////////////////////////////////////////////////////////
static void init_io_mappings(int position)
{
	if (SLAVE_IO != GET_SLAVE_TYPE(position)) return;
	uint8_t pdo_count 			= slave_xml_configs[position].init_pdo_count;

	uint8_t i, j;
	uint8_t physical;
	slave_entries_info_t *slave_domain_config;
	const init_pdo_t *init_pdo;
	uint16_t cur;
	uint16_t last_bit_offset, last_bit_length;

	physical = slave_xml_configs[position].physical;
	for (i = 0; i < pdo_count; ++i)
	{
		init_pdo = &(slave_xml_configs[position].init_pdos[i]);
		if (PDO_DIR_RX == init_pdo->dir)
		{
			slave_domain_config	= &(domain_configs[DOMAIN_OUT_IDX].entries[physical]);
		}
		else
		{
			slave_domain_config	= &(domain_configs[DOMAIN_IN_IDX].entries[physical]);
		}

		cur = slave_domain_config->n_entries;

		slave_domain_config->n_entries += init_pdo->n_entries;
		slave_domain_config->pdo_infos[slave_domain_config->pdo_num].pdo_index = init_pdo->index;
		slave_domain_config->pdo_infos[slave_domain_config->pdo_num].entries 	 = &(slave_domain_config->entry_vec[cur]);
		slave_domain_config->pdo_infos[slave_domain_config->pdo_num].n_entries = init_pdo->n_entries;

		for (j = 0; j < init_pdo->n_entries; ++j, ++cur)
		{
			slave_domain_config->entry_vec[cur].index 		= init_pdo->entries[j].index;
			slave_domain_config->entry_vec[cur].subindex	= init_pdo->entries[j].subindex;
			slave_domain_config->entry_vec[cur].bit_length 	= init_pdo->entries[j].bitlen;

			if (cur)
			{
				last_bit_offset = slave_domain_config->bit_offset_vec[cur - 1];
				last_bit_length = slave_domain_config->entry_vec[cur - 1].bit_length;

				if (last_bit_offset + last_bit_length >= 8)
				{
					slave_domain_config->byte_offset_vec[cur] =
						slave_domain_config->byte_offset_vec[cur - 1] + (last_bit_offset + last_bit_length) / 8;
				}
				else
				{
					slave_domain_config->byte_offset_vec[cur] = slave_domain_config->byte_offset_vec[cur - 1];
				}
				slave_domain_config->bit_offset_vec[cur] = (last_bit_offset + last_bit_length) % 8;
			}
		}

		slave_domain_config->pdo_num++;
	}
}

////////////////////////////////////////////////////////////////
//从配置文件解析结果中构建输入输出域中的pdo映射
////////////////////////////////////////////////////////////////
static int init_driver_mappings(int logical)
{
	if (SLAVE_AXIS != GET_SLAVE_TYPE(logical)) return -1;

	int physical = GET_SLAVE_PHYSICAL(logical);

	uint8_t pdo_count 			= slave_xml_configs[logical].init_pdo_count;

	uint8_t i, j;

	slave_entries_info_t *slave_domain_config;
	const init_pdo_t *init_pdo;
	uint16_t curr;

	uint8_t found;
	uint16_t front_count, end_count;

	for (i = 0; i < pdo_count; ++i)
	{
		init_pdo = &(slave_xml_configs[logical].init_pdos[i]);
		if (PDO_DIR_RX == init_pdo->dir)
		{
			slave_domain_config	= &(domain_configs[DOMAIN_OUT_IDX].entries[physical]);
		}
		else
		{
			slave_domain_config	= &(domain_configs[DOMAIN_IN_IDX].entries[physical]);
		}

		if ( (slave_domain_config->n_entries + init_pdo->n_entries)> MAX_SLAVE_DOMAIN_ENTRIES)
		{
			return -1;
		}

		//查找PDO是否已经存在
		found = front_count = end_count = 0;
		for(j = 0; j < slave_domain_config->pdo_num; ++j)
		{
			front_count += slave_domain_config->pdo_infos[j].n_entries;
			if (slave_domain_config->pdo_infos[j].pdo_index == init_pdo->index)
			{
				end_count = slave_domain_config->n_entries - front_count;
				found = 1;
				break;
			}
		}

		ec_pdo_entry_info_t		entry_buffer[MAX_SLAVE_DOMAIN_ENTRIES];
		if (found)
		{
			curr = slave_domain_config->pdo_infos[j].entries - slave_domain_config->entry_vec
				+ slave_domain_config->pdo_infos[j].n_entries;

			if (end_count > 0)
				memcpy(entry_buffer, slave_domain_config->pdo_infos[j].entries + slave_domain_config->pdo_infos[j].n_entries, end_count * sizeof(ec_pdo_entry_info_t));

			slave_domain_config->pdo_infos[j].n_entries += init_pdo->n_entries;
		}
		else
		{//在末尾处新增PDO
			curr = slave_domain_config->n_entries;
			slave_domain_config->pdo_infos[slave_domain_config->pdo_num].n_entries = init_pdo->n_entries;
			slave_domain_config->pdo_infos[slave_domain_config->pdo_num].pdo_index = init_pdo->index;
			slave_domain_config->pdo_infos[slave_domain_config->pdo_num].entries 	 = &(slave_domain_config->entry_vec[curr]);
			slave_domain_config->pdo_num++;
		}

		//增加记录条目信息
		for (j = 0; j < init_pdo->n_entries; ++j, ++curr)
		{
			slave_domain_config->entry_vec[curr].index 		= init_pdo->entries[j].index;
			slave_domain_config->entry_vec[curr].subindex	= init_pdo->entries[j].subindex;
			slave_domain_config->entry_vec[curr].bit_length 	= init_pdo->entries[j].bitlen;
			slave_domain_config->logical[curr]				= logical;
			slave_domain_config->meaning[curr]			 	= init_pdo->entries[j].meaning;
		}

		if (found
		    && end_count > 0)
		{
			memcpy(slave_domain_config->entry_vec + curr, entry_buffer,  end_count * sizeof(ec_pdo_entry_info_t));
		}

		slave_domain_config->n_entries += init_pdo->n_entries;
	}

	return 0;
}

////////////////////////////////////////////////////////////////
//CSP模式驱动器PDO条目
////////////////////////////////////////////////////////////////
static void init_out_domain_csp(int logical)
{
	int physical = GET_SLAVE_PHYSICAL(logical);
	slave_entries_info_t	*entries = domain_configs[DOMAIN_OUT_IDX].entries + physical;

	//控制字		Control Word
	entries->entry_vec[0].index 	= 0x6040;
	entries->entry_vec[0].subindex 	= 0x0;
	entries->entry_vec[0].bit_length= 16;
	entries->byte_offset_vec[0]		= 0;
	entries->meaning[0]		= PDO_ENTRY_MEANING_ControlWord;
	entries->logical[0]		= logical;

	//操作模式		Mode of Operation
	entries->entry_vec[1].index 	= 0x6060;
	entries->entry_vec[1].subindex 	= 0x0;
	entries->entry_vec[1].bit_length= 8;
	entries->byte_offset_vec[1]		= 2;
	entries->meaning[1]		= PDO_ENTRY_MEANING_OpMode;
	entries->logical[1]		= logical;

	//目标位置		Target postion
	entries->entry_vec[2].index 	= 0x607A;
	entries->entry_vec[2].subindex 	= 0x0;
	entries->entry_vec[2].bit_length= 32;
	entries->byte_offset_vec[2]		= 3;
	entries->meaning[2]		= PDO_ENTRY_MEANING_TargetPosition;
	entries->logical[2]		= logical;

	entries->n_entries = 3;

	entries->pdo_num = 1;
	entries->pdo_infos[0].pdo_index = 0x1600;
	entries->pdo_infos[0].entries   = entries->entry_vec;
	entries->pdo_infos[0].n_entries = entries->n_entries;
}

static void init_in_domain_csp(int logical)
{
	int physical = GET_SLAVE_PHYSICAL(logical);
	slave_entries_info_t	*entries = domain_configs[DOMAIN_IN_IDX].entries + physical;

	//报警码 	Alarm Code
	entries->entry_vec[0].index 	= 0x603F;
	entries->entry_vec[0].subindex	= 0x0;
	entries->entry_vec[0].bit_length= 16;
	entries->byte_offset_vec[0] 	= 0;
	entries->meaning[0]		= PDO_ENTRY_MEANING_AlarmCode;
	entries->logical[0]		= logical;

	//状态字 	Status Word
	entries->entry_vec[1].index 	= 0x6041;
	entries->entry_vec[1].subindex	= 0x0;
	entries->entry_vec[1].bit_length= 16;
	entries->byte_offset_vec[1] 	= 2;
	entries->meaning[1]		= PDO_ENTRY_MEANING_StatusWord;
	entries->logical[1]		= logical;

	//当前位置		Current Postion
	entries->entry_vec[2].index 	= 0x6064;
	entries->entry_vec[2].subindex	= 0x0;
	entries->entry_vec[2].bit_length= 32;
	entries->byte_offset_vec[2] 	= 4;
	entries->meaning[2]		= PDO_ENTRY_MEANING_ActualPosition;
	entries->logical[2]		= logical;

	//操作模式回显		Modes of Operation Display
	entries->entry_vec[3].index 	= 0x6061;
	entries->entry_vec[3].subindex	= 0x0;
	entries->entry_vec[3].bit_length= 8;
	entries->byte_offset_vec[3] 	= 8;
	entries->meaning[3]		= PDO_ENTRY_MEANING_OpModeDisplay;
	entries->logical[3]		= logical;

	//数字输入		Digital Inputs
	entries->entry_vec[4].index 	= 0x60FD;
	entries->entry_vec[4].subindex	= 0x0;
	entries->entry_vec[4].bit_length= 32;
	entries->byte_offset_vec[4] 	= 9;
	entries->meaning[4]		= PDO_ENTRY_MEANING_DigitalInput;
	entries->logical[4]		= logical;

	entries->n_entries = 5;

	entries->pdo_num = 1;
	entries->pdo_infos[0].pdo_index = 0x1A00;
	entries->pdo_infos[0].entries   = entries->entry_vec;
	entries->pdo_infos[0].n_entries = entries->n_entries;
}

////////////////////////////////////////////////////////////////
//CST模式驱动器PDO条目
////////////////////////////////////////////////////////////////
static void init_out_domain_cst(int logical)
{
	int physical = GET_SLAVE_PHYSICAL(logical);
	slave_entries_info_t	*entries = domain_configs[DOMAIN_OUT_IDX].entries + physical;

	//控制字		Control Word
	entries->entry_vec[0].index 	= 0x6040;
	entries->entry_vec[0].subindex 	= 0x0;
	entries->entry_vec[0].bit_length= 16;
	entries->byte_offset_vec[0]		= 0;
	entries->meaning[0] = PDO_ENTRY_MEANING_ControlWord;
	entries->logical[0]		= logical;

	//操作模式		Mode of Operation
	entries->entry_vec[1].index 	= 0x6060;
	entries->entry_vec[1].subindex 	= 0x0;
	entries->entry_vec[1].bit_length= 8;
	entries->byte_offset_vec[1]		= 2;
	entries->meaning[1] = PDO_ENTRY_MEANING_OpMode;
	entries->logical[1]		= logical;

	//目标扭矩		Target torque
	entries->entry_vec[2].index 	= 0x6071;
	entries->entry_vec[2].subindex 	= 0x0;
	entries->entry_vec[2].bit_length= 16;
	entries->byte_offset_vec[2]		= 3;
	entries->meaning[2] = PDO_ENTRY_MEANING_TargetTorque;
	entries->logical[2]		= logical;

	entries->n_entries = 3;

	entries->pdo_num = 1;
	entries->pdo_infos[0].pdo_index = 0x1600;
	entries->pdo_infos[0].entries   = entries->entry_vec;
	entries->pdo_infos[0].n_entries = entries->n_entries;
}

static void init_in_domain_cst(int logical)
{
	int physical = GET_SLAVE_PHYSICAL(logical);
	slave_entries_info_t	*entries = domain_configs[DOMAIN_IN_IDX].entries + physical;

	int i = 0;
	//报警码 	Alarm Code
	entries->entry_vec[i].index 	= 0x603F;
	entries->entry_vec[i].subindex	= 0x0;
	entries->entry_vec[i].bit_length= 16;
	entries->byte_offset_vec[i] 	= 0;
	entries->byte_offset_vec[i] 	= 0;
	entries->meaning[i] = PDO_ENTRY_MEANING_AlarmCode;
	entries->logical[i] = logical;
	++i;

	//状态字 	Status Word
	entries->entry_vec[i].index 	= 0x6041;
	entries->entry_vec[i].subindex	= 0x0;
	entries->entry_vec[i].bit_length= 16;
	entries->byte_offset_vec[i] 	= entries->byte_offset_vec[i-1] + entries->entry_vec[i-1].bit_length / 8;
	entries->meaning[i] = PDO_ENTRY_MEANING_StatusWord;
	entries->logical[i] = logical;
	++i;

	//当前位置		Current Postion
	entries->entry_vec[i].index 	= 0x6064;
	entries->entry_vec[i].subindex	= 0x0;
	entries->entry_vec[i].bit_length= 32;
	entries->byte_offset_vec[i] 	= entries->byte_offset_vec[i-1] + entries->entry_vec[i-1].bit_length / 8;
	entries->meaning[i] = PDO_ENTRY_MEANING_ActualPosition;
	entries->logical[i] = logical;
	++i;

	//操作模式回显		Modes of Operation Display
	entries->entry_vec[i].index 	= 0x6061;
	entries->entry_vec[i].subindex	= 0x0;
	entries->entry_vec[i].bit_length= 8;
	entries->byte_offset_vec[i] 	= entries->byte_offset_vec[i-1] + entries->entry_vec[i-1].bit_length / 8;
	entries->meaning[i] = PDO_ENTRY_MEANING_OpModeDisplay;
	entries->logical[i] = logical;
	++i;

	//数字输入		Digital Inputs
	entries->entry_vec[i].index 	= 0x60FD;
	entries->entry_vec[i].subindex	= 0x0;
	entries->entry_vec[i].bit_length= 32;
	entries->byte_offset_vec[i] 	= entries->byte_offset_vec[i-1] + entries->entry_vec[i-1].bit_length / 8;
	entries->meaning[i] = PDO_ENTRY_MEANING_DigitalInput;
	entries->logical[i] = logical;
	++i;

	//内部扭矩		Demand Torque
	entries->entry_vec[i].index 	= 0x6074;
	entries->entry_vec[i].subindex	= 0x0;
	entries->entry_vec[i].bit_length= 16;
	entries->byte_offset_vec[i] 	= entries->byte_offset_vec[i-1] + entries->entry_vec[i-1].bit_length / 8;
	entries->meaning[i] = PDO_ENTRY_MEANING_DemandTorque;
	entries->logical[i] = logical;
	++i;

	//扭矩回授		Actual Torque
	entries->entry_vec[i].index 	= 0x6077;
	entries->entry_vec[i].subindex	= 0x0;
	entries->entry_vec[i].bit_length= 16;
	entries->byte_offset_vec[i] 	= entries->byte_offset_vec[i-1] + entries->entry_vec[i-1].bit_length / 8;
	entries->meaning[i] = PDO_ENTRY_MEANING_ActualTorque;
	entries->logical[i] = logical;
	++i;

	entries->n_entries = i;

	entries->pdo_num = 1;
	entries->pdo_infos[0].pdo_index = 0x1A00;
	entries->pdo_infos[0].entries   = entries->entry_vec;
	entries->pdo_infos[0].n_entries = entries->n_entries;
}

static void axis_state_init(void )
{
	int 			i;
	axis_state_t 	*axis_state_ptr;
	for (i = 0 ; i < MAX_SLAVE_COUNT; ++i)
	{
		axis_state_ptr = axis_states + i;
		axis_state_ptr->off_control_word	= (unsigned int)(-1);
		axis_state_ptr->off_opmode			= (unsigned int)(-1);
		axis_state_ptr->off_target_pos		= (unsigned int)(-1);
		axis_state_ptr->off_target_torque	= (unsigned int)(-1);

		axis_state_ptr->off_alarm_code		= (unsigned int)(-1);
		axis_state_ptr->off_status_word		= (unsigned int)(-1);
		axis_state_ptr->off_cur_pos			= (unsigned int)(-1);
		axis_state_ptr->off_demand_torque	= (unsigned int)(-1);
		axis_state_ptr->off_actual_torque	= (unsigned int)(-1);
		axis_state_ptr->off_opmode_display	= (unsigned int)(-1);
		axis_state_ptr->off_digital_input	= (unsigned int)(-1);

		axis_state_ptr->off_output			= (unsigned int)(-1);
		axis_state_ptr->off_input			= (unsigned int)(-1);
		axis_state_ptr->output_size			= 0;
		axis_state_ptr->input_size			= 0;
	}
}

static int init_slave_data_structs(void)
{
	int 				logical, physical;
	slave_type_t		type;
	int					i;
	domain_config_t 	*domain_config;

	axis_state_init();

	for (logical = 0; logical < initconfig_slave_count; ++logical)
	{
		type 		= GET_SLAVE_TYPE(logical);
		physical	= GET_SLAVE_PHYSICAL(logical);
		if (SLAVE_IO == type)
		{
			if (GET_SLAVE_INIT_PDO_COUNT(logical))
			{	//IO模块配置了PDO,按配置文件进行映射
				init_io_mappings(logical);
			}
			else
			{
				ECK_INFO("io module physical %d is not configured in xml.\n", physical);
			}
		}
		else
		{//Axis
			if (GET_SLAVE_INIT_PDO_COUNT(logical))
			{
				init_driver_mappings(logical);
			}
			else
			{//驱动器未配置PDO,使用默认配置
				switch (GET_SLAVE_XML_OPMODE(logical))
				{
				case OPMODE_CST:
					init_out_domain_cst(logical);
					init_in_domain_cst(logical);
					break;
				case OPMODE_CSP:
				default:
					init_out_domain_csp(logical);
					init_in_domain_csp(logical);
					break;
				}
			}
		}
	}

	//计算域中所有pdo条目的字节偏移
	//累加域中所有pdo条目总和
	for (i = 0; i < 2; ++i)
	{
		domain_config = &domain_configs[i];
		for (physical = 0; physical < initconfig_physical_count; ++physical)
		{
			slave_entries_info_t *slave_domain_config = domain_config->entries + physical;
			int curr;
			uint16_t last_bit_offset, last_bit_length;
			for (curr = 0; curr < slave_domain_config->n_entries; ++curr)
			{
				if (curr)
				{
					last_bit_offset = slave_domain_config->bit_offset_vec[curr - 1];
					last_bit_length = slave_domain_config->entry_vec[curr - 1].bit_length;

					if (last_bit_offset + last_bit_length >= 8)
					{
						slave_domain_config->byte_offset_vec[curr] =
							slave_domain_config->byte_offset_vec[curr - 1] + (last_bit_offset + last_bit_length) / 8;
					}
					else
					{
						slave_domain_config->byte_offset_vec[curr] = slave_domain_config->byte_offset_vec[curr - 1];
					}
					slave_domain_config->bit_offset_vec[curr] = (last_bit_offset + last_bit_length) % 8;
				}
			}

			domain_config->total_entries += domain_config->entries[physical].n_entries;
		}
	}

	//根据domain_configs初始化pdo_configs
	for (physical = 0; physical < initconfig_physical_count; ++physical)
	{
		//输出
		for (i = 0; i < domain_configs[DOMAIN_OUT_IDX].entries[physical].pdo_num; ++i)
		{
			pdo_configs[physical].rxpdos[i].index		= domain_configs[DOMAIN_OUT_IDX].entries[physical].pdo_infos[i].pdo_index;
			pdo_configs[physical].rxpdos[i].n_entries	= domain_configs[DOMAIN_OUT_IDX].entries[physical].pdo_infos[i].n_entries;
			pdo_configs[physical].rxpdos[i].entries 	= domain_configs[DOMAIN_OUT_IDX].entries[physical].pdo_infos[i].entries;
		}
		//输入
		for (i = 0; i < domain_configs[DOMAIN_IN_IDX].entries[physical].pdo_num; ++i)
		{
			pdo_configs[physical].txpdos[i].index		= domain_configs[DOMAIN_IN_IDX].entries[physical].pdo_infos[i].pdo_index;
			pdo_configs[physical].txpdos[i].n_entries	= domain_configs[DOMAIN_IN_IDX].entries[physical].pdo_infos[i].n_entries;
			pdo_configs[physical].txpdos[i].entries 	= domain_configs[DOMAIN_IN_IDX].entries[physical].pdo_infos[i].entries;
		}

		//研控Mini3E混合伺服，ethercat pdos 显示SM0 默认配置RxPDO 0x1601，包含0x6060操作模式回显
		//ecrt_slave_config_reg_pdo_entry()时从SM0开始查找对应SM0,导致过程数据映射错误
		//2022-7-4 增加SM0 SM1的配置，ecrt_slave_config_pdo_assign_clear()将清除默认SM0 SM1的配置
		//SM0 接收邮箱
		pdo_configs[physical].syncs[0].index	   = 0;
		pdo_configs[physical].syncs[0].dir		   = EC_DIR_OUTPUT;
		pdo_configs[physical].syncs[0].n_pdos	   = 0;				//清除默认配置
		pdo_configs[physical].syncs[0].pdos		   = NULL;
		pdo_configs[physical].syncs[0].watchdog_mode= EC_WD_DISABLE;

		//SM1 发送邮箱
		pdo_configs[physical].syncs[1].index	   = 1;
		pdo_configs[physical].syncs[1].dir		   = EC_DIR_INPUT;
		pdo_configs[physical].syncs[1].n_pdos	   = 0;				//清除默认配置
		pdo_configs[physical].syncs[1].pdos		   = NULL;
		pdo_configs[physical].syncs[1].watchdog_mode= EC_WD_DISABLE;

		//SM2 RxPdo
		pdo_configs[physical].syncs[2].index		= 2;
		pdo_configs[physical].syncs[2].dir			= EC_DIR_OUTPUT;
		pdo_configs[physical].syncs[2].n_pdos		= domain_configs[0].entries[physical].pdo_num;
		pdo_configs[physical].syncs[2].pdos			= pdo_configs[physical].rxpdos;
		pdo_configs[physical].syncs[2].watchdog_mode= EC_WD_ENABLE;

		//SM3 TxPdo
		pdo_configs[physical].syncs[3].index		= 3;
		pdo_configs[physical].syncs[3].dir			= EC_DIR_INPUT;
		pdo_configs[physical].syncs[3].n_pdos		= domain_configs[1].entries[physical].pdo_num;
		pdo_configs[physical].syncs[3].pdos			= pdo_configs[physical].txpdos;
		pdo_configs[physical].syncs[3].watchdog_mode= EC_WD_ENABLE;

		//结束
		pdo_configs[physical].syncs[4].index		= 0xff;
		pdo_configs[physical].syncs[4].dir			= EC_DIR_INVALID;
		pdo_configs[physical].syncs[4].n_pdos		= 0;
		pdo_configs[physical].syncs[4].pdos			= NULL;
		pdo_configs[physical].syncs[4].watchdog_mode= EC_WD_DEFAULT;
	}

	return 0;
}

static int configure_slave_pdo(int physical, slave_type_t	type, const slave_entries_info_t	*slave_def, ec_direction_t dir)
{
	int wkc;
	uint16 slave;
	uint16 index;
	uint8  subindex;
	uint32 count;
	uint16 count16b;
	uint8_t pdono, entryno;
	uint32_t entry;
	const slave_pdo_info_t		*pdo_def;
	const ec_pdo_entry_info_t	*entry_def;
	int logical;

	slave = physical + 1;

	if (SLAVE_IO == type)
	{
		logical = GET_FIRST_LOGICAL(physical);
		if (0 == GET_SLAVE_INIT_PDO_COUNT(logical))
		{//IO模块未配置PDO映射，使用默认值
			ECK_INFO("physical %d IO module is not configured in XML, keep its default PDO configuration.", physical);
			return 0;
		}
	}

	//PDO定义
	for (pdono = 0; pdono < slave_def->pdo_num; ++pdono)
	{
		pdo_def = &slave_def->pdo_infos[pdono];

		index = pdo_def->pdo_index;
		subindex = 0;
		count = 0;
		wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(count), &count, EC_TIMEOUTRXM);
		if (1 != wkc)
		{
			ECK_ERR("downloading sdo physical:%d index:0x%x sub:0x%d failed.\n", physical, index, subindex);
			return -1;
		}

		for (entryno = 0; entryno < pdo_def->n_entries; ++entryno)
		{
			entry_def = &pdo_def->entries[entryno];
			subindex = entryno + 1;
			entry = MAKE_PDO_ENTRY(entry_def->index, entry_def->subindex, entry_def->bit_length);
			wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(entry), &entry, EC_TIMEOUTRXM);
			if (1 != wkc)
			{
				ECK_ERR("downloading sdo physical:%d index:0x%x sub:0x%d failed.\n", physical, index, subindex);
				return -1;
			}
		}

		subindex = 0;
		count = pdo_def->n_entries;
		wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(count), &count, EC_TIMEOUTRXM);
		if (1 != wkc)
		{
			ECK_ERR("downloading sdo physical:%d index:0x%x sub:0x%d failed.\n", physical, index, subindex);
			return -1;
		}
	}

	//RXPDO PXPDO分配
	index = 0x1C12;
	if (EC_DIR_INPUT == dir)
		index = 0x1C13;

	subindex = 0;
	count16b = 0;
	wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(count16b), &count16b, EC_TIMEOUTRXM);
	if (1 != wkc)
	{
		ECK_ERR("downloading sdo 0x%x 0x%d failed.\n", index, subindex);
		return -1;
	}

	for (pdono = 0; pdono < slave_def->pdo_num; ++pdono)
	{
		pdo_def = &slave_def->pdo_infos[pdono];

		subindex = pdono + 1;
		wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(pdo_def->pdo_index), &(pdo_def->pdo_index), EC_TIMEOUTRXM);
		if (1 != wkc)
		{
			ECK_ERR("downloading sdo 0x%x 0x%d failed.\n", index, subindex);
			return -1;
		}
	}

	subindex = 0;
	count16b = slave_def->pdo_num;
	wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(count16b), &count16b, EC_TIMEOUTRXM);
	if (1 != wkc)
	{
		ECK_ERR("downloading sdo 0x%x 0x%d failed.\n", index, subindex);
		return -1;
	}

	return 0;
}

static void calculate_slave_offset(int physical, slave_type_t	type, const slave_entries_info_t	*slave_def, ec_direction_t dir, unsigned int *offset)
{
	int i;
	uint16 logical;
	const ec_pdo_entry_info_t	*entry_def;
	pdo_entry_meaning_t meaning;
	uint16_t slave_size;
	unsigned int tmpoff = *offset;

	if (EC_DIR_OUTPUT == dir)
	{
		slave_size = ec_slave[physical + 1].Obytes;
	}
	else
	{
		slave_size = ec_slave[physical + 1].Ibytes;
	}

	if (SLAVE_IO == type)
	{
		logical = GET_FIRST_LOGICAL(physical);
		if (0 == GET_SLAVE_INIT_PDO_COUNT(logical))
		{//IO模块未配置PDO映射，使用默认值
			if (EC_DIR_OUTPUT == dir)
			{
				*SLAVE_STATUS_OFF_OUTPUT_PTR(logical) = tmpoff;
				*SLAVE_STATUS_OUTPUT_SIZE_PTR(logical) = slave_size;
			}
			else
			{
				*SLAVE_STATUS_OFF_INPUT_PTR(logical) = tmpoff;
				*SLAVE_STATUS_INPUT_SIZE_PTR(logical) = slave_size;
			}
			goto UPDATE;
		}
	}

	//轴数据偏移位置计算
	for(i = 0; i < slave_def->n_entries; ++i)
	{
		entry_def = &slave_def->entry_vec[i];
		logical   = slave_def->logical[i];
		meaning   = slave_def->meaning[i];

		if (EC_DIR_OUTPUT == dir)
		{
			if (-1 == *SLAVE_STATUS_OFF_OUTPUT_PTR(logical))
			{
				*SLAVE_STATUS_OFF_OUTPUT_PTR(logical) = tmpoff;
			}
		}
		else
		{
			if (-1 == *SLAVE_STATUS_OFF_INPUT_PTR(logical))
			{
				*SLAVE_STATUS_OFF_INPUT_PTR(logical) = tmpoff;
			}
		}

		switch(meaning)
		{
		case PDO_ENTRY_MEANING_ControlWord		://控制字
			*AXIS_STATUS_OFF_CONTROL_WORD_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_OUTPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_OpMode			://操作模式
			*AXIS_STATUS_OFF_OPMODE_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_OUTPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_TargetPosition	://目标位置
			*AXIS_STATUS_OFF_TARGET_POS_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_OUTPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_TargetTorque 	://目标扭矩
			*AXIS_STATUS_OFF_TARGET_TORQUE_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_OUTPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_AlarmCode		://告警码
			*AXIS_STATUS_OFF_ALARM_CODE_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_INPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_StatusWord		://状态字
			*AXIS_STATUS_OFF_STATUS_WORD_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_INPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_ActualPosition	://实际位置
			*AXIS_STATUS_OFF_CUR_POS_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_INPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_OpModeDisplay	://操作模式回显
			*AXIS_STATUS_OFF_OPMODE_DISPLAY_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_INPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_DigitalInput 	://数字DI
			*AXIS_STATUS_OFF_DIGITAL_INPUT_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_INPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_DemandTorque 	://内部扭矩
			*AXIS_STATUS_OFF_DEMAND_TORQUE_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_INPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		case PDO_ENTRY_MEANING_ActualTorque 	://扭矩回授
			*AXIS_STATUS_OFF_ACTUAL_TORQUE_PTR(logical) = tmpoff;
			tmpoff += (entry_def->bit_length / 8);
			*SLAVE_STATUS_INPUT_SIZE_PTR(logical) += (entry_def->bit_length / 8);
			break;
		default:
			break;
		}

	}

UPDATE:

	*offset += slave_size;

	return ;
}

static void init_process_data(void)
{
	uint16_t i;
	for(i = 0; i < initconfig_slave_count; ++i)
	{
		if (SLAVE_AXIS == GET_SLAVE_TYPE(i))
			SET_AXIS_OPMODE(i, GET_SLAVE_XML_OPMODE(i))		//设置操作模式
	}

	for (i = 0; i < output_port_num; ++i)
		IO_STATUS_SET_OUT_DATA(i, 0x00);

	period_startout = 1;
}

static int configure_pdo(void)
{
	int i;
	uint16 slave;
	int physical;
	unsigned int off;
	domain_config_t 		*domain_config;
	slave_entries_info_t	*slave_def;
	slave_type_t	type;
	int	io_map_size;

	//重置数据
	memset(domain_configs, 0, sizeof(domain_configs));
	domain_configs[0].dir = EC_DIR_OUTPUT;
	domain_configs[1].dir = EC_DIR_INPUT;
	memset(pdo_configs, 0, sizeof(pdo_configs));

	//生成数据到domain_configs中
	init_slave_data_structs();

	for (i = 0; i < 2; ++i)
	{
		domain_config = &domain_configs[i];

		for (physical = 0; physical < initconfig_physical_count; ++physical)
		{
			type = GET_SLAVE_TYPE(GET_FIRST_LOGICAL(physical));
			slave_def = &domain_config->entries[physical];

			//从站PDO设置
			if (configure_slave_pdo(physical, type, slave_def, domain_config->dir))
			{
				ECK_ERR("configure_axis_pdo() failed physical:%d dir:%u.\n", physical, domain_config->dir);
				return -1;
			}
		}
	}

	ECK_INFO("starting configure mappings...");
	io_map_size = ec_config_map(domain0_pd);
	ECK_INFO("iomap size :%d.", io_map_size);

	for (slave = 0; slave <= initconfig_physical_count; ++slave)
	{
		pr_info("Slave[%d]outputs %uB @0x%px , inputs %uB @0x%px\n", slave,
			ec_slave[slave].Obytes, ec_slave[slave].outputs,
			ec_slave[slave].Ibytes,ec_slave[slave].inputs);
	}

	domain1_pd = ec_slave[0].inputs;
	domain0_size = ec_slave[0].Obytes;
	domain1_size = ec_slave[0].Ibytes;

	for (i = 0; i < 2; ++i)
	{
		domain_config = &domain_configs[i];
		off = 0;
		for (physical = 0; physical < initconfig_physical_count; ++physical)
		{
			type = GET_SLAVE_TYPE(GET_FIRST_LOGICAL(physical));
			slave_def = &domain_config->entries[physical];
			//从站偏移计算
			calculate_slave_offset(physical, type, slave_def, domain_config->dir, &off);
		}
	}

	//IO模块偏移计算
	calculate_io_offset();

	init_process_data();
	return 0;
}

static int wait_dc_synced(void)
{
	int slave, wkc;
	int32_t val;
	uint16	adr;
	uint32  abs_sync_diff;
	int32  retries;
	slave_type_t type;
	unsigned long timeout;

	for (slave = 1; slave <= initconfig_physical_count; ++slave)
	{
		pr_info("Slave[%d] hasdc:%d pdelay:%d.\n", slave, ec_slave[slave].hasdc, ec_slave[slave].pdelay);
		if (!ec_slave[slave].hasdc)
			continue;

#ifdef CONFIG_ECAT_IO_FREERUN
		//IO模块采取FreeRun模式,跳过等待同步
		type = GET_SLAVE_TYPE(GET_FIRST_LOGICAL(slave - 1));
		if (SLAVE_IO == type)
			continue;
#endif

		adr = ec_slave[slave].configadr;
		retries = 5;
		do
		{
			wkc = ec_FPRD(adr, ECT_REG_DCSYSDIFF, sizeof(val), &val, EC_TIMEOUTRET);
			if (wkc > EC_NOFRAME)
			{
				abs_sync_diff = etohl(val) & 0x7fffffff;
				if (abs_sync_diff < EC_DC_MAX_SYNC_DIFF_NS)
				{
					pr_info("Slave[%d] synced with ref-clock, timediff : %u(ns).\n", slave, abs_sync_diff);
					break;
				}
				else
				{
					pr_info("Slave[%d] syncing... timediff : %u(ns).\n", slave, abs_sync_diff);
				}
			}

			if (retries-- <= 0)
			{
				period_warn |= PERIOD_WNOTSYNC;
				if (wkc > EC_NOFRAME)
					pr_warn("Slave[%d] not synced, timediff : %u(ns).", slave, abs_sync_diff);
				else
					pr_warn("Slave[%d] not synced, timediff read failed.", slave);
				break;
			}

			timeout = msleep_interruptible(1000);
			if (timeout)
			{//收到用户空间信号
				return -1;
			}
			if (period_run == 0) return -1;
		}while(1);

	}

	return 0;
}

//配置从站0x980 Register Cyclic Unit Control
//0x981 Register Activation register
//Master.xml中从站配置比如：<DCSync AssignActivate="0x0300"/>
//AssignActivate默认配置值0x0300
//注意对比ESI的<AssignActivate>标签
static int configure_sync0(void)
{
	uint16 slave;
	uint16 logical, assignAct;
	uint16 sync0, sync1;
	slave_type_t type;

	int32 cycleShift = initconfig_dc_start_shift_us * 1000;
	uint32 cycleTime = CONFIG_ECAT_DC_CYCLE * 1000;
	for (slave = 1; slave <= initconfig_physical_count; ++slave)
	{
		if (!ec_slave[slave].hasdc)
			continue;

#ifdef CONFIG_ECAT_IO_FREERUN
		//IO模块采取FreeRun模式
		type = GET_SLAVE_TYPE(GET_FIRST_LOGICAL(slave - 1));
		if (SLAVE_IO == type)
			continue;
#endif
		logical = slave - 1;
		assignAct = slave_xml_configs[logical].assign_activate;
		ECK_INFO("logical[%u] AssignActivate:0x%x\n", logical, assignAct);
		sync0 = assignAct & 0x0200;
		sync1 = assignAct & 0x0400;

		if (sync0 && sync1)
			ec_dcsync01(slave, TRUE, cycleTime,  0, cycleShift);
		else if (sync0)
			ec_dcsync0(slave, TRUE, cycleTime, cycleShift);
		else if (sync1)
		{
			ECK_ERR("logical[%u] slave DC AssignActivate not supported", logical);
			return -1;
		}
		else
			ec_dcsync0(slave, FALSE, cycleTime, cycleShift);
	}

	return 0;
}

int ecbus_do_work_preop(void)
{
	int retry;
	retry = 3;
	//BWR 0x900寄存器
	do{
		bwr900_state = 0;
		tick_sethook(bwr900_hook);

		while (tick_gethook() != NULL)
			msleep(10);

		if (bwr900_wkc > 0)
			break;
		--retry;
	}while(retry);

	if (!retry)
	{
		period_err |= PERIOD_BWR900;
		ECK_ERR("BWR 0x900 failed, retry:%d.\n", retry);
		return -1;
	}

	//写系统时钟偏移寄存器[0x0920:0x0927], 计算传播时延，写传播时延寄存器0x0928:0x092B
	boolean hasdc= nicdrv_configdc(bwr900_mastertime);
	if (FALSE == hasdc)
	{
		period_err |= PERIOD_EDC;
		ECK_ERR("No DC capable slave found.\n");
		return -1;
	}

	period_start();
	msleep(20);

	//在PREOP状态下载所有从站初始化SDO
	retry = 3;
	do
	{
		if (download_init_sdos())
		{
			ECK_ERR("download_init_sdos() failed, retry:%d.\n", retry);
			msleep(100);
		}
		else{
			break;
		}
		--retry;
	}while(retry);

	if (!retry)
	{
		period_err |= PERIOD_ESDO;
		pr_err("download_init_sdos() failed, retry:%d.\n", retry);
		goto OUT_STOP_PERIOD;
	}

	//在PREOP状态配置PDO
	retry = 3;
	do
	{
		if (configure_pdo())
		{
			pr_err("configure_pdo() failed, retry:%d.\n", retry);
			msleep(100);
		}
		else{
			break;
		}
		--retry;
	}while(retry);

	if (!retry)
	{
		period_err |= PERIOD_EPDO;
		pr_err("configure_pdo() failed, retry:%d.\n", retry);
		goto OUT_STOP_PERIOD;
	}

	//在PREOP状态等待所有从站时钟同步完成 寄存器0x092C < 10us
	if (wait_dc_synced())
	{
		pr_err("wait_dc_synced() failed.\n");
		goto OUT_STOP_PERIOD;
	}

	//在PREOP状态配置DC SYNC0使能 SYNC1禁能, 设置起始Sync信号时间 [0x0990:0x0997]
	if (configure_sync0())
	{
		period_err |= PERIOD_ESYNC0;
		pr_err("configure_sync0() failed.\n");
		goto OUT_STOP_PERIOD;
	}

	//各从站延时SyncDelay(100ms)后产生SYNC0，再产生SYNC0之后进入SAFEOP???
	pr_info("delay before entering safeop.\n");
	msleep(CONFIG_ECAT_ENTER_SAFEOP_DELAY);
	pr_info("delay %d(ms) before entering safeop.\n", CONFIG_ECAT_ENTER_SAFEOP_DELAY);

	return 0;

OUT_STOP_PERIOD:
	return -1;
}

static void ecbus_configure_process_data_watchdog(void)
{
	int wkc;
	uint16 wd_pd = 30;
	uint8 wd_counter = 0;

	//Process Data Watchdog超时计数清0
	wkc =  ec_BWR(0, 0x0442, sizeof(wd_counter), &wd_counter, EC_TIMEOUTRET);
	pr_info("clear watchdog counter returns wkc=%d\n", wkc);

	//Process Data Watchdog超时时间设置为3ms
	wkc =  ec_BWR(0, 0x0420, sizeof(wd_pd), &wd_pd, EC_TIMEOUTRET);
	pr_info("configure WD_PD returns wkc=%d\n", wkc);
}

int 	ecbus_wait_enter_safeop(void)
{
	uint8	success = 0;
	uint16 state;
	int retry = 10;
	int slave;
	unsigned long timeout;

	while (retry--)
	{
		ec_slave[0].state = EC_STATE_SAFE_OP;
		ec_writestate(0);
		state = ec_statecheck(0, EC_STATE_SAFE_OP,	EC_TIMEOUTSTATE * 4);

		if (EC_STATE_SAFE_OP == state)
		{
			success = 1;
			break;
		}

		pr_info("waiting enter safe op... current state : 0x%x.\n", state);

		if (state & EC_STATE_ERROR)
		{
			ec_readstate();
			for (slave = 1; slave <= ec_slavecount; slave++)
			{
				pr_info("slave[%d] in state:0x%x\n", slave, ec_slave[slave].state);
				if (ec_slave[slave].state & EC_STATE_ERROR)
				{
					ec_slave[slave].state |= EC_STATE_ACK;
					ec_writestate(slave);
				}
			}
		}
		timeout = msleep_interruptible(1000);
		if (timeout)
		{//收到用户空间信号
			return -1;
		}
	}

	if (!success)
		return -1;

	ec_readstate();
	for (slave = 1; slave <= ec_slavecount; slave++)
	{
		pr_info("slave[%d] in state:0x%x\n", slave, ec_slave[slave].state);
	}

	pr_info("slaves all in safe op.\n");
	msleep(CONFIG_ECAT_ENTER_OP_DELAY);
	pr_info("delay %d(ms) before entering op.\n", CONFIG_ECAT_ENTER_OP_DELAY);
	return 0;
}

int 	ecbus_wait_enter_op(void)
{
	uint8	success = 0;
	uint16 state;
	int retry = 10;
	int slave;
	unsigned long timeout;

	//ec_slave[0].state = EC_STATE_OPERATIONAL;
	//ec_writestate(0);
	while (retry--)
	{
		ec_slave[0].state = EC_STATE_OPERATIONAL;
		ec_writestate(0);

#if 1
		state = ec_statecheck(0, EC_STATE_OPERATIONAL,	EC_TIMEOUTSTATE * 4);
		if (EC_STATE_OPERATIONAL == state)
		{
			success = 1;
			break;
		}
#else
		pr_info("delay 5s ...\n");
		success = 1;
		msleep(5000);
		break;
#endif

		ec_readstate();
		for (slave = 1; slave <= ec_slavecount; slave++)
		{
			pr_info("slave[%d] in state:0x%x\n", slave, ec_slave[slave].state);
			if (ec_slave[slave].state & EC_STATE_ERROR)
			{
				ec_slave[slave].state |= EC_STATE_ACK;
				ec_writestate(slave);
			}
		}

		pr_info("waiting enter op... current state :0x%x.", state);
		timeout = msleep_interruptible(1000);
		if (timeout)
		{//收到用户空间信号
			return -1;
		}
	}

	if (!success)
		return -1;

	expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
	pr_info("slaves all in op, expectedWKC=%d.\n", expectedWKC);
	period_op = 1;

	return 0;
}

void ecbus_check(int wkc)
{
	if (expectedWKC == wkc)
	{
		master_state->ec_bus_inited = 1;
		master_state->accumlated = 0;
	}
	else if (master_state->ec_bus_inited)	//总线初始化完成之后开始累计
	{
		++master_state->accumlated;
	}

	if (master_state->accumlated > DOMAIN_WKC_TOLERANCE)
	{
		master_state->rt_lrw_ready = 0;
		master_state->ec_bus_error = 1; //报警变量置1
		master_state->ec_bus_inited = 0;
	}
	else
		master_state->rt_lrw_ready = 1;

	master_state->ms.slaves_responding = initconfig_physical_count;
}

int ecbus_activate(void)
{
	period_stop();
	msleep(20);

	master_state->accumlated = 0;
	master_state->rt_lrw_ready = 0;
	master_state->ec_bus_inited = 0;
	master_state->al_status = EC_STATE_INIT;

	if (ecbus_wait_enter_preop())
	{
		ECK_ERR("enter preop failed.\n");
		goto OUT_ERR;
	}
	master_state->al_status = EC_STATE_PRE_OP;

	if (ecbus_do_work_preop())
	{
		ECK_ERR("do work preop failed.\n");
		goto OUT_ERR;
	}
	master_state->al_status = EC_STATE_BOOT;

#ifdef CONFIG_ECAT_SETUP_PDWD
	//设置Process Data Watchdog功能，取决于SM  的是否使能看门狗
	//S.A. ethercatmain.h 将#define EC_SMENABLEMASK 0xfffeffbf开启所有SM的WatchDog
	ecbus_configure_process_data_watchdog();
#endif

	if (ecbus_wait_enter_safeop())
	{
		ECK_ERR("enter safeop failed.\n");
		goto OUT_ERR;
	}
	master_state->al_status = EC_STATE_SAFE_OP;

	if (ecbus_wait_enter_op())
	{
		ECK_ERR("enter op failed.\n");
		goto OUT_ERR;
	}
	master_state->al_status = EC_STATE_OPERATIONAL;

	return 0;

OUT_ERR:
	master_state->al_status = EC_STATE_NONE;
	period_stop();
	msleep(20);
	return -EACCES;
}

static int bus_reset_proc(void *data)
{
	int ret;
	ret = ecbus_activate();

	ecbus_reset_kthread = NULL;

	module_put_and_kthread_exit(0);
}

int ecbus_reset(void)
{
	if (ecbus_reset_kthread != NULL)
		return -EACCES;

	//创建内核线程并运行
	if (!try_module_get(THIS_MODULE))
		return -EACCES;

	master_state->al_status = EC_STATE_INIT;
	ecbus_reset_kthread = kthread_run(bus_reset_proc, NULL, "kecbusreset");
	if (IS_ERR(ecbus_reset_kthread))
		goto out_put_module;

	return 0;

out_put_module:
	module_put(THIS_MODULE);

	return -EACCES;
}
