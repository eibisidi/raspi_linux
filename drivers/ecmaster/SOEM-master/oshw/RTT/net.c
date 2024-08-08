#define LOG_TAG              "net"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

#include <rtthread.h>
#include <raspi4.h>
#include "netif/ethernetif.h"
#include "ethernet.h"
#include "lwip/tcpip.h"
#include "ethercattype.h"

#define NET_TIME_MEASURE 0

//static struct netif *netif = RT_NULL;
static struct pbuf *tx_pbuf = RT_NULL;
extern void bcmgenet_turnoff_irq();

extern int bcmgenet_gmac_eth_send(rt_uint32_t packet, int length, struct pbuf *p);
extern rt_uint32_t bcmgenet_gmac_eth_recv(rt_uint8_t **packetp);

int net_init(const char *ifname)
{
	tx_pbuf = pbuf_alloc(PBUF_RAW, EC_MAXECATFRAME, PBUF_RAM);
	if (RT_NULL == tx_pbuf)
	{	
		LOG_E("pbuf_alloc()  error.");
		return 0;
	}

	return 1;
}

int net_finish()
{
	if (tx_pbuf) 
	{
		pbuf_free(tx_pbuf);
		tx_pbuf = 0;
	}
	return 1;
}

int net_send (void *packet, int length)
{
	int ret = 0;

	RT_ASSERT(length <= EC_MAXECATFRAME);
	rt_memcpy(tx_pbuf->payload, packet, length);

	tx_pbuf->len = tx_pbuf->tot_len = length;
	
	bcmgenet_gmac_eth_send((rt_uint32_t)SEND_DATA_NO_CACHE, tx_pbuf->tot_len, tx_pbuf);
	ret = 1;

	return ret;
}

/*
*接收网卡数据
*返回： 0       网卡未接收到数据
*	    -1   接收到的数据非EtherCAT帧
*       >0   EtherCAT帧长度
*/
int net_recv (uint8_t * packet, size_t size)
{
	struct eth_hdr *ethhdr;
	u16_t type;

	int recv_len = 0;
	rt_uint8_t *addr_point = RT_NULL;

#if NET_TIME_MEASURE
	rt_uint64_t t0 , t1;
	GET_CNTPCT(t0);
#endif
	recv_len = bcmgenet_gmac_eth_recv(&addr_point);
#if NET_TIME_MEASURE
	GET_CNTPCT(t1);
#endif

	if (recv_len <= 0)
	{//网卡无数据
		//rt_kprintf("no data\n");
		return 0;
	}

	if (recv_len < SIZEOF_ETH_HDR
		|| recv_len > size)
	{
		//rt_kprintf("length wrong data\n");
		return -1;
	}
	ethhdr = (struct eth_hdr *)(addr_point);
	type = ethhdr->type;

	if (type != PP_HTONS(ETHTYPE_ETHERCAT))
	{
		//ulog_hex("net", 16, addr_point, recv_len);
	
		//rt_kprintf("type wrong data\n");
		return -1;
	}

	//rt_kprintf("right data\n");
	rt_memcpy(packet, addr_point, recv_len);

	return 1;
}

int net_test(void)
{
	unsigned char frame[] = "\xff\xff\xff\xff\xff\xff\x01\x01\x01\x01\x01\x01\x88\xa4\x0d\x10\
	\x08\x01\x00\x00\x03\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\
	\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\
	\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";

	//net_init("e0");
	net_send(frame, sizeof(frame));
	//net_finish();
	return 0;
}

MSH_CMD_EXPORT(net_test, test net);

#if 0
static uint8 run;
static void net_poll_thread_entry(void* parameter)
{
    struct eth_device* device;
	struct pbuf *p;
	
	device = ( struct eth_device*)netif->state;
    while (run)
    {
        /* receive all of buffer */
        while (1)
        {
            if(device->eth_rx == RT_NULL) break;

            p = device->eth_rx(&(device->parent));
            if (p != RT_NULL)
            {
                /* notify to upper layer */
                if( device->netif->input(p, device->netif) != ERR_OK )
                {
                    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: Input error\n"));
                    pbuf_free(p);
                    p = NULL;
                }
            }
            else break;
        }

	   rt_thread_mdelay(1);
    }
}

static char poll_thread_stack[4096];
static struct rt_thread poll_thread;

int net_start_poll()
{
	if (netif == RT_NULL)
	{
		if (FALSE == net_init("e0"))
		{
			LOG_E("net_init() failed.");
			return -1;
		}
	}
	 rt_thread_init(&poll_thread,
                   "poll_thread",
                   net_poll_thread_entry,
                   RT_NULL,
                   &poll_thread_stack[0],
                   sizeof(poll_thread_stack),
                   5, 20);

	run = 1;
    rt_thread_startup(&poll_thread);

	return 0;
}
MSH_CMD_EXPORT(net_start_poll, net_start_poll);

int net_end_poll()
{
	run = 0;
	return 0;
}
MSH_CMD_EXPORT(net_end_poll, net_end_poll);
#endif
