#include <linux/netdevice.h>

#include "osal.h"

#define NET_TIME_MEASURE 0
#define MAX_PACKET_LEN  (1518)

#if HUSY_CHECKED
extern struct net_device *w5500_netdev;
extern int w5500_packet_read_irq(struct net_device *ndev, u8 *buf, int buflen);
extern void w5500_packet_write_irq(struct net_device *netdev, u16 len, const u8 *data);
extern void w5500_packet_queue_irq(struct net_device *netdev, u16 len, const u8 *data);
extern void w5500_packet_send_irq(struct net_device *netdev);
#else
struct net_device *w5500_netdev;
int w5500_packet_read_irq(struct net_device *ndev, u8 *buf, int buflen) {return 0;}
void w5500_packet_write_irq(struct net_device *netdev, u16 len, const u8 *data){return;}
void w5500_packet_queue_irq(struct net_device *netdev, u16 len, const u8 *data){return;}
void w5500_packet_send_irq(struct net_device *netdev){return ;}
#endif

int net_init(const char *ifname)
{
	int ret = 0;

	if (NULL == w5500_netdev)
	{
		pr_err("W5500 device not found!\n");
		return FALSE;
	}

	if ((ret = w5500_netdev->netdev_ops->ndo_open(w5500_netdev)))
	{
		pr_err("enc28j60_net_open() failed with %d!\n", ret);
		return FALSE;
	}
	
	return TRUE;
}

int net_finish(void)
{
	if (w5500_netdev)
	{
		w5500_netdev->netdev_ops->ndo_stop(w5500_netdev);
	}
	return TRUE;
}

int net_queue_and_send (uint8_t *packet, int length)
{
	if (length < 0 
		|| length > MAX_PACKET_LEN)
		return FALSE;

	w5500_packet_write_irq(w5500_netdev, (u16)length, packet);
	return TRUE;
}

int net_queue(uint8_t *packet, int length)
{
	if (length < 0 
		|| length > MAX_PACKET_LEN)
		return FALSE;
	
	w5500_packet_queue_irq(w5500_netdev, (u16)length, packet);

	return TRUE;
}

int net_send(void)
{
	w5500_packet_send_irq(w5500_netdev);
	return TRUE;
}

/*
*接收网卡数据
*返回： 0       网卡未接收到数据
*	    -1   接收到的数据非EtherCAT帧
*       >0   EtherCAT帧长度
*/
int net_recv (uint8_t * packet, size_t size)
{
	struct ethhdr *ethhdr;
	u16 type;

	int recv_len = 0;

	recv_len = w5500_packet_read_irq(w5500_netdev, packet, size);

	if (recv_len <= 0)
	{//网卡无数据
		return 0;
	}

	if (recv_len < ETH_HLEN
		|| recv_len > size)
	{
		w5500_netdev->stats.rx_length_errors++;
		return -1;
	}
	ethhdr = (struct ethhdr *)(packet);
	type = ethhdr->h_proto;

	if (type != htons(ETH_P_ETHERCAT))
	{
		w5500_netdev->stats.rx_frame_errors++;
		return -1;
	}

	return recv_len;
}

void net_show_stats(struct seq_file *seq, void *v)
{
	struct net_device *ndev = w5500_netdev;
	if (!ndev) return;
	
	seq_printf(seq, "------W5500------\n");
	seq_printf(seq, "tx_packets:%ld rx_packets:%ld\n", ndev->stats.tx_packets, ndev->stats.rx_packets);
	seq_printf(seq, "rx_dropped:%ld rx_missed_errors:%ld\n",  ndev->stats.rx_dropped, ndev->stats.rx_missed_errors);
	seq_printf(seq, "rx_length_errors:%ld rx_frame_errors:%ld\n",  ndev->stats.rx_length_errors, ndev->stats.rx_frame_errors);
	seq_printf(seq, "tx_dropped:%ld tx_errors:%ld\n",  ndev->stats.tx_dropped, ndev->stats.tx_errors);
	seq_printf(seq, "tx_fifo_errors:%ld\n",  ndev->stats.tx_fifo_errors);
}

