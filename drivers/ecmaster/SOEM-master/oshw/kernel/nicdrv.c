#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/delay.h> 

#include "osal.h"
#include "oshw.h"
#include "ethercatbase.h"
#include "globals.h"
#include "net.h"

#define EC_NRT_MAXBUF          (16)
#define EC_RT_STARTBUF		   (EC_NRT_MAXBUF)
#define EC_RT_SYNCBUF		   (EC_MAXBUF - 1)		/*FRMW单独占用最后的buf*/
#define EC_RT_ENDBUF		   (EC_RT_SYNCBUF)
#define EC_RT_MAXBUF		   (EC_RT_ENDBUF - EC_RT_STARTBUF)
#define SIDE_RING_SIZE		   (16)
#define EC_SND_RETRY		   (3)
#define EC_QUEUE_RD_092C	   (0)					/*在LRW报文后增加读取DC从站092C寄存器，用于抓取时间差距报文进行分析*/

#define PORTM0 0x01
#define PORTM1 0x02
#define PORTM2 0x04
#define PORTM3 0x08

enum {
	NICDRV_EOK = 0,
	NICDRV_ESEND = 1,
};

/*Ring Buffer between rt side and nrt side*/
struct SideRing{
	ecx_portt * ports[SIDE_RING_SIZE];
	uint8 		indexes[SIDE_RING_SIZE];
	int 		stacknumbers[SIDE_RING_SIZE];
	uint8		wr;
	uint8		rd;
}side_ring;

#ifndef MAX
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/** Redundancy modes */
enum
{
   /** No redundancy, single NIC mode */
   ECT_RED_NONE,
   /** Double redundant NIC connection */
   ECT_RED_DOUBLE
};

/** Primary source MAC address used for EtherCAT.
 * This address is not the MAC address used from the NIC.
 * EtherCAT does not care about MAC addressing, but it is used here to
 * differentiate the route the packet traverses through the EtherCAT
 * segment. This is needed to find out the packet flow in redundant
 * configurations. */
//const uint16 priMAC[3] = { 0x3ad8, 0x8ddd, 0xfad6 };
const uint16 priMAC[3] = { 0x0101, 0x0101, 0x0101 };
//const uint16 priMAC[3] = { 0xf1fa, 0x1df3, 0xa4bc };
/** Secondary source MAC address used for EtherCAT. */
const uint16 secMAC[3] = { 0x0404, 0x0404, 0x0404 };

/** second MAC word is used for identification */
#define RX_PRIM priMAC[1]
/** second MAC word is used for identification */
#define RX_SEC secMAC[1]

extern int ecx_pullindex(ecx_contextt *context);
extern void ecx_pushindex(ecx_contextt *context, uint8 idx, void *data, uint16 length, uint16 DCO);
extern void ecx_clearindex(ecx_contextt *context);

static inline uint8 side_ring_full(void)
{
	if (((side_ring.wr + 1) % SIDE_RING_SIZE) == side_ring.rd)
		return 1;
	return 0;
}

static inline uint8 side_ring_empty(void)
{
	if (side_ring.wr == side_ring.rd)
		return 1;
	return 0;
}

static void ecx_clear_rxbufstat(int *rxbufstat)
{
   int i;
   for(i = 0; i < EC_MAXBUF; i++)
   {
      rxbufstat[i] = EC_BUF_EMPTY;
   }
}

/** Basic setup to connect NIC to socket.
 * @param[in] port        = port context struct
 * @param[in] ifname      = Name of NIC device, f.e. "eth0"
 * @param[in] secondary   = if >0 then use secondary stack instead of primary
 * @return >0 if succeeded
 */
int ecx_setupnic(ecx_portt *port, const char *ifname, int secondary)
{
	int i;
	int *psock;

	if (secondary)
	{
	  /* secondary port struct available? */
	  if (port->redport)
	  {
	     /* when using secondary socket it is automatically a redundant setup */
	     psock = &(port->redport->sockhandle);
	     *psock = -1;
	     port->redstate                   = ECT_RED_DOUBLE;
	     port->redport->stack.sock        = &(port->redport->sockhandle);
	     port->redport->stack.txbuf       = &(port->txbuf);
	     port->redport->stack.txbuflength = &(port->txbuflength);
	     port->redport->stack.tempbuf     = &(port->redport->tempinbuf);
	     port->redport->stack.rxbuf       = &(port->redport->rxbuf);
	     port->redport->stack.rxbufstat   = &(port->redport->rxbufstat);
	  	 port->stack.sndtime	  		  = &(port->redport->sndtime);
	     port->redport->stack.rxsa        = &(port->redport->rxsa);
	     ecx_clear_rxbufstat(&(port->redport->rxbufstat[0]));
	  }
	  else
	  {
	     /* fail */
	     return 0;
	  }
	}
	else
	{
	  mutex_init(&port->getindex_mutex);
	  mutex_init(&port->tx_mutex);
	  mutex_init(&port->rx_mutex);
	  port->sockhandle        = -1;
	  port->lastidx           = 0;
	  port->lastidx_rt		  = EC_RT_STARTBUF;
	  port->redstate          = ECT_RED_NONE;
	  port->stack.sock        = &(port->sockhandle);
	  port->stack.txbuf       = &(port->txbuf);
	  port->stack.txbuflength = &(port->txbuflength);
	  port->stack.tempbuf     = &(port->tempinbuf);
	  port->stack.rxbuf       = &(port->rxbuf);
	  port->stack.rxbufstat   = &(port->rxbufstat);
	  port->stack.sndtime	  = &(port->sndtime);
	  port->stack.rxsa        = &(port->rxsa);
	  ecx_clear_rxbufstat(&(port->rxbufstat[0]));
	  psock = &(port->sockhandle);
	  port->error			  = NICDRV_EOK;
	  port->txs				  = 0;
	  port->rxs				  = 0;
	  if(net_init(ifname) == FALSE){
			pr_err("net_init() failed."); 
			return 0; //fail
	  }
	}

	/* setup ethernet headers in tx buffers so we don't have to repeat it */
	for (i = 0; i < EC_MAXBUF; i++)
	{
	  ec_setupheader(&(port->txbuf[i]));
	  port->rxbufstat[i] = EC_BUF_EMPTY;
	}
	ec_setupheader(&(port->txbuf2));

	side_ring.rd = side_ring.wr = 0;

	return 1;
}

/** Close sockets used
 * @param[in] port        = port context struct
 * @return 0
 */
int ecx_closenic(ecx_portt *port)
{
	mutex_destroy(&port->getindex_mutex);
	mutex_destroy(&port->tx_mutex);
	mutex_destroy(&port->rx_mutex);
	net_finish();

   return 0;
}

/** Fill buffer with ethernet header structure.
 * Destination MAC is always broadcast.
 * Ethertype is always ETH_P_ECAT.
 * @param[out] p = buffer
 */
void ec_setupheader(void *p)
{
   ec_etherheadert *bp;
   bp = p;
   bp->da0 = oshw_htons(0xffff);
   bp->da1 = oshw_htons(0xffff);
   bp->da2 = oshw_htons(0xffff);
   bp->sa0 = oshw_htons(priMAC[0]);
   bp->sa1 = oshw_htons(priMAC[1]);
   bp->sa2 = oshw_htons(priMAC[2]);
   bp->etype = oshw_htons(ETH_P_ECAT);
}

static uint8 ecx_getindex_nrt(ecx_portt *port)
{
   uint8 idx;
   uint8 cnt;

   WARN_ON(in_interrupt());

   mutex_lock(&port->getindex_mutex);

   idx = port->lastidx + 1;
   /* index can't be larger than buffer array */
   if (idx >= EC_NRT_MAXBUF)
   {
	  idx = 0;
   }
   cnt = 0;
   /* try to find unused index */
   while ((port->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_NRT_MAXBUF))
   {
	  idx++;
	  cnt++;
	  if (idx >= EC_NRT_MAXBUF)
	  {
		 idx = 0;
	  }
   }
   port->rxbufstat[idx] = EC_BUF_ALLOC;
   if (port->redstate != ECT_RED_NONE)
   {
	  port->redport->rxbufstat[idx] = EC_BUF_ALLOC;
   }
   port->lastidx = idx;

   mutex_unlock(&port->getindex_mutex);

   return idx;
}

static uint8 ecx_getindex_rt(ecx_portt *port)
{
	uint8 idx;
	uint8 cnt;

	WARN_ON(!in_interrupt());

	idx = port->lastidx_rt + 1;
	/* index can't be larger than buffer array */
	if (idx >= EC_RT_ENDBUF)
	{
	  idx = EC_RT_STARTBUF;
	}
	cnt = 0;
	/* try to find unused index */
	while ((port->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_RT_MAXBUF))
	{
	  idx++;
	  cnt++;
	  if (idx >= EC_RT_ENDBUF)
	  {
		 idx = EC_RT_STARTBUF;
	  }
	}
	port->rxbufstat[idx] = EC_BUF_ALLOC;
	if (port->redstate != ECT_RED_NONE)
	{
	  port->redport->rxbufstat[idx] = EC_BUF_ALLOC;
	}
	port->lastidx_rt = idx;

	return idx;
}


/** Get new frame identifier index and allocate corresponding rx buffer.
 * @param[in] port        = port context struct
 * @return new index.
 */
uint8 ecx_getindex(ecx_portt *port)
{
	if (in_interrupt())
	{
		return ecx_getindex_rt(port);
	}

	return ecx_getindex_nrt(port);
}

/** Set rx buffer status.
 * @param[in] port     = port context struct
 * @param[in] idx      = index in buffer array
 * @param[in] bufstat  = status to set
 */
void ecx_setbufstat(ecx_portt *port, uint8 idx, int bufstat)
{
   port->rxbufstat[idx] = bufstat;
   if (port->redstate != ECT_RED_NONE)
   {
      port->redport->rxbufstat[idx] = bufstat;
   }
}

static int ecx_queue_side_buffer(ecx_portt *port, uint8 idx, int stacknumber)
{
	WARN_ON(in_interrupt());
	
	mutex_lock(&port->tx_mutex);

	if (side_ring_full())
	{
		set_nrt_error(NRT_ERR_SIDERING_FULL);
		mutex_unlock(&port->tx_mutex);
		return 0;
	}
	
	side_ring.ports[side_ring.wr]   = port;
	side_ring.indexes[side_ring.wr] = idx;
	side_ring.stacknumbers[side_ring.wr] = stacknumber;
	side_ring.wr = (side_ring.wr + 1) % SIDE_RING_SIZE;

	mutex_unlock(&port->tx_mutex);

	return 1;
}

/** Transmit buffer over socket (non blocking).
 * @param[in] port        = port context struct
 * @param[in] idx         = index in tx buffer array
 * @param[in] stacknumber  = 0=Primary 1=Secondary stack
 * @return socket send result
 */
int ecx_outframe(ecx_portt *port, uint8 idx, int stacknumber)
{
	int lp, rval;
	ec_stackT *stack;

	WARN_ON(stacknumber != 0);

	if (!stacknumber)
	{
		stack = &(port->stack);
	}
	else
	{
		stack = &(port->redport->stack);
	}
	lp = (*stack->txbuflength)[idx];
	(*stack->rxbufstat)[idx] = EC_BUF_TX;
	(*stack->sndtime)[idx] = get_jiffies_64();
	rval = net_queue_and_send((*stack->txbuf)[idx], lp);
	++port->txs;

	if (rval == FALSE)
	{
		if (!in_interrupt())
			pr_err("net_queue_and_send() failed.");
		port->error = -NICDRV_ESEND;
		return 0;
	}

	return TRUE;
}

uint64 last_master_time;
int32_t  val;
uint16_t u16_tmp_value;
/** Transmit buffer over socket (non blocking).
 * @param[in] port        = port context struct
 * @param[in] idx = index in tx buffer array
 * @return socket send result
 */
 //将数据包写入W5500发送缓存,仅在中断ISR中调用
 static int ecx_queue_frame(ecx_portt *port, uint8 idx)
 {
	 ec_etherheadert *ehp;
	 int rval;
	 int lp;
	 ec_stackT *stack;

#if EC_QUEUE_RD_092C
	 boolean more = TRUE;
#else
	 boolean more = FALSE;
#endif

	 //获得下一次中断时钟
#ifdef CONFIG_ECAT_DC_SYNC64
	 uint64 mastertime =  tick_get_expected_ns();
#else
	 uint32 mastertime = (uint32 )(tick_get_expected_ns());
#endif

	 last_master_time = mastertime;
	//将主站时钟同步到DC从站
	ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_FPWR, idx, more,
			 ecx_context.slavelist[ecx_context.grouplist[0].DCnext].configadr,
			 ECT_REG_DCSYSTIME, sizeof(mastertime), &mastertime);

#if EC_QUEUE_RD_092C
	//读取DC从站0x092C System Time Difference 
	ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_FPRD, idx, TRUE,
			 0x1001,
			 ECT_REG_DCSYSDIFF, sizeof(val), &val);

	ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_FPRD, idx, TRUE,
			 0x1002,
			 ECT_REG_DCSYSDIFF, sizeof(val), &val);

	 ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_FPRD, idx, TRUE,
			  0x1003,
			  ECT_REG_DCSYSDIFF, sizeof(val), &val);

	 ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_BRD, idx, FALSE,
		  	  0, ECT_REG_ALSTAT, sizeof(u16_tmp_value), &u16_tmp_value);

#endif

	 ehp = (ec_etherheadert *)&(port->txbuf[idx]);
	 /* rewrite MAC source address 1 to primary */
	 ehp->sa1 = oshw_htons(priMAC[1]);

	 stack = &(port->stack);

	 lp = (*stack->txbuflength)[idx];
	 (*stack->rxbufstat)[idx] = EC_BUF_TX;
	 (*stack->sndtime)[idx] = get_jiffies_64();
	 rval = net_queue((*stack->txbuf)[idx], lp);
	 ++port->txs;
 
	 if (rval == FALSE)
	 {
		 port->error = -NICDRV_ESEND;
		 return 0;
	 }

	return rval;
 }

 //在FRMR报文帧后加入FPWR用于同步DC从站时钟
 //加入W5500缓存后立刻发送
 //时钟同步相关仅能在ISR中调用，osal_current_time_ns()获得时钟才有效
int ecx_outframe_red(ecx_portt *port, uint8 idx)
{
	ec_etherheadert *ehp;
	int rval;

	//添加主站同步DC Slave报文
#ifdef CONFIG_ECAT_DC_SYNC64
	uint64 mastertime =  osal_current_time_ns();
	ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_FPWR, idx, FALSE,
			ecx_context.slavelist[ecx_context.grouplist[0].DCnext].configadr,
			ECT_REG_DCSYSTIME, sizeof(mastertime), &mastertime);
#else
	uint32 mastertime = (uint32 )osal_current_time_ns();
	ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_FPWR, idx, FALSE,
			ecx_context.slavelist[ecx_context.grouplist[0].DCnext].configadr,
			ECT_REG_DCSYSTIME, sizeof(mastertime), &mastertime);
#endif

	ehp = (ec_etherheadert *)&(port->txbuf[idx]);
	/* rewrite MAC source address 1 to primary */
	ehp->sa1 = oshw_htons(priMAC[1]);
	/* transmit over primary socket*/
	rval = ecx_outframe(port, idx, 0);
	if (port->redstate != ECT_RED_NONE)
	{
		WARN_ON(1);
	}

   return rval;
}

/** Non blocking read of socket. Put frame in temporary buffer.
 * @param[in] port        = port context struct
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return >0 if frame is available and read
 */
static int ecx_recvpkt(ecx_portt *port, int stacknumber)
{
	int lp, bytesrx;
	ec_stackT *stack;

	if (!stacknumber)
	{
	  stack = &(port->stack);
	}
	else
	{
	  stack = &(port->redport->stack);
	}

	lp = sizeof(port->tempinbuf);
	do{
		bytesrx = net_recv((*stack->tempbuf), lp);
		if (0 == bytesrx) break;       //网卡未接收到数据

	}while(bytesrx < 0);               //网卡接收到非EtherCAT帧

	//rt_kprintf("----jumped out.\n");

	++port->rxs;
	port->tempinbufs = bytesrx;

	return (bytesrx > 0);
}

/** Non blocking receive frame function. Uses RX buffer and index to combine
 * read frame with transmitted frame. To compensate for received frames that
 * are out-of-order all frames are stored in their respective indexed buffer.
 * If a frame was placed in the buffer previously, the function retrieves it
 * from that buffer index without calling ec_recvpkt. If the requested index
 * is not already in the buffer it calls ec_recvpkt to fetch it. There are
 * three options now, 1 no frame read, so exit. 2 frame read but other
 * than requested index, store in buffer and exit. 3 frame read with matching
 * index, store in buffer, set completed flag in buffer status and exit.
 *
 * @param[in] port        = port context struct
 * @param[in] idx         = requested index of frame
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME or EC_OTHERFRAME.
 */
int remain_budget = 0;

int ecx_inframe(ecx_portt *port, uint8 idx, int stacknumber)
{
   uint16  l;
   int	   rval;
   uint8   idxf;
   ec_etherheadert *ehp;
   ec_comt *ecp;
   ec_stackT *stack;
   ec_bufT *rxbuf;

   WARN_ON(in_interrupt());

   if (!stacknumber)
   {
	  stack = &(port->stack);
   }
   else
   {
	  stack = &(port->redport->stack);
   }
   rval = EC_NOFRAME;
   rxbuf = &(*stack->rxbuf)[idx];
   /* check if requested index is already in buffer ? */
   if ((idx < EC_MAXBUF) && (	(*stack->rxbufstat)[idx] == EC_BUF_RCVD))
   {
	  l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
	  /* return WKC */
	  rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
	  /* mark as completed */
	  (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
   }
   else
   {
	  mutex_lock(&port->rx_mutex);
	  /* non blocking call to retrieve frame from socket */
	  if (ecx_recvpkt(port, stacknumber))
	  {
		 rval = EC_OTHERFRAME;
		 ehp =(ec_etherheadert*)(stack->tempbuf);
		 /* check if it is an EtherCAT frame */
		 if (ehp->etype == oshw_htons(ETH_P_ECAT))
		 {
			ecp =(ec_comt*)(&(*stack->tempbuf)[ETH_HEADERSIZE]);
			l = etohs(ecp->elength) & 0x0fff;
			idxf = ecp->index;
			/* found index equals requested index ? */
			if (idxf == idx)
			{
			   /* yes, put it in the buffer array (strip ethernet header) */
			   memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idx] - ETH_HEADERSIZE);
			   /* return WKC */
			   rval = ((*rxbuf)[l] + ((uint16)((*rxbuf)[l + 1]) << 8));
			   /* mark as completed */
			   (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
			   /* store MAC source word 1 for redundant routing info */
			   (*stack->rxsa)[idx] = oshw_ntohs(ehp->sa1);
			}
			else
			{
			   /* check if index exist and someone is waiting for it */
			   if (idxf < EC_MAXBUF && (*stack->rxbufstat)[idxf] == EC_BUF_TX)
			   {
				  rxbuf = &(*stack->rxbuf)[idxf];
				  /* put it in the buffer array (strip ethernet header) */
				  memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
				  /* mark as received */
				  (*stack->rxbufstat)[idxf] = EC_BUF_RCVD;
				  (*stack->rxsa)[idxf] = oshw_ntohs(ehp->sa1);
			   }
			   else
			   {
				  /* strange things happened */
                  pr_err("strange things happened.\n");
			   }
			}
		 }
	  }
	  mutex_unlock(&port->rx_mutex);
   }

   /* WKC if matching frame found */
   return rval;
}

/** Blocking redundant receive frame function. If redundant mode is not active then
 * it skips the secondary stack and redundancy functions. In redundant mode it waits
 * for both (primary and secondary) frames to come in. The result goes in an decision
 * tree that decides, depending on the route of the packet and its possible missing arrival,
 * how to reroute the original packet to get the data in an other try.
 *
 * @param[in] port        = port context struct
 * @param[in] idx = requested index of frame
 * @param[in] timer = absolute timeout time
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME.
 */
static int ecx_waitinframe_red(ecx_portt *port, uint8 idx, osal_timert timer)
{
   int wkc	= EC_NOFRAME;
   int wkc2 = EC_NOFRAME;
   int primrx, secrx;

   /* if not in redundant mode then always assume secondary is OK */
   if (port->redstate == ECT_RED_NONE)
   {
	  wkc2 = 0;
   }
   do
   {
   	  msleep(CONFIG_ECAT_SRCONFIRM_RECV_DELAY_MS);					//休眠一段时间等待接收
	  /* only read frame if not already in */
	  if (wkc <= EC_NOFRAME)
	  {
		 wkc  = ecx_inframe(port, idx, 0);
	  }
	  /* only try secondary if in redundant mode */
	  if (port->redstate != ECT_RED_NONE)
	  {
		 /* only read frame if not already in */
		 if (wkc2 <= EC_NOFRAME)
			wkc2 = ecx_inframe(port, idx, 1);
	  }
   /* wait for both frames to arrive or timeout */
   } while (((wkc <= EC_NOFRAME) || (wkc2 <= EC_NOFRAME)) && (osal_timer_is_expired(&timer) == FALSE));
   /* only do redundant functions when in redundant mode */
   if (port->redstate != ECT_RED_NONE)
   {
      WARN_ON(1);		/*暂不支持线缆冗余*/
	  /* primrx if the received MAC source on primary socket */
	  primrx = 0;
	  if (wkc > EC_NOFRAME)
	  {
		 primrx = port->rxsa[idx];
	  }
	  /* secrx if the received MAC source on psecondary socket */
	  secrx = 0;
	  if (wkc2 > EC_NOFRAME)
	  {
		 secrx = port->redport->rxsa[idx];
	  }
	  /* primary socket got secondary frame and secondary socket got primary frame */
	  /* normal situation in redundant mode */
	  if ( ((primrx == RX_SEC) && (secrx == RX_PRIM)) )
	  {
		 /* copy secondary buffer to primary */
		 memcpy(&(port->rxbuf[idx]), &(port->redport->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
		 wkc = wkc2;
	  }
	  /* primary socket got nothing or primary frame, and secondary socket got secondary frame */
	  /* we need to resend TX packet */
	  if ( ((primrx == 0) && (secrx == RX_SEC)) ||
		   ((primrx == RX_PRIM) && (secrx == RX_SEC)) )
	  {
		 osal_timert read_timer;

		 /* If both primary and secondary have partial connection retransmit the primary received
		  * frame over the secondary socket. The result from the secondary received frame is a combined
		  * frame that traversed all slaves in standard order. */
		 if ( (primrx == RX_PRIM) && (secrx == RX_SEC) )
		 {
			/* copy primary rx to tx buffer */
			memcpy(&(port->txbuf[idx][ETH_HEADERSIZE]), &(port->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
		 }
		 osal_timer_start(&read_timer, EC_TIMEOUTRET);
		 /* resend secondary tx */
		 ecx_outframe(port, idx, 1);
		 do
		 {
			/* retrieve frame */
			wkc2 = ecx_inframe(port, idx, 1);
		 } while ((wkc2 <= EC_NOFRAME) && (osal_timer_is_expired(&read_timer) == FALSE));
		 if (wkc2 > EC_NOFRAME)
		 {
			/* copy secondary result to primary rx buffer */
			memcpy(&(port->rxbuf[idx]), &(port->redport->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
			wkc = wkc2;
		 }
	  }
   }

   /* return WKC or EC_NOFRAME */
   return wkc;
}

/** Blocking receive frame function. Calls ec_waitinframe_red().
 * @param[in] port        = port context struct
 * @param[in] idx       = requested index of frame
 * @param[in] timeout   = timeout in us
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME.
 */
int ecx_waitinframe(ecx_portt *port, uint8 idx, int timeout)
{
	uint16	l;
	int rval, state;
	ec_stackT *stack;
	ec_bufT   *rxbuf;
	int stacknumber = 0;

	WARN_ON(!in_interrupt());
	UNUSED(timeout);
	
	if (!stacknumber)
	{
		stack = &(port->stack);
	}
	else
	{
		stack = &(port->redport->stack);
	}

	rval = EC_NOFRAME;
	rxbuf = &(*stack->rxbuf)[idx];

	if (idx > EC_MAXBUF)
		return rval;
	
	state = (*stack->rxbufstat)[idx];
	if (EC_BUF_RCVD == state)
	{
		l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
		rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
	}

    return rval;
}

/**
**period线程已经启动，等待报文接收完成,period线程意外退出时返回错误
**EC_BUF_RCVD报文成功接收，EC_BUF_COMPLETE接收报文超时
** !ecx_in_period
**返回: 实际的wkc, 接收超时返回EC_NOFRAME
 */
static int ecx_waitinframe_nrt(ecx_portt *port, uint8 idx)
{
	uint16  l;
	int rval, state;
	ec_stackT *stack;
	ec_bufT	  *rxbuf;
	int stacknumber = 0;

	WARN_ON(in_interrupt());
	
	if (!stacknumber)
	{
		stack = &(port->stack);
	}
	else
	{
		stack = &(port->redport->stack);
	}

	rval = EC_NOFRAME;
	rxbuf = &(*stack->rxbuf)[idx];

	if (idx > EC_MAXBUF)
		return rval;
	
	do
	{
		msleep(CONFIG_ECAT_SRCONFIRM_RECV_DELAY_MS);					//休眠一段时间等待接收
		state = (*stack->rxbufstat)[idx];
		if (EC_BUF_RCVD == state)
		{
			l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
			/* return WKC */
			rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
			break;
		}

	}while (state != EC_BUF_COMPLETE && period_run);	//等待报文超时

	return rval;
}

int ecx_srconfirm(ecx_portt *port, uint8 idx, int timeout)
{
	int wkc = EC_NOFRAME;
	osal_timert timer1, timer2;

	UNUSED(timeout);
	WARN_ON(in_interrupt());

	if (period_run)
	{//period周期ISR已经开始
		int retry = EC_SND_RETRY;
		while (retry-- && wkc <= EC_NOFRAME)
		{
			ecx_queue_side_buffer(port, idx, 0);
			wkc = ecx_waitinframe_nrt(port, idx);
		}
		return wkc;
	}

	//period周期ISR尚未开始
	osal_timer_start (&timer1, CONFIG_ECAT_SRCONFIRM_RE_SEND_TIMEOUT_MS * 1000);		//重发
	do
	{
		mutex_lock(&port->tx_mutex);
		ecx_outframe(port, idx, 0);
		mutex_unlock(&port->tx_mutex);

		osal_timer_start(&timer2, CONFIG_ECAT_SRCONFIRM_RECV_TIMEOUT_MS * 1000);		//接收超时

		if (osal_timer_is_expired (&timer1))
		{//接收响应超时
			set_nrt_error(NRT_ERR_CONFIRM_TIMEOUT);
			break;
		}

		wkc = ecx_waitinframe_red(port, idx, timer2);
	} while ((wkc <= EC_NOFRAME));

   	return wkc;
}

#ifdef EC_VER1
int ec_setupnic(const char *ifname, int secondary)
{
   return ecx_setupnic(&ecx_port, ifname, secondary);
}

int ec_closenic(void)
{
   return ecx_closenic(&ecx_port);
}

uint8 ec_getindex(void)
{
   return ecx_getindex(&ecx_port);
}

void ec_setbufstat(uint8 idx, int bufstat)
{
   ecx_setbufstat(&ecx_port, idx, bufstat);
}

int ec_outframe(uint8 idx, int stacknumber)
{
   return ecx_outframe(&ecx_port, idx, stacknumber);
}

int ec_outframe_red(uint8 idx)
{
   WARN_ON(1);
   return ecx_outframe_red(&ecx_port, idx);
}

int ec_inframe(uint8 idx, int stacknumber)
{
   return ecx_inframe(&ecx_port, idx, stacknumber);
}

int ec_waitinframe(uint8 idx, int timeout)
{
   return ecx_waitinframe(&ecx_port, idx, timeout);
}

int ec_srconfirm(uint8 idx, int timeout)
{
   return ecx_srconfirm(&ecx_port, idx, timeout);
}
#endif

int nicdrv_send_side_buffer(void)
{
	ecx_portt *port; 
	uint8 idx;
	int stacknumber;

	int lp, rval;
	ec_stackT *stack;

	if (side_ring_empty())
		return 0;

	port         = side_ring.ports[side_ring.rd];
	idx          = side_ring.indexes[side_ring.rd];
	stacknumber  = side_ring.stacknumbers[side_ring.rd];
	side_ring.rd = (side_ring.rd + 1) % SIDE_RING_SIZE;

	if (!stacknumber)
	{
		stack = &(port->stack);
	}
	else
	{
		stack = &(port->redport->stack);
	}
	lp = (*stack->txbuflength)[idx];
	(*stack->rxbufstat)[idx] = EC_BUF_TX;
	(*stack->sndtime)[idx] = get_jiffies_64();
	rval = net_queue_and_send((*stack->txbuf)[idx], lp);
	++port->txs;
	return rval;
}

int nicdrv_recv_frames(void)
{
	uint8	i;
	uint8	idxf;
	ec_etherheadert *ehp;
	ec_comt *ecp;
	ec_stackT *stack;
	ec_bufT *rxbuf;
	int recvcnt = 0;
	u64	sndtime, now;

	ecx_portt *port = &ecx_port;
	int stacknumber = 0;
	if (!stacknumber)
	{
		stack = &(port->stack);
	}
	else
	{
		stack = &(port->redport->stack);
	}

	/*超时检查*/
	now = get_jiffies_64();
	for (i = 0; i < EC_MAXBUF; ++i)
	{
		if (EC_BUF_TX == (*stack->rxbufstat)[i])
		{
			sndtime = (*stack->sndtime)[i];
			if (now > sndtime + CONFIG_ECAT_RCV_TIMEOUT)
			{
				period_warn |= PERIOD_WRECV_TIMO;
				(*stack->rxbufstat)[i] = EC_BUF_COMPLETE;
			}
		}
	}

	while (recvcnt < 8)
	{
		if(ecx_recvpkt(port, stacknumber) <= 0)
		{/*未接收到EtherCAT包*/
			return recvcnt;
		}
		++recvcnt;
		ehp =(ec_etherheadert*)(stack->tempbuf);
		ecp =(ec_comt*)(&(*stack->tempbuf)[ETH_HEADERSIZE]);
		idxf = ecp->index;

		if (idxf < EC_MAXBUF && (*stack->rxbufstat)[idxf] == EC_BUF_TX)
		{
			rxbuf = &(*stack->rxbuf)[idxf];
			/* put it in the buffer array (strip ethernet header) */
			memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
			/* mark as received */
			(*stack->rxbufstat)[idxf] = EC_BUF_RCVD;
			(*stack->rxsa)[idxf] = oshw_ntohs(ehp->sa1);
		}
	}

	return recvcnt;
}

int nicdrv_send_processdata(void)
{
	net_send();
	return 0;
}

//参考ethercatmain.c:ecx_main_send_processdata()
//由于ecx_outframe_red()会在FRMW报文后增加FPWR报文用于RefClock从站与主站的时钟同步
//ecx_adddatagram()的more参数修改为TRUE
int nicdrv_queue_processdata(void)
{
	uint32 LogAdr;
	uint16 w1, w2;
	int length;
	uint16 sublength;
	uint8 idx;
	int wkc;
	uint8* data;
	boolean first=FALSE;
	uint16 currentsegment = 0;
	uint32 iomapinputoffset;
	uint16 DCO;
	const boolean MORE =  TRUE;  //FRMV后增加FPWR
	
	ecx_contextt *context = &ecx_context;
	uint8 group = 0;
	boolean use_overlap_io = 0;

	wkc = 0;
	if(context->grouplist[group].hasdc)
	{
	  first = TRUE;
	}

	/* For overlapping IO map use the biggest */
	if(use_overlap_io == TRUE)
	{
	  /* For overlap IOmap make the frame EQ big to biggest part */
	  length = (context->grouplist[group].Obytes > context->grouplist[group].Ibytes) ?
	     context->grouplist[group].Obytes : context->grouplist[group].Ibytes;
	  /* Save the offset used to compensate where to save inputs when frame returns */
	  iomapinputoffset = context->grouplist[group].Obytes;
	}
	else
	{
	  length = context->grouplist[group].Obytes + context->grouplist[group].Ibytes;
	  iomapinputoffset = 0;
	}

	LogAdr = context->grouplist[group].logstartaddr;
	if(length)
	{

	  wkc = 1;
	  /* LRW blocked by one or more slaves ? */
	  if(context->grouplist[group].blockLRW)
	  {
	     /* if inputs available generate LRD */
	     if(context->grouplist[group].Ibytes)
	     {
	        currentsegment = context->grouplist[group].Isegment;
	        data = context->grouplist[group].inputs;
	        length = context->grouplist[group].Ibytes;
	        LogAdr += context->grouplist[group].Obytes;
	        /* segment transfer if needed */
	        do
	        {
	           if(currentsegment == context->grouplist[group].Isegment)
	           {
	              sublength = (uint16)(context->grouplist[group].IOsegment[currentsegment++] - context->grouplist[group].Ioffset);
	           }
	           else
	           {
	              sublength = (uint16)context->grouplist[group].IOsegment[currentsegment++];
	           }
	           /* get new index */
	           idx = ecx_getindex(context->port);
	           w1 = LO_WORD(LogAdr);
	           w2 = HI_WORD(LogAdr);
	           DCO = 0;
	           ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_LRD, idx, w1, w2, sublength, data);
	           if(first)
	           {
	              /* FPRMW in second datagram */
	              DCO = ecx_adddatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_FRMW, idx, MORE,
	                                       context->slavelist[context->grouplist[group].DCnext].configadr,
	                                       ECT_REG_DCSYSTIME, sizeof(int64), context->DCtime);
	              first = FALSE;
	           }
	           /* send frame */
	           ecx_queue_frame(context->port, idx);
	           /* push index and data pointer on stack */
	           ecx_pushindex(context, idx, data, sublength, DCO);
	           length -= sublength;
	           LogAdr += sublength;
	           data += sublength;
	        } while (length && (currentsegment < context->grouplist[group].nsegments));
	     }
	     /* if outputs available generate LWR */
	     if(context->grouplist[group].Obytes)
	     {
	        data = context->grouplist[group].outputs;
	        length = context->grouplist[group].Obytes;
	        LogAdr = context->grouplist[group].logstartaddr;
	        currentsegment = 0;
	        /* segment transfer if needed */
	        do
	        {
	           sublength = (uint16)context->grouplist[group].IOsegment[currentsegment++];
	           if((length - sublength) < 0)
	           {
	              sublength = (uint16)length;
	           }
	           /* get new index */
	           idx = ecx_getindex(context->port);
	           w1 = LO_WORD(LogAdr);
	           w2 = HI_WORD(LogAdr);
	           DCO = 0;
	           ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_LWR, idx, w1, w2, sublength, data);
	           if(first)
	           {
	              /* FPRMW in second datagram */
	              DCO = ecx_adddatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_FRMW, idx, MORE,
	                                       context->slavelist[context->grouplist[group].DCnext].configadr,
	                                       ECT_REG_DCSYSTIME, sizeof(int64), context->DCtime);
	              first = FALSE;
	           }
	           /* send frame */
	           ecx_queue_frame(context->port, idx);
	           /* push index and data pointer on stack */
	           ecx_pushindex(context, idx, data, sublength, DCO);
	           length -= sublength;
	           LogAdr += sublength;
	           data += sublength;
	        } while (length && (currentsegment < context->grouplist[group].nsegments));
	     }
	  }
	  /* LRW can be used */
	  else
	  {
	     if (context->grouplist[group].Obytes)
	     {
	        data = context->grouplist[group].outputs;
	     }
	     else
	     {
	        data = context->grouplist[group].inputs;
	        /* Clear offset, don't compensate for overlapping IOmap if we only got inputs */
	        iomapinputoffset = 0;
	     }
	     /* segment transfer if needed */
	     do
	     {
	        sublength = (uint16)context->grouplist[group].IOsegment[currentsegment++];
	        /* get new index */
	        idx = ecx_getindex(context->port);
	        w1 = LO_WORD(LogAdr);
	        w2 = HI_WORD(LogAdr);
	        DCO = 0;
	        ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_LRW, idx, w1, w2, sublength, data);
	        if(first)
	        {
	           /* FPRMW in second datagram */
	           DCO = ecx_adddatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_FRMW, idx, MORE,
	                                    context->slavelist[context->grouplist[group].DCnext].configadr,
	                                    ECT_REG_DCSYSTIME, sizeof(int64), context->DCtime);
	           first = FALSE;
	        }
	        /* send frame */
	        ecx_queue_frame(context->port, idx);
	        /* push index and data pointer on stack.
	         * the iomapinputoffset compensate for where the inputs are stored 
	         * in the IOmap if we use an overlapping IOmap. If a regular IOmap
	         * is used it should always be 0.
	         */
	        ecx_pushindex(context, idx, (data + iomapinputoffset), sublength, DCO);  
			
	        length -= sublength;
	        LogAdr += sublength;
	        data += sublength; 
	     } while (length && (currentsegment < context->grouplist[group].nsegments));
	  }
	}

	return wkc;
}

int nicdrv_queue_sync_frame(void)
{
	ecx_contextt *context = &ecx_context;
	uint8 group = 0;
	uint8 idx;

	idx = EC_RT_SYNCBUF;

#ifdef CONFIG_ECAT_DC_SYNC64
	uint64 mastertime = 0;
#else
	uint32 mastertime = 0;
#endif
	ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_FRMW, idx,
			context->slavelist[context->grouplist[group].DCnext].configadr, ECT_REG_DCSYSTIME,
			sizeof(mastertime), &mastertime);

	ecx_queue_frame(context->port, idx);

	return TRUE;
}

//参考ethercatmain.c:ecx_receive_processdata_group()
//LRW报文不要使用报文内容覆盖输出缓存
int nicdrv_receive_processdata(void)
{
	uint8 idx;
	int pos;
	int wkc = 0, wkc2;
	uint16 le_wkc = 0;
	int valid_wkc = 0;
	int64 le_DCtime;
	ec_idxstackT *idxstack;
	ec_bufT *rxbuf;
	uint8	*input_beg, *data;
	uint16   output_len, length;

	int group = 0;
	ecx_contextt *context = &ecx_context;

	
	wkc2 = group;
	idxstack = context->idxstack;
	rxbuf = context->port->rxbuf;
	/* get first index */
	pos = ecx_pullindex(context);
	/* read the same number of frames as send */
	while (pos >= 0)
	{
		idx = idxstack->idx[pos];
		wkc2 = ecx_waitinframe(context->port, idx, 0/*无用参数*/);
		/* check if there is input data in frame */
		if (wkc2 > EC_NOFRAME)
		{
			if (rxbuf[idx][EC_CMDOFFSET] == EC_CMD_LRD)
			{
				if (idxstack->dcoffset[pos] > 0)
				{
					memcpy(idxstack->data[pos], &(rxbuf[idx][EC_HEADERSIZE]), idxstack->length[pos]);
					memcpy(&le_wkc, &(rxbuf[idx][EC_HEADERSIZE + idxstack->length[pos]]), EC_WKCSIZE);
					wkc = etohs(le_wkc);
					memcpy(&le_DCtime, &(rxbuf[idx][idxstack->dcoffset[pos]]), sizeof(le_DCtime));
					*(context->DCtime) = etohll(le_DCtime);
				}
				else
				{
					/* copy input data back to process data buffer */
					memcpy(idxstack->data[pos], &(rxbuf[idx][EC_HEADERSIZE]), idxstack->length[pos]);
					wkc += wkc2;
				}
				valid_wkc = 1;
			}
			else if (EC_CMD_LRW == rxbuf[idx][EC_CMDOFFSET])
			{
				input_beg = ec_slave[0].inputs;
				data      = idxstack->data[pos];
				length    = idxstack->length[pos];
				
				if (data >= input_beg)
				{
					memcpy(data, &(rxbuf[idx][EC_HEADERSIZE]), length);
				}
				else if (data < input_beg 
					&& data + length > input_beg)
				{//仅覆盖输入部分的内容
					output_len  = input_beg - data;
					memcpy(input_beg, &(rxbuf[idx][EC_HEADERSIZE]) + output_len, length - output_len);
				}
				
				if (idxstack->dcoffset[pos] > 0)
				{
					memcpy(&le_wkc, &(rxbuf[idx][EC_HEADERSIZE + idxstack->length[pos]]), EC_WKCSIZE);
					wkc = etohs(le_wkc);
					memcpy(&le_DCtime, &(rxbuf[idx][idxstack->dcoffset[pos]]), sizeof(le_DCtime));
					*(context->DCtime) = etohll(le_DCtime);
				}
				else
				{
					wkc += wkc2;
				}
				valid_wkc = 1;
			}
		 	else if(rxbuf[idx][EC_CMDOFFSET]==EC_CMD_LWR)
			{
				if (idxstack->dcoffset[pos] > 0)
				{
					memcpy(idxstack->data[pos], &(rxbuf[idx][EC_HEADERSIZE]), idxstack->length[pos]);
					memcpy(&le_wkc, &(rxbuf[idx][EC_HEADERSIZE + idxstack->length[pos]]), EC_WKCSIZE);
					wkc = etohs(le_wkc);
					memcpy(&le_DCtime, &(rxbuf[idx][idxstack->dcoffset[pos]]), sizeof(le_DCtime));
					*(context->DCtime) = etohll(le_DCtime);
				}
				else
				{
					/* copy input data back to process data buffer */
					memcpy(idxstack->data[pos], &(rxbuf[idx][EC_HEADERSIZE]), idxstack->length[pos]);
					wkc += wkc2;
				}
				valid_wkc = 1;
			}
		}
		/* release buffer */
		ecx_setbufstat(context->port, idx, EC_BUF_EMPTY);
		/* get next index */
		pos = ecx_pullindex(context);
	}

	ecx_clearindex(context);

	/* if no frames has arrived */
	if (valid_wkc == 0)
	{
		return EC_NOFRAME;
	}
	return wkc;
}

#if 0
//发送对时报文包括FRMW + FPWR
int nicdrv_send_sync_datagrams(void)
{
	ecx_contextt *context = &ecx_context;
	uint8 group = 0;
	uint8 idx;

	idx = EC_RT_SYNCBUF;

#ifdef CONFIG_ECAT_DC_SYNC64
	uint64 mastertime = 0;
#else
	uint32 mastertime = 0;
#endif
	ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_FRMW, idx,
			context->slavelist[context->grouplist[group].DCnext].configadr, ECT_REG_DCSYSTIME,
			sizeof(mastertime), &mastertime);

	return ecx_outframe_red(context->port, idx);
}
#endif

/* latched port time of slave */
static int32 ecx_porttime(ecx_contextt *context, uint16 slave, uint8 port)
{
   int32 ts;
   switch (port)
   {
      case 0:
         ts = context->slavelist[slave].DCrtA;
         break;
      case 1:
         ts = context->slavelist[slave].DCrtB;
         break;
      case 2:
         ts = context->slavelist[slave].DCrtC;
         break;
      case 3:
         ts = context->slavelist[slave].DCrtD;
         break;
      default:
         ts = 0;
         break;
   }
   return ts;
}

/* calculate previous active port of a slave */
static uint8 ecx_prevport(ecx_contextt *context, uint16 slave, uint8 port)
{
   uint8 pport = port;
   uint8 aport = context->slavelist[slave].activeports;
   switch(port)
   {
      case 0:
         if(aport & PORTM2)
            pport = 2;
         else if (aport & PORTM1)
            pport = 1;
         else if (aport & PORTM3)
            pport = 3;
         break;
      case 1:
         if(aport & PORTM3)
            pport = 3;
         else if (aport & PORTM0)
            pport = 0;
         else if (aport & PORTM2)
            pport = 2;
         break;
      case 2:
         if(aport & PORTM1)
            pport = 1;
         else if (aport & PORTM3)
            pport = 3;
         else if (aport & PORTM0)
            pport = 0;
         break;
      case 3:
         if(aport & PORTM0)
            pport = 0;
         else if (aport & PORTM2)
            pport = 2;
         else if (aport & PORTM1)
            pport = 1;
         break;
   }
   return pport;
}


/* search unconsumed ports in parent, consume and return first open port */
static uint8 ecx_parentport(ecx_contextt *context, uint16 parent)
{
   uint8 parentport = 0;
   uint8 b;
   /* search order is important, here 3 - 1 - 2 - 0 */
   b = context->slavelist[parent].consumedports;
   if (b & PORTM3)
   {
      parentport = 3;
      b &= (uint8)~PORTM3;
   }
   else if (b & PORTM1)
   {
      parentport = 1;
      b &= (uint8)~PORTM1;
   }
   else if (b & PORTM2)
   {
      parentport = 2;
      b &= (uint8)~PORTM2;
   }
   else if (b & PORTM0)
   {
      parentport = 0;
      b &= (uint8)~PORTM0;
   }
   context->slavelist[parent].consumedports = b;
   return parentport;
}


boolean nicdrv_configdc(   uint64 mastertime64)
{
   uint16 i, slaveh, parent, child;
   uint16 parenthold = 0;
   uint16 prevDCslave = 0;
   int32 ht, dt1, dt2, dt3;
   int64 hrt;
   uint8 entryport;
   int8 nlist;
   int8 plist[4];
   int32 tlist[4];
   ecx_contextt *context = &ecx_context;

   context->slavelist[0].hasdc = FALSE;
   context->grouplist[0].hasdc = FALSE;
   ht = 0;

   for (i = 1; i <= *(context->slavecount); i++)
   {
      context->slavelist[i].consumedports = context->slavelist[i].activeports;
      if (context->slavelist[i].hasdc)
      {
         if (!context->slavelist[0].hasdc)
         {
            context->slavelist[0].hasdc = TRUE;
            context->slavelist[0].DCnext = i;
            context->slavelist[i].DCprevious = 0;
            context->grouplist[context->slavelist[i].group].hasdc = TRUE;
            context->grouplist[context->slavelist[i].group].DCnext = i;
         }
         else
         {
            context->slavelist[prevDCslave].DCnext = i;
            context->slavelist[i].DCprevious = prevDCslave;
         }
         /* this branch has DC slave so remove parenthold */
         parenthold = 0;
         prevDCslave = i;
         slaveh = context->slavelist[i].configadr;
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtA = etohl(ht);
         /* 64bit latched DCrecvTimeA of each specific slave */
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCSOF, sizeof(hrt), &hrt, EC_TIMEOUTRET);
         /* use it as offset in order to set local time around 0 + mastertime */
         hrt = htoell(-etohll(hrt) + mastertime64);
         /* save it in the offset register */
         (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYSOFFSET, sizeof(hrt), &hrt, EC_TIMEOUTRET);
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME1, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtB = etohl(ht);
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME2, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtC = etohl(ht);
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME3, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtD = etohl(ht);

         /* make list of active ports and their time stamps */
         nlist = 0;
         if (context->slavelist[i].activeports & PORTM0)
         {
            plist[nlist] = 0;
            tlist[nlist] = context->slavelist[i].DCrtA;
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM3)
         {
            plist[nlist] = 3;
            tlist[nlist] = context->slavelist[i].DCrtD;
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM1)
         {
            plist[nlist] = 1;
            tlist[nlist] = context->slavelist[i].DCrtB;
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM2)
         {
            plist[nlist] = 2;
            tlist[nlist] = context->slavelist[i].DCrtC;
            nlist++;
         }
         /* entryport is port with the lowest timestamp */
         entryport = 0;
         if((nlist > 1) && (tlist[1] < tlist[entryport]))
         {
            entryport = 1;
         }
         if((nlist > 2) && (tlist[2] < tlist[entryport]))
         {
            entryport = 2;
         }
         if((nlist > 3) && (tlist[3] < tlist[entryport]))
         {
            entryport = 3;
         }
         entryport = plist[entryport];
         context->slavelist[i].entryport = entryport;
         /* consume entryport from activeports */
         context->slavelist[i].consumedports &= (uint8)~(1 << entryport);

         /* finding DC parent of current */
         parent = i;
         do
         {
            child = parent;
            parent = context->slavelist[parent].parent;
         }
         while (!((parent == 0) || (context->slavelist[parent].hasdc)));
         /* only calculate propagation delay if slave is not the first */
         if (parent > 0)
         {
            /* find port on parent this slave is connected to */
            context->slavelist[i].parentport = ecx_parentport(context, parent);
            if (context->slavelist[parent].topology == 1)
            {
               context->slavelist[i].parentport = context->slavelist[parent].entryport;
            }

            dt1 = 0;
            dt2 = 0;
            /* delta time of (parentport - 1) - parentport */
            /* note: order of ports is 0 - 3 - 1 -2 */
            /* non active ports are skipped */
            dt3 = ecx_porttime(context, parent, context->slavelist[i].parentport) -
                  ecx_porttime(context, parent,
                    ecx_prevport(context, parent, context->slavelist[i].parentport));
            /* current slave has children */
            /* those children's delays need to be subtracted */
            if (context->slavelist[i].topology > 1)
            {
               dt1 = ecx_porttime(context, i,
                        ecx_prevport(context, i, context->slavelist[i].entryport)) -
                     ecx_porttime(context, i, context->slavelist[i].entryport);
            }
            /* we are only interested in positive difference */
            if (dt1 > dt3) dt1 = -dt1;
            /* current slave is not the first child of parent */
            /* previous child's delays need to be added */
            if ((child - parent) > 1)
            {
               dt2 = ecx_porttime(context, parent,
                        ecx_prevport(context, parent, context->slavelist[i].parentport)) -
                     ecx_porttime(context, parent, context->slavelist[parent].entryport);
            }
            if (dt2 < 0) dt2 = -dt2;

            /* calculate current slave delay from delta times */
            /* assumption : forward delay equals return delay */
            context->slavelist[i].pdelay = ((dt3 - dt1) / 2) + dt2 +
               context->slavelist[parent].pdelay;
            ht = htoel(context->slavelist[i].pdelay);
            /* write propagation delay*/
            (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYSDELAY, sizeof(ht), &ht, EC_TIMEOUTRET);
         }
      }
      else
      {
         context->slavelist[i].DCrtA = 0;
         context->slavelist[i].DCrtB = 0;
         context->slavelist[i].DCrtC = 0;
         context->slavelist[i].DCrtD = 0;
         parent = context->slavelist[i].parent;
         /* if non DC slave found on first position on branch hold root parent */
         if ( (parent > 0) && (context->slavelist[parent].topology > 2))
            parenthold = parent;
         /* if branch has no DC slaves consume port on root parent */
         if ( parenthold && (context->slavelist[i].topology == 1))
         {
            ecx_parentport(context, parenthold);
            parenthold = 0;
         }
      }
   }

   return context->slavelist[0].hasdc;
}

