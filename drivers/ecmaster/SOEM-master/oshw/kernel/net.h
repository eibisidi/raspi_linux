#ifndef _NET_H
#define _NET_H

int net_init(const char *ifname);
int net_finish(void);

int net_queue_and_send (uint8_t *packet, int length);
int net_recv (uint8_t * packet, size_t size);
int net_queue(uint8_t *packet, int length);
int net_send(void);
void net_show_stats(struct seq_file *seq, void *v);
#endif

