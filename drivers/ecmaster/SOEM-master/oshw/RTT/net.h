#ifndef _NET_H
#define _NET_H

boolean net_init(const char *ifname);
boolean net_finish();

int net_send (void *packet, int length);
int net_recv (uint8_t * packet, size_t size);

#endif

