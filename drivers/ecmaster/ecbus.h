#ifndef ECBUS_H
#define ECBUS_H

int 	ecbus_wait_enter_preop(void);
int 	ecbus_activate(void);
int 	ecbus_reset(void);
void 	ecbus_check(int wkc);
#endif
