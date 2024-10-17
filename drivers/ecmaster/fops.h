#ifndef __FOPS_H
#define __FOPS_H

#include "globals.h"
#include "eck_ioctl.h"

int eck_file_open(eck_t *eck);
void eck_file_close(eck_t *eck);
long eck_ioctl(struct file *filp, unsigned int cmd, void __user *arg);

#endif
