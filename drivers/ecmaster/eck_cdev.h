#ifndef ECK_CDEV_H
#define ECK_CDEV_H

#include <linux/fs.h>
#include <linux/cdev.h>

#include "globals.h"

typedef struct eck_cdev {
	struct ECK		*eck; /**< Master owning the device. */
	struct cdev  cdev; /**< Character device. */
} eck_cdev_t;

typedef struct {
	unsigned int writable; /**< Device was opened with write permission. */
	void		*src_vaddr;		//kernel vaddr to map
} eck_ioctl_context_t;

typedef struct {
	eck_cdev_t		*cdev;
	eck_ioctl_context_t	 ctx;
} eck_cdev_priv_t;

int eck_cdev_init(eck_cdev_t *cdev, struct ECK *eck, dev_t dev_num);
void eck_cdev_clear(eck_cdev_t *cdev);

#endif
