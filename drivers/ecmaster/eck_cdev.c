#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/mman.h>

#include "globals.h"
#include "eck_cdev.h"
#include "eck.h"
#include "eck_ioctl.h"
#include "fops.h"

static int eck_cdev_open(struct inode *inode, struct file *filp);
static int eck_cdev_release(struct inode *inode, struct file *filp);
static long eck_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int eck_cdev_mmap(struct file *filp, struct vm_area_struct *vma);

static const struct file_operations eck_cdev_fops = {
	.owner          = THIS_MODULE,
	.open           = eck_cdev_open,
	.release        = eck_cdev_release,
	.unlocked_ioctl = eck_cdev_ioctl,
	.mmap           = eck_cdev_mmap,
};

int eck_cdev_init(eck_cdev_t *cdev, struct ECK *eck, dev_t dev_num)
{
	int ret;

	cdev->eck = eck;

	cdev_init(&cdev->cdev, &eck_cdev_fops);
	cdev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&cdev->cdev, MKDEV(MAJOR(dev_num), eck->index), 1);
	if (ret)
		pr_err("Failed to add character device!\n");

	return ret;
}

void eck_cdev_clear(eck_cdev_t *cdev)
{
	cdev_del(&cdev->cdev);
}

int eck_cdev_open(struct inode *inode, struct file *filp)
{
	int ret;
	eck_cdev_priv_t *priv;
	eck_cdev_t *eck_cdev = container_of(inode->i_cdev, eck_cdev_t, cdev);
	eck_t *eck = eck_cdev->eck;

	priv = kmalloc(sizeof(eck_cdev_priv_t), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->cdev = eck_cdev;
	filp->private_data = priv;

	ret = eck_file_open(eck);
	if (ret)
		goto FREE_PRIV;

	return 0;

FREE_PRIV:
	kfree(priv);

	return ret;
}

int eck_cdev_release(struct inode *inode, struct file *filp)
{
	eck_cdev_priv_t *priv = (eck_cdev_priv_t *) filp->private_data;
	eck_t *eck = priv->cdev->eck;

	eck_file_close(eck);

	kfree(priv);

	return 0;
}

static int mmap_kmem_helper(struct vm_area_struct *vma, void *va)
{
	unsigned long addr, len, pfn, to;
	int ret = 0;

	to = (unsigned long)va;
	addr = vma->vm_start;
	len = vma->vm_end - vma->vm_start;

	if (to != PAGE_ALIGN(to) || (len & ~PAGE_MASK) != 0)
		return -EINVAL;

#ifndef CONFIG_MMU
	pfn = __pa(to) >> PAGE_SHIFT;
	ret = remap_pfn_range(vma, addr, pfn, len, PAGE_SHARED);
#else
	if (to < VMALLOC_START || to >= VMALLOC_END) {
		/* logical address. */
		pfn = __pa(to) >> PAGE_SHIFT;
		ret = remap_pfn_range(vma, addr, pfn, len, PAGE_SHARED);
		if (ret)
			return ret;
	} else {
		/* vmalloc memory. */
		while (len > 0) {
			struct page *page = vmalloc_to_page((void *)to);

			if (vm_insert_page(vma, addr, page))
				return -EAGAIN;
			addr += PAGE_SIZE;
			to += PAGE_SIZE;
			len -= PAGE_SIZE;
		}
	}
#endif

	return ret;
}

int eck_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	eck_cdev_priv_t *priv = (eck_cdev_priv_t *) filp->private_data;

	if (!priv->ctx.src_vaddr)
		return -EFAULT;

	ret =  mmap_kmem_helper(vma, priv->ctx.src_vaddr);
	return ret;
}

long eck_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return eck_ioctl(filp, cmd, (void __user *)arg);
}

