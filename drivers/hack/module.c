#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/kobject.h>

MODULE_AUTHOR("HuSuYang");
MODULE_DESCRIPTION("Hack Module");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");

static uint8_t digest_data[] __attribute__((section("HACK"))) = {
								0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8,
								0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8};

static int __init hack_init_module(void)
{
	int i = digest_data[0];
	pr_info("i=%d\n", i);
	return 0;
}
module_init(hack_init_module);

static void __exit hack_exit_module(void)
{
}
module_exit(hack_exit_module);


