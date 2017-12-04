#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>

static struct i2c_board_info gt1151_info = {
	I2C_BOARD_INFO("gt1151",0x14),
};
static struct i2c_client *gt1151_client;

static int gt1151_dev_init(void)
{
	struct i2c_adapter *i2c_adap;

	i2c_adap = i2c_get_adapter(1);
	printk("adap:%d\r\n",i2c_adap);
	gt1151_client = i2c_new_device(i2c_adap,&gt1151_info);
	i2c_put_adapter(i2c_adap);
	return 0;

}
static void gt1151_dev_exit(void)
{
	i2c_unregister_device(gt1151_client);
}
module_init(gt1151_dev_init);
module_exit(gt1151_dev_exit);
MODULE_LICENSE("GPL");