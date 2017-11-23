/*
* multi touch screen driver for gt1151
* base on multi-touch protocol A
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/delay.h>


/* register address */
#define GT_CTRL_REG 	0X8040   	//GT1151控制寄存器
#define GT_CFGS_REG 	0X8050   	//GT1151配置起始地址寄存器
#define GT_CHECK_REG 	0X813C   	//GT1151校验和寄存器
#define GT_PID_REG 		0X8140   	//GT1151产品ID寄存器

#define GT_GSTID_REG 	0X814E   	//GT1151当前检测到的触摸情况
#define GT_TP1_REG 		0X814F  	//第一个触摸点数据地址
#define GT_TP2_REG 		0X8157		//第二个触摸点数据地址
#define GT_TP3_REG 		0X815F		//第三个触摸点数据地址
#define GT_TP4_REG 		0X8167		//第四个触摸点数据地址
#define GT_TP5_REG 		0X816F		//第五个触摸点数据地址  
#define GT_TP6_REG 		0X8177
#define GT_TP7_REG 		0X817F
#define GT_TP8_REG 		0X8187
#define GT_TP9_REG 		0X818F
#define GT_TP10_REG 	0X8197


#define GPIO_ADDR  0
//GT1151配置参数表
const u8 GT911_CFG_TBL[]=
{
	
0xFF,0xD0,0x02,0x00,0x05,0x0A,0x0D,0x00,0x01,0x40,
0x00,0x0B,0x78,0x64,0x53,0x04,0x00,0x00,0x00,0x00,
0x28,0x00,0x00,0x00,0x08,0x04,0x00,0x02,0x28,0x78,
0x00,0x00,0x00,0x41,0x00,0x00,0x00,0x9B,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x89,0x28,0x1E,0x4E,
0x50,0x85,0x0C,0x38,0x6D,0x38,0xDD,0x03,0x26,0x00,
0x05,0x5A,0x87,0x00,0x4F,0x55,0x32,0x01,0x04,0x68,
0x61,0x6E,0x68,0x75,0x70,0x75,0x77,0x8B,0x7F,0x9C,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x23,0x23,0x81,0x00,0x0F,0x0F,0x04,
0x0E,0x88,0x88,0x88,0x08,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x37,
0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,
0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x12,0x13,0x14,0x15,
0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,
0xFF,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x12,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x13,0x1E,0x6D,0x01,0x00,0x00,0x00,0x0A,
0x00,0x00,0x00,0xD0,0x07,0x50,0x39,0x8E,0x01
}; 



volatile unsigned long *gpbcon = NULL;
volatile unsigned long *gpbdat = NULL;

/* 构造一个触摸屏设备结构体 */
struct gt1151_ts_priv {
	struct i2c_client *client; /* I2C 设备 */
	struct input_dev *input;   /* 输入设备结构体 */
	//struct delayed_work work;  /* 延迟工作队列 */
	//struct mutex mutex;        /* 互斥体 */
	int irq;                   /* 中断 */
};



/**
 * 发送和接受函数，虽然内核中提供了i2c_master_recv和i2c_master_send，
 * 但是这两个函数只适合单个msg的情况
*/
/**
* i2c_master_read_gt1151 - issue two I2C message in master receive mode
* @client: handler to slave device
* @buf_index: buffer address
* @buf_data: where to store data read from slave
* @len_data: the bytes of buf_data to read 
*
* returns negative errno, or else the number of bytes read
*/
static int i2c_master_read_gt1151(struct i2c_client *client, 
		__u16  buf_index, unsigned char *buf_data, 
		unsigned short len_data)
{
	int ret;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = I2C_M_NOSTART,
			.len = 1,
			.buf = &buf_index,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len_data,
			.buf = buf_data,
		}
	};

	ret = i2c_transfer(client->adapter, msgs, 2);

	return (ret == 2) ? len_data : ret;
}

/**
* i2c_master_write_gt1151 - issue a single I2C message in master transmit mode
* @client: handler to slave device
* @buf_index: buffer address
* @buf_data: data that wile be write to the slave
* @len_data: the bytes of buf_data to write
*
* returns negative errno, or else the number of bytes written
*/
static int i2c_master_write_gt1151(struct i2c_client *client, 
		__u16 buf_index, unsigned char const *buf_data,
		unsigned short len_data)
{
	unsigned char buf[2];
	int ret;

	struct i2c_msg msgs[1] = {
		{
			.addr = client->addr,
			.flags = 0, /* default write flag */
			.len = len_data + 1, 
			.buf = buf,
		}
	};

	buf[0] = buf_index;
	memcpy(&buf[1], buf_data, len_data);

	ret = i2c_transfer(client->adapter, msgs, 1);
	
	return (ret == 1) ? sizeof(buf) : ret;
}

/**
* 延迟工作，当产生中断时调用，负责从I2C总线上读取数据，然后按照数
* 据手册上的进行解析，然后进行上报。
*/

static void gt1151_ts_poscheck(struct work_struct *work)
{
	struct gt1151_ts_priv *priv = container_of(work, 
	 			struct gt1151_ts_priv, work.work);
	unsigned char buf[14];
	unsigned short xpos[3] = {0}, ypos[3] = {0};
	unsigned char event[3] = {0};
	unsigned char query = 0;
	int ret, i;

	mutex_lock(&priv->mutex);

	i2c_master_read_gt1151(priv->client, QUERY_BUF, &query, 1);
	if (!(query & 0x80)) {
		dev_err(&priv->client->dev, "no finger touch\n");
		goto out;
	}

	memset(&buf, 0, sizeof(buf));

	ret = i2c_master_read_gt1151(priv->client, POINT_INFO_BUF, buf, 14);
	if (ret != 14) {
		dev_err(&priv->client->dev, "failed to read point info buffer\n");
		goto out;
	}

	/* touch key */
	if (buf[0] == 0x41) {
		dev_info(&priv->client->dev, "the key number %d\n", buf[1]);
		if (buf[1] == 0x04)
			input_report_key(priv->input, KEY_HOME, !!buf[2]);
		else if (buf[1] == 0x03)
			input_report_key(priv->input, KEY_MENU, !!buf[2]);
		else if (buf[1] == 0x02)
			input_report_key(priv->input, KEY_BACK, !!buf[2]);
		else if (buf[1] == 0x01)
			input_report_key(priv->input, KEY_POWER, !!buf[2]);
		else
			goto out;

		input_sync(priv->input);
		goto out;
	}

	/* finger 0 */
	if (buf[0] & 0x01) {
		xpos[0] = ((buf[3] & 0x0F) << 8) | buf[2];
		ypos[0] = ((buf[3] & 0xF0) << 4) | buf[4];
		event[0] = buf[5] & 0x0F;
	}

	/* finger 1 */
	if (buf[0] & 0x02) {
		xpos[1] = ((buf[7] & 0x0F) << 8) | buf[6];
		ypos[1] = ((buf[7] & 0xF0) << 4) | buf[8];
		event[1] = buf[9] & 0x0F;
	}

	/* finger 2 */
	if (buf[0] & 0x04) {
		xpos[2] = ((buf[11] & 0x0F) << 8) | buf[10];
		ypos[2] = ((buf[11] & 0xF0) << 4) | buf[12];
		event[2] = buf[13] & 0x0F;
	}

	for (i = 0; i < 3; i++) {
		input_report_abs(priv->input, ABS_MT_POSITION_X, ypos[i]);
		input_report_abs(priv->input, ABS_MT_POSITION_Y, xpos[i]);
		input_report_abs(priv->input, ABS_MT_TOUCH_MAJOR, !!event[i]);
		input_report_abs(priv->input, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(priv->input);
		dev_info(&priv->client->dev, "finger %d > xpos = %d, \
		ypos = %d, event = %d\n", i, ypos[i], xpos[i], event[i]);
	}
	input_sync(priv->input);

out:
	mutex_unlock(&priv->mutex);
	enable_irq(priv->irq);
}
typedef struct _tp_str{
    u16 tp_x[10];
	u16 tp_y[10];
	u8  tp_id[10];
;
}tp_str;
const u16 GT1151_TPX_TBL[10]={GT_TP1_REG,GT_TP2_REG,GT_TP3_REG,GT_TP4_REG,GT_TP5_REG,GT_TP6_REG,GT_TP7_REG,GT_TP8_REG,GT_TP9_REG,GT_TP10_REG};
tp_str tp_str_now;

/* 中断服务子程序*/
static irqreturn_t gt1151_ts_irq(int irq, void *dev_id)
{
	struct gt1151_ts_priv *priv = dev_id;
    int num=0,i;
	unsigned char temp;
	u8 buf[5];
	//读取触摸点的状态
	i2c_master_read_gt1151(priv->client,GT_GSTID_REG, temp, 1);
	num=temp&0x0f;
	for(i=0;i<num;i++)
	{
		i2c_master_read_gt1151(priv->client,GT1151_TPX_TBL[i], buf, 5);
		tp_str_now.tp_id[i]=buf[0]&0x0f;
		tp_str_now.tp_x[i]=((u16)buf[2]<<8)+buf[1];
		tp_str_now.tp_y[i]=((u16)buf[4]<<8)+buf[3];
		
		input_report_key(priv->input,ABS_MT_TRACKING_ID,tp_str_now.tp_id[i]);
		input_report_key(priv->input, BTN_TOUCH, 1);
		input_report_abs(priv->input, ABS_MT_POSITION_X, tp_str_now.tp_x[i]);
		input_report_abs(priv->input, ABS_MT_POSITION_Y, tp_str_now.tp_y[i]);
		input_mt_sync(priv->input);
		
	}
	input_sync(priv->input);

	return IRQ_HANDLED;
}

/**
* gt1151_identify_capsensor - identify capacitance sensor model
*
* returns error -1, or else suc 0
*/
static int gt1151_identify_capsensor(struct i2c_client *client)
{
	unsigned char buf[4];
	unsigned char query = 0;

    i2c_master_read_gt1151(client, GT_PID_REG, buf, 4);
    if(strcmp((char*)buf,"1151")==0)
		return 0

	return -1;
}


//i2c地址选择
void i2c_addr_select(void)
{
	pass;
}

void updata_cfg_reg(struct i2c_client *client)
{
	unsigned char temp;
	temp=0x2a;
	//软复位
	i2c_master_write_gt1151(client,GT_CTRL_REG,temp,1);
	//更新配置
	i2c_master_write_gt1151(client,GT_CFGS_REG,GT911_CFG_TBL,239);
	mdelay(10);
	temp=0x00;
	//结束复位
	i2c_master_write_gt1151(client,GT_CTRL_REG,temp,1);
	
}

/* probe函数，在i2c设备和i2c驱动匹配时会调用此函数来完成相应的工作 */
static int gt1151_ts_probe(struct i2c_client *client, 
			const struct i2c_device_id *idp)
{
	struct gt1151_ts_priv *priv;
	struct input_dev *input;
	int error;

	//i2c地址选择 0x28/0x29
	i2c_addr_select();
	
	
	/* 识别此设备型号是否为gt1151 */
	error = gt1151_identify_capsensor(client);
	if (error) {
		dev_err(&client->dev, "cannot identify the touch screen\n");
		goto err0;
	}

	//配置更新
	updata_cfg_reg();
	
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		error = -ENOMEM;
		goto err0;
	}

	dev_set_drvdata(&client->dev, priv);

	/* 分配一个input设备 */
	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto err1;
	}

	/* 设置input设备所支持的事件类型 */
	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0,720 ,0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,1280,0, 0);

	input->name = "gt1151 touch screen";
	input->phys = "I2C";
	input->id.bustype = BUS_I2C;

	input_set_drvdata(input, priv);

	priv->client = client;
	priv->input = input;

	/* 向输入子系统注册此input设备 */
	error = input_register_device(input);
	if (error) {
		dev_err(&client->dev, "failed to register input device\n");
		goto err1;
	}
	
	priv->irq = client->irq;

	
	
	/* 注册中断，低电平触发 */
	error = request_irq(priv->irq, gt1151_ts_irq, IRQF_TRIGGER_LOW,
						client->name, priv);
	if (error) {
		dev_err(&client->dev, "unable to request touchscreen IRQ\n");
		goto err2;
	}

	device_init_wakeup(&client->dev, 1);
	return 0;
	
err2:
	input_unregister_device(input);
	input = NULL;
err1:
	input_free_device(input);
	kfree(priv);
err0:
	dev_set_drvdata(&client->dev, NULL);
	return error;
}

/* 当没有使用此设备时调用移除函数进行注销 */
static int __devexit gt1151_ts_remove(struct i2c_client *client)
{
	struct gt1151_ts_priv *priv = dev_get_drvdata(&client->dev);

	free_irq(priv->irq, priv);
	input_unregister_device(priv->input);
	kfree(priv);

	dev_set_drvdata(&client->dev, NULL);

	return 0;
}



/* 驱动支持的设备列表，用来匹配 */
static const struct i2c_device_id gt1151_ts_id[] = {
	{"gt1151", 0},
	{}			/* should not omitted */
};
MODULE_DEVICE_TABLE(i2c, gt1151_ts_id);

static struct i2c_driver gt1151_ts_driver = {
	.driver = {
		.name = "gt1151-ts",
	},
	.probe = gt1151_ts_probe,
	.remove = __devexit_p(gt1151_ts_remove),
	
	.id_table = gt1151_ts_id,
};

/* 模块加载函数 */
static int __init gt1151_ts_init(void)
{
	gpbcon = (volatile unsigned long *)ioremap(GPIO_ADDR, 16);
	gpbdat = gpbcon + 1;
	
	i2c_add_driver(&gt1151_ts_driver);	
}

/* 模块卸载函数 */
static void __exit gt1151_ts_exit(void)
{
	iounmap(gpbcon);
	
	i2c_del_driver(&gt1151_ts_driver);
}

module_init(gt1151_ts_init);
module_exit(gt1151_ts_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CJOK <cjok.liao@gmail.com>");
MODULE_DESCRIPTION("gt1151 touchscreen driver");