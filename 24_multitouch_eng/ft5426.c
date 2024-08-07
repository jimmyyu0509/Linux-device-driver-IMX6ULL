#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>

#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/input/edt-ft5x06.h>
#include <linux/delay.h>

//#include <linux/timer.h>
//#include <linux/jiffies.h>
//#include <sys/ioctl.h>

#define MAX_SUPPORT_POINTS		5			/* 5 point touch */
#define TOUCH_EVENT_DOWN		0x00		/* down 	*/
#define TOUCH_EVENT_UP			0x01		/* up 	*/
#define TOUCH_EVENT_ON			0x02		/* on 	*/
#define TOUCH_EVENT_RESERVED	0x03		/* reserved */

/* FT5X06 reg relative define */
#define FT5X06_TD_STATUS_REG	0X02		/*	status reg address */
#define FT5x06_DEVICE_MODE_REG	0X00 		/*  mode reg */
#define FT5426_IDG_MODE_REG		0XA4		/*  INT mod */
#define FT5X06_READLEN			29			/*  read reg number */

struct ft5x06_dev{
	struct device_node *nd;                 /* device node	*/
	int irq_pin,reset_pin;                  /* INT and reset IO */ 
	int irq_num;                            /* INT NO. */
	void *private_data;                     /* private data */
	struct i2c_client *client;              /* I2C client end */
	struct input_dev *input;				/* input device struct */

};
static struct ft5x06_dev ft5x06;


/*
 * @description     : reset FT5X06
 * @param - client 	: operate i2c
 * @param - multidev: defined multitouch device
 * @return          : 0，success; other negative value,failure
 */
static int ft5x06_ts_reset(struct i2c_client *client , struct ft5x06_dev *dev){
	int ret = 0;

	if (gpio_is_valid(dev->reset_pin)) {  		/* check IO whether effect */
		/* apply reset IO，and default output low level 0 */
		ret = devm_gpio_request_one(&client->dev,	
					dev->reset_pin, GPIOF_OUT_INIT_LOW,
					"edt-ft5x06 reset");
		if (ret) {
			return ret;
		}

		msleep(5);
		gpio_set_value(dev->reset_pin, 1);	/* output high level 1, stop to reset */
		msleep(300);
	}

	return 0;

}

/*
 * @description	: read some regs from FT5X06
 * @param - dev:  ft5x06 device
 * @param - reg:  read first reg address
 * @param - val:  read data
 * @param - len:  read data length
 * @return 		: operation result
 */
static int ft5x06_read_regs(struct ft5x06_dev *dev, u8 reg, void *val, int len)
{
	int ret;
	struct i2c_msg msg[2];
	struct i2c_client *client = (struct i2c_client *)dev->client;

	/* msg[0] is send to read first address */
	msg[0].addr = client->addr;			/* ft5x06 address */
	msg[0].flags = 0;					/* mark to send data */
	msg[0].buf = &reg;					/* read first address */
	msg[0].len = 1;						/* reg length*/

	/* msg[1] read data */
	msg[1].addr = client->addr;			/* ft5x06 address */
	msg[1].flags = I2C_M_RD;			/* mark to read data */
	msg[1].buf = val;					/* read data buffer */
	msg[1].len = len;					/* read data length */

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret == 2) {
		ret = 0;
	} else {
		ret = -EREMOTEIO;
	}
	return ret;
}

/*
 * @description	: Write data to some regs
 * @param - dev:  ft5x06 device
 * @param - reg:  write first reg address
 * @param - buf:  write data buffer
 * @param - len:  write data length
 * @return 	  :   operation results
 */
static s32 ft5x06_write_regs(struct ft5x06_dev *dev, u8 reg, u8 *buf, u8 len)
{
	u8 b[256];
	struct i2c_msg msg;
	struct i2c_client *client = (struct i2c_client *)dev->client;
	
	b[0] = reg;					/* first reg address */
	memcpy(&b[1],buf,len);		/* copy write data to matrix b */
		
	msg.addr = client->addr;	/* ft5x06 address */
	msg.flags = 0;				/* mark to write data */

	msg.buf = b;				/* write data buffer */
	msg.len = len + 1;			/* write data length */

	return i2c_transfer(client->adapter, &msg, 1);
}

#if 0
/*
 * @description	: read a signed ft5x06 reg value，read a reg
 * @param - dev:  ft5x06 device
 * @param - reg:  read reg
 * @return 	  :   read reg value
 */
static unsigned char ft5x06_read_reg(struct ft5x06_dev *dev, u8 reg)
{
	u8 data = 0;
	ft5x06_read_regs(dev, reg, &data, 1);
	return data;
}
#endif

/*
 * @description	: write a asigned value to a ft5x06 asigned reg
 * @param - dev:  ft5x06 device
 * @param - reg:  write reg
 * @param - data: write value
 * @return   :    NO
 */
static void ft5x06_write_reg(struct ft5x06_dev *dev, u8 reg, u8 data)
{
	u8 buf = 0;
	buf = data;
	ft5x06_write_regs(dev, reg, &buf, 1);
}

/*
 * @description     : FT5X06 INT fuction
 * @param - irq 	: INT NO. 
 * @param - dev_id	: device structure
 * @return 			: INT execute result
 */
static irqreturn_t ft5x06_handler(int irq, void *dev_id)
{
	struct ft5x06_dev *multidata = dev_id;

	u8 rdbuf[29];
	int i, type, x, y, id;
	int offset, tplen;
	int ret;
	bool down;

	offset = 1; 	/* offset 1，is 0X02+1=0x03,start from 0X03 is touch value */
	tplen = 6;		/* One touch point has six regs to save touch value */

    //printk("ft5x06_handler\r\n");
	memset(rdbuf, 0, sizeof(rdbuf));		/* clear */

	/* read FT5X06 touch point coordinate from 0x02 reg,
	continue to read 29 regs . */
	ret = ft5x06_read_regs(multidata, FT5X06_TD_STATUS_REG, rdbuf, FT5X06_READLEN);
	if (ret) {
		goto fail;
	}

	/* upload every touch point coordinate */
	for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
		/* get every touch point coordinate,upload Type B format*/
		u8 *buf = &rdbuf[i * tplen + offset]; /* get every touch point original data start address */

		/* Taking the first touch point as an example，reg TOUCH1_XH(address0X03), each bit describes as follows：
		 * bit7:6  Event flag  0:down 1:up 2：on 3：no event
		 * bit5:4  reversed
		 * bit3:0  X coordinat touch point's 11~8 bit。
		 */
		type = buf[0] >> 6;     /* get touch type */
		if (type == TOUCH_EVENT_RESERVED)
			continue;
 
		/* The touch panel data we use is reverse and FT5X06 */
		x = ((buf[2] << 8) | buf[3]) & 0x0fff;
		y = ((buf[0] << 8) | buf[1]) & 0x0fff;
		
		/* Taking the first touch point as an example,reg TOUCH1_YH(address0X05), each bit describes as follows：
		 * bit7:4  Touch ID ，indicate which touch point 
		 * bit3:0  Y coordinat touch point's 11~8 bit。
		 */
		/*upload data*/
		id = (buf[2] >> 4) & 0x0f;
		down = type != TOUCH_EVENT_UP;

		input_mt_slot(multidata->input, id);/*ABS_MT_SLOT*/
		input_mt_report_slot_state(multidata->input, MT_TOOL_FINGER, down);
		/* ABS_MT_TRACKING_ID */

		if (!down)
			continue;

		input_report_abs(multidata->input, ABS_MT_POSITION_X, x); /* ABS_MT_POSITION_X */
		input_report_abs(multidata->input, ABS_MT_POSITION_Y, y); /* ABS_MT_POSITION_Y */
	}

	input_mt_report_pointer_emulation(multidata->input, true);
	input_sync(multidata->input);    /*SYN_REPORT*/

fail:
	return IRQ_HANDLED;

}

/*
 * @description     : FT5x06 INT initialization
 * @param - client 	: operate i2c
 * @param - multidev: defined multitouch device
 * @return          : 0，success ; other negative value,failure
 */
static int ft5x06_ts_irq(struct i2c_client *client, struct ft5x06_dev *dev)
{
	int ret = 0;

	/* 1,apply INT GPIO */
	if (gpio_is_valid(dev->irq_pin)) {
		ret = devm_gpio_request_one(&client->dev, dev->irq_pin,
					GPIOF_IN, "edt-ft5x06 irq");
		if (ret) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				dev->irq_pin, ret);
			return ret;
		}
	}

	/* 2，apply INT,client->irq is IO INT */
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					ft5x06_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, &ft5x06);
	if (ret) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		return ret;
	}

	return 0;
}

/*
  * @description     : i2c driver's probe function ，while driver and device match,
  *                    this fucntion will execute.
  * @param - client  : i2c device
  * @param - id      : i2c device ID
  * @return          : 0，success;other negative value,failure
  */
static int ft5x06_ts_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{	
	int ret = 0;
	//test
	//int value = 0;
	printk("ft5x06_probe!\r\n");

	ft5x06.client = client;

	/* 1，get INT and reset IO of device tree */
	ft5x06.irq_pin = of_get_named_gpio(client->dev.of_node, "interrupt-gpios", 0);
	ft5x06.reset_pin = of_get_named_gpio(client->dev.of_node, "reset-gpios", 0);

	/* 2，reset FT5x06 */
	ret = ft5x06_ts_reset(client, &ft5x06);
	if(ret < 0) {
		goto fail;
	}

	/* 3，initial INT */
	ret = ft5x06_ts_irq(client, &ft5x06);
	if(ret < 0) {
		goto fail;
	}

	/* 4，initial FT5X06 */
	ft5x06_write_reg(&ft5x06 , FT5x06_DEVICE_MODE_REG, 0); 	/* enter normal mode */
	ft5x06_write_reg(&ft5x06 , FT5426_IDG_MODE_REG, 1); 		/* FT5426 INT mode */

    //test
	//value = ft5x06_read_reg(&ft5x06 ,FT5426_IDG_MODE_REG);
	//printk("FT5426_IDG_MODE_REG =%#X\r\n", value);

	/* 5，input device register */
	ft5x06.input = devm_input_allocate_device(&client->dev);
	if (!ft5x06.input) {
		ret = -ENOMEM;
		goto fail;
	}
	ft5x06.input->name = client->name;
	ft5x06.input->id.bustype = BUS_I2C;
	ft5x06.input->dev.parent = &client->dev;
    
	__set_bit(EV_SYN, ft5x06.input->evbit);
	__set_bit(EV_KEY, ft5x06.input->evbit);
	__set_bit(EV_ABS, ft5x06.input->evbit);
	__set_bit(BTN_TOUCH, ft5x06.input->keybit);
    
	/* Single touch */
	input_set_abs_params(ft5x06.input, ABS_X, 0, 1024, 0, 0);
	input_set_abs_params(ft5x06.input, ABS_Y, 0, 600, 0, 0);
	/* Multi touch */
	input_set_abs_params(ft5x06.input, ABS_MT_POSITION_X,0, 1024, 0, 0);
	input_set_abs_params(ft5x06.input, ABS_MT_POSITION_Y,0, 600, 0, 0);	     
	ret = input_mt_init_slots(ft5x06.input, MAX_SUPPORT_POINTS, 0);
	if (ret) {
		goto fail;
	}

	ret = input_register_device(ft5x06.input);
	if (ret)
		goto fail;

	return 0;

fail:
	return ret;

}

/*
 * @description     : i2c driver's remove function，while removeing i2c driver,this function will execute.
 * @param - client 	: i2c device
 * @return          : 0，success;other negative value,failute
 */
static int ft5x06_ts_remove(struct i2c_client *client)
{	
	/* remove input_dev */
	input_unregister_device(ft5x06.input);
	return 0;
}


/*traditional match table*/
static struct i2c_device_id ft5x06_ts_id[] = {
	{"edt-ft5206", 0},
	{"edt-ft5426", 0},
	{ }
};

/*device tree match table*/
static struct of_device_id ft5x06_of_match[] = {
	{ .compatible = "edt,edt-ft5206" },
	{ .compatible = "edt,edt-ft5406" },
	{ /* sentinel*/}
};

/*i2c_driver*/
static struct i2c_driver ft5x06_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "edt_ft5x06",
		.of_match_table = of_match_ptr(ft5x06_of_match),
	},
	.id_table = ft5x06_ts_id,
    .probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	
	
};

/*driver init function*/
static int __init ft5x06_init(void)
{
    int ret;
	ret = i2c_add_driver(&ft5x06_ts_driver);	
	return ret;
}
/*driver exit function*/
static void __exit ft5x06_exit(void)
{
     i2c_del_driver(&ft5x06_ts_driver);     
}

module_init(ft5x06_init);
module_exit(ft5x06_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jimmyyu");

