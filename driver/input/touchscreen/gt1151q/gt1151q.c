/***************************************************************
文件名		: gt1151q.c
作者	  	: 潘志爱
版本	   	: V1.0
描述	   	: Linux gt1151q电容屏驱动，单点触摸
其他	   	: 无
日志	   	: 初版V1.0 2021/11/25 潘志爱创建
***************************************************************/
#include "gt1151q.h"

struct  gt1151q_dev {
	int reset_pin;		//复位脚
	int int_pin;		//中断脚
    int max_touch_num;	//最大触摸点数
	int abs_x_max;		//触摸屏横向最大长度
	int abs_y_max;		//触摸屏纵向最大长度
	s32 irq_is_disable;
	s32 use_irq;
	struct input_dev *inputdev;
	struct i2c_client *client; 
	struct work_struct work;
	spinlock_t irq_lock;           
};

struct gt1151q_dev *gt1151qdev;//gt1151q设备指针


/*
  * @description : 读取 I2C 设备多个寄存器数据
  * @param – dev : I2C 设备
  * @param – reg : 要读取的寄存器首地址
  * @param – val : 读取到的数据
  * @param – len : 要读取的数据长度
  * @return : 操作结果
*/
static int gt1151q_read_regs(struct gt1151q_dev *dev, u16 reg, void *val,int len)
{
	int ret;
	struct i2c_msg msg[2];
	u8 regdata[2];
	struct i2c_client *client = (struct i2c_client *)dev->client;
	regdata[0] = reg >> 8;
    regdata[1] = reg & 0xFF;

	/* msg[0]，第一条写消息，发送要读取的寄存器首地址 */
	msg[0].addr = client->addr;					//I2C 器件地址 
	msg[0].flags = 0;							//标记为发送数据
	msg[0].buf = &regdata[0];					//读取的首地址
	msg[0].len = 2; 							//reg地址长度

	/* msg[1]，第二条读消息，读取寄存器数据 */
	msg[1].addr = client->addr; 				//I2C 器件地址
	msg[1].flags = I2C_M_RD;					//标记为读取数据
	msg[1].buf = val;							//读取数据缓冲区
	msg[1].len = len;							//要读取的数据长度
	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret == 2) {
		ret = 0;
	} else {
		ret = -EREMOTEIO;
	}
	return ret;
}

/*
    * @description : 向 I2C 设备多个寄存器写入数据
    * @param – dev : 要写入的设备结构体
    * @param – reg : 要写入的寄存器首地址
    * @param – buf : 要写入的数据缓冲区
    * @param – len : 要写入的数据长度
    * @return : 操作结果
*/
static s32 gt1151q_write_regs(struct gt1151q_dev *dev, u16 reg, u8 *buf, u8 len)
{
	u8 b[256];
	struct i2c_msg msg;
	struct i2c_client *client = (struct i2c_client *)dev->client;

	b[0] = reg >> 8;			//寄存器首地址低8位
    b[1] = reg & 0XFF;			//寄存器首地址高8位
	memcpy(&b[2],buf,2);		//将要写入的数据拷贝到数组b里面

	msg.addr = client->addr;	//I2C 器件地址
	msg.flags = 0;				//标记为写数据

	msg.buf = b;				//要发送的数据缓冲区
	msg.len = len + 2;			//要发送的数据长度,寄存器地址长度+数据长度

	return i2c_transfer(client->adapter, &msg, 1);
}


/*
    * @function : 中断失能
    * @param – ts : 要写入的gt1151q_dev设备结构体
    * @return : 无
*/
void gtp_irq_disable(struct gt1151q_dev *ts)
{
    unsigned long irqflags;

   //GTP_DEBUG_FUNC();
    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*
    * @function : 中断使能
    * @param – ts : 要写入的gt1151q_dev设备结构体
    * @return : 无
*/
void gtp_irq_enable(struct gt1151q_dev *ts)
{
    unsigned long irqflags = 0;

   // GTP_DEBUG_FUNC();
    
    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable) 
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0; 
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*
    * @function : 触摸点按下，上报触摸点信息
    * @param – dev : 要写入的gt1151q_dev设备结构体
	* @param - id  : 触摸点的track_id
	* @param - x   : 要上报的x坐标
	* @param - y   : 要上报的y坐标
	* @param - w   : 要上报触摸点的宽度
    * @return : 无
*/
static void gtp_touch_down(struct gt1151q_dev *dev,s32 id,s32 x,s32 y,s32 w)
{
  
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(dev->inputdev, id);
    input_report_abs(dev->inputdev, ABS_MT_TRACKING_ID, id);
    input_report_abs(dev->inputdev, ABS_MT_POSITION_X, dev->abs_x_max - x);
    input_report_abs(dev->inputdev, ABS_MT_POSITION_Y, dev->abs_y_max - y);
    input_report_abs(dev->inputdev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(dev->inputdev, ABS_MT_WIDTH_MAJOR, w);
#else
    input_report_key(dev->inputdev, BTN_TOUCH, 1);
    input_report_abs(dev->inputdev, ABS_X, x);
    input_report_abs(dev->inputdev, ABS_Y, y);
    input_report_abs(dev->inputdev, ABS_Z, 0);
    input_report_abs(dev->inputdev, ABS_PRESSURE, 1);
    input_sync(dev->inputdev);
#endif
}


/*
    * @function : 触摸松开，移除触摸点信息
    * @param – dev : 要写入的gt1151q_dev设备结构体
	* @param - id  : 触摸点的track_id
    * @return : 无
*/
static void gtp_touch_up(struct gt1151q_dev *ts, s32 id)
{


#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->inputdev, id);
    input_report_abs(ts->inputdev, ABS_MT_TRACKING_ID, -1);
    GTP_DEBUG("Touch id[%2d] release!", id);
#else
    input_report_key(ts->inputdev, BTN_TOUCH, 0);
    input_report_abs(ts->inputdev, ABS_PRESSURE, 0);
    input_sync(ts->inputdev);
#endif
   
}

/*
    * @function : 触摸点处理，判断触摸点是否按下，读取触摸点点数和坐标，并上报和移除处理
    * @param – dev : 要写入的gt1151q_dev设备结构体
    * @return : 无
*/
static void gt1151q_process_events(struct gt1151q_dev *dev)
{
	u8  point_data[GT_TP_REG_NUM * dev->max_touch_num]; //所有触摸点数据
	u8 *coor_data;					//每个触摸点的数据
	static u8 touch_num = 0;		//当前触摸点数
	static u8 pre_touch = 0;		//当前移除的触摸点数
	u8 c_data;						//用于清除0x814E的数据
	u8 reg_point = 0;				//当前0x814E寄存器数据
	int i ;
	int id ;   						//trak_id
	int input_x, input_y, input_w;  
	int error;

	c_data = 0x00;
	gt1151q_write_regs(dev, GT_TP_NUM_REG, &c_data, 1);	//清除0x814E寄存器，每次读完都要清零
	msleep(50);
	error = gt1151q_read_regs(dev, GT_TP_NUM_REG, &reg_point, 1); //读取0x814E寄存器数据
	if (error) {
		dev_err(&dev->client->dev, "I2C transfer error: %d\n", error);
		return error;
	}
	if(reg_point&0X80 == 0) {		//触摸点数据未就绪
		goto exit_work_func;
	}
	touch_num = reg_point & 0x0F;  //获取触摸点数bit0:bit3
	if (touch_num > dev->max_touch_num)
		goto touch_num_error;
	//printk("touch_num = %d\n",touch_num);
	if (touch_num) {
        for(i=0; i<touch_num; i++){
           	error = gt1151q_read_regs(dev, GT_TP1_TRACK_ID_REG + GT_CONTACT_SIZE*i, &point_data[i*GT_TP_REG_NUM], GT_TP_REG_NUM);
			if (error)
				goto error;
			coor_data = &point_data[i*GT_TP_REG_NUM];
			id = coor_data[0] & 0x0F;   /*trak_id*/
			input_x = get_unaligned_le16(&coor_data[1]);
			input_y = get_unaligned_le16(&coor_data[3]);
			input_w = get_unaligned_le16(&coor_data[5]);
			gtp_touch_down(dev, id, input_x, input_y, input_w);
		}
	}
	else if(pre_touch) {
		gtp_touch_up(dev, 0);
	}

	pre_touch = touch_num;

	exit_work_func:
		printk("NO TOUCH DOWN!\n");
	touch_num_error:
		printk("touch_num > max_touch_num!\n");
	error:
		printk("Read GT_TP1_TRACK_ID_REG is error!\n");
}


/*
    * @function : 中断下半部处理函数，采用队列方式
    * @param – work : work_struct结构体，即对应的工作
    * @return : 无
*/
void gt1151_work_func_t(struct work_struct *work)
{
	struct gt115q_dev *ts = NULL;
	ts = container_of(work, struct gt1151q_dev, work);
	gt1151q_process_events(ts);
}


/*
    * @function : 中断处理函数，这里在下半部进行数据的处理
    * @param – work : work_struct结构体，即对应的工作
    * @return : 无
*/
static irqreturn_t gt1151q_irq_handler(int irq, void *dev_id)
{
	struct gt1151q_dev *dev = dev_id ;
	//gtp_irq_disable(dev);
	//gt1151q_process_events(dev);
	schedule_work(&dev->work);		//work工作的调度
	//gtp_irq_enable(dev);
	return IRQ_HANDLED;
}


/*
    * @function : 复位脚reset_pin和中断脚int_pin初始化,并做上电时序
    * @param – client : i2c_client设备结构体
	* @param – dev : gt1151q_dev设备结构体
    * @return : 0:成功，其他值失败
*/
static int gt115q_ts_reset(struct i2c_client *client, struct gt1151q_dev *dev)
{
	int error;
//	int reg_data = 0;

	if (gpio_is_valid(dev->int_pin)) {
		error = devm_gpio_request_one(&client->dev,
					dev->int_pin, GPIOF_IN,
					"gt1151q int");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as int pin, error %d\n",
				dev->int_pin, error);
			return error;
		}
	}

	if (gpio_is_valid(dev->reset_pin)) {
		error = devm_gpio_request_one(&client->dev,
					dev->reset_pin, GPIOF_OUT_INIT_HIGH,
					"gt1151q reset");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				dev->reset_pin, error);
			return error;
		}
	}

	gpio_set_value(dev->int_pin, 1);    //拉低INT引脚
    msleep(50);
    //gpio_direction_input(dev->int_pin); //INT引脚设置为输入

	gpio_set_value(dev->reset_pin, 0); //复位GT9147
    msleep(20);
    gpio_set_value(dev->reset_pin, 1); //停止复位GT9147
    msleep(60);
//	gt1151q_read_regs(dev, GT_REQUEST_REG, &reg_data, 1);
//	printk("reg_data = %d\n", reg_data);
	return 0;
}


/*
    * @function : input_dev结构体申请并注册,设置按键事件类型、键值，设置上报类型
	* @param – dev : gt1151q_dev设备结构体
    * @return : 0:成功，其他值失败
*/
static int gt1151q_request_input_dev(struct gt1151q_dev *dev)
{
	int error;

	dev->inputdev = devm_input_allocate_device(&dev->client->dev);
	if (!dev->inputdev) {
		dev_err(&dev->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	dev->inputdev->evbit[0] = BIT_MASK(EV_SYN) |
				  BIT_MASK(EV_KEY) |
				  BIT_MASK(EV_ABS);
	dev->inputdev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	__set_bit(INPUT_PROP_DIRECT, dev->inputdev->propbit);

	//input_set_abs_params(dev->inputdev,ABS_MT_POSITION_X, 0, dev->abs_x_max, 0, 0);
	//input_set_abs_params(dev->inputdev,ABS_MT_POSITION_Y, 0, dev->abs_y_max, 0, 0);
	//input_set_abs_params(dev->inputdev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	//input_set_abs_params(dev->inputdev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	//input_set_abs_params(dev->inputdev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(dev->inputdev,ABS_X, 0, dev->abs_x_max, 0, 0);
	input_set_abs_params(dev->inputdev,ABS_Y, 0, dev->abs_y_max, 0, 0);
	input_set_abs_params(dev->inputdev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(dev->inputdev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	input_mt_init_slots(dev->inputdev, dev->max_touch_num, 0);

	dev->inputdev->name = "GT1151Q Capacitive TouchScreen";
	dev->inputdev->phys = "input/ts";
	dev->inputdev->id.bustype = BUS_I2C;
	/*ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = 0x1001;
	ts->input_dev->id.version = 10427;*/

	error = input_register_device(dev->inputdev);
	if (error) {
		dev_err(&dev->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

/*
    * @function : I2C probe
	* @param – dev : gt1151q_dev设备结构体
	* @param – id : device id
    * @return : 0:成功，其他值失败
*/
static int gt1151q_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error;
	u8 data;
	//u8 data1=0;
	//struct gt1151q_dev *gt1151qdev;
	gt1151qdev =  devm_kzalloc(&client->dev, sizeof(*gt1151qdev), GFP_KERNEL);	/**/

	if(!gt1151qdev) {
		return -ENOMEM;
	}
	gt1151qdev->client = client;
	i2c_set_clientdata(client, gt1151qdev);
	//gt1151qdev.client = client;

	gt1151qdev->reset_pin = of_get_named_gpio(gt1151qdev->client->dev.of_node, "goodix,rst-gpio", 0);
	gt1151qdev->int_pin = of_get_named_gpio(gt1151qdev->client->dev.of_node, "goodix,irq-gpio", 0);

	error = gt115q_ts_reset(gt1151qdev->client, gt1151qdev);
	if(error < 0) {
		return error;
	}
	
	data =  0x02;
	gt1151q_write_regs(gt1151qdev, GT_CTRL_REG, &data, 1);
	//gt1151q_read_regs(gt1151qdev, GT_CTRL_REG, &data1, 1);
	//printk("GT_CTRL_REG=%d", data1);
	msleep(100);
	data =  0x00;
	gt1151q_write_regs(gt1151qdev, GT_CTRL_REG, &data, 1);
	//gt1151q_read_regs(gt1151qdev, GT_CTRL_REG, &data1, 1);
	//printk("GT_CTRL_REG=%d", data1);
	msleep(100);

	gt1151qdev->abs_x_max = ABS_X_MAX;
	gt1151qdev->abs_y_max = ABS_Y_MAX;
	gt1151qdev->max_touch_num = MAX_SUPPORT_POINTS;
	error = gt1151q_request_input_dev(gt1151qdev);
	if(error){
		return error;
	}
	INIT_WORK(&gt1151qdev->work, gt1151_work_func_t);
	error = devm_request_threaded_irq(&gt1151qdev->client->dev, gt1151qdev->client->irq, NULL, gt1151q_irq_handler,
					IRQ_TYPE_LEVEL_LOW | IRQF_ONESHOT, gt1151qdev->client->name, gt1151qdev);
	//error = request_irq(gt1151qdev->client->irq, gt1151q_irq_handler, IRQ_TYPE_EDGE_FALLING, gt1151qdev->client->name, gt1151qdev);
	if(error) {
		dev_err(&gt1151qdev->client->dev, "request IRQ failed: %d\n", error);
		return error;
	}

	return 0;
}


/*
    * @function : I2C remove
	* @param – dev : i2c_client设备结构体
    * @return : 0:成功，其他值失败
*/
static int gt1151q_remove(struct i2c_client *client)
{
	/* 函数具体程序 */
	struct gt1151q_dev *dev = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);
	//free_irq(dev->client->irq, dev);
	input_unregister_device(dev->inputdev);
	return 0;
}

/* 传统匹配方式 ID 列表 */
static const struct i2c_device_id gt1151q_id[] = {
	{"goodix,gt1151", 0},
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id gt1151q_of_match[] = {
	{ .compatible = "goodix,gt1151" },
	{ /* Sentinel */ }
};
/* i2c 驱动结构体 */
static struct i2c_driver gt1151q_driver = {
	.probe = gt1151q_probe,
	.remove = gt1151q_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "gt1151q",
		.of_match_table = gt1151q_of_match,
	},
	.id_table = gt1151q_id,
};


/*
    * @function : 驱动入口函数
    * @return : 0:成功，其他值失败
*/
static int __init gt1151q_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&gt1151q_driver);
	return ret;
}


/*
    * @function : 驱动出口函数
    * @return : 无
*/
static void __exit gt1151q_exit(void)
{
	printk("GT1151Q exit !\n");
	i2c_del_driver(&gt1151q_driver);
}

module_init(gt1151q_init);
module_exit(gt1151q_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("PZA");
