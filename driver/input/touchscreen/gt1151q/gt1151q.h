/***************************************************************
文件名		: gt1151q.h
作者	  	: 潘志爱
版本	   	: V1.0
描述	   	: Linux gt1151q电容屏驱动
其他	   	: 无
日志	   	: 初版V1.0 2021/11/25 潘志爱创建
***************************************************************/

#ifndef __GT1151Q_H
#define __GT1151Q_H

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/unaligned/le_struct.h>
#include <linux/regulator/consumer.h>


#define GTP_ICS_SLOT_REPORT 0  //1：多点触摸，0：单点触摸


#define GT1151Q_ADDR	0x14    /**GT1151Q的器件地址**/
#define GT_CTRL_REG	0x8040	/*GT1151Q的控制寄存器*/
#define GT_TP1_TRACK_ID_REG	0x814F/*第一个触摸点的track_id :bit0-bit3, 第2个触摸点的track_id  0x814F+8   第3个触摸点的track_id0X814F+2*8*/
#define GT_TP1_REG 		        0X8150  /* 第一个触摸点数据地址 */
#define GT_TP2_REG 		        0X8158	/* 第二个触摸点数据地址 */
#define GT_TP3_REG 		        0X8160  /* 第三个触摸点数据地址 */
#define GT_TP4_REG 		        0X8168  /* 第四个触摸点数据地址  */
#define GT_TP5_REG 		        0X8170	/* 第五个触摸点数据地址   */
#define MAX_SUPPORT_POINTS      1       /* 最多10点电容触摸 */
#define GT_TP_NUM_REG		0x814E      /*触摸的点数,bit0:bit3*/
#define GT_REQUEST_REG      0X8044      /*Request REG*/

#define GT_CONTACT_SIZE		8       /*每个触摸点数据地址间隔*/
#define GT_TP_REG_NUM       7       /*每个触摸点信息的寄存器个数：trak_id、x(L)、x(H)...W、H*/
#define ABS_X_MAX   800
#define ABS_Y_MAX   480

#define GTP_DEBUG_ON          0

#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-GTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)


#endif