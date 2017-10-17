/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
/* #include <mach/mt6333.h> */

#include "kd_flashlight.h"

/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
#include <mach/gpio_const.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <mt-plat/mt_gpio.h>
#include <linux/delay.h>

#define GPIO_FLASH_EN         (GPIO10 | 0x80000000)
#define GPIO_FLASH_EN_M_GPIO  GPIO_MODE_00
#define GPIO_FLASH_STROBE         (GPIO9 | 0x80000000)
#define GPIO_FLASH_STROBE_M_GPIO  GPIO_MODE_00
#define GPIO_FLASH_GPIO         (GPIO8 | 0x80000000)
#define GPIO_FLASH_GPIO_M_GPIO  GPIO_MODE_00

/* TPV Mobile/WillHuang, 20151005, PMS# ,} */



/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[strobe_main_sid2_part1.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)


//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */
static struct work_struct workTimeOut;
static int g_timeOutTimeMs=0;
static u32 strobe_Res=0;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
extern int get_dutyH(void);
extern unsigned long get_timeoutTimeMsH(void);
extern unsigned char get_flashLedCloseFlagH(void);
static DEFINE_MUTEX(g_strobeI2CSem);

#define e_DutyNum 18

static unsigned char m_flashLedCloseFlagL = 1;
static int m_DutyL = -1;
static int gIsTorch[e_DutyNum] = {1, 1, 1, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; 

//75,150,225,300 mA
static int gLedTorchDuty[e_DutyNum] = { 4, 8, 12, 16,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

/* current(mA) 75,150,225,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000 mA */
static int gLedFlashDuty[e_DutyNum] = { 4, 8, 12, 16, 19, 22, 24, 27, 30, 33, 36, 38, 41, 44, 46, 49, 52, 60};

static unsigned char g_Timer_is_enabled=0;
/*****************************************************************************
Functions
*****************************************************************************/
static struct i2c_client *SGM3784_i2c_client;

struct SGM3784_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct SGM3784_chip_data {
	struct i2c_client *client;
	struct SGM3784_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

/* i2c access*/
static int SGM3784_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct SGM3784_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("SGM3784 failed writting at 0x%02x\n", reg);
	return ret;
}

static int SGM3784_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct SGM3784_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}

static int SGM3784_chip_init(struct SGM3784_chip_data *chip)
{

	/* Pull UP GPIO8->SGM3784 HWEN */
	mt_set_gpio_mode(GPIO_FLASH_EN, GPIO_FLASH_EN_M_GPIO);
	mt_set_gpio_dir(GPIO_FLASH_EN, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_FLASH_STROBE, GPIO_FLASH_STROBE_M_GPIO);
	mt_set_gpio_dir(GPIO_FLASH_STROBE, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_FLASH_GPIO, GPIO_FLASH_GPIO_M_GPIO);
	mt_set_gpio_dir(GPIO_FLASH_GPIO, GPIO_DIR_OUT);
	msleep(1);
	mt_set_gpio_out(GPIO_FLASH_EN, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
	msleep(1);

	return 0;
}

static int SGM3784_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct SGM3784_chip_data *chip;
	struct SGM3784_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("SGM3784_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_ERR("SGM3784 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct SGM3784_chip_data), GFP_KERNEL);
	chip->client = client;
    chip->lock = g_strobeI2CSem;
	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_ERR("SGM3784 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct SGM3784_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (SGM3784_chip_init(chip) < 0)
		goto err_chip_init;

	SGM3784_i2c_client = client;

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("SGM3784 probe is failed\n");
	return -ENODEV;
}

static int SGM3784_remove(struct i2c_client *client)
{
	struct SGM3784_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define SGM3784_NAME "leds-SGM3784"
static const struct i2c_device_id SGM3784_id[] = {
	{SGM3784_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id SGM3784_of_match[] = {
	{.compatible = "mediatek,STROBE_MAIN"},
	{},
};
#endif

static struct i2c_driver SGM3784_i2c_driver = {
	.driver = {
		   .name = SGM3784_NAME,
#ifdef CONFIG_OF
		   .of_match_table = SGM3784_of_match,
#endif
		   },
	.probe = SGM3784_probe,
	.remove = SGM3784_remove,
	.id_table = SGM3784_id,
};

static int __init SGM3784_init(void)
{
	//PK_DBG("SGM3784_init\n");	
	return i2c_add_driver(&SGM3784_i2c_driver);
}

static void __exit SGM3784_exit(void)
{
	i2c_del_driver(&SGM3784_i2c_driver);
}


module_init(SGM3784_init);
module_exit(SGM3784_exit);

MODULE_DESCRIPTION("Flash driver for SGM3784");
MODULE_AUTHOR("xx <xxx.xxx@tpv-tech.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;
	val = SGM3784_read_reg(SGM3784_i2c_client, reg);
	return (int)val;
}


static int FL_Enable(void)
{
	/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
		int buf[2];

		PK_DBG(" SGM3784 FL_Enable : H_LED_Duty=%d L_LED_Duty=%d, line=%d\n", (int)get_dutyH(), (int)m_DutyL, __LINE__);
		PK_DBG(" SGM3784 FL_Enable : H_LEDCloseFlag=%d L_LEDCloseFlag=%d, line=%d\n", (int)get_flashLedCloseFlagH(), (int)m_flashLedCloseFlagL, __LINE__);

		//	mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);
		//	mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
			
		//	buf[0] = 0x0F;
		//	buf[1] = 0x00;
		//	SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);  // 2 LEDs Closed		


		if( (get_flashLedCloseFlagH() == 1) && (m_flashLedCloseFlagL == 1) )
		{
			PK_DBG(" SGM3784 FL_Enable CK1, line=%d\n", __LINE__);
			buf[0] = 0x0F;
			buf[1] = 0x00;
			SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);  // 2 LEDs Closed		
			mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);
			mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
		}
		else if(get_flashLedCloseFlagH() == 1)
		{

			if(gIsTorch[m_DutyL] == 1)
			{
				PK_DBG(" SGM3784 FL_Enable L_LED Torch mode, line=%d\n", __LINE__);			
#if 0				
				buf[0] = 0x01;
				buf[1] = 0xF8;//0xFA;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				buf[0] = 0x02;
				buf[1] = 0xFF; //0xCF;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
#endif				
				buf[0] = 0x0F;
				buf[1] = 0x01; //0x02;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);
			}
			else
			{
			    PK_DBG(" SGM3784 FL_Enable L_LED Flash mode, line=%d\n", __LINE__);
#if 1	
				buf[0] = 0x0F;
				buf[1] = 0x00;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);	
				mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);				
				buf[0] = 0x01;
				buf[1] = 0xFB;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				buf[0] = 0x02;
				buf[1] = 0xCF;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
#endif				
				buf[0] = 0x0F;
				buf[1] = 0x01; //0x02;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ONE);
			}
			
		}
		else if(m_flashLedCloseFlagL == 1)
		{

			if(gIsTorch[get_dutyH()] == 1)
			{
				PK_DBG(" SGM3784 FL_Enable H_LED Torch mode, line=%d\n", __LINE__);
#if 0					
				buf[0] = 0x01;
				buf[1] = 0xF8;//0xFA;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				buf[0] = 0x02;
				buf[1] = 0xFF; //0xCF;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
#endif				
				buf[0] = 0x0F;
				buf[1] = 0x02; //0x01;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);
			}
			else
			{
				PK_DBG(" SGM3784 FL_Enable H_LED Flash mode, line=%d\n", __LINE__);
#if 1				
				buf[0] = 0x0F;
				buf[1] = 0x00;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);	
				mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);	
				buf[0] = 0x01;
				buf[1] = 0xFB;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				buf[0] = 0x02;
				buf[1] = 0xCF;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
#endif				
				buf[0] = 0x0F;
				buf[1] = 0x02; //0x01;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ONE);
				//mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
			}

		}
		else
		{
			if((gIsTorch[get_dutyH()] == 1) && (gIsTorch[m_DutyL] == 1))
			{
				PK_DBG(" SGM3784 FL_Enable H_LED & L_LED Torch mode, line=%d\n", __LINE__);
#if 0					
				buf[0] = 0x01;
				buf[1] = 0xF8;//0xAA;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				buf[0] = 0x02;
				buf[1] = 0xFF;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
#endif				
				buf[0] = 0x0F;
				buf[1] = 0x03;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ONE);		
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);
			}
			else
			{
				PK_DBG(" SGM3784 FL_Enable H_LED & L_LED Flash mode, line=%d\n", __LINE__);
#if 1
				buf[0] = 0x0F;
				buf[1] = 0x00;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);	
				mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);					
				buf[0] = 0x01;
				buf[1] = 0xFB;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				buf[0] = 0x02;
				buf[1] = 0xCF;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
#endif				
				buf[0] = 0x0F;
				buf[1] = 0x03;
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ONE);
				//mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
			}
		}
	/* TPV Mobile/WillHuang, 20151005, PMS# ,} */

	return 0;
}

static int FL_Disable(void)
{
#if 1
	/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
		int buf[2];

		PK_DBG(" SGM3784 FL_Disable line=%d\n", __LINE__);			

		mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);

		buf[0] = 0x0F;
		buf[1] = 0x00;
		SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);

		buf[0] = 0x01;
		buf[1] = 0xF8;
		SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
	
		buf[0] = 0x02;
		buf[1] = 0xFF; //0xCF;
		SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);		

	/* TPV Mobile/WillHuang, 20151005, PMS# ,} */
#endif
	return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{	
	/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
		int buf[2];
		if (duty >= e_DutyNum)
			duty = e_DutyNum - 1;
		if (duty < 0)
			duty = 0;
		m_DutyL = duty;


		PK_DBG(" SGM3784 FL_dim_duty H_LED=%d L_LED=%d, line=%d\n", (int)get_dutyH(), (int)m_DutyL, __LINE__);
		PK_DBG(" SGM3784 FL_dim_duty H_LEDCloseFlag=%d L_LEDCloseFlag=%d, line=%d\n", (int)get_flashLedCloseFlagH(), (int)m_flashLedCloseFlagL, __LINE__);

		if((get_flashLedCloseFlagH()==1) && (m_flashLedCloseFlagL==1))
		{
			PK_DBG(" SGM3784 FL_dim_duty CK1, line=%d\n", __LINE__);	
		}
		else if(get_flashLedCloseFlagH()==1)
		{
			if(gIsTorch[m_DutyL] == 1)
			{
				PK_DBG(" SGM3784 FL_dim_duty L_LED Torch, line=%d\n", __LINE__);			
				buf[0] = 0x08; //0x0B;
				buf[1] = gLedTorchDuty[m_DutyL];
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
			}
			else
			{
				PK_DBG(" SGM3784 FL_dim_duty L_LED Flash, line=%d\n", __LINE__);
				buf[0] = 0x06; //0x09;
				buf[1] = gLedFlashDuty[m_DutyL];
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
			}
		}
		else if(m_flashLedCloseFlagL == 1)
		{
			if(gIsTorch[get_dutyH()] == 1)
			{
				PK_DBG(" SGM3784 FL_dim_duty H_LED Torch, line=%d\n", __LINE__);			
				buf[0] = 0x0B; //0x08;
				buf[1] = gLedTorchDuty[get_dutyH()];
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
			}
			else
			{
			    PK_DBG(" SGM3784 FL_dim_duty H_LED Flash, line=%d\n", __LINE__);
				buf[0] = 0x09; //0x06;
				buf[1] = gLedFlashDuty[get_dutyH()];
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
			}

		}
		else
		{
			if((gIsTorch[get_dutyH()] == 1) && (gIsTorch[m_DutyL] == 1))
			{
				PK_DBG(" SGM3784 FL_dim_duty H_LED&L_LED Torch, line=%d\n", __LINE__);
				buf[0] = 0x0B; //0x08;
				buf[1] = gLedTorchDuty[get_dutyH()];
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);

				buf[0] = 0x08; //0x0B;
				buf[1] = gLedTorchDuty[m_DutyL];
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);				
			}
			else
			{
				PK_DBG(" SGM3784 FL_dim_duty H_LED&L_LED Flash, line=%d\n", __LINE__);			
				
				buf[0] = 0x09; //0x06;
				buf[1] = gLedFlashDuty[get_dutyH()];
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);

				buf[0] = 0x06; //0x09;
				buf[1] = gLedFlashDuty[m_DutyL];
				SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);
				
			}

		}

	/* TPV Mobile/WillHuang, 20151005, PMS# ,} */	
	return 0;
}

/*
static int g_lowPowerLevel=LOW_BATTERY_LEVEL_0;
static void lowPowerCB(LOW_BATTERY_LEVEL lev)
{
	g_lowPowerLevel=lev;
}*/

static int FL_Init(void)
{
	/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
		int buf[2];
		mt_set_gpio_out(GPIO_FLASH_EN, GPIO_OUT_ONE);		
		msleep(5);

		buf[0] = 0x0F;
		buf[1] = 0x00;
		SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);

		buf[0] = 0x01;
		buf[1] = 0xF8; //0xA8;
		SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);

		buf[0] = 0x02;
		buf[1] = 0xFF; //0xCF;
		SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);

		buf[0] = 0x03;
		buf[1] = 0x48;
		SGM3784_write_reg(SGM3784_i2c_client, buf[0], buf[1]);

	/* TPV Mobile/WillHuang, 20151005, PMS# ,} */

	return 0;
}

static int FL_Uninit(void)
{
	FL_Disable();

	/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
		mt_set_gpio_out(GPIO_FLASH_EN, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_FLASH_STROBE, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_FLASH_GPIO, GPIO_OUT_ZERO);
		//msleep(5);
	/* TPV Mobile/WillHuang, 20151005, PMS# ,} */
	
	return 0;
}

static int FL_hasLowPowerDetect(void)
{

	return 1;
}

static int detLowPowerStart(void)
{

/* g_lowPowerLevel=LOW_BATTERY_LEVEL_0; */
	return 0;
}


static int detLowPowerEnd(void)
{

	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static struct hrtimer g_timeOutTimer;

static int g_b1stInit = 1;

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static void timerInit(void)
{
	if (g_b1stInit == 1) {
		g_b1stInit = 0;

		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}



}


static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	/* kal_uint8 valTemp; */
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	//PK_DBG
	//    ("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	//     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("L_FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("L_FLASHLIGHT_DUTY: %d\n", (int)arg);
		/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
			m_DutyL = arg;
			///FL_dim_duty(arg);
		/* TPV Mobile/WillHuang, 20151005, PMS# ,} */		
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
		/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
			if ( (g_timeOutTimeMs != 0) && (g_Timer_is_enabled != 1)) {
				ktime_t ktime;
                g_Timer_is_enabled = 1;				
				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
		/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
			m_flashLedCloseFlagL = 0;
			FL_dim_duty(m_DutyL);
		/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */				
			FL_Enable();
		
		} else {
		
		/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */
			m_flashLedCloseFlagL = 1;

			if(get_flashLedCloseFlagH() == 0)
			{
			 if ((get_timeoutTimeMsH() != 0) && (g_Timer_is_enabled != 1)) {
				ktime_t ktime;
                                g_Timer_is_enabled = 1;
				ktime = ktime_set(0, get_timeoutTimeMsH() * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			 }
				FL_dim_duty(m_DutyL);
				FL_Enable();
			}
			else
			{
				FL_dim_duty(m_DutyL);
			        FL_Disable();
				if(g_Timer_is_enabled)
				{
				hrtimer_cancel(&g_timeOutTimer);
					g_Timer_is_enabled = 0;
				}
			}
		/* TPV Mobile/WillHuang, 20151005, PMS# ,{ */

		}
		break;
/*
	case FLASH_IOC_PRE_ON:
		PK_DBG("FLASH_IOC_PRE_ON\n");
			FL_preOn();
		break;
	case FLASH_IOC_GET_PRE_ON_TIME_MS:
		PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",(int)arg);
		temp=13;
		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
	    {
		PK_DBG(" ioctl copy to user failed\n");
		return -1;
	    }
		break;
*/
	case FLASH_IOC_SET_REG_ADR:
		PK_DBG("FLASH_IOC_SET_REG_ADR: %d\n", (int)arg);
		/* g_reg = arg; */
		break;
	case FLASH_IOC_SET_REG_VAL:
		PK_DBG("FLASH_IOC_SET_REG_VAL: %d\n", (int)arg);
		/* g_val = arg; */
		break;
	case FLASH_IOC_SET_REG:
		/* PK_DBG("FLASH_IOC_SET_REG: %d %d\n",g_reg, g_val); */

		break;

	case FLASH_IOC_GET_REG:
		PK_DBG("FLASH_IOC_GET_REG: %d\n", (int)arg);

		/* i4RetValue = valTemp; */
		/* PK_DBG("FLASH_IOC_GET_REG: v=%d\n",valTemp); */
		break;

	case FLASH_IOC_HAS_LOW_POWER_DETECT:
		PK_DBG("FLASH_IOC_HAS_LOW_POWER_DETECT");
		temp = FL_hasLowPowerDetect();
		if (copy_to_user((void __user *)arg, (void *)&temp, 4)) {
			PK_DBG(" ioctl copy to user failed\n");
			return -1;
		}
		break;
	case FLASH_IOC_LOW_POWER_DETECT_START:
		detLowPowerStart();
		break;
	case FLASH_IOC_LOW_POWER_DETECT_END:
		i4RetValue = detLowPowerEnd();
		break;

	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_ERR(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;


		spin_unlock_irq(&g_strobeSMPLock);
		FL_Uninit();
	}
	PK_DBG(" Done\n");
	return 0;
}


static FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};



MUINT32 strobeInit_main_sid2_part1(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}
