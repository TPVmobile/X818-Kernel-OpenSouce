/* drivers/hwmon/mt6516/amit/helix.c - helix ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <mach/upmu_sw.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>

#include <linux/wakelock.h>
#include <asm/io.h>
#include "helix.h"
#include <helix_cust_alsps.h>
#include <alsps.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>



#define POWER_NONE_MACRO MT65XX_POWER_NONE
#define GPIO_ALS_EINT_PIN 6
#define HELIX_USE_PS_INT
#define OPEN_PROX_ARITHMETIC                          //yzb menxian
struct proc_dir_entry *proc_helix_close_away;
static int gpio_irq = 0;

static int flag_init = 0;

//#define MTK_AUTO_DETECT_ALSPS


/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define helix_DEV_NAME     "helix"
/*----------------------------------------------------------------------------*/
#if 0
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args) 
#else
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN()               pr_debug(APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    pr_err(APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    pr_debug(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    pr_debug(APS_TAG fmt, ##args) 
#endif

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1


#define FAR_OFFSET            2000
#define NEAR_OFFSET           3000
#define DEF_CT                7500
#define DEF_BH                9500
#define NG                    12000
#define NG_TS                 64535


#define PS_SHOT_MODE     2

/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

/*----------------------------------------------------------------------------*/
static struct i2c_client *helix_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id helix_i2c_id[] = {{helix_DEV_NAME,0},{}};

/*----------------------------------------------------------------------------*/
static int helix_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int helix_i2c_remove(struct i2c_client *client);
static int helix_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int helix_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int helix_i2c_resume(struct i2c_client *client);

extern struct alsps_hw *helix_get_cust_alsps_hw(void);
extern int hwmsen_alsps_sensor_add(struct sensor_init_info* obj) ;
int helix_get_ps_rawdata(void);

static int helix_clear_intr(struct i2c_client *client);

#define C_I2C_FIFO_SIZE     8

static DEFINE_MUTEX(helix_mutex);

static struct helix_priv *g_helix_ptr = NULL;

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static int helix_local_init(void);
static int helix_local_uninit(void);
static int helix_init_flag =-1; // 0<==>OK -1 <==> fail
static struct alsps_init_info helix_init_info = {
		.name = "helix",
		.init = helix_local_init,
		.uninit = helix_local_uninit,	
};


static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct helix_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct helix_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
    struct workqueue_struct *alsps_workqueue;
    /*i2c address group*/
    struct helix_i2c_addr  addr;

    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/    
};

/*----------------------------------------------------------------------------*/

static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
static struct i2c_driver helix_i2c_driver = {	
	.probe      = helix_i2c_probe,
	.remove     = helix_i2c_remove,
	.detect     = helix_i2c_detect,
	.suspend    = helix_i2c_suspend,
	.resume     = helix_i2c_resume,
	.id_table   = helix_i2c_id,
	.driver = {
		.name   = helix_DEV_NAME,
        .of_match_table = alsps_of_match,
	},
};

static struct helix_priv *helix_obj = NULL;
#ifndef MTK_AUTO_DETECT_ALSPS
static struct platform_driver helix_alsps_driver;
#endif

/*------------------------i2c function for 89-------------------------------------*/
int helix_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	mutex_lock(&helix_mutex);
	switch(i2c_flag){	
	case I2C_FLAG_WRITE:
	client->addr &=I2C_MASK_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	
	case I2C_FLAG_READ:
	client->addr &=I2C_MASK_FLAG;
	client->addr |=I2C_WR_FLAG;
	client->addr |=I2C_RS_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	default:
	APS_LOG("helix_i2c_master_operate i2c_flag command not support!\n");
	break;
	}
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&helix_mutex);
	return res;
	EXIT_ERR:
	mutex_unlock(&helix_mutex);
	APS_ERR("helix_i2c_transfer fail\n");
	return res;
}


/*----------------------------------------------------------------------------*/
int helix_get_addr(struct alsps_hw *hw, struct helix_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}


/*----------------------------------------------------------------------------*/
static void helix_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");
    /*
	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "helix")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "helix")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	*/
	power_on = on;
}

#define PROX_DATA_SAFE_RANGE_MAX_VALUE  6000
#define PROX_DATA_SAFE_RANGE_MIN_VALUE   1000
#define PROX_DATA_PROX_THRES_MAX_VALUE   7000
#define PROX_DATA_PROX_THRES_MIN_VALUE    2000

static int read_file(char *filename)
{
    struct file *filp;
    char bufs[100];
    int ret;
	int data_val=0;
 
   // strcpy(bufs, "hello my world");
 
    /* kernel memory access setting */
    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);
 
    /* open a file */
    filp = filp_open(filename, O_RDWR, 0664);
    if (IS_ERR(filp)) {
        APS_ERR("open error\n");
        return 0;
    }
    else {
        APS_LOG("open success\n");
    }
 
 
    /* read example */
    APS_LOG(KERN_DEBUG);
    
    APS_LOG("filp->f_pos=%d \n: ", (int)filp->f_pos);
    memset(bufs,'\n',60);
    ret = vfs_read(filp, bufs, 60, &filp->f_pos);
    APS_LOG("char_cnt =%d \n: ", ret);
	
    if (ret != 0) {
        APS_ERR("filp->f_pos=%d\n", (int)filp->f_pos);
    }
    APS_LOG("\n");

 	ret= sscanf(bufs,"%d\n",&data_val);
	APS_LOG("data_val=%d\n", data_val);
    filp_close(filp, NULL);  /* filp_close(filp, current->files) ?  */
    /* restore kernel memory setting */

    set_fs(old_fs);
	return data_val;
}


int offset_start_flag = 0;
int cali_start_flag = 0;

static ssize_t helix_show_prox_data_safe_range_max(struct device_driver *ddri, char *buf)
{
	APS_LOG("helix_show_prox_data_safe_range_max\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", PROX_DATA_SAFE_RANGE_MAX_VALUE);     
	
}


static ssize_t helix_show_prox_data_safe_range_min(struct device_driver *ddri, char *buf)
{
	APS_LOG("helix_show_prox_data_safe_range_min\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", PROX_DATA_SAFE_RANGE_MIN_VALUE);     
	
}


static long helix_enable_ps_for_cali(struct i2c_client *client, int enable)
{
	u8 databuf[2]; 	
	long res = 0;

    APS_LOG("helix_enable_ps  = %x\n",enable);

	if(enable)
	{

		databuf[1] = 23;
		databuf[0] = helix_CMM_LED_Frequency;			
		APS_LOG("helix_CMM_LED_Frequency  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

		databuf[1] = 3;  //0x13
		databuf[0] = helix_CMM_PS_Samle_Delay;			
		APS_LOG("helix_CMM_PS_Samle_Delay  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}		
		
		databuf[1] = 0x14;    //80mA
		databuf[0] = helix_CMM_CONTROL;			// LED Current 100mA
		APS_LOG("helix_CMM_CONTROL  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

		databuf[1] = 0xFF;
		databuf[0] = helix_CMM_INT_HIGH_THD_HIGH;
		APS_LOG("helix_CMM_INT_HIGH_THD_HIGH = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = 0xFF;
		databuf[0] = helix_CMM_INT_HIGH_THD_LOW;
		APS_LOG("helix_CMM_INT_HIGH_THD_LOW  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = 0;
		databuf[0] = helix_CMM_INT_LOW_THD_HIGH;
		APS_LOG("helix_CMM_INT_LOW_THD_HIGH  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = 0;
		databuf[0] = helix_CMM_INT_LOW_THD_LOW;
		APS_LOG("helix_CMM_INT_LOW_THD_LOW  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = 0x11;
		databuf[0] = helix_CMM_Persistence;
		APS_LOG("helix_CMM_Persistence  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = 0x0F;//0x3B  4.8ms int time
		databuf[0] = helix_CMM_PS_CONFIG;
		APS_LOG("helix_CMM_PS_CONFIG  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}			
		

		databuf[1] = 0x09;   // 10ms          //0701 0x01 => 0x08 => 0x09    10 + N*10 ms
		databuf[0] = helix_CMM_PS_Interval;
		APS_LOG("helix_CMM_PS_Interval  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				

		databuf[1] = 0x02;		// 0x02 : repeat mode , 0x01 : oneshot mode
		databuf[0] = helix_CMM_PS_Control;
		APS_LOG("helix_CMM_PS_Control  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
			
	}
	else
              {
                          databuf[1] = 0;   
		databuf[0] = helix_CMM_CONTROL;			
		APS_LOG("helix_CMM_CONTROL  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}                
	
		databuf[1] = 0x00;
		databuf[0] = helix_CMM_PS_Control;
		APS_LOG("helix_CMM_PS_Control  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
	{
			goto EXIT_ERR;
		}                
		
 	}

		return 0;
	
EXIT_ERR:
	APS_ERR("helix_enable_ps fail\n");
	return res;
	}


static int helix_simple_read_ps(struct i2c_client *client)
{
                u8 databuf[2];    
                u8 psdatabuf[2]; 
                int  ps_data = 0;

                helix_enable_ps_for_cali(client,1);   
                
                databuf[0] = helix_CMM_PS_Control;  databuf[1] = 0x01;	
                helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
                mdelay(5);
                
                databuf[0]=helix_CMM_PDATA_H;
                helix_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
                
                psdatabuf[0]=helix_CMM_PDATA_L;
                helix_i2c_master_operate(client, psdatabuf, 0x101, I2C_FLAG_READ);
                
                ps_data = psdatabuf[1] | (databuf[0]<<8);	
                
                databuf[0] = helix_CMM_STATUS;  
                helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);

                helix_enable_ps_for_cali(client, 0);       
                
                return ps_data;
}


 
static ssize_t helix_show_data_val(struct device_driver *ddri, char *buf)
{
    int val;
	if(!helix_obj)
	{
        APS_ERR("helix_obj is null!!\n");
        return 0;
	}             

    val = helix_simple_read_ps (helix_i2c_client);

    return scnprintf(buf, PAGE_SIZE, "0x%04X\n", val);
}

int cal_flag_val = 0;
int cal1_flag_val = 0;
int cal2_flag_val = 0;
int cal_finish_flag = 0;
static ssize_t helix_show_prox_offset_cal(struct device_driver *ddri, char *buf)
{
	APS_LOG("helix_show_prox_offset_cal\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}

	return  0;//snprintf(buf, PAGE_SIZE, "0x%04X\n", cal1_flag_val); 
}


static ssize_t helix_store_prox_offset_cal(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	APS_LOG("helix_store_prox_offset_cal\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	APS_LOG("helix_store_prox_offset_cal cal1_flag_val=%d\n",cal1_flag_val);
	cal1_flag_val = value;

	offset_start_flag = 0;
	
	return count;    
}

static ssize_t helix_show_prox_thres_max(struct device_driver *ddri, char *buf)
{

	APS_LOG("helix_show_prox_thres_max\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", PROX_DATA_PROX_THRES_MAX_VALUE);     
	
}

static ssize_t helix_show_prox_thres_min(struct device_driver *ddri, char *buf)
{
	APS_LOG("helix_show_prox_thres_min\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", PROX_DATA_PROX_THRES_MIN_VALUE);     
	
}

static ssize_t helix_store_prox_thres_max(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	APS_LOG("helix_store_prox_thres_max\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	APS_LOG("helix_store_prox_thres_max cal_finish_flag=%d\n",cal_finish_flag);
	cal_finish_flag = value;

	return count;    
}

static ssize_t helix_store_prox_thres_min(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	APS_LOG("helix_store_prox_thres_min\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))

	cal_flag_val = value;

	return count;    
}

static ssize_t helix_show_prox_data_max(struct device_driver *ddri, char *buf)
{
	APS_LOG("helix_show_prox_data_max\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	

	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", 4000);     
	
}

static ssize_t helix_show_prox_thres(struct device_driver *ddri, char *buf)
{
	APS_LOG("helix_show_prox_thres\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	

	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", helix_obj->ps);     
	
}

static ssize_t helix_show_prox_calibrate_start(struct device_driver *ddri, char *buf)
{
	APS_LOG("helix_show_prox_thres\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	
   
	return  0;//return scnprintf(buf, PAGE_SIZE, "0x%04X\n", calibrate_start_flag);     
	
}

static ssize_t helix_store_prox_calibrate_start(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	APS_LOG("helix_store_prox_calibrate_start\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	        APS_LOG("helix_store_prox_calibrate_start calibrate_start_flag=%d\n",value);
	cali_start_flag = value;

	return count;    
}

static ssize_t helix_store_prox_thres(struct device_driver *ddri, const char *buf, size_t count)
{
    int value;
    int crosstalk;
    int threshold;
    int cali_finish_flag;
    APS_LOG("helix_store_prox_thres\n");
    
    if(!helix_obj)
    {
        APS_ERR("helix_obj is null!!\n");
        return 0;
    }
    
	cali_finish_flag = 0;
    if(1 == sscanf(buf, "%d ", &value))
    {
        cali_finish_flag = value;
    }
	
    cali_start_flag = 0;

    APS_LOG("helix_store_prox_calibrate_start cali_finish_flag=%d\n",cali_finish_flag);
    //restore cal values. +++
    crosstalk = read_file("/persist/proxdata/crosstalk");
    threshold = read_file("/persist/proxdata/threshold");
    cali_finish_flag = read_file("/persist/proxdata/prox_thres");
    APS_LOG("helix_enable_als crosstalk=%d, threshold=%d , cali_finish_flag=%d\n",crosstalk,threshold,cali_finish_flag);
    if(cali_finish_flag ==1)
    {
        ps_cali.close = threshold;
        ps_cali.far_away= crosstalk +200;
        ps_cali.valid = 1;

        helix_obj->hw->ps_threshold_high = ps_cali.close;
        helix_obj->hw->ps_threshold_low = ps_cali.far_away;
        atomic_set(&helix_obj->ps_thd_val_high,  helix_obj->hw->ps_threshold_high);
        atomic_set(&helix_obj->ps_thd_val_low,  helix_obj->hw->ps_threshold_low);
        //restore cal values. ---	
    }
	return count;    
}

static ssize_t helix_show_prox_offset_start(struct device_driver *ddri, char *buf)
{
    APS_LOG("helix_show_prox_thres\n");
    if(!helix_obj)
    {
        APS_ERR("helix_obj is null!!\n");
        return 0;
    }

    return  0;//return scnprintf(buf, PAGE_SIZE, "0x%04X\n", offset_start_flag);     
}
static ssize_t helix_store_prox_offset_start(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	APS_LOG("helix_store_prox_calibrate_start\n");
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	APS_LOG("helix_store_prox_offset_start offset_start_flag=%d\n",offset_start_flag);
	offset_start_flag = value;

	return count;    
}

int eint_start_flag = 0;

static ssize_t helix_show_prox_eint_start(struct device_driver *ddri, char *buf)
{
  printk("iii = %s \n",__FUNCTION__);        

    if(!helix_obj)
    {
        APS_ERR("helix_obj is null!!\n");
        return 0;
    }

    return  eint_start_flag;//return scnprintf(buf, PAGE_SIZE, "0x%04X\n", offset_start_flag);     
}

static ssize_t helix_store_prox_eint_start(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
  printk("iii = %s \n",__FUNCTION__);        

	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	printk("helix_store_prox_offset_start offset_start_flag=%d\n",eint_start_flag);
	eint_start_flag = value;

	if(eint_start_flag==0)
		disable_irq_nosync(gpio_irq);
	else
		enable_irq(gpio_irq);
	
	return count;    
}

static DRIVER_ATTR(prox_data_safe_range_max ,     0664, helix_show_prox_data_safe_range_max , NULL);
static DRIVER_ATTR(prox_data_safe_range_min ,      0664, helix_show_prox_data_safe_range_min, NULL);
static DRIVER_ATTR(prox_data_val ,  0664, helix_show_data_val ,	NULL);
static DRIVER_ATTR(prox_offset_cal ,   0664, helix_show_prox_offset_cal, helix_store_prox_offset_cal);
static DRIVER_ATTR(prox_thres_max ,  0664, helix_show_prox_thres_max, helix_store_prox_thres_max);
static DRIVER_ATTR(prox_thres_min ,   0664, helix_show_prox_thres_min,		helix_store_prox_thres_min);
static DRIVER_ATTR(prox_data_max ,  0664, helix_show_prox_data_max, NULL);
static DRIVER_ATTR(prox_thres ,    0664, helix_show_prox_thres, helix_store_prox_thres);
static DRIVER_ATTR(prox_calibrate_start  ,    0664, helix_show_prox_calibrate_start, helix_store_prox_calibrate_start);
static DRIVER_ATTR(prox_offset_start ,     0664, helix_show_prox_offset_start, helix_store_prox_offset_start);
static DRIVER_ATTR(prox_eint_start ,     0664, helix_show_prox_eint_start, helix_store_prox_eint_start);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *helix_attr_list[] = {
    &driver_attr_prox_data_safe_range_max,
    &driver_attr_prox_data_safe_range_min,    
    &driver_attr_prox_data_val,       
    &driver_attr_prox_offset_cal,
    &driver_attr_prox_thres_max,
    &driver_attr_prox_thres_min,
    &driver_attr_prox_data_max,
    &driver_attr_prox_thres,
    &driver_attr_prox_calibrate_start,
    &driver_attr_prox_offset_start,
    &driver_attr_prox_eint_start,
};

/*----------------------------------------------------------------------------*/
static int helix_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(helix_attr_list)/sizeof(helix_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, helix_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", helix_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int helix_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(helix_attr_list)/sizeof(helix_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, helix_attr_list[idx]);
	}
	
	return err;
}



/*----------------------------------------------------------------------------*/
static long helix_enable_als(struct i2c_client *client, int enable)
{
	struct helix_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];	  
	long res = 0;

    APS_LOG("helix_enable_als  = %x\n",enable);
	
	if(enable)
	{

		
		databuf[1] = 0xC5;         //0xCC; 	// 0xC5    // integration 50ms 100 count per lux
		databuf[0] = helix_CMM_ALS_CONFIG;
		APS_LOG("helix_CMM_ALS_CONFIG  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}		


		databuf[1] =0x42;    // 50ms  interval  50ms *2   //0701
		databuf[0] = helix_CMM_ALS_Interval;
		APS_LOG("helix_CMM_ALS_Interval  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}	


		databuf[1] = 0x02;		// 0x02 : repeat mode , 0x01 : oneshot mode
		databuf[0] = helix_CMM_ALS_Control;
		APS_LOG("helix_CMM_ALS_Control  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}		
	
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));

	}
	else 
              { 
                            databuf[1] = 1;    // 50ms  interval  50ms *2   //0701
		databuf[0] = helix_CMM_ALS_Interval;
		APS_LOG("helix_CMM_ALS_Interval  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

		databuf[1] = 0x00;                      //0701
		databuf[0] = helix_CMM_ALS_CONFIG;
		APS_LOG("helix_CMM_ALS_CONFIG  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}	

		databuf[1] = 0x00;
		databuf[0] = helix_CMM_ALS_Control;
		APS_LOG("helix_CMM_ALS_Control  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

		atomic_set(&obj->als_deb_on, 0);

	}

															
	return 0;
		
EXIT_ERR:
	APS_ERR("helix_enable_als fail\n");
	return res;
}


/*----------------------------------------------------------------------------*/
static long helix_enable_ps(struct i2c_client *client, int enable)
{
	struct helix_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];	
	long res = 0;
	int crosstalk=0;
	int threshold = 0;
	int cali_finish_flag = 0;
	
        APS_LOG("helix_enable_ps  = %x\n",enable);


	if(enable)
	{
		helix_clear_intr(obj->client);
		if(flag_init == 1){
			enable_irq(gpio_irq);
			
		}else{
			flag_init = 1;
		}
			    //restore cal values. +++
		crosstalk = read_file("/persist/proxdata/crosstalk");
		threshold = read_file("/persist/proxdata/threshold");
		cali_finish_flag = read_file("/persist/proxdata/prox_thres");
		APS_LOG("helix_enable_als crosstalk=%d, threshold=%d , cali_finish_flag=%d\n",crosstalk,threshold,cali_finish_flag);
		if(cali_finish_flag == 1)
		{
			ps_cali.close = threshold;
			ps_cali.far_away= crosstalk +200;
			ps_cali.valid = 1;
		
			helix_obj->hw->ps_threshold_high = ps_cali.close;
			helix_obj->hw->ps_threshold_low = ps_cali.far_away;
			atomic_set(&helix_obj->ps_thd_val_high,  helix_obj->hw->ps_threshold_high);
			atomic_set(&helix_obj->ps_thd_val_low,  helix_obj->hw->ps_threshold_low);
			//restore cal values. ---	
		}

		
		databuf[1] = 23;
		databuf[0] = helix_CMM_LED_Frequency;			
		APS_LOG("helix_CMM_LED_Frequency  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	
		databuf[1] = 0x03;  //0x13
		databuf[0] = helix_CMM_PS_Samle_Delay;			
		APS_LOG("helix_CMM_PS_Samle_Delay  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}		
		
        databuf[1] = 0x02;
	    databuf[0] = helix_CMM_INT_Config;
	    APS_LOG("helix_CMM_INT_Config  = %x\n",databuf[1]);
	    res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	    if(res <= 0)
	    {
		    goto EXIT_ERR;
	    }

		databuf[1] = 0x14;    //50mA
		databuf[0] = helix_CMM_CONTROL;			// LED Current 100mA
		APS_LOG("helix_CMM_CONTROL  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("yzb------:%d-------%d-----\n",atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
		databuf[1] = ((atomic_read(&obj->ps_thd_val_high) & 0xFF00)>>8);
		databuf[0] = helix_CMM_INT_HIGH_THD_HIGH;
		APS_LOG("helix_CMM_INT_HIGH_THD_HIGH = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = (atomic_read(&obj->ps_thd_val_high) & 0x00FF);
		databuf[0] = helix_CMM_INT_HIGH_THD_LOW;
		APS_LOG("helix_CMM_INT_HIGH_THD_LOW  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = ((atomic_read(&obj->ps_thd_val_low) & 0xFF00)>>8);
		databuf[0] = helix_CMM_INT_LOW_THD_HIGH;
		APS_LOG("helix_CMM_INT_LOW_THD_HIGH  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = (atomic_read(&obj->ps_thd_val_low) & 0x00FF);
		databuf[0] = helix_CMM_INT_LOW_THD_LOW;
		APS_LOG("helix_CMM_INT_LOW_THD_LOW  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = 0x11;
		databuf[0] = helix_CMM_Persistence;
		APS_LOG("helix_CMM_Persistence  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				
		
		databuf[1] = 0x0F;//0x3B  
		databuf[0] = helix_CMM_PS_CONFIG;
		APS_LOG("helix_CMM_PS_CONFIG  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}			
		
			databuf[1] = 0x09;   // 190ms          //0701 0x01 => 0x08 => 0x09    10 + N*10 ms
			databuf[0] = helix_CMM_PS_Interval;
			APS_LOG("helix_CMM_PS_Interval  = %x\n",databuf[1]);
			res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}	

						
		databuf[1] = 2;		// 0x02 : repeat mode , 0x01 : oneshot mode
		databuf[0] = helix_CMM_PS_Control;
		APS_LOG("helix_CMM_PS_Control  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}				

		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
			
	}
	else
              {
                            databuf[1] = 0;   
		databuf[0] = helix_CMM_CONTROL;			
		APS_LOG("helix_CMM_CONTROL  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}                
	
		databuf[1] = 0x00;
		databuf[0] = helix_CMM_PS_Control;
		APS_LOG("helix_CMM_PS_Control  = %x\n",databuf[1]);
		res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
                
		atomic_set(&obj->ps_deb_on, 0);
		
		//Pingping.liu masked 2016.04.25 for ps failure problem
		//if(0 == obj->hw->polling_mode_ps)
		//{
		        //cancel_work_sync(&obj->eint_work);
		//}

		cali_start_flag = 0;
		offset_start_flag = 0;
 	}

	return 0;
	
EXIT_ERR:
	APS_ERR("helix_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
#if 0
static int helix_check_and_clear_intr(struct i2c_client *client) 
{
	int res;
	u8 buffer[2];
	
	buffer[0] = helix_CMM_STATUS;
	res = helix_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	
    if(res < 0)
    {
    	goto EXIT_ERR;
    }
	
	return res;

EXIT_ERR:
	APS_ERR("helix_check_and_clear_intr fail\n");
	return 1;
}
#endif
/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int helix_check_intr(struct i2c_client *client) 
{
	int res,intp,intl;
	u8 buffer[2];

    if (gpio_get_value(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
        return 0;

	buffer[0] = helix_CMM_STATUS;
	res = helix_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = 0;
	intp = 0;
	intl = 0;
	if((buffer[0] & 0x01) || (buffer[0] & 0x02))	// 0x02 : ps high threshold
	{
		res = 0;
		intp = 1;
	}
    APS_LOG("helix_check_intr 0x%X\n", buffer[0]);

	return res;

EXIT_ERR:
	APS_ERR("helix_check_intr fail\n");
	return 1;
}

static int helix_clear_intr(struct i2c_client *client) 
{

	int res;
	u8 buffer[2];
    buffer[1] = PS_SHOT_MODE;		// 0x02 : repeat mode , 0x01 : oneshot mode
	buffer[0] = helix_CMM_PS_Control;
	APS_LOG("helix_CMM_PS_Control  = %x\n",buffer[1]);
	res = helix_i2c_master_operate(client, buffer, 0x2, I2C_FLAG_WRITE);

	return res;
}


/*-----------------------------------------------------------------------------*/
static irqreturn_t helix_eint_func(void)
{
	struct helix_priv *obj = g_helix_ptr;
	if(!obj)
	{
		return IRQ_NONE;
	}

    disable_irq_nosync(gpio_irq);
	if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
        queue_work(obj->alsps_workqueue, &obj->eint_work);

    return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int helix_setup_eint(struct i2c_client *client)
{
	struct helix_priv *obj = i2c_get_clientdata(client);        
    struct device_node *irq_node;
    struct platform_device *alspsPltFmDev;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_cfg;
    int ret;

    g_helix_ptr = obj;
    if(gpio_irq == 0){
        alspsPltFmDev = get_alsps_platformdev();
        pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
    	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    	if (IS_ERR(pins_cfg)) {
    		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
    	}
    	pinctrl_select_state(pinctrl, pins_cfg);
        
        irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
        if (irq_node) {
    		gpio_irq = irq_of_parse_and_map(irq_node, 0);
    		APS_LOG("alsps helix irq = %d\n", gpio_irq);
    		if (!gpio_irq) {
    			APS_ERR("irq_of_parse_and_map fail!!\n");
    			return -EINVAL;
    		}

            ret = request_irq(gpio_irq, (irq_handler_t)helix_eint_func, IRQF_TRIGGER_NONE, "als_eint", NULL);
    		if (ret) {
    			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
    			return -EINVAL;
    		}

        } else {
    		APS_ERR("null irq node!!\n");
    		return -EINVAL;
    	}
    } else {
        enable_irq(gpio_irq);
    }

    return 0;
}

/*----------------------------------------------------------------------------*/

static int helix_init_client(struct i2c_client *client)
{
	u8 databuf[2];    
	int res = 0;

//	khpark
	databuf[1] = 0x01;
	databuf[0] = helix_CMM_Reset;
	APS_LOG("helix_CMM_Reset  = %x\n",databuf[1]);
	res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	mdelay(10);

///*	khpark
	databuf[1] = 0x02;
	databuf[0] = helix_CMM_INT_Config;
	APS_LOG("helix_CMM_INT_Config  = %x\n",databuf[1]);
	res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
#ifdef HELIX_USE_PS_INT
    if((res = helix_setup_eint(client))!=0)
    {
        APS_ERR("setup eint: %d\n", res);
        return res;
    }
#endif
	
	return helix_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int helix_read_als(struct i2c_client *client, u16 *data)
{	 
	u16 c0_value, c1_value;	 
	u8 c0_buffer[2];
	u8 c0_buffer_tmp[2];
	u8 c1_buffer[2];
	u8 c1_buffer_tmp[2];
	u8 databuf[2];
	int lux = 0;
	
	int res = 0;

              
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
    //*	khpark	
	c0_buffer[0]=helix_CMM_C0DATA_H;
	res = helix_i2c_master_operate(client, c0_buffer, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	c0_buffer_tmp[0]=helix_CMM_C0DATA_L;
	res = helix_i2c_master_operate(client, c0_buffer_tmp, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	c0_value = c0_buffer_tmp[0] | (c0_buffer[0]<<8);

	c1_buffer[0]=helix_CMM_C1DATA_H;
	res = helix_i2c_master_operate(client, c1_buffer, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	c1_buffer_tmp[0]=helix_CMM_C1DATA_L;
	res = helix_i2c_master_operate(client, c1_buffer_tmp, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}


	c1_value = c1_buffer_tmp[0] | (c1_buffer[0]<<8);		

    if(c0_value >= c1_value)
    {
        if(c1_value >= 0 && c1_value < 3000)
        {
            lux = c1_value * 1078 / 1000;
        }
        else if(c1_value >= 3000 && c1_value < 9000)
        {
            lux = c1_value * 1092 / 1000;
            lux = lux - 42;
        }
        else
        {
            lux = c1_value * 1065 / 1000;
            lux = lux + 201;
        }

    }
    else
    {
        if(c0_value >= 0 && c0_value < 1500)
        {
            lux = c0_value * 60 / 100;
        }
        else if(c0_value >= 1500 && c0_value < 15500)
        {
            lux = c0_value * 61 / 100;
            lux = lux - 15;
        }
        else
        {
            lux = c0_value * 59 / 100;
            lux = lux + 295;
        }
    }
	/*Eric , 20160325, S2,{*/
	/*enhance ALS accuracy*/
	lux=lux * 55/100;
	/*Eric , 20160325, S2,}*/
    if(lux > MAX_LUX)
    {
        lux = MAX_LUX;
    }

    if(lux < 0)
    {
        lux = 0;
    }

	*data = lux;
	
	APS_LOG("ALS - c0 %05d, c1 %05d, lux = %06d\n", c0_value, c1_value, *data);

    databuf[1] = 0x01;		// 0x02 : repeat mode , 0x01 : oneshot mode
	databuf[0] = helix_CMM_ALS_Control;
	APS_LOG("helix_CMM_ALS_Control  = %x\n",databuf[1]);
	res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	
	return 0;	 

EXIT_ERR:

    databuf[1] = 0x01;		// 0x02 : repeat mode , 0x01 : oneshot mode
	databuf[0] = helix_CMM_ALS_Control;
	APS_LOG("helix_CMM_ALS_Control  = %x\n",databuf[1]);
	res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	
	APS_ERR("helix_read_als fail\n");
	return res;
}

int helix_read_als_ch0(struct i2c_client *client, u16 *data)
{	 
	//struct helix_priv *obj = i2c_get_clientdata(client);
	u16 c0_value;	 
	u8 databuf[2];
	u8 databuf_tmp[2];

	int res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

//get adc channel 0 value
///*	khpark
	databuf[0]=helix_CMM_C0DATA_H;
	res = helix_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	databuf_tmp[0]=helix_CMM_C0DATA_L;
	res = helix_i2c_master_operate(client, databuf_tmp, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	c0_value = databuf_tmp[0] | (databuf[0]<<8);
	*data = c0_value;

    databuf[1] = 0x01;		// 0x02 : repeat mode , 0x01 : oneshot mode
	databuf[0] = helix_CMM_ALS_Control;
	APS_LOG("helix_CMM_ALS_Control  = %x\n",databuf[1]);
	res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
        
	return 0;	 
	
EXIT_ERR:

    databuf[1] = 0x01;		// 0x02 : repeat mode , 0x01 : oneshot mode
	databuf[0] = helix_CMM_ALS_Control;
	APS_LOG("helix_CMM_ALS_Control  = %x\n",databuf[1]);
	res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	
	APS_ERR("helix_read_als_ch0 fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int helix_get_als_value(struct helix_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("helix_get_als_value exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_ERR("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
long helix_read_ps(struct i2c_client *client, u16 *data)
{
	u8 databuf[2];
	u8 databuf_tmp[2];
	long res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}


	databuf[0]=helix_CMM_PDATA_H;
	res = helix_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	databuf_tmp[0]=helix_CMM_PDATA_L;
	res = helix_i2c_master_operate(client, databuf_tmp, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	*data = databuf_tmp[0] | (databuf[0]<<8);

    APS_LOG("helix_read_ps  ps = %d\n", *data);
	return 0;  
    
    databuf[1] = 0x01;		// 0x02 : repeat mode , 0x01 : oneshot mode
	databuf[0] = helix_CMM_PS_Control;
	res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);  

EXIT_ERR:
	APS_ERR("helix_read_ps fail\n");

              databuf[1] = 2;		// 0x02 : repeat mode , 0x01 : oneshot mode
	databuf[0] = helix_CMM_PS_Control;
	APS_LOG("helix_CMM_PS_Control  = %x\n",databuf[1]);
	res = helix_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	        
	return res;
}

/*----------------------------------------------------------------------------*/
static int helix_get_ps_value(struct helix_priv *obj, u16 ps)
{
	int val;// mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=1;

	if(ps_cali.valid == 1)
	{
		if((ps >ps_cali.close))
		{
			val = 0;  /*close*/
			val_temp = 0;
			intr_flag_value = 1;
		}
		
		else if((ps < ps_cali.far_away))
		{
			val = 1;  /*far away*/
			val_temp = 1;
			intr_flag_value = 0;
		}
		else
			val = val_temp;

		APS_LOG("helix_get_ps_value val  = %d\n",val);
		APS_LOG("helix_get_ps_value intr_flag_value  = %d\n",intr_flag_value);
		APS_LOG("helix_read_data_for_cali close  = %d,far_away = %d,valid = %d",ps_cali.close,ps_cali.far_away,ps_cali.valid);
	}
	else
	{
		if((ps  > atomic_read(&obj->ps_thd_val_high)))
		{
			val = 0;  /*close*/
			val_temp = 0;
			intr_flag_value = 1;
		}
		else if((ps  < atomic_read(&obj->ps_thd_val_low)))
		{
			val = 1;  /*far away*/
			val_temp = 1;
			intr_flag_value = 0;
		}
		else
		       val = val_temp;	
		       APS_LOG("helix_get_ps_value intr_flag_value  = %d",intr_flag_value);
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		//return 1;  /*far away*/
	}

	if(!invalid)
	{
//		APS_DBG("PS:  %05d  (d) \n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
#define DEBUG_helix	    1

static void helix_eint_work(struct work_struct *work)
{
	struct helix_priv *obj = (struct helix_priv *)container_of(work, struct helix_priv, eint_work);
	int err;
	int	values[2];
	u8 databuf[3];
	int res = 0;

    APS_FUN();

	if((err = helix_check_intr(obj->client)))
	{
		APS_ERR("helix_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		helix_read_ps(obj->client, &obj->ps);
		helix_read_als_ch0(obj->client, &obj->als);
		values[0] = helix_get_ps_value(obj, obj->ps);
		values[1] = obj->ps;
		APS_LOG("helix_eint_work rawdata ps=%d als_ch0=%d,values=%d!\n",obj->ps, obj->als, values[0]);
		
		if(obj->als > 40000)
		{
			APS_LOG("helix_eint_work ALS too large may under lighting als_ch0=%d!\n",obj->als);
			//return;
		}

		if(intr_flag_value){

            databuf[0] = helix_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			res = helix_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			
			databuf[0] = helix_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			res = helix_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = helix_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(0x00FF);
			res = helix_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			
			databuf[0] = helix_CMM_INT_HIGH_THD_HIGH; 
			databuf[1] = (u8)((0xFF00) >> 8);
			res = helix_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
				
		}
		else
        {       
			
			databuf[0] = helix_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(0 & 0x00FF);
			res = helix_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			
			databuf[0] = helix_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((0 & 0xFF00) >> 8);
			res = helix_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			
			databuf[0] = helix_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			res = helix_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		
			databuf[0] = helix_CMM_INT_HIGH_THD_HIGH; 
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);
			res = helix_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}

#ifdef DEBUG_helix
		databuf[0] = helix_CMM_INT_LOW_THD_LOW;
		res = helix_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;

		}
		APS_LOG("helix_eint_work helix_CMM_INT_LOW_THD_LOW after databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);

		databuf[0] = helix_CMM_INT_HIGH_THD_LOW;
		res = helix_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("helix_eint_work helix_CMM_INT_HIGH_THD_LOW after databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);
#endif

		if(ps_report_interrupt_data(values[0]))
		{
		  APS_ERR("call ps_report_interrupt_data fail\n");
		}
	}
	
	helix_clear_intr(obj->client);
	enable_irq(gpio_irq);
	return;
EXIT_ERR:
	helix_clear_intr(obj->client);
	//mt_eint_unmask(CUST_EINT_ALS_NUM); 
	enable_irq(gpio_irq);
	APS_ERR("i2c_transfer error = %d\n", res);
	return;
}


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int helix_open(struct inode *inode, struct file *file)
{
	file->private_data = helix_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int helix_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long helix_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct helix_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_result;

    APS_LOG("helix_unlocked_ioctl = %d \n",  cmd);
              
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
                                          APS_LOG("ALSPS_SET_PS_MODE = %d \n",  enable);
			if(enable)
			{
				if((err = helix_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
				//mt_eint_unmask(CUST_EINT_ALS_NUM);
				enable_irq(gpio_irq);
			}
			else
			{
				if((err = helix_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
				//mt_eint_mask(CUST_EINT_ALS_NUM);
				disable_irq_nosync(gpio_irq);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
                                          APS_LOG("ALSPS_GET_PS_MODE = %d \n",  enable);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
                        
                                          #ifndef HELIX_USE_PS_INT
			if((err = helix_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			#endif
                        
			dat = helix_get_ps_value(obj, obj->ps);
                                          
                                          
                                          APS_LOG("ALSPS_GET_PS_DATA = %d \n",  dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
                        
                                          #ifndef HELIX_USE_PS_INT
			if((err = helix_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			#endif
                        
			dat = obj->ps;
                                          APS_LOG("ALSPS_GET_PS_RAW_DATA = %d \n",  dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
                                          APS_LOG("ALSPS_SET_ALS_MODE = %d \n",  enable);
			if(enable)
			{
				if((err = helix_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = helix_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
                                          APS_LOG("ALSPS_GET_ALS_MODE = %d \n",  enable);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = helix_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = helix_get_als_value(obj, obj->als);
                                          APS_LOG("ALSPS_GET_ALS_DATA = %d \n",  dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = helix_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
                                          APS_LOG("ALSPS_GET_ALS_RAW_DATA = %d \n",  dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			if((err = helix_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			if(obj->ps > atomic_read(&obj->ps_thd_val_high))
				{
					ps_result = 0;
				}
			else	ps_result = 1;
                        
			APS_LOG("ALSPS_GET_PS_TEST_RESULT = %d \n",  ps_result);	
			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}			   
			break;
			/*------------------------------------------------------------------------------------------*/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations helix_fops = {
	.owner = THIS_MODULE,
	.open = helix_open,
	.release = helix_release,
	.unlocked_ioctl = helix_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice helix_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &helix_fops,
};
/*----------------------------------------------------------------------------*/
static int helix_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	
	APS_FUN();    
#if 0                //0701
              struct helix_priv *obj = i2c_get_clientdata(client);    
	int err;
        
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if(err = helix_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = helix_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		helix_power(obj->hw, 0);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int helix_i2c_resume(struct i2c_client *client)
{
	
	APS_FUN();
        
#if 0      //0701
              struct helix_priv *obj = i2c_get_clientdata(client);        
	int err;
        
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	helix_power(obj->hw, 1);
	if(err = helix_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = helix_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = helix_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/

#if 0
int helix_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct helix_priv *obj = (struct helix_priv *)self;

    APS_LOG("helix_als_operate: %d\n", command);  

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if((err = helix_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = helix_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing known issue*/
				helix_read_als(obj->client, &obj->als);
				#if defined(MTK_AAL_SUPPORT)
				sensor_data->values[0] = obj->als;
				#else
				if(obj->als == 0)
				{
					sensor_data->values[0] = temp_als;				
				}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
					helix_read_als(obj->client, &obj->als);
					b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = helix_get_als_value(obj, obj->als);
					temp_als = sensor_data->values[0];
				}
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
        
}
#endif

static int als_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("helix_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_LIGHT;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return -1;
	}
	res=	helix_enable_als(helix_obj->client, en);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_get_data(int* value, int* status)
{
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#else
    struct helix_priv *obj = NULL;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_LIGHT;
    req.get_data_req.action = SENSOR_HUB_GET_DATA;
    len = sizeof(req.get_data_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
        APS_ERR("SCP_sensorHub_req_send fail!\n");
    }
    else
    {
        *value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    if(atomic_read(&helix_obj->trace) & CMC_TRC_PS_DATA)
	{
        APS_LOG("value = %d\n", *value);
        //show data
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return -1;
	}
	obj = helix_obj;
	if((err = helix_read_als(obj->client, &obj->als)))
	{
		err = -1;
	}
	else
	{
		*value = helix_get_als_value(obj, obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return err;
}

static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int ps_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("helix_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_PROXIMITY;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return -1;
	}
	res=	helix_enable_ps(helix_obj->client, en);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_PROXIMITY;
    req.get_data_req.action = SENSOR_HUB_GET_DATA;
    len = sizeof(req.get_data_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
        APS_ERR("SCP_sensorHub_req_send fail!\n");
    }
    else
    {
        *value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    if(atomic_read(&helix_obj->trace) & CMC_TRC_PS_DATA)
	{
        APS_LOG("value = %d\n", *value);
        //show data
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if(!helix_obj)
	{
		APS_ERR("helix_obj is null!!\n");
		return -1;
	}
    
    if((err = helix_read_ps(helix_obj->client, &helix_obj->ps)))
    {
        err = -1;;
    }
    else
    {
        *value = helix_get_ps_value(helix_obj, helix_obj->ps);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	return 0;
}



int helix_close_away_proc_write( struct file *filp, const char __user *buf,unsigned long len, void *data )
{
	char str_buf[256]={0};
	int close,away;
	
    APS_LOG("helix_close_away_proc_write +++\n");

	
	if (copy_from_user(str_buf,buf,len)){
        APS_LOG("copy_from_user error +++\n");
        return 0;
    }

	sscanf(str_buf,"%d:%d",&close,&away);
	
	APS_LOG("helix_close_away_proc_write str_buf=%s,close=%d,away=%d\n",  str_buf,close,away);

             // if( (close  == away) && (away!=0)){
             //     int margin = close * 24 /100;              
             //     close += margin;
             //     away+= margin; 
             //     if(away<0) away = 0;
             // }
	
	ps_cali.close = close + NEAR_OFFSET;
	ps_cali.far_away= away + FAR_OFFSET;
	ps_cali.valid = 1;
	
	helix_obj->hw->ps_threshold_high = ps_cali.close;
	helix_obj->hw->ps_threshold_low = ps_cali.far_away;
	atomic_set(&helix_obj->ps_thd_val_high,  helix_obj->hw->ps_threshold_high);
	atomic_set(&helix_obj->ps_thd_val_low,  helix_obj->hw->ps_threshold_low);

              APS_LOG("helix_close_away_proc_write ---\n");

	return len;	
        
}

#define CALI_LOOP  20

void helix_read_data(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
         int i=0 ,err = 0,j = 0;
         u16 data[22]={0};
	 u32 sum = 0,data_cali=0;
         
         
         for(i = 0;i<=CALI_LOOP;i++)
         {
	mdelay(5);
	if((err = helix_read_ps(client,&data[i])))
	{
		APS_LOG("helix_read_data fail: %d\n", i); 
		break;
	}  
              helix_clear_intr(client);        //0701
	mdelay(190);
         }
         
         
         for(j = 1;j<=CALI_LOOP;j++){
         	APS_LOG("[data%d]:%d,",j,data[j]);
        	sum += data[j];
         }
         
         if(i == (CALI_LOOP+1)) {	
	data_cali = sum/CALI_LOOP;             
              APS_LOG("helix_read_data >>>>>>> data = %d \n",data_cali);

              if(data_cali > DEF_BH)  data_cali = DEF_CT;
              if(data_cali > NG)  data_cali = NG_TS;
              
	ps_data_cali->close = data_cali;
	ps_data_cali->far_away = data_cali;
              ps_data_cali->valid = 1;
         } else  {
         	ps_data_cali->valid = 0;
         }
         
}


#ifdef OPEN_PROX_ARITHMETIC 

static int helix_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
         int i=0 ,err = 0,j = 0;
         u16 data[22]={0};
	 u32 sum = 0,data_cali=0;

         APS_FUN();
         
         for(i = 0;i<=CALI_LOOP;i++)
         {
         		mdelay(5);
                err = helix_read_ps(client,&data[i]);
        		if(err)
        		{
        			APS_ERR("helix_read_data_for_cali fail: %d\n", i); 
        			break;
        		}
                helix_clear_intr(client);    //0701
        		mdelay(190);
         }
         APS_LOG("helix_read_data_for_cali data:\n");
         
         for(j = 1;j<=CALI_LOOP;j++){
         	APS_LOG("[data%d]:%d,",j,data[j]);
        	sum += data[j];
         }
         
         if(i == (CALI_LOOP+1)) {
        	data_cali = sum/CALI_LOOP;        

              if(data_cali > DEF_BH)  data_cali = DEF_CT;
              if(data_cali > NG)  data_cali = NG_TS;
              
        	ps_data_cali->close = data_cali + NEAR_OFFSET;
        	ps_data_cali->far_away = data_cali + FAR_OFFSET;
              ps_data_cali->valid =1;
        	APS_LOG("helix_read_data_for_cali >>>>> data = %d \n",data_cali );

        	ps_cali.close = ps_data_cali->close;
        	ps_cali.far_away= ps_data_cali->far_away;
        	ps_cali.valid = 1;

        	helix_obj->hw->ps_threshold_high = ps_cali.close;
        	helix_obj->hw->ps_threshold_low = ps_cali.far_away;
        	atomic_set(&helix_obj->ps_thd_val_high,  helix_obj->hw->ps_threshold_high);
        	atomic_set(&helix_obj->ps_thd_val_low,  helix_obj->hw->ps_threshold_low);
        	
        	APS_LOG("helix_read_data_for_cali close  = %d,far_away = %d,valid = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);

         }	 else	 {
         	ps_data_cali->valid = 0;
         	err=  -1;
         }
        
         return err;
	 	

}


int helix_get_ps_rawdata(void)
{
	struct helix_priv *obj = helix_obj;  
	struct PS_CALI_DATA_STRUCT ps_cali_temp; 
	long err = 0;
        
	APS_FUN();

    disable_irq_nosync(gpio_irq);
	err = helix_enable_ps(obj->client,1);        
	if(err)
	{
		APS_LOG("helix_init_client_for_cali error,return\n");
		return -1;
	}
	else
	{
        err = 0;
	}
	err = helix_read_data_for_cali(obj->client,&ps_cali_temp);
	if(err)
	{
		APS_LOG("helix_read_data_for_cali error,set default vale\n");
		return -1;
	}
	err = helix_init_client(obj->client);
	if(err)
	{
		APS_LOG("helix_init_client error\n");
	              return -1;
	}
	err = helix_enable_ps(obj->client, 0);
	if(err)
	{
        APS_LOG("helix_enable error\n");
        return -1;
	}	
    APS_LOG("helix_get_ps_rawdata---\n");
          
	return 0;
}

#endif


#if 0
int helix_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int value;
	int err = 0;
	
	hwm_sensor_data* sensor_data;
	struct helix_priv *obj = (struct helix_priv *)self;
	
	APS_LOG("helix_ps_operate: %d\n", command);  
        
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{	
				value = *(int *)buff_in;
				if(value)
				{
				       //helix_get_ps_rawdata();
					//APS_LOG("helix_get_ps_rawdata \n"); 

					
					if((err = helix_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					#if 1
					if(!test_bit(CMC_BIT_ALS, &obj->enable))
					{
						ALS_FLAG = 1;
						if((err = helix_enable_als(obj->client, 1)))
						{
							APS_ERR("enable als fail: %d\n", err); 
							return -1;
						}
					}
					#endif
					
					
					
					
					//mt_eint_unmask(CUST_EINT_ALS_NUM);
					enable_irq(gpio_irq);
				}
				else
				{
					if((err = helix_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					#if 1
					if(ALS_FLAG == 1)
					{
						if((err = helix_enable_als(obj->client, 0)))
						{
							APS_ERR("disable als fail: %d\n", err); 
							return -1;
						}
						ALS_FLAG = 0;
					}
					#endif
					//mt_eint_mask(CUST_EINT_ALS_NUM);
					disable_irq_nosync(gpio_irq);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				helix_read_ps(obj->client, &obj->ps);
				helix_read_als_ch0(obj->client, &obj->als);
				APS_ERR("helix_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = helix_get_ps_value(obj, obj->ps);
				sensor_data->values[1]=obj->ps;				
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}


/*----------------------------------------------------------------------------*/
static int helix_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, helix_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int helix_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct helix_priv *obj;
	int err = 0;
	
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(*obj));
	helix_obj = obj;
	obj->hw = helix_get_cust_alsps_hw();
	helix_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
#ifdef HELIX_USE_PS_INT
    obj->alsps_workqueue = create_singlethread_workqueue("alsps");
    INIT_WORK(&obj->eint_work, helix_eint_work);
#endif
              
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
    obj->als_modulus = (400*100)/(1*150);

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	helix_i2c_client = client;
    
    err = helix_init_client(client);
	if(err)
	{
		goto exit_init_failed;
	}

	#ifdef	OPEN_PROX_ARITHMETIC
	helix_get_ps_rawdata();
    #endif		
	
	APS_LOG("helix_init_client() OK!\n");

	if((err = misc_register(&helix_device)))
	{
		APS_ERR("helix_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	als_ctl.is_use_common_factory =false;
	ps_ctl.is_use_common_factory = false;

	if((err = helix_create_attr(&(helix_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}	

	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = obj->hw->is_batch_supported_als;
#else
	als_ctl.is_support_batch = false;
#endif
	
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	
	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
	ps_ctl.is_support_batch = false;
#endif
	
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
	}
	
	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
	}

	helix_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
#if defined(MTK_AUTO_DETECT_ALSPS)
	helix_init_flag = 0;
#endif
	return 0;

	exit_sensor_obj_attach_fail:
	exit_create_attr_failed:
	misc_deregister(&helix_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	if(0 == obj->hw->polling_mode_ps)
	{
		//mt_eint_mask(CUST_EINT_ALS_NUM);
		disable_irq_nosync(gpio_irq);
	}
	//exit_kfree:
	kfree(obj);
	obj = NULL;
	exit:
	helix_i2c_client = NULL; 
	helix_init_flag = -1;

#if defined(MTK_AUTO_DETECT_ALSPS)
	helix_init_flag = -1;
#endif
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int helix_i2c_remove(struct i2c_client *client)
{
	int err;	
	
	if((err = helix_delete_attr(&(helix_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("helix_delete_attr fail: %d\n", err);
	} 

    //          remove_proc_entry("helixcloseaway",proc_helix_close_away);
			  
	if((err = misc_deregister(&helix_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	helix_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

/*----------------------------------------------------------------------------*/
static int helix_local_init(void)
{
	struct alsps_hw *hw = helix_get_cust_alsps_hw();

	helix_power(hw, 1); 

	if(i2c_add_driver(&helix_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 

	if(-1 == helix_init_flag)
	{
	   return -1;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int helix_local_uninit(void)
{
	struct alsps_hw *hw = helix_get_cust_alsps_hw();
	APS_FUN();    
	helix_power(hw, 0);
	i2c_del_driver(&helix_i2c_driver);
	return 0;
}



/*----------------------------------------------------------------------------*/
static int __init helix_init(void)
{
	alsps_driver_add(&helix_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit helix_exit(void)
{
	APS_FUN();
#ifndef MTK_AUTO_DETECT_ALSPS
	platform_driver_unregister(&helix_alsps_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(helix_init);
module_exit(helix_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("helix driver");
MODULE_LICENSE("GPL");


