/************************************************************************************
** File: - mediatek\custom\common\kernel\touchpanel\synaOT\synaOT_drv.c
** OPPO_R819
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      touch panel driver for synaptics
**      can change MAX_POINT_NUM value to support multipoint
** Version: 1.0
** Date created: 10:49:46,17/12/2012
** Author: rendong.shi@BasicDrv.TP
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** 
************************************************************************************/


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/mt_gpio.h>
//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/eint.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <linux/rtpm_prio.h>
#include <linux/sysfs.h>
#include <linux/proc_fs.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_pm_ldo.h>

static unsigned int tp_debug = 1;
#define micro_DEVICE  "micro"

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
			printk(micro_DEVICE ": " a,##arg);\
	}while(0)
#define VKNUMBER 4
static int vkey_points[VKNUMBER] = {60,160,260,360};	
static unsigned int boot_mode = 0;
#define KEY_UNPRESS			(0)
#define KEY_PRESS			(1)
#define TOUCH_KEY

#ifdef   TOUCH_KEY
#define GPIO_BACK	(GPIO74)
#define GPIO_HOME	(GPIO75)
#define GPIO_MENU	    (GPIO73)
#define EINT_BACK_NUM	(74)
#define EINT_HOME_NUM	(75)
#define EINT_MENU_NUM	(73)

static DEFINE_SEMAPHORE(work_sem);
static void kpd_back_eint(void);
static void kpd_back_handler(unsigned long data);
static DECLARE_TASKLET(kpd_back_tasklet, kpd_back_handler, 0);
static void kpd_home_eint(void);
static void kpd_home_handler(unsigned long data);
static DECLARE_TASKLET(kpd_home_tasklet, kpd_home_handler, 0);
static void kpd_menu_eint(void);
static void kpd_menu_handler(unsigned long data);
static DECLARE_TASKLET(kpd_menu_tasklet, kpd_menu_handler, 0);	


int back_key_down_press = 0;
int menu_key_down_press = 0;
int home_key_down_press = 0;

static struct input_dev *micro_input_dev;
static struct early_suspend micro_early_suspend;
#endif	
	
static void micro_real_key_report(struct input_dev  *micro_input_dev,
    int raw_x, int raw_y, int x, int y, int p) {
		
    if(micro_input_dev) 
	{
		input_report_key(micro_input_dev, BTN_TOUCH, 1);
        input_report_abs(micro_input_dev, ABS_MT_TOUCH_MAJOR, p);
        input_report_abs(micro_input_dev, ABS_MT_WIDTH_MAJOR, (raw_x+raw_y)/2);
        input_report_abs(micro_input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(micro_input_dev, ABS_MT_POSITION_Y, y);
        input_mt_sync(micro_input_dev);
        if(p)
        	TPD_DEBUG("D[%4d %4d %4d]\n", x, y, p);
	}		  
}
extern unsigned int Touchpanel_Detected(void);
static void micro_key_report(struct input_dev  *micro_input_dev)
{
    down(&work_sem);
    menu_key_down_press = !mt_get_gpio_in(GPIO_MENU);
	home_key_down_press = !mt_get_gpio_in(GPIO_HOME);
    back_key_down_press = !mt_get_gpio_in(GPIO_BACK);	
	if(!Touchpanel_Detected())
	{
		if(back_key_down_press || menu_key_down_press || home_key_down_press)
		{
		   if(boot_mode!= RECOVERY_BOOT)
		   {
			if(back_key_down_press)
				micro_real_key_report(micro_input_dev, 10,10,700,1400, 200); 
			if(home_key_down_press)
				micro_real_key_report(micro_input_dev, 10,10,500,1400, 200); 
			if(menu_key_down_press)
				micro_real_key_report(micro_input_dev, 10,10,100,1400, 200);
		   }
		   else
		   {
			if(home_key_down_press)
			{
				input_report_key(micro_input_dev, KEY_HOME, 1);
				printk("miro home down!!!\n");
			}
		   }
			
		//	atomic_set(&touch_key_touched,1);
		}
   	
		else
		{
		   if(boot_mode!= RECOVERY_BOOT) 
		   {
				input_report_key(micro_input_dev, BTN_TOUCH, 0);
				TPD_DEBUG("U[0 0 0]\n");
				input_mt_sync(micro_input_dev);
		   }
		   else
		   {
		     if(!home_key_down_press)
			 {
		        input_report_key(micro_input_dev, KEY_HOME, 0);
			     printk("miro home up!!!\n");
			  }
		   }
	    }
	    
        input_sync(micro_input_dev);
	}
	up(&work_sem);

}

	

	
void kpd_back_handler(unsigned long data)
{
        
        back_key_down_press = !mt_get_gpio_in(GPIO_BACK);                          
        micro_key_report(micro_input_dev); 
		TPD_DEBUG("%s is called.", __func__);
		

}

static void kpd_back_eint(void)
{
	int tmp ;             

	tmp = mt_get_gpio_in(GPIO_BACK);            

	mt65xx_eint_set_polarity(EINT_BACK_NUM, !tmp);

	mt65xx_eint_unmask(EINT_BACK_NUM);         
	
	tasklet_schedule(&kpd_back_tasklet);
	
	TPD_DEBUG("%s is called.", __func__);
}



static void kpd_home_handler(unsigned long data)
{
 	
        
        home_key_down_press = !mt_get_gpio_in(GPIO_HOME);
                                    
        micro_key_report(micro_input_dev); 	
		TPD_DEBUG("%s is called.", __func__);
		

}
static void kpd_home_eint(void)
{
	int tmp ;             
	TPD_DEBUG("%s is called.", __func__);
	
	tmp = mt_get_gpio_in(GPIO_HOME);            
	mt65xx_eint_set_polarity(EINT_HOME_NUM, !tmp);
	mt65xx_eint_unmask(EINT_HOME_NUM);
	tasklet_schedule(&kpd_home_tasklet);
}



static void kpd_menu_handler(unsigned long data)
{
       
        menu_key_down_press = !mt_get_gpio_in(GPIO_MENU); 
		
		micro_key_report(micro_input_dev); 
               	
		TPD_DEBUG(KERN_ALERT"%s is called.", __func__);
}

static void kpd_menu_eint(void)
{
	int tmp ;             

	tmp = mt_get_gpio_in(GPIO_MENU);            

	mt65xx_eint_set_polarity(EINT_MENU_NUM, !tmp);

	mt65xx_eint_unmask(EINT_MENU_NUM);         

	tasklet_schedule(&kpd_menu_tasklet);
	TPD_DEBUG("%s is called.", __func__);
}

	
	
	
	
	
	
	
	



static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf){
//static ssize_t cap_vk_show(char *buf){
      /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
      return sprintf(buf,
        	__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":100:1400:80:80"
         ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":500:1400:80:80"
         ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":700:1400:80:80"
         "\n");

}

static struct kobj_attribute mtk_virtual_keys_attr = 
{
    .attr = 
	{
		.name = "virtualkeys."micro_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *mtk_properties_attrs[] = 
{
    &mtk_virtual_keys_attr.attr,
    NULL
};


static struct attribute_group mtk_properties_attr_group = 
{
    .attrs = mtk_properties_attrs,
};

static struct kobject *micro_properties_kobj;

static int micro_virtual_keys_init()
{
	int ret;

	micro_properties_kobj = kobject_create_and_add("board_properties", NULL);
	
    if(micro_properties_kobj)
        ret = sysfs_create_group(micro_properties_kobj, &mtk_properties_attr_group);
	
    if(!micro_properties_kobj || ret)
	    printk("failed to create board_properties\n");		

	return ret;
}
/*----------------------------------------------------------------------------*/
static void micro_key_early_suspend(struct early_suspend *h)
{
     printk("micro_early_suspend is called\n");
		mt65xx_eint_mask(EINT_MENU_NUM);
		mt65xx_eint_mask(EINT_HOME_NUM);
		mt65xx_eint_mask(EINT_BACK_NUM);
		//hwPowerDown(MT65XX_POWER_LDO_VMCH1,"micro_3000");
		mt_set_gpio_out(79, 0); 

}

static void micro_key_late_resume(struct early_suspend *h)
{
 
    mt_set_gpio_out(79, 1); 
	//hwPowerOn(MT65XX_POWER_LDO_VMCH1,VOL_3000,"micro_3000");
	printk("micro_late_resume is called\n");
	mt65xx_eint_unmask(EINT_MENU_NUM);
	mt65xx_eint_unmask(EINT_HOME_NUM);
	mt65xx_eint_unmask(EINT_BACK_NUM);
	
}
/*----------------------------------------------------------------------------*/

static int micro_probe()
{

    int ret = -1;
	boot_mode = get_boot_mode();
	printk("MICRO_______ boot_mode = %d\n",boot_mode);
    hwPowerOn(MT65XX_POWER_LDO_VMCH1,VOL_3000,"micro_3000");
	micro_input_dev = input_allocate_device();
	printk(" allocate input device ok");
	if (micro_input_dev == NULL) 
	{
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_OT_probe: Failed to allocate input device\n");
	}
	printk("success to allocate input device");
	micro_input_dev ->name = micro_DEVICE;
    set_bit(EV_SYN, micro_input_dev->evbit);
	set_bit(EV_ABS, micro_input_dev->evbit);
	set_bit(EV_KEY, micro_input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, micro_input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,micro_input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, micro_input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, micro_input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, micro_input_dev->propbit);
	input_set_abs_params(micro_input_dev, ABS_MT_TOUCH_MAJOR,
			     0, 255, 0, 0);
	input_set_abs_params(micro_input_dev, ABS_MT_POSITION_X,
		     0,720, 0, 0);
	input_set_abs_params(micro_input_dev, ABS_MT_POSITION_Y,
		     0, 1280, 0, 0);
	set_bit(BTN_TOUCH, micro_input_dev->keybit);
	
	set_bit(KEY_MENU, micro_input_dev->keybit);
	set_bit(KEY_HOME, micro_input_dev->keybit);
	set_bit(KEY_BACK, micro_input_dev->keybit);
  
	printk("micro input_register_device begin\n");
	if(input_register_device(micro_input_dev))
	{
		printk(KERN_ERR "syna_OT_probe: Failed to register input device\n");
		goto err_input_dev_register;
	}
	printk("micro input_register_device OK\n");

	
	
	
	#ifdef TOUCH_KEY
	mt_set_gpio_mode(GPIO_BACK, GPIO_MODE_02);
	mt_set_gpio_dir(GPIO_BACK, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_BACK, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_BACK, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO_HOME, GPIO_MODE_02);
	mt_set_gpio_dir(GPIO_HOME, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HOME, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_HOME, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO_MENU, GPIO_MODE_02);
	mt_set_gpio_dir(GPIO_MENU, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MENU, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_MENU, GPIO_PULL_UP);
	mt65xx_eint_set_sens(EINT_BACK_NUM, CUST_EINT_EDGE_SENSITIVE);
	mt65xx_eint_set_polarity(EINT_BACK_NUM, CUST_EINT_POLARITY_LOW);
	mt65xx_eint_set_hw_debounce(EINT_BACK_NUM,(0XA0 >> 5));
	mt65xx_eint_registration(EINT_BACK_NUM, 
					true, 
					CUST_EINT_POLARITY_LOW,
	                         	kpd_back_eint, 
					false);
	mt65xx_eint_set_sens(EINT_HOME_NUM, CUST_EINT_EDGE_SENSITIVE);
	mt65xx_eint_set_polarity(EINT_HOME_NUM, CUST_EINT_POLARITY_LOW);
	mt65xx_eint_set_hw_debounce(EINT_HOME_NUM, (0XA0 >> 5));
	mt65xx_eint_registration(EINT_HOME_NUM, 
					true, 
					CUST_EINT_POLARITY_LOW,
	                         	kpd_home_eint, 
					false);
	mt65xx_eint_set_sens(EINT_MENU_NUM, CUST_EINT_EDGE_SENSITIVE);
	mt65xx_eint_set_polarity(EINT_MENU_NUM, CUST_EINT_POLARITY_LOW);
	mt65xx_eint_set_hw_debounce(EINT_MENU_NUM, (0XA0 >> 5));
	mt65xx_eint_registration(EINT_MENU_NUM, 
					true, 
					CUST_EINT_POLARITY_LOW,
	                         	kpd_menu_eint, 
					false);
	mt65xx_eint_unmask(EINT_BACK_NUM);         
	mt65xx_eint_unmask(EINT_HOME_NUM);
	mt65xx_eint_unmask(EINT_MENU_NUM); 
	
	ret = micro_virtual_keys_init();
	if (ret < 0) {
	    printk("[%s] fail to add kobject\n", "board_properties");
	}
	
	micro_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	micro_early_suspend.suspend = micro_key_early_suspend;
	micro_early_suspend.resume = micro_key_late_resume;
	register_early_suspend(&micro_early_suspend);
    return 0;
    #endif
	
err_input_dev_register:
	input_free_device(micro_input_dev);
	hwPowerDown(MT65XX_POWER_LDO_VMCH1,"micro_3000");
	return ret;

	
}


static int micro_remove(struct platform_device *pdev)
{

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&micro_early_suspend);
#endif
	
	input_unregister_device(micro_input_dev);
	input_free_device(micro_input_dev);
	return 0;
}



/*----------------------------------------------------------------------------*/
static struct platform_driver micro_platform_driver = {
    .probe      =micro_probe,
    .remove     = micro_remove,    
    .driver     = {
    .name  = "micro-platform",
        //		.owner = THIS_MODULE,
    }
};

static int __init micro_driver_init(void)
{
  
	printk(" micro_driver_init!!!!!!!!!!!\n");
    if(platform_driver_register(&micro_platform_driver))
    {
        printk("failed to register platform driver");
        return -ENODEV;
    }
    return 0;
}
static void __exit micro_driver_exit(void) {
  platform_driver_unregister(&micro_platform_driver);
}

module_init(micro_driver_init);
module_exit(micro_driver_exit);

MODULE_DESCRIPTION("Synaptics touch key Driver");
MODULE_LICENSE("GPL");
