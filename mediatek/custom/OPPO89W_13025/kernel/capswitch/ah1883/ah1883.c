#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>

#include <linux/hwmsen_helper.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/semaphore.h>
#include <linux/wakelock.h>

#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/hwmsensor.h>
/*----------------------------------------------------------------*/
#define HALL_DEV_NAME "hall"

#define GPIO_HALL_EINT_PIN         GPIO116
#define CUST_EINT_HALL_NUM 		11

static unsigned int is_device_suspend = 0;
static struct early_suspend hall_early_suspend;
/*----------------------------------------------------------------*/
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern s32 mt_get_gpio_pull_select_chip(u32 pin);
/*----------------------------------------------------------------*/
#define HALL_INPUT

#ifdef HALL_INPUT
static struct input_dev *hall_input = NULL;
static struct workqueue_struct *hall_workqueue = NULL;
static struct work_struct hall_report_work;
#endif
static unsigned int hall_signal = 0;
/*----------------------------------------------------------------*/
static void hall_eint_func(void);
/*----------------------------------------------------------------*/
static ssize_t hall_signal_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
    char page[8]; 	
    char *p = page;	
    int len = 0; 	
    p += sprintf(p, "%d\n", hall_signal);	
    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf,page,len < count ? len  : count))		
        return -EFAULT;	
    *pos = *pos + (len < count ? len  : count);	

    return len < count ? len  : count;
}

static ssize_t hall_signal_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{	
    char tmp[2] = {0, 0};	
    int ret;		
    if (count > 2)		
        return -EINVAL;		
    ret = copy_from_user(tmp, buf, 2);
    if ('1' == tmp[0])		
        hall_signal = 1;	
    else if ('0' == tmp[0])		
        hall_signal = 0;	
    else		
        return -EINVAL;		
    return count;	
}
/*----------------------------------------------------------------*/
static struct file_operations hall_fops = {
	.read = hall_signal_read,
	.write = hall_signal_write,
};
/*----------------------------------------------------------------*/
static void hall_setup_eint_wait_far(void)
{
	mt_set_gpio_mode(GPIO_HALL_EINT_PIN, GPIO_MODE_01);
	mt_set_gpio_dir(GPIO_HALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_HALL_EINT_PIN, GPIO_PULL_DOWN);

	mt65xx_eint_set_sens(CUST_EINT_HALL_NUM, CUST_EINT_LEVEL_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_HALL_NUM, CUST_EINT_POLARITY_HIGH);
	mt65xx_eint_set_hw_debounce(CUST_EINT_HALL_NUM, 0);
	mt65xx_eint_registration(CUST_EINT_HALL_NUM, CUST_EINT_DEBOUNCE_DISABLE, CUST_EINT_POLARITY_HIGH, hall_eint_func, 0);
	mdelay(5);
	mt65xx_eint_unmask(CUST_EINT_HALL_NUM);
}
/*----------------------------------------------------------------*/
static void hall_setup_eint_wait_close(void)
{
	mt_set_gpio_mode(GPIO_HALL_EINT_PIN, GPIO_MODE_01);
	mt_set_gpio_dir(GPIO_HALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_HALL_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_HALL_NUM, CUST_EINT_LEVEL_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_HALL_NUM, CUST_EINT_POLARITY_LOW);
	mt65xx_eint_set_hw_debounce(CUST_EINT_HALL_NUM, 0);
	mt65xx_eint_registration(CUST_EINT_HALL_NUM, CUST_EINT_DEBOUNCE_DISABLE, CUST_EINT_POLARITY_LOW, hall_eint_func, 0);
	mdelay(5);
	mt65xx_eint_unmask(CUST_EINT_HALL_NUM);

}
/*----------------------------------------------------------------*/
#ifdef HALL_INPUT
static int hall_ev_report_func(struct work_struct *work)
{
	if ((!hall_signal && !is_device_suspend) || (hall_signal && is_device_suspend)){
		printk("%s report status\n",__func__);
		input_report_key(hall_input, KEY_POWER, 1);
		input_sync(hall_input);
		input_report_key(hall_input, KEY_POWER, 0);
		input_sync(hall_input);
	} else printk("%s not report status\n",__func__);
	printk("%s now status is %s\n",__func__,hall_signal==0?"close":"far");
	if (hall_signal)
		hall_setup_eint_wait_close();
	else hall_setup_eint_wait_far(); /*eint pin has been pulled down*/
}
#endif
/*----------------------------------------------------------------*/
static void hall_eint_func(void)
{
	printk("%s\n",__func__);
	hall_signal = !hall_signal;
#ifdef HALL_INPUT
	queue_work(hall_workqueue, &hall_report_work);
#endif
}
/*----------------------------------------------------------------*/
static void hall_early_suspend_func(struct early_suspend *h)
{
	is_device_suspend = 1;
	printk("%s\n",__func__);
}
/*----------------------------------------------------------------*/
static void hall_early_resume_func(struct early_suspend *h)
{
	is_device_suspend = 0;
	printk("%s\n",__func__);
}
/*----------------------------------------------------------------*/
static int hall_probe(struct platform_device *pdev)
{
	int ret;
	
	printk("%s  Enter\n",__func__);

	hwmsen_make_debug_flag(&hall_fops, "hall_signal");

#ifdef HALL_INPUT
	INIT_WORK(&hall_report_work, hall_ev_report_func);
	hall_input = input_allocate_device();
	if (hall_input == NULL){
		printk("%s  allocate input device fail\n",__func__);
		goto exit;
	}
	hall_input->name = HALL_DEV_NAME;
	set_bit(EV_SYN, hall_input->evbit);
	set_bit(EV_KEY, hall_input->evbit);
	set_bit(KEY_POWER, hall_input->keybit);

	ret = input_register_device(hall_input);
	if (ret < 0){
		printk("%s  register input fail\n",__func__);
		goto exit;
	}
#endif

	hall_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	hall_early_suspend.suspend = hall_early_suspend_func;
	hall_early_suspend.resume = hall_early_resume_func;
	register_early_suspend(&hall_early_suspend);

	hall_signal = mt_get_gpio_pull_select_chip(GPIO_HALL_EINT_PIN); //default high level
	printk("%s  hall_signal = %d\n",__func__,hall_signal);
	if (hall_signal)
		hall_setup_eint_wait_close(); 
	else hall_setup_eint_wait_far(); /*eint pin has been pulled down*/
	
	printk("%s  Exit  OK\n",__func__);
	return 0;
exit:
	printk("%s  Exit  error\n",__func__);
	return -EINVAL;
}
/*----------------------------------------------------------------*/
static int hall_remove(struct platform_device *pdev)
{
	return 0;
}
/*----------------------------------------------------------------*/
static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.remove = hall_remove,
	.driver = {
		.name = HALL_DEV_NAME,
	},
};
/*----------------------------------------------------------------*/
static int __init hall_mod_init(void)
{
	int ret;
	printk("%s Enter\n",__func__);
	ret = platform_driver_register(&hall_driver);
	if (ret < 0)
		printk("%s  register driver fail !\n",__func__);
	
#ifdef HALL_INPUT
	hall_workqueue = create_singlethread_workqueue("hall_work_thread");
#endif
	printk("%s Exit\n",__func__);
	return ret;
}
/*----------------------------------------------------------------*/
static void __exit hall_mod_exit(void)
{
#ifdef HALL_INPUT
	destroy_workqueue(hall_workqueue);
#endif
	platform_driver_unregister(&hall_driver);
}
/*----------------------------------------------------------------*/
module_init(hall_mod_init);
module_exit(hall_mod_exit);

MODULE_AUTHOR("ye.zhang <zhye@oppo.com>");
MODULE_DESCRIPTION("OPPO hall driver");
MODULE_LICENSE("GPL");
