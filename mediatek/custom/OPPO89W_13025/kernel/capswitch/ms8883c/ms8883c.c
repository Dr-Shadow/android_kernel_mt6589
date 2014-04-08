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

#define CAPSWITCH_DEV_NAME "capswitch"

#define GPIO_CAPSWITCH_EINT_PIN         GPIO73
#define CUST_EINT_CAPSWITCH_NUM 		73

static DEFINE_SEMAPHORE(capswitch_eint_timer_sem);

extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

#define CAP_INPUT

#ifdef CAP_INPUT
static struct input_dev *capswitch_input = NULL;

static struct delayed_work capswitch_delayed_work;
#endif
static struct hrtimer eint_timer;
static unsigned int eint_count = 0;

static unsigned int capswitch_signal = 0;

static ssize_t capswitch_signal_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
    char page[8]; 	
    char *p = page;	
    int len = 0; 	
    p += sprintf(p, "%d\n", capswitch_signal);	
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

static ssize_t capswitch_signal_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{	
    char tmp[2] = {0, 0};	
    int ret;		
    if (count > 2)		
        return -EINVAL;		
    ret = copy_from_user(tmp, buf, 2);
    if ('1' == tmp[0])		
        capswitch_signal = 1;	
    else if ('0' == tmp[0])		
        capswitch_signal = 0;	
    else		
        return -EINVAL;		
    return count;	
}

static struct file_operations capswitch_fops = {
	.read = capswitch_signal_read,
	.write = capswitch_signal_write,
};

#ifdef CAP_INPUT
static int capswitch_delayed_work_func(struct work_struct *work)
{
	hrtimer_start(&eint_timer, ktime_set(0, 300000000/*200ms*/), HRTIMER_MODE_REL);
}
#endif

static enum hrtimer_restart capswitch_eint_timer_func(struct hrtimer *timer)
{
	if (eint_count >= 2){
		capswitch_signal = 1;
	#ifdef CAP_INPUT
		input_report_key(capswitch_input, KEY_PAUSE, 1);
		input_sync(capswitch_input);
		input_report_key(capswitch_input, KEY_PAUSE, 0);
		input_sync(capswitch_input);
	#endif
		printk("%s  touched count=%d \n",__func__, eint_count);
	}
	eint_count = 0;
	return HRTIMER_NORESTART;
}

static void capswitch_eint_func(void)
{
	eint_count++;
	if (eint_count == 1){
		mt65xx_eint_unmask(CUST_EINT_CAPSWITCH_NUM);
		hrtimer_start(&eint_timer, ktime_set(0, 500000000/*200ms*/), HRTIMER_MODE_REL);
	} 
	else 
		mt65xx_eint_unmask(CUST_EINT_CAPSWITCH_NUM);
	
	printk("%s  eint_count=%d\n",__func__,eint_count);
}

static void capswitch_setup_eint(void)
{
	mt_set_gpio_mode(GPIO_CAPSWITCH_EINT_PIN, GPIO_MODE_02);
	mt_set_gpio_dir(GPIO_CAPSWITCH_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CAPSWITCH_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CAPSWITCH_EINT_PIN, GPIO_PULL_DOWN);
	
	mt65xx_eint_set_sens(CUST_EINT_CAPSWITCH_NUM, CUST_EINT_EDGE_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_CAPSWITCH_NUM, CUST_EINT_POLARITY_HIGH);
	mt65xx_eint_set_hw_debounce(CUST_EINT_CAPSWITCH_NUM, 0);
	mt65xx_eint_registration(CUST_EINT_CAPSWITCH_NUM, CUST_EINT_DEBOUNCE_DISABLE, CUST_EINT_POLARITY_HIGH, capswitch_eint_func, false);

	mt65xx_eint_unmask(CUST_EINT_CAPSWITCH_NUM);

}

static int capswitch_probe(struct platform_device *pdev)
{
	int ret;
	
	printk("%s  Enter\n",__func__);
	
	hrtimer_init(&eint_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	eint_timer.function = capswitch_eint_timer_func;

	hwmsen_make_debug_flag(&capswitch_fops, "capswitch_signal");

#ifdef CAP_INPUT
	INIT_DELAYED_WORK(&capswitch_delayed_work, capswitch_delayed_work_func);
	capswitch_input = input_allocate_device();
	if (capswitch_input == NULL){
		printk("%s  allocate input device fail\n",__func__);
		goto exit;
	}
	capswitch_input->name = CAPSWITCH_DEV_NAME;
	set_bit(EV_SYN, capswitch_input->evbit);
	set_bit(EV_KEY, capswitch_input->evbit);
	set_bit(KEY_PAUSE, capswitch_input->keybit);

	ret = input_register_device(capswitch_input);
	if (ret < 0){
		printk("%s  register input fail\n",__func__);
		goto exit;
	}
#endif
	capswitch_setup_eint();
	printk("%s  Exit  OK\n",__func__);

	return 0;
exit:
	printk("%s  Exit  error\n",__func__);
	return -EINVAL;
}

static int capswitch_suspend(struct platform_device *pdev, pm_message_t mes)
{
	//mt65xx_eint_mask(CUST_EINT_CAPSWITCH_NUM);
	return 0;
}

static int capswitch_resume(struct platform_device *pdev)
{
	//mt65xx_eint_unmask(CUST_EINT_CAPSWITCH_NUM);
	return 0;
}

static int capswitch_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver capswitch_driver = {
	.probe = capswitch_probe,
	.remove = capswitch_remove,
	.suspend = capswitch_suspend,
	.resume = capswitch_resume,
	.driver = {
		.name = CAPSWITCH_DEV_NAME,
	},
};
static int __init capswitch_mod_init(void)
{
	int ret;
	printk("%s Enter\n",__func__);
	ret = platform_driver_register(&capswitch_driver);
	if (ret < 0)
		printk("%s  register driver fail !\n",__func__);
	
	printk("%s Exit\n",__func__);
	return ret;
}

static void __exit capswitch_mod_exit(void)
{
	platform_driver_unregister(&capswitch_driver);
}

module_init(capswitch_mod_init);
module_exit(capswitch_mod_exit);

MODULE_AUTHOR("ye.zhang <zhye@oppo.com>");
MODULE_DESCRIPTION("OPPO capswitch driver");
MODULE_LICENSE("GPL");
