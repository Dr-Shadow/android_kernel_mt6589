/***
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK-DISTRIBUTED SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK-DISTRIBUTED SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK-DISTRIBUTED
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK-DISTRIBUTED SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/xlog.h>
#include <linux/types.h>
#include <linux/kobject.h>

#include "mach/mtk_thermal_monitor.h"
#include <mach/system.h>

#if 0
#define mtk_cooler_dropcall_dprintk(fmt, args...) \
  do { xlog_printk(ANDROID_LOG_DEBUG, "thermal/cooler/dropcall", fmt, ##args); } while(0)
#else
#define mtk_cooler_dropcall_dprintk(fmt, args...) 
#endif

static struct thermal_cooling_device *cl_dropcall_dev;
static unsigned int cl_dropcall_state = 0;

static int 
mtk_cl_dropcall_get_max_state(struct thermal_cooling_device *cdev,
                              unsigned long *state)
{        
  *state = 1;
  mtk_cooler_dropcall_dprintk("mtk_cl_dropcall_get_max_state() %d\n", *state);
  return 0;
}

static int 
mtk_cl_dropcall_get_cur_state(struct thermal_cooling_device *cdev,
                              unsigned long *state)
{        
  *state = cl_dropcall_state;
  mtk_cooler_dropcall_dprintk("mtk_cl_dropcall_get_cur_state() %d\n", *state);
  return 0;
}

static int 
mtk_cl_dropcall_set_cur_state(struct thermal_cooling_device *cdev,
                              unsigned long state)
{
  mtk_cooler_dropcall_dprintk("mtk_cl_dropcall_set_cur_state() %d\n", state);
  cl_dropcall_state = state;
  if(cl_dropcall_state == 1)
  {
    // send uevent to notify current call must be dropped
    char event[] = "DROPCALL=1";
    char *envp[] = { event, NULL };
    
    kobject_uevent_env(&(cl_dropcall_dev->device.kobj), KOBJ_CHANGE, envp);
  }
  else
  {
    // send uevent to notify no drop call is necessary
    char event[] = "DROPCALL=0";
    char *envp[] = { event, NULL };
    
    kobject_uevent_env(&(cl_dropcall_dev->device.kobj), KOBJ_CHANGE, envp);
  }
    
  return 0;
}

/* bind fan callbacks to fan device */
static struct thermal_cooling_device_ops mtk_cl_dropcall_ops = {
  .get_max_state = mtk_cl_dropcall_get_max_state,
  .get_cur_state = mtk_cl_dropcall_get_cur_state,
  .set_cur_state = mtk_cl_dropcall_set_cur_state,
};

static int mtk_cooler_dropcall_register_ltf(void)
{
  mtk_cooler_dropcall_dprintk("register ltf\n");
  cl_dropcall_dev = mtk_thermal_cooling_device_register("mtk-cooler-dropcall", 
                                                         NULL,
                                                         &mtk_cl_dropcall_ops);

  return 0;
}

static void mtk_cooler_dropcall_unregister_ltf(void)
{
  mtk_cooler_dropcall_dprintk("unregister ltf\n");
    
  if (cl_dropcall_dev) 
  {
    mtk_thermal_cooling_device_unregister(cl_dropcall_dev);
    cl_dropcall_dev = NULL;
  }
}


static int __init mtk_cooler_dropcall_init(void)
{
  int err = 0;

  cl_dropcall_dev = NULL;

  mtk_cooler_dropcall_dprintk("init\n");

  err = mtk_cooler_dropcall_register_ltf();
  if (err)
    goto err_unreg;

  return 0;

err_unreg:
  mtk_cooler_dropcall_unregister_ltf();
  return err;
}

static void __exit mtk_cooler_dropcall_exit(void)
{
  mtk_cooler_dropcall_dprintk("exit\n");
    
  mtk_cooler_dropcall_unregister_ltf();
}

module_init(mtk_cooler_dropcall_init);
module_exit(mtk_cooler_dropcall_exit);


