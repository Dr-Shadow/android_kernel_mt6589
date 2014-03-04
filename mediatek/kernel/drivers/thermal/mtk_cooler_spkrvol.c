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
#define mtk_cooler_spkrvol_dprintk(fmt, args...) \
  do { xlog_printk(ANDROID_LOG_DEBUG, "thermal/cooler/spkrvol", fmt, ##args); } while(0)
#else
#define mtk_cooler_spkrvol_dprintk(fmt, args...) 
#endif

#define MAX_NUM_INSTANCE_MTK_COOLER_SPKRVOL  3

static struct thermal_cooling_device *cl_spkrvol_dev[MAX_NUM_INSTANCE_MTK_COOLER_SPKRVOL] = {0};
static unsigned long cl_spkrvol_state[MAX_NUM_INSTANCE_MTK_COOLER_SPKRVOL] = {0};

static int 
mtk_cl_spkrvol_get_max_state(struct thermal_cooling_device *cdev,
                              unsigned long *state)
{        
  *state = 1;
  mtk_cooler_spkrvol_dprintk("mtk_cl_spkrvol_get_max_state() %s %d\n", cdev->type, *state);
  return 0;
}

static int 
mtk_cl_spkrvol_get_cur_state(struct thermal_cooling_device *cdev,
                              unsigned long *state)
{
#if 1 // cannot use this way for now since devdata is used by mtk_thermal_monitor
  *state = *((unsigned long*) cdev->devdata);
#else
  *state = cl_spkrvol_state[(int) cdev->type[15]];
#endif
  mtk_cooler_spkrvol_dprintk("mtk_cl_spkrvol_get_cur_state() %s %d\n", cdev->type, *state);
  return 0;
}

static int 
mtk_cl_spkrvol_set_cur_state(struct thermal_cooling_device *cdev,
                              unsigned long state)
{
  mtk_cooler_spkrvol_dprintk("mtk_cl_spkrvol_set_cur_state() %s %d\n", cdev->type, state);
#if 1
  *((unsigned long*) cdev->devdata) = state;
#else
  cl_spkrvol_state[(int) cdev->type[15]] = state;
#endif
  if(1 == state)
  {
    // send uevent to notify current call must be dropped
    char event[] = "SPKRVOL=70";
    char *envp[] = { event, NULL };
    
    kobject_uevent_env(&(cdev->device.kobj), KOBJ_CHANGE, envp);
  }
#if 0 // unnecessary...only send uevent when needed
  else
  {
    // send uevent to notify no drop call is necessary
    char event[] = "SPKRVOL=0";
    char *envp[] = { event, NULL };
    
    kobject_uevent_env(&(cdev->device.kobj), KOBJ_CHANGE, envp);
  }
#endif
    
  return 0;
}

/* bind fan callbacks to fan device */
static struct thermal_cooling_device_ops mtk_cl_spkrvol_ops = {
  .get_max_state = mtk_cl_spkrvol_get_max_state,
  .get_cur_state = mtk_cl_spkrvol_get_cur_state,
  .set_cur_state = mtk_cl_spkrvol_set_cur_state,
};

static int mtk_cooler_spkrvol_register_ltf(void)
{
  int i;
  mtk_cooler_spkrvol_dprintk("register ltf\n");

  for (i = MAX_NUM_INSTANCE_MTK_COOLER_SPKRVOL; i-- > 0; )
  {
    char temp[20] = {0};
    sprintf(temp, "mtk-cl-spkrvol%02d", i);
    cl_spkrvol_dev[i] = mtk_thermal_cooling_device_register(temp, 
                                                             (void*) &cl_spkrvol_state[i],
                                                             &mtk_cl_spkrvol_ops);
  }

#if 0
  cl_spkrvol_dev = mtk_thermal_cooling_device_register("mtk-cl-spkrvol", 
                                                         NULL,
                                                         &mtk_cl_spkrvol_ops);
#endif

  return 0;
}

static void mtk_cooler_spkrvol_unregister_ltf(void)
{
  int i;
  mtk_cooler_spkrvol_dprintk("unregister ltf\n");

  for (i = MAX_NUM_INSTANCE_MTK_COOLER_SPKRVOL; i-- > 0; )
  {
    if (cl_spkrvol_dev[i])
    {
      mtk_thermal_cooling_device_unregister(cl_spkrvol_dev[i]);
      cl_spkrvol_dev[i] = NULL;
      cl_spkrvol_state[i] = 0;
    }
  }
#if 0
  if (cl_spkrvol_dev) 
  {
    mtk_thermal_cooling_device_unregister(cl_spkrvol_dev);
    cl_spkrvol_dev = NULL;
  }
#endif
}


static int __init mtk_cooler_spkrvol_init(void)
{
  int err = 0;
  int i;

  for (i = MAX_NUM_INSTANCE_MTK_COOLER_SPKRVOL; i-- > 0; )
  {
    cl_spkrvol_dev[i] = NULL;
    cl_spkrvol_state[i] = 0;
  }

  //cl_spkrvol_dev = NULL;

  mtk_cooler_spkrvol_dprintk("init\n");

  err = mtk_cooler_spkrvol_register_ltf();
  if (err)
    goto err_unreg;

  return 0;

err_unreg:
  mtk_cooler_spkrvol_unregister_ltf();
  return err;
}

static void __exit mtk_cooler_spkrvol_exit(void)
{
  mtk_cooler_spkrvol_dprintk("exit\n");
    
  mtk_cooler_spkrvol_unregister_ltf();
}

module_init(mtk_cooler_spkrvol_init);
module_exit(mtk_cooler_spkrvol_exit);




