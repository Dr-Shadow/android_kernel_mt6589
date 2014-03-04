/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#if defined(MTK_HDMI_SUPPORT)
#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#include <linux/debugfs.h>

#if defined(CONFIG_ARCH_MT6575)
    #include <mach/mt6575_typedefs.h>
#else
    #error "unknown arch"
#endif

#include "hdmitx.h"
#include "hdmi_drv.h"


void DBG_Init(void);
void DBG_Deinit(void);

extern void hdmi_log_enable(int enable);



// ---------------------------------------------------------------------------
//  External variable declarations
// ---------------------------------------------------------------------------

//extern LCM_DRIVER *lcm_drv;
// ---------------------------------------------------------------------------
//  Debug Options
// ---------------------------------------------------------------------------


static char STR_HELP[] =
    "\n"
    "USAGE\n"
    "        echo [ACTION]... > hdmitx\n"
    "\n"
    "ACTION\n"
    "        hdmitx:[on|off]\n"
    "             enable hdmi video output\n"
    "\n";

extern void hdmi_log_enable(int enable);

// TODO: this is a temp debug solution
//extern void hdmi_cable_fake_plug_in(void);
//extern int hdmi_drv_init(void);
static void process_dbg_opt(const char *opt)
{
	if (0 == strncmp(opt, "on", 2))
    {
		hdmi_power_on();
    }
    else if (0 == strncmp(opt, "off", 3))
    {
		hdmi_power_off();
    }
	else if(0 == strncmp(opt, "suspend", 7))
	{
		hdmi_suspend();
	}
	else if(0 == strncmp(opt, "resume", 6))
	{
		hdmi_resume();
	}
       else if(0 == strncmp(opt, "colorbar", 8))
        {
            
       }
       else if(0 == strncmp(opt, "ldooff", 6))
        {
            
       }
       else if (0 == strncmp(opt, "log:", 3))
    {
        if (0 == strncmp(opt + 9, "on", 2)) {
            hdmi_log_enable(true);
        } else if (0 == strncmp(opt + 9, "off", 3)) {
            hdmi_log_enable(false);
        } else {
            goto Error;
        }
    }
	else
	{
		goto Error;
	}

    return;

Error:
    printk("[hdmitx] parse command error!\n\n%s", STR_HELP);
}

static void process_dbg_cmd(char *cmd)
{
    char *tok;
    
    printk("[hdmitx] %s\n", cmd);
    
    while ((tok = strsep(&cmd, " ")) != NULL)
    {
        process_dbg_opt(tok);
    }
}

// ---------------------------------------------------------------------------
//  Debug FileSystem Routines
// ---------------------------------------------------------------------------

struct dentry *hdmitx_dbgfs = NULL;


static ssize_t debug_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}


static char debug_buffer[2048];

static ssize_t debug_read(struct file *file,
                          char __user *ubuf, size_t count, loff_t *ppos)
{
    const int debug_bufmax = sizeof(debug_buffer) - 1;
    int n = 0;

    n += scnprintf(debug_buffer + n, debug_bufmax - n, STR_HELP);
    debug_buffer[n++] = 0;

    return simple_read_from_buffer(ubuf, count, ppos, debug_buffer, n);
}


static ssize_t debug_write(struct file *file,
                           const char __user *ubuf, size_t count, loff_t *ppos)
{
    const int debug_bufmax = sizeof(debug_buffer) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax) 
        count = debug_bufmax;

	if (copy_from_user(&debug_buffer, ubuf, count))
		return -EFAULT;

	debug_buffer[count] = 0;

    process_dbg_cmd(debug_buffer);

    return ret;
}


static struct file_operations debug_fops = {
	.read  = debug_read,
    .write = debug_write,
	.open  = debug_open,
};


void HDMI_DBG_Init(void)
{
    hdmitx_dbgfs = debugfs_create_file("hdmi",
        S_IFREG|S_IRUGO, NULL, (void *)0, &debug_fops);
}


void HDMI_DBG_Deinit(void)
{
    debugfs_remove(hdmitx_dbgfs);
}

#endif
