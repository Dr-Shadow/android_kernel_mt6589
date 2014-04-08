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
#include "extmd_mt6252d.h"
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/module.h>
//#include <mach/mt6575_gpio.h>
#include <cust_eint.h>

extern unsigned int mt65xx_eint_set_sens(unsigned int, unsigned int);
extern void mt65xx_eint_set_polarity(unsigned char, unsigned char);
extern void mt65xx_eint_set_hw_debounce(unsigned char, unsigned int);
extern void mt65xx_eint_registration(unsigned char, unsigned char, unsigned char, void(*func)(void),
					unsigned char);
extern void mt65xx_eint_unmask(unsigned int);
extern void mt65xx_eint_mask(unsigned int);

int cm_do_md_power_on(void)
{
	return 0;
}

void cm_hold_rst_signal(void)
{
	mt_set_gpio_dir(GPIO_DT_MD_RST_PIN, 1);
	mt_set_gpio_out(GPIO_DT_MD_RST_PIN, 0);
}

void cm_relese_rst_signal(void)
{
	mt_set_gpio_out(GPIO_DT_MD_RST_PIN, 1);
	mt_set_gpio_dir(GPIO_DT_MD_RST_PIN, 0);
}

int cm_do_md_go(void)
{
	//int high_signal_check_num=0;
	int ret = 0;
	unsigned int retry = 100;

	#if 0
	cm_relese_rst_signal();
	msleep(10);

	mt_set_gpio_dir(GPIO_DT_MD_PWR_KEY_PIN, 1);
	mt_set_gpio_out(GPIO_DT_MD_PWR_KEY_PIN, EXT_MD_PWR_KEY_ACTIVE_LVL);

	msleep(5000); 
	mt_set_gpio_dir(GPIO_DT_MD_PWR_KEY_PIN, 0);
	//mt_set_gpio_out(GPIO_OTG_DRVVBUS_PIN, 1); // VBus
	#endif
	// Release download key to let md can enter normal boot
	//mt_set_gpio_dir(102, 1);
	//mt_set_gpio_out(102, 1);
	mt_set_gpio_dir(GPIO_DT_MD_DL_PIN, 1);
	mt_set_gpio_out(GPIO_DT_MD_DL_PIN, 1);
	// Press power key
	mt_set_gpio_dir(GPIO_DT_MD_PWR_KEY_PIN, 1);
	mt_set_gpio_out(GPIO_DT_MD_PWR_KEY_PIN, 1);
	msleep(10);
	cm_relese_rst_signal();

	// Check WDT pin to high
	while(retry>0){
		retry--;
		if(mt_get_gpio_in(GPIO_DT_MD_WDT_PIN)==0)
			msleep(10);
		else
			return 100-retry;
	}
	//msleep(5000); 
	ret = -1;

	return ret;
}

void cm_do_md_rst_and_hold(void)
{
}

void cm_hold_wakeup_md_signal(void)
{
	mt_set_gpio_out(GPIO_DT_AP_WK_MD_PIN, 0);
}

void cm_release_wakeup_md_signal(void)
{
	mt_set_gpio_out(GPIO_DT_AP_WK_MD_PIN, 1);
}

void cm_gpio_setup(void)
{
	// MD wake up AP pin
	mt_set_gpio_pull_enable(GPIO_DT_MD_WK_AP_PIN, !0);
	mt_set_gpio_pull_select(GPIO_DT_MD_WK_AP_PIN, 1);
	mt_set_gpio_dir(GPIO_DT_MD_WK_AP_PIN, 0);
	mt_set_gpio_mode(GPIO_DT_MD_WK_AP_PIN, GPIO_DT_MD_WK_AP_PIN_M_EINT); // EINT3

	// AP wake up MD pin
	mt_set_gpio_mode(GPIO_DT_AP_WK_MD_PIN, GPIO_DT_AP_WK_MD_PIN_M_GPIO); // GPIO Mode
	mt_set_gpio_dir(GPIO_DT_AP_WK_MD_PIN, 1);
	mt_set_gpio_out(GPIO_DT_AP_WK_MD_PIN, 0);

	// Rest MD pin
	mt_set_gpio_mode(GPIO_DT_MD_RST_PIN, GPIO_DT_MD_RST_PIN_M_GPIO); //GPIO202 is reset pin
	mt_set_gpio_pull_enable(GPIO_DT_MD_RST_PIN, 0);
	mt_set_gpio_pull_select(GPIO_DT_MD_RST_PIN, 1);
	mt_set_gpio_dir(GPIO_DT_MD_RST_PIN, 1);
	mt_set_gpio_out(GPIO_DT_MD_RST_PIN, 0);// Default @ reset state

	// MD power key pin
	mt_set_gpio_mode(GPIO_DT_MD_PWR_KEY_PIN, GPIO_DT_MD_PWR_KEY_PIN_M_GPIO); //GPIO 200 is power key
	mt_set_gpio_pull_enable(GPIO_DT_MD_PWR_KEY_PIN, 0);
	mt_set_gpio_dir(GPIO_DT_MD_PWR_KEY_PIN, 0);// Using input floating
	//mt_set_gpio_out(GPIO_DT_MD_PWR_KEY_PIN, 1);// Default @ reset state

	// MD WDT irq pin
	mt_set_gpio_pull_enable(GPIO_DT_MD_WDT_PIN, !0);
	mt_set_gpio_pull_select(GPIO_DT_MD_WDT_PIN, 1);
	mt_set_gpio_dir(GPIO_DT_MD_WDT_PIN, 0);
	mt_set_gpio_mode(GPIO_DT_MD_WDT_PIN, GPIO_DT_MD_WDT_PIN_M_EINT); // EINT9

	// MD Download pin
	//.......
}

void cm_ext_md_rst(void)
{
	cm_hold_rst_signal();
	mt_set_gpio_out(GPIO_OTG_DRVVBUS_PIN, 0); // VBus EMD_VBUS_TMP_PIN
}

void cm_enable_ext_md_wdt_irq(void)
{
	mt65xx_eint_unmask(CUST_EINT_DT_EXT_MD_WDT_NUM);
}

void cm_disable_ext_md_wdt_irq(void)
{
	mt65xx_eint_mask(CUST_EINT_DT_EXT_MD_WDT_NUM);
}

void cm_enable_ext_md_wakeup_irq(void)
{
	mt65xx_eint_unmask(CUST_EINT_DT_EXT_MD_WK_UP_NUM);
}

void cm_disable_ext_md_wakeup_irq(void)
{
	mt65xx_eint_mask(CUST_EINT_DT_EXT_MD_WK_UP_NUM);
}


