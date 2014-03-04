/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
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

#include <mach/mt_reg_base.h>
//Register address define
//#define PMIC_RESERVE_CON2 0xF0007500

#define ACCDET_BASE                 0x00000000
#define TOP_RST_ACCDET_SET 		 ACCDET_BASE + 0x0116
#define TOP_RST_ACCDET_CLR 		 ACCDET_BASE + 0x0118

#define INT_CON_ACCDET           ACCDET_BASE + 0x017E
#define INT_CON_ACCDET_SET		 ACCDET_BASE + 0x0180  //6320 Design
#define INT_CON_ACCDET_CLR       ACCDET_BASE + 0x0182

#define INT_STATUS_ACCDET        ACCDET_BASE + 0x0186

//6320 clock register
#define TOP_CKPDN_SET            ACCDET_BASE + 0x0104
#define TOP_CKPDN_CLR            ACCDET_BASE + 0x0106


#define ACCDET_RSV               ACCDET_BASE + 0x0582

#define ACCDET_CTRL              ACCDET_BASE + 0x0584
#define ACCDET_STATE_SWCTRL      ACCDET_BASE + 0x0586
#define ACCDET_PWM_WIDTH         	 ACCDET_BASE + 0x0588
#define ACCDET_PWM_THRESH        	 ACCDET_BASE + 0x058A
#define ACCDET_EN_DELAY_NUM      ACCDET_BASE + 0x058c
#define ACCDET_DEBOUNCE0         ACCDET_BASE + 0x058E
#define ACCDET_DEBOUNCE1         ACCDET_BASE + 0x0590
#define ACCDET_DEBOUNCE2         ACCDET_BASE + 0x0592
#define ACCDET_DEBOUNCE3         ACCDET_BASE + 0x0594

#define ACCDET_DEFAULT_STATE_RG  ACCDET_BASE + 0x0596


#define ACCDET_IRQ_STS           ACCDET_BASE + 0x0598

#define ACCDET_CONTROL_RG        ACCDET_BASE + 0x059A
#define ACCDET_STATE_RG          ACCDET_BASE + 0x059C

#define ACCDET_CUR_DEB			 ACCDET_BASE + 0x059E
#define ACCDET_RSV_CON0			 ACCDET_BASE + 0x05A0
#define ACCDET_RSV_CON1			 ACCDET_BASE + 0x05A2


/*
#define ACCDET_FSM_STATE         ACCDET_BASE + 0x50
#define ACCDET_CURR_DEBDS        ACCDET_BASE + 0x54
#define ACCDET_TV_START_LINE0    ACCDET_BASE + 0x58
#define ACCDET_TV_END_LINE0      ACCDET_BASE + 0x5C
#define ACCDET_TV_START_LINE1    ACCDET_BASE + 0x60
#define ACCDET_TV_END_LINE1      ACCDET_BASE + 0x64
#define ACCDET_TV_PRE_LINE       ACCDET_BASE + 0x68
#define ACCDET_TV_START_PXL      ACCDET_BASE + 0x6C
#define ACCDET_TV_END_PXL        ACCDET_BASE + 0x70
#define ACCDET_TV_EN_DELAY_NUM   ACCDET_BASE + 0x74
#define ACCDET_TV_DIV_RATE       ACCDET_BASE + 0x78
*/




//Register value define


#define ACCDET_CTRL_EN           (1<<0)
#define ACCDET_MIC_PWM_IDLE      (1<<6)
#define ACCDET_VTH_PWM_IDLE      (1<<5)
#define ACCDET_CMP_PWM_IDLE      (1<<4)
#define ACCDET_CMP_EN            (1<<0)
#define ACCDET_VTH_EN            (1<<1)
#define ACCDET_MICBIA_EN         (1<<2)


#define ACCDET_ENABLE			 (1<<0)
#define ACCDET_DISABLE			 (0<<0)

#define ACCDET_RESET_SET             (1<<4)
#define ACCDET_RESET_CLR             (1<<4)

#define IRQ_CLR_BIT           0x100
#define IRQ_STATUS_BIT        (1<<0)

#define RG_ACCDET_IRQ_SET        (1<<2)
#define RG_ACCDET_IRQ_CLR        (1<<2)
#define RG_ACCDET_IRQ_STATUS_CLR (1<<2)

//CLOCK
#define RG_ACCDET_CLK_SET        (1<<14)
#define RG_ACCDET_CLK_CLR        (1<<14)


#define ACCDET_PWM_EN_SW         (1<<15)
#define ACCDET_MIC_EN_SW         (1<<15)
#define ACCDET_VTH_EN_SW         (1<<15)
#define ACCDET_CMP_EN_SW         (1<<15)

#define ACCDET_SWCTRL_EN         0x07

#define ACCDET_IN_SW             0x10

#define ACCDET_PWM_SEL_CMP       0x00
#define ACCDET_PWM_SEL_VTH       0x01
#define ACCDET_PWM_SEL_MIC       0x10
#define ACCDET_PWM_SEL_SW        0x11
#define ACCDET_SWCTRL_IDLE_EN    (0x07<<4)


#define ACCDET_TEST_MODE5_ACCDET_IN_GPI        (1<<5)
#define ACCDET_TEST_MODE4_ACCDET_IN_SW        (1<<4)
#define ACCDET_TEST_MODE3_MIC_SW        (1<<3)
#define ACCDET_TEST_MODE2_VTH_SW        (1<<2)
#define ACCDET_TEST_MODE1_CMP_SW        (1<<1)
#define ACCDET_TEST_MODE0_GPI        (1<<0)



//#define ACCDET_DEFVAL_SEL        (1<<15)


