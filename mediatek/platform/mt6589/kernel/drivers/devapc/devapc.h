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

 
#define MOD_NO_IN_1_DEVAPC                  16
#define DEVAPC_MODULE_MAX_NUM               32  
#define DEVAPC_TAG                          "DEVAPC"
#define MAX_TIMEOUT                         100
#define ABORT_EMI                           0x20000008 
 
 
// device apc attribute
 typedef enum
 {
     E_L0=0,
     E_L1,
     E_L2,
     E_L3,
     E_MAX_APC_ATTR
 }APC_ATTR;
 
 // device apc index 
 typedef enum
 {
     E_DEVAPC0=0,
     E_DEVAPC1,  
     E_DEVAPC2,
     E_DEVAPC3,
     E_DEVAPC4,
     E_MAX_DEVAPC
 }DEVAPC_NUM;
 
 // domain index 
 typedef enum
 {
     E_AP_MCU = 0,
     E_MD1_MCU ,
     E_MD2_MCU , 
     E_MM_MCU ,
     E_MAX
 }E_MASK_DOM;
 
 
 typedef struct {
     const char      *device_name;
     bool            forbidden;
 } DEVICE_INFO;
 
  
 static DEVICE_INFO D_APC0_Devices[] = {
     {"0",              FALSE},
     {"1",              TRUE},
     {"2",              TRUE},
     {"3",              FALSE},
     {"4",              TRUE},
     {"5",              TRUE},
     {"6",              TRUE},
     {"7",              TRUE},
     {"8",              TRUE},
     {"9",              TRUE},
     {"10",             FALSE},
     {"11",             TRUE},
     {"12",             TRUE},
     {"13",             TRUE},
     {"14",             TRUE},
     {"15",             TRUE},
     {"16",             TRUE},
     {"17",             TRUE},
     {"18",             TRUE},
     {"19",             TRUE},
     {"20",             TRUE},
     {"21",             FALSE},
     {"22",             TRUE},
     {"23",             TRUE},
     {NULL,             FALSE},
 };
 
 static DEVICE_INFO D_APC1_Devices[] = {
     {"0",              TRUE},
     {"1",              FALSE},
     {"2",              TRUE},
     {"3",              FALSE},
     {"4",              TRUE},
     {"5",              TRUE},
     {"6",              TRUE},
     {"7",              TRUE},
     {"(Reserved)",     FALSE},
     {"(Reserved)",     FALSE},
     {"10",             TRUE},
     {"11",             TRUE},
     {"(Reserved)",     FALSE},
     {"13",             TRUE},
     {"14",             TRUE},
     {"(Reserved)",     FALSE},
     {"16",             TRUE},
     {"17",             TRUE},
     {"18",             TRUE},
     {NULL,             FALSE},
 };
 
 
 static DEVICE_INFO D_APC2_Devices[] = {
     {"0",              TRUE},
     {"1",              FALSE},
     {"2",              TRUE},
     {"3",              FALSE},
     {"4",              FALSE},
     {"5",              FALSE},
     {"6",              TRUE},
     {"(reserved)",     FALSE},
     {"8",              TRUE},
     {"9",              FALSE},
     {"10",             TRUE},
     {"11",             TRUE},
     {"12",             TRUE},
     {"13",             FALSE},
     {"14",             TRUE},
     {"15",             FALSE},
     {"16",             TRUE},
     {"17",             TRUE},
     {"18",             TRUE},
     {"19",             TRUE},
     {"20",             FALSE},
     {"21",             TRUE},
     {"22",             TRUE},
     {"23",             TRUE},
     {"24",             TRUE},
     {"25",             TRUE},
     {"26",             TRUE},
     {"27",             TRUE},
     {"28",             TRUE},
     {"29",             TRUE},
     {NULL,             TRUE},
 };

 
 static DEVICE_INFO D_APC3_Devices[] = {
     {"0",              TRUE},
     {"1",              TRUE},
     {"2",              TRUE},
     {"3",              TRUE},
     {"4",              TRUE},
     {"5",              TRUE},
     {"6",              TRUE},
     {"7",              TRUE},
     {"8",              TRUE},
     {"9",              TRUE},
     {"10",             TRUE},
     {"11",             TRUE},
     {"12",             TRUE},
     {"13",             TRUE},
     {"14",             TRUE},
     {"15",             TRUE},
     {"16",             TRUE},
     {"17",             TRUE},
     {"18",             TRUE},
     {"19",             TRUE},
     {"20",             TRUE},
     {NULL,             FALSE},
 };
 
 
 static DEVICE_INFO D_APC4_Devices[] = {
     {"0",              TRUE},
     {"1",              TRUE},
     {"2",              TRUE},
     {"3",              TRUE},
     {"4",              TRUE},
     {"5",              TRUE},
     {"(reserved)",     FALSE},
     {"(reserved)",     FALSE},
     {"8",              TRUE},
     {"9",              TRUE},
     {"10",             TRUE},
     {"(reserved)",     FALSE},
     {"(reserved)",     FALSE},
     {"(reserved)",     FALSE},
     {"(reserved)",     FALSE},
     {"(reserved)",     FALSE},
     {"16",             TRUE},
     {"17",             TRUE},
     {"18",             TRUE},
     {"19",             TRUE},
     {"20",             TRUE},
     {"21",             TRUE},
     {"22",             FALSE},
     {"23",             TRUE},
     {NULL,             FALSE},
 };                         
 
 
#define SET_SINGLE_MODULE(apcnum, domnum, index, module, permission_control)     \
 {                                                                               \
     mt65xx_reg_sync_writel(readl(DEVAPC##apcnum##_D##domnum##_APC_##index) & ~(0x3 << (2 * module)), DEVAPC##apcnum##_D##domnum##_APC_##index); \
     mt65xx_reg_sync_writel(readl(DEVAPC##apcnum##_D##domnum##_APC_##index) | (permission_control << (2 * module)),DEVAPC##apcnum##_D##domnum##_APC_##index); \
 }                                                                               \
 
#define UNMASK_SINGLE_MODULE_IRQ(apcnum, domnum, module_index)                  \
 {                                                                               \
     mt65xx_reg_sync_writel(readl(DEVAPC##apcnum##_D##domnum##_VIO_MASK) & ~(module_index),      \
         DEVAPC##apcnum##_D##domnum##_VIO_MASK);                                 \
 }                                                                               \
 
#define CLEAR_SINGLE_VIO_STA(apcnum, domnum, module_index)                     \
 {                                                                               \
     mt65xx_reg_sync_writel(readl(DEVAPC##apcnum##_D##domnum##_VIO_STA) | (module_index),        \
         DEVAPC##apcnum##_D##domnum##_VIO_STA);                                  \
 }                                                                               \

 
