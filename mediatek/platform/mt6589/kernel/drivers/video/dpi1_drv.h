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

#ifndef __DPI1_DRV_H__
#define __DPI1_DRV_H__

#include "dpi_drv.h"
#include "disp_drv.h"
#include "lcm_drv.h"
#include "hdmi_drv.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DPI_OUTPUT_BIT_NUM_8BITS = 0,
    DPI_OUTPUT_BIT_NUM_10BITS = 1,
    DPI_OUTPUT_BIT_NUM_12BITS = 2
} DPI_OUTPUT_BIT_NUM;

typedef enum
{
    DPI_OUTPUT_CHANNEL_SWAP_RGB = 0,
    DPI_OUTPUT_CHANNEL_SWAP_GBR = 1,
    DPI_OUTPUT_CHANNEL_SWAP_BRG = 2,
    DPI_OUTPUT_CHANNEL_SWAP_RBG = 3,
    DPI_OUTPUT_CHANNEL_SWAP_GRB = 4,
    DPI_OUTPUT_CHANNEL_SWAP_BGR = 5
} DPI_OUTPUT_CHANNEL_SWAP;

typedef enum
{
    DPI_OUTPUT_YC_MAP_RGB_OR_CrYCb = 0, // {R[7:4],G[7:4],B[7:4]} or {Cr[7:4],Y[7:4],Cb[7:4]}
    DPI_OUTPUT_YC_MAP_CYCY         = 4, // {C[11:4],Y[11:4],C[3:0],Y[3:0]}
    DPI_OUTPUT_YC_MAP_YCYC         = 5, // {Y[11:4],C[11:4],Y[3:0],C[3:0]}
    DPI_OUTPUT_YC_MAP_CY           = 6, // {C[11:0],Y[11:0]}
    DPI_OUTPUT_YC_MAP_YC           = 7  // {Y[11:0],C[11:0]}
} DPI_OUTPUT_YC_MAP;

// ---------------------------------------------------------------------------
DPI_STATUS DPI1_Init_PLL(HDMI_VIDEO_RESOLUTION resolution);
DPI_STATUS DPI1_Init(BOOL isDpiPoweredOn);
DPI_STATUS DPI1_Deinit(void);

DPI_STATUS DPI1_Set_DrivingCurrent(LCM_PARAMS *lcm_params);

DPI_STATUS DPI1_PowerOn(void);
DPI_STATUS DPI1_PowerOff(void);

DPI_STATUS DPI1_EnableClk(void);
DPI_STATUS DPI1_DisableClk(void);

DPI_STATUS DPI1_EnableSeqOutput(BOOL enable);
DPI_STATUS DPI1_SetRGBOrder(DPI_RGB_ORDER input, DPI_RGB_ORDER output);

DPI_STATUS DPI1_ConfigPixelClk(DPI_POLARITY polarity, UINT32 divisor, UINT32 duty);
DPI_STATUS DPI1_ConfigDataEnable(DPI_POLARITY polarity);
DPI_STATUS DPI1_ConfigVsync(DPI_POLARITY polarity,
                           UINT32 pulseWidth, UINT32 backPorch, UINT32 frontPorch);
DPI_STATUS DPI1_ConfigHsync(DPI_POLARITY polarity,
                           UINT32 pulseWidth, UINT32 backPorch, UINT32 frontPorch);

DPI_STATUS DPI1_FBSyncFlipWithLCD(BOOL enable);
DPI_STATUS DPI1_SetDSIMode(BOOL enable);
BOOL 	   DPI1_IsDSIMode(void);
DPI_STATUS DPI1_FBSetFormat(DPI_FB_FORMAT format);
DPI_FB_FORMAT DPI1_FBGetFormat(void);
DPI_STATUS DPI1_FBSetSize(UINT32 width, UINT32 height);
DPI_STATUS DPI1_FBEnable(DPI_FB_ID id, BOOL enable);
DPI_STATUS DPI1_FBSetAddress(DPI_FB_ID id, UINT32 address);
DPI_STATUS DPI1_FBSetPitch(DPI_FB_ID id, UINT32 pitchInByte);

DPI_STATUS DPI1_SetFifoThreshold(UINT32 low, UINT32 high);

// Debug
DPI_STATUS DPI1_DumpRegisters(void);

DPI_STATUS DPI1_Capture_Framebuffer(unsigned int pvbuf, unsigned int bpp);

//FM De-sense
DPI_STATUS DPI1_FMDesense_Query(void);
DPI_STATUS DPI1_FM_Desense(unsigned long freq);
DPI_STATUS DPI1_Get_Default_CLK(unsigned int *clk);
DPI_STATUS DPI1_Get_Current_CLK(unsigned int *clk);
DPI_STATUS DPI1_Change_CLK(unsigned int clk);
DPI_STATUS DPI1_Reset_CLK(void);

void DPI1_mipi_switch(bool on);
void DPI1_DisableIrq(void);
void DPI1_EnableIrq(void);
DPI_STATUS DPI1_FreeIRQ(void);

DPI_STATUS DPI1_EnableInterrupt(DISP_INTERRUPT_EVENTS eventID);
DPI_STATUS DPI1_SetInterruptCallback(void (*pCB)(DISP_INTERRUPT_EVENTS eventID));
void DPI1_WaitVSYNC(void);
void DPI1_InitVSYNC(unsigned int vsync_interval);
void DPI1_PauseVSYNC(bool enable);

DPI_STATUS DPI1_ConfigHDMI(void);
DPI_STATUS DPI1_EnableColorBar(void);
DPI_STATUS DPI1_EnableBlackScreen(void);
DPI_STATUS DPI1_DisableInternalPattern(void);
DPI_STATUS DPI1_ESAVVTimingControlLeft(UINT8 offsetOdd, UINT8 widthOdd, UINT8 offsetEven, UINT8 widthEven);
DPI_STATUS DPI1_ESAVVTimingControlRight(UINT8 offsetOdd, UINT8 widthOdd, UINT8 offsetEven, UINT8 widthEven);
DPI_STATUS DPI1_MatrixCoef(UINT16 c00, UINT16 c01, UINT16 c02,
                           UINT16 c10, UINT16 c11, UINT16 c12,
                           UINT16 c20, UINT16 c21, UINT16 c22);
DPI_STATUS DPI1_MatrixPreOffset(UINT16 preAdd0, UINT16 preAdd1, UINT16 preAdd2);
DPI_STATUS DPI1_MatrixPostOffset(UINT16 postAdd0, UINT16 postAdd1, UINT16 postAdd2);
DPI_STATUS DPI1_SetChannelLimit(UINT16 yBottom, UINT16 yTop, UINT16 cBottom, UINT16 cTop);
DPI_STATUS DPI1_CLPFSetting(UINT8 clpfType, BOOL roundingEnable);
DPI_STATUS DPI1_EmbeddedSyncSetting(BOOL embSync_R_Cr, BOOL embSync_G_Y, BOOL embSync_B_Cb,
                                    BOOL esavFInv, BOOL esavVInv, BOOL esavHInv,
                                    BOOL esavCodeMan);
DPI_STATUS DPI1_OutputSetting(DPI_OUTPUT_BIT_NUM outBitNum, BOOL outBitSwap, DPI_OUTPUT_CHANNEL_SWAP outChSwap, DPI_OUTPUT_YC_MAP outYCMap);
// ---------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif // __DPI1_DRV_H__
