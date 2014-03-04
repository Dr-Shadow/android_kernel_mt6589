# Copyright Statement:
#
# This software/firmware and related d.oumentation ("MediaTek Software") are
# prot.oted under relevant copyright laws. The information contained herein
# is.oonfidential and proprietary to MediaTek Inc. and/or its licensors.
# Without the prior written permission of MediaTek i.o. and/or its licensors,
# any reprod.otion, modification, use or disclosure of MediaTek Software,
# and information.oontained herein, in whole or in part, shall be strictly prohibited.

# MediaTek I.o. (C) 2010. All rights reserved.
#
# BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
# THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
# RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
# AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
# NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
# SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
# SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
# THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
# THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
# CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
# SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
# STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
# CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
# AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
# OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
# MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
#
# The following software/firmware and/or related d.oumentation ("MediaTek Software")
# have been modified by MediaTek I.o. All revisions are subject to any receiver's
# appl.oable license agreements with MediaTek Inc.


# drivers/video/mtk/Makefile
#
# Makefile for MT6516 video driver.
#

ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/hdmitx/  \
             -I$(MTK_PATH_SOURCE)/drivers/hdmitx/	\
			 -I$(MTK_PATH_PLATFORM)/drivers/video/  \
             -I$(MTK_PATH_SOURCE)/drivers/video/	\
               -I$(MTK_PATH_PLATFORM)/drivers/hdmitx/  \
             	-I$(MTK_PATH_SOURCE)/drivers/hdmitx/	\
		-DTMFL_LINUX_OS_KERNEL_DRIVER -DTMFL_TDA19989 -DTMFL_NO_RTOS


obj-y	:= hdmitx.o debug.o				\
	hdmi_drv.o					\
	tmbslTDA9989_HDCP.o			\
	tmbslTDA9989_local.o		\
	tmbslTDA9989_State.o		\
	tmdlHdmiTx_local.o			\
	tmbslTDA9989_Edid.o			\
	tmbslTDA9989_InOut.o		\
	tmbslTDA9989_Misc.o			\
	tmdlHdmiTx.o				\
	tmdlHdmiTx_cfg.o			\
	I2C_api.o					\




