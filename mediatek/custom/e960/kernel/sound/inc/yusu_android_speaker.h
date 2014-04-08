/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2009
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


/*******************************************************************************
 *
 * Filename:
 * ---------
 * Yusu_android_speaker.h
 *
 * Project:
 * --------
 *   Yusu
 *
 * Description:
 * ------------
 *   speaker select
 *
 * Author:
 * -------
 *   ChiPeng Chang (mtk02308)
 *
 *
 *------------------------------------------------------------------------------
 * $Revision$
 * $Modtime:$
 * $Log:$
 *
 * 06 17 2012 weiguo.li
 * [ALPS00302429] [Need Patch] [Volunteer Patch]modify speaker driver
 * .
 *
 * 12 14 2011 weiguo.li
 * [ALPS00102848] [Need Patch] [Volunteer Patch] build waring in yusu_android_speaker.h
 * .
 *
 * 11 10 2011 weiguo.li
 * [ALPS00091610] [Need Patch] [Volunteer Patch]chang yusu_android_speaker.c function name and modules use it
 * .
 *
 * 09 28 2011 weiguo.li
 * [ALPS00076254] [Need Patch] [Volunteer Patch]LGE audio driver using Voicebuffer for incall
 * .
 *
 * 07 08 2011 weiguo.li
 * [ALPS00059378] poring lge code to alps(audio)
 * .
 *
 * 07 23 2010 chipeng.chang
 * [ALPS00122386][Music]The playing music is no sound after below steps. 
 * when mode change , record deivce for volume setting.
 *
 * 07 03 2010 chipeng.chang
 * [ALPS00002838][Need Patch] [Volunteer Patch] for speech volume step 
 * modify for headset customization.
 *
 *******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>

#if defined(MT6577)
#include <mach/mt_typedefs.h>
#elif defined(MT6575) 
#include <mach/mt_typedefs.h>
#elif defined(MT6573)
#include <mach/mt_typedefs.h>
#else
#include <mach/mt_typedefs.h>
#endif

#ifndef _YUSU_ANDROID_SPEAKER_H_
#define _YUSU_ANDROID_SPEAKER_H_

  enum EAMP_CONTROL_SUBCOMMAND
 {
	 EAMP_SPEAKER_CLOSE =0,
	 EAMP_SPEAKER_OPEN,
	 EAMP_HEADPHONE_OPEN,
	 EAMP_HEADPHONE_CLOSE,
	 EAMP_EARPIECE_OPEN,
	 EAMP_EARPIECE_CLOSE,
	 EAMP_GETREGISTER_VALUE,
	 EAMP_SETREGISTER_VALUE,
	 EAMP_SETAMP_GAIN,
	 EAMP_GETAMP_GAIN,
	 EAMP_GET_CTRP_NUM ,
	 EAMP_GET_CTRP_BITS,
	 EAMP_GET_CTRP_TABLE,
	 EAMP_SETMODE,
 };

enum AUDIO_AMP_CONTROL_COMMAND{
    AUD_AMP_GET_CTRP_NUM ,
    AUD_AMP_GET_CTRP_BITS,
    AUD_AMP_GET_CTRP_TABLE,
    AUD_AMP_GET_REGISTER,
    AUD_AMP_SET_REGISTER,
    AUD_AMP_SET_AMPGAIN,  // gain is use for low 24bits as external amp , device should base on control point set to AMPLL_CON0_REG
    AUD_AMP_GET_AMPGAIN,
    AUD_AMP_SET_MODE,
    NUM_AUD_AMP_COMMAND
};

typedef struct {
	unsigned long int	command;
	unsigned long int 	param1;
	unsigned long int 	param2;
}AMP_Control;

enum SPEAKER_CHANNEL
{
      Channel_None = 0 ,
      Channel_Right,
      Channel_Left,
      Channel_Stereo
};

bool Speaker_Init(void);
bool Speaker_DeInit(void);
bool Speaker_Register(void);
int  ExternalAmp(void);

void Sound_Speaker_Turnon(int channel);
void Sound_Speaker_Turnoff(int channel);
void Sound_Speaker_SetVolLevel(int level);

void Sound_Headset_Turnon(void);
void Sound_Headset_Turnoff(void);

//now for  kernal use
void AudioAMPDevice_Suspend(void);
void AudioAMPDevice_Resume(void);
// used for AEE beep sound
void AudioAMPDevice_SpeakerLouderOpen(void); //some times kernal need to force  speaker for notification
void AudioAMPDevice_SpeakerLouderClose(void);
void AudioAMPDevice_mute(void);


int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count);




kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size);


#endif


